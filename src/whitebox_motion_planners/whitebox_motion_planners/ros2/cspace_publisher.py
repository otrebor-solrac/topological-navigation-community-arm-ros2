import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
import numpy as np
import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from ..collision.foam_collider import FoamCollider
from ..collision.grid_discretizer import GridDiscretizer
from ..kinematics import get_kinematics

class CSpaceVoxelPublisher(Node):
    def __init__(self):
        super().__init__('cspace_voxel_publisher')
        self.cached_voxels_msg = None
        self.cache_dirty = True
        self.last_sub_count = 0
        self.last_publish_time = 0.0
        # Seconds between periodic fallback republishes (covers race conditions on page reload)
        self._republish_interval_s = 8.0
        
        # 1. Initialize parameters
        (robot_type, step_size, use_horizontal, use_obstacles, 
         thinning_dist, acm_margin, robot_urdf, obstacles_urdf, cache_dir,
         base_yaw_offset, shoulder_pitch_offset, elbow_pitch_offset,
         base_yaw_dir, shoulder_pitch_dir, elbow_pitch_dir, link_lengths,
         singularity_threshold) = self._init_parameters()
        
        self.base_yaw_offset = base_yaw_offset
        self.shoulder_pitch_offset = shoulder_pitch_offset
        self.elbow_pitch_offset = elbow_pitch_offset
        self.base_yaw_dir = base_yaw_dir
        self.shoulder_pitch_dir = shoulder_pitch_dir
        self.elbow_pitch_dir = elbow_pitch_dir
        
        # 2. Get kinematics model
        self.kinematics = get_kinematics(robot_type, use_horizontal_constraint=use_horizontal, link_lengths=link_lengths)
        
        # 3. Setup collider and load obstacles
        obstacles_hash = self._setup_collider_and_obstacles(
            robot_type, use_obstacles, thinning_dist, robot_urdf, obstacles_urdf, singularity_threshold
        )
        
        # 4. Setup grid discretizer
        self.grid = GridDiscretizer(step_size_deg=step_size, num_dof=self.kinematics.get_dof())
        
        # 5. Configure persistent cache
        self._setup_cache(step_size, thinning_dist, obstacles_hash, cache_dir)
        
        # 6. Configure ROS 2 publishers, services, and timers
        self._setup_ros_interfaces(step_size)

    def _init_parameters(self):
        """
        Declare and read ROS 2 parameters.
        
        :return: Tuple containing parameter values.
        """
        # Robot model type (e.g. 'community_arm' or 'open_manipulator')
        self.declare_parameter('robot_type', 'community_arm')
        # Discrete grid step size in degrees for C-Space discretization
        self.declare_parameter('step_size_deg', 15.0) 
        # Enable horizontal leveling kinematic constraint for end-effector
        self.declare_parameter('use_horizontal_constraint', True)
        # Enable environment obstacle collision checks
        self.declare_parameter('use_obstacles', True)
        # Collision sphere radius offset distance (thinning) in meters
        self.declare_parameter('sphere_thinning_dist', 0.015)
        # ACM margin (meters) added to sum of radii when computing allowed collision pairs
        self.declare_parameter('acm_margin', 0.005)
        # Optional override path to the robot URDF file
        self.declare_parameter('robot_urdf_path', '')
        # Optional override path to the obstacles URDF file
        self.declare_parameter('obstacles_urdf_path', '')
        # Optional override directory path to save/load persistent cache
        self.declare_parameter('cache_dir', '')
        
        # Link lengths for kinematics
        self.declare_parameter('link_lengths.base_height', 0.130)
        self.declare_parameter('link_lengths.lower_shank', 0.140)
        self.declare_parameter('link_lengths.upper_shank', 0.140)
        self.declare_parameter('link_lengths.gripper_dx', 0.05467)
        self.declare_parameter('link_lengths.gripper_dz', 0.0)
        self.declare_parameter('link_lengths.gripper_k_elbow', 0.0)
        
        # Configurable joint mapping to global world frame
        self.declare_parameter('joint_offsets.base_yaw', 0.0)
        self.declare_parameter('joint_offsets.shoulder_pitch', 0.0)
        self.declare_parameter('joint_offsets.elbow_pitch', 0.0)
        self.declare_parameter('joint_directions.base_yaw', 1)
        self.declare_parameter('joint_directions.shoulder_pitch', 1)
        self.declare_parameter('joint_directions.elbow_pitch', 1)
        self.declare_parameter('singularity_threshold', 0.0)
        
        import math
        link_lengths = {
            'base_height': self.get_parameter('link_lengths.base_height').value,
            'lower_shank': self.get_parameter('link_lengths.lower_shank').value,
            'upper_shank': self.get_parameter('link_lengths.upper_shank').value,
            'gripper_dx': self.get_parameter('link_lengths.gripper_dx').value,
            'gripper_dz': self.get_parameter('link_lengths.gripper_dz').value,
            'gripper_k_elbow': self.get_parameter('link_lengths.gripper_k_elbow').value
        }
        return (
            self.get_parameter('robot_type').value,
            self.get_parameter('step_size_deg').value,
            self.get_parameter('use_horizontal_constraint').value,
            self.get_parameter('use_obstacles').value,
            self.get_parameter('sphere_thinning_dist').value,
            self.get_parameter('acm_margin').value,
            self.get_parameter('robot_urdf_path').value,
            self.get_parameter('obstacles_urdf_path').value,
            self.get_parameter('cache_dir').value,
            math.radians(self.get_parameter('joint_offsets.base_yaw').value),
            math.radians(self.get_parameter('joint_offsets.shoulder_pitch').value),
            math.radians(self.get_parameter('joint_offsets.elbow_pitch').value),
            float(self.get_parameter('joint_directions.base_yaw').value),
            float(self.get_parameter('joint_directions.shoulder_pitch').value),
            float(self.get_parameter('joint_directions.elbow_pitch').value),
            link_lengths,
            self.get_parameter('singularity_threshold').value
        )

    def _setup_collider_and_obstacles(
        self, 
        robot_type, 
        use_obstacles, 
        thinning_dist, 
        robot_urdf, 
        obstacles_urdf,
        singularity_threshold
    ) -> str:
        """
        Configure the robot collider and optionally load environment obstacles.
        
        :param robot_type: Type of the robot.
        :param use_obstacles: Flag to load environment obstacles from URDF.
        :param thinning_dist: Safety distance to shrink the collision spheres.
        :param robot_urdf: User-defined path to robot URDF.
        :param obstacles_urdf: User-defined path to obstacles URDF.
        :param singularity_threshold: Threshold below which states are marked singular.
        :return: MD5 hash of the obstacles URDF file, or "no_obstacles".
        """
        # Resolve robot URDF dynamically if not provided
        if not robot_urdf:
            if robot_type == 'community_arm':
                try:
                    pkg_share = get_package_share_directory('community_robot_arm')
                    robot_urdf = os.path.join(pkg_share, 'urdf', 'spherized', 'community_robot_arm_slim_spherized.urdf')
                except Exception as e:
                    self.get_logger().warn(f"Could not resolve robot URDF path: {e}")
                    robot_urdf = None

        self.collider = FoamCollider(
            urdf_path=robot_urdf,
            sphere_thinning_dist=thinning_dist
        )
        self.collider.singularity_threshold = singularity_threshold
        self.collider.set_joint_transforms(
            offset_base_yaw=self.base_yaw_offset,
            offset_shoulder_pitch=self.shoulder_pitch_offset,
            offset_elbow_pitch=self.elbow_pitch_offset,
            dir_base_yaw=self.base_yaw_dir,
            dir_shoulder_pitch=self.shoulder_pitch_dir,
            dir_elbow_pitch=self.elbow_pitch_dir
        )
        
        obstacles_hash = "no_obstacles"
        if use_obstacles:
            # Resolve obstacles URDF dynamically if not provided
            if not obstacles_urdf:
                try:
                    pkg_share = get_package_share_directory('community_robot_arm')
                    obstacles_urdf = os.path.join(pkg_share, 'urdf', 'spherized', 'obstacles', 'box_obstacle_spherized.urdf')
                except Exception as e:
                    self.get_logger().error(f"Could not resolve obstacles URDF path: {e}")
                    obstacles_urdf = None

            if obstacles_urdf and os.path.exists(obstacles_urdf):
                try:
                    self.get_logger().info(f"CSpace Voxelizer: Loading environment obstacles from: {obstacles_urdf}")
                    obstacles = self._load_obstacles_from_urdf(obstacles_urdf)
                    
                    for center, radius in obstacles:
                        self.collider.add_obstacle(center, radius)
                        self.get_logger().info(f"CSpace Voxelizer: Added obstacle sphere: center={center}, radius={radius:.3f}")
                        
                    # Calculate MD5 hash of obstacles URDF to invalidate cache
                    if "no_obstacles" in os.path.basename(obstacles_urdf):
                        obstacles_hash = "no_obstacles"
                    else:
                        import hashlib
                        hash_md5 = hashlib.md5()
                        with open(obstacles_urdf, "rb") as f:
                            for chunk in iter(lambda: f.read(4096), b""):
                                hash_md5.update(chunk)
                        obstacles_hash = hash_md5.hexdigest()[:8]
                except Exception as e:
                    self.get_logger().error(f"CSpace Voxelizer: Failed to load obstacles: {e}")
            else:
                self.get_logger().warn(f"CSpace Voxelizer: Obstacles URDF file not found at: {obstacles_urdf}")
                
        return obstacles_hash

    def _setup_cache(
        self, 
        step_size, 
        thinning_dist, 
        obstacles_hash, 
        cache_dir
    ):
        """
        Configure the persistent cache directory and load cached data.
        
        :param step_size: Discrete grid step size.
        :param thinning_dist: Safety distance offset.
        :param obstacles_hash: MD5 hash of environment obstacles.
        :param cache_dir: User-defined cache directory.
        """
        if not cache_dir:
            if os.path.exists('/home/ros_ws/cspace_cache'):
                self.cache_dir = '/home/ros_ws/cspace_cache'
            else:
                try:
                    # Use whitebox package directory
                    pkg_share = get_package_share_directory('whitebox_motion_planners')
                    self.cache_dir = os.path.join(pkg_share, 'cspace_cache')
                except Exception:
                    # Fallback to local python module dir
                    src_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
                    self.cache_dir = os.path.join(src_dir, 'cspace_cache')
        else:
            self.cache_dir = cache_dir

        os.makedirs(self.cache_dir, exist_ok=True)
        sing_thresh = self.collider.singularity_threshold
        if sing_thresh > 0.0:
            self.cache_filename = f"cspace_cache_{step_size}deg_{thinning_dist}m_{obstacles_hash}_singularity{sing_thresh}.json"
        else:
            self.cache_filename = f"cspace_cache_{step_size}deg_{thinning_dist}m_{obstacles_hash}.json"
        self.cache_filepath = os.path.join(self.cache_dir, self.cache_filename)
        
        self.get_logger().info(f"Cache filepath: {self.cache_filepath}")
        
        self.warned_no_cache = False
        self._load_cspace_if_exists()

    def _setup_ros_interfaces(self, step_size):
        """
        Setup ROS 2 publishers, services, and timers.
        
        :param step_size: Discrete grid step size.
        """

        # String (JSON) publisher for Rosbridge
        self.publisher_ = self.create_publisher(String, '/cspace_voxels', 10)

        from rclpy.qos import QoSProfile, DurabilityPolicy
        desc_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.obstacles_desc_pub = self.create_publisher(
            String,
            '/obstacles_description',
            desc_qos
        )
        
        # Timer to publish voxels when there are active subscribers (e.g. web dashboard)
        self.timer = self.create_timer(0.5, self.publish_voxels)

        # Register service to generate the cache on demand
        self.srv = self.create_service(Trigger, 'generate_cspace', self.generate_cspace_callback)

        # Dashboard control interface
        self.web_cmd_sub = self.create_subscription(
            String,
            '/web_commands',
            self.web_command_callback,
            10
        )

        self.get_logger().info(f"C-Space Voxelizer started (Resolution: {step_size} deg)")

    def _load_cspace_if_exists(self) -> bool:
        """
        Load C-Space cache from disk if it exists.

        :return: True if cache loaded successfully, False otherwise.
        """
        if os.path.exists(self.cache_filepath):
            self.get_logger().info(f"[CSpace] Cache found at: {self.cache_filepath}")
            try:
                with open(self.cache_filepath, 'r') as f:
                    cached_json = f.read()
                
                # Validate JSON syntax and structure
                raw_data = json.loads(cached_json)
                
                # Log what keys are present and their counts
                if isinstance(raw_data, dict):
                    for k, v in raw_data.items():
                        count = len(v) if isinstance(v, list) else v
                        self.get_logger().info(f"[CSpace] Cache key '{k}': {count}")
                
                if isinstance(raw_data, dict) and "self_collision_voxels" in raw_data and "obstacle_voxels" in raw_data:
                    if "step_rad" not in raw_data:
                        self.get_logger().info(f"[CSpace] step_rad missing in cache, injecting: {self.grid.step_rad}")
                        raw_data["step_rad"] = float(self.grid.step_rad)
                        cached_json = json.dumps(raw_data)
                    msg = String()
                    msg.data = cached_json
                    self.cached_voxels_msg = msg
                    self.get_logger().info(f"[CSpace] Cache format OK. self_collision={len(raw_data.get('self_collision_voxels',[]))}, obstacle={len(raw_data.get('obstacle_voxels',[]))}, forbidden={len(raw_data.get('forbidden_voxels',[]))}")
                else:
                    # Old cache format! Auto-regenerate using the new Rust solver
                    self.get_logger().warn(f"[CSpace] Old cache format detected (keys: {list(raw_data.keys()) if isinstance(raw_data, dict) else type(raw_data)}). Auto-regenerating...")
                    cspace_data = self._compute_cspace_voxels()
                    self._save_cspace_cache(cspace_data)
                
                self.cache_dirty = True
                self.get_logger().info("[CSpace] C-Space loaded from cache successfully!")
                
                return True
            
            except Exception as e:
                self.get_logger().error(f"Error loading cache: {e}.")
                self.cached_voxels_msg = None
        else:
            self.get_logger().warn("No C-Space cache file found.")
            self.get_logger().info("The planner will use real-time collisions and the dashboard will not display obstacles.")
            self.get_logger().info("You can generate the cache by calling the service: ros2 service call /generate_cspace std_srvs/srv/Trigger")
        
        return False

    def _compute_cspace_voxels(self) -> list:
        """
        Iterate through all grid states in parallel using Rust solver,
        falling back to sequential Python if execution fails.

        :return: List of forbidden configuration coordinates [q0, q1, q2].
        """
        states = self.grid.get_all_states()
        total_states = len(states)
        
        # 1. Try Rust solver
        rust_binary = '/home/ros_ws/src/tools/cspace_solver/target/release/cspace_solver'
        if not os.path.exists(rust_binary):
            # Fallback path relative to source directory
            src_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
            rust_binary = os.path.join(src_dir, 'tools', 'cspace_solver', 'target', 'release', 'cspace_solver')
            
        if os.path.exists(rust_binary):
            self.get_logger().info(f"Using Rust parallel C-Space solver: {rust_binary}")
            try:
                import subprocess
                
                # Format JSON input payload for the Rust binary
                input_data = {
                    'joints': [
                        {
                            'name': jinfo['name'],
                            'parent': jinfo['parent'],
                            'child': child,
                            'static_t': jinfo['static_T'].tolist(),
                            'axis': [float(x) for x in jinfo['axis']]
                        }
                        for child, jinfo in self.collider.urdf_parser.joints.items()
                    ],
                    'root_link': self.collider.urdf_parser.root_link,
                    'thinned_spheres': [
                        {
                            'link': s['link'],
                            'local_c': s['local_center'].tolist(),
                            'radius': float(s['radius'])
                        }
                        for s in self.collider.urdf_parser.thinned_spheres
                    ],
                    'active_pairs': list(self.collider.urdf_parser.active_checking_pairs),
                    'obstacles': [
                        {
                            'center': obs.center.tolist(),
                            'radius': float(obs.radius)
                        }
                        for obs in self.collider.spherical_obstacles
                    ],
                    'steps_per_circle': int(self.grid.steps_per_circle),
                    'num_dof': int(self.grid.num_dof),
                    'step_rad': float(self.grid.step_rad),
                    'offset_base_yaw': float(self.base_yaw_offset),
                    'offset_shoulder_pitch': float(self.shoulder_pitch_offset),
                    'offset_elbow_pitch': float(self.elbow_pitch_offset),
                    'dir_base_yaw': float(self.base_yaw_dir),
                    'dir_shoulder_pitch': float(self.shoulder_pitch_dir),
                    'dir_elbow_pitch': float(self.elbow_pitch_dir),
                }
                
                json_input = json.dumps(input_data)
                
                # Run the Rust binary and pass the JSON via stdin
                process = subprocess.Popen(
                    [rust_binary],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                stdout, stderr = process.communicate(input=json_input)
                
                if process.returncode == 0:
                    output_data = json.loads(stdout)
                    output_data["step_rad"] = float(self.grid.step_rad)
                    self.get_logger().info(f"Rust C-Space solver success! Found {len(output_data.get('forbidden_voxels', []))}/{total_states} forbidden states.")
                    return output_data
                else:
                    self.get_logger().error(f"Rust solver failed (exit code {process.returncode}): {stderr}")
            except Exception as e:
                self.get_logger().error(f"Failed to execute Rust solver: {e}")
                
        # 2. Fallback to Python sequential computation
        self.get_logger().warn("Falling back to Python sequential C-Space computation...")
        forbidden_voxels = []
        self_collision_voxels = []
        obstacle_voxels = []
        for idx, q_discrete in enumerate(states):
            if idx % 10000 == 0 and idx > 0:
                self.get_logger().info(f"Progress: {idx}/{total_states} states processed...")
                
            q_radians = self.grid.get_radians(q_discrete)
            
            # Convert q_radians (World) to URDF joint coordinates
            yaw_w, pitch1_w, pitch2_w = q_radians[:3] if len(q_radians) >= 3 else (q_radians[0], q_radians[1], 0.0)
            base_yaw = self.base_yaw_offset + self.base_yaw_dir * yaw_w
            shoulder_pitch = self.shoulder_pitch_offset + self.shoulder_pitch_dir * pitch1_w
            elbow_pitch = self.elbow_pitch_offset + self.elbow_pitch_dir * (pitch1_w + pitch2_w)
            q_urdf = (base_yaw, shoulder_pitch, elbow_pitch)
            
            is_self_collision = False
            if self.collider.urdf_parser is not None:
                is_self_collision = self.collider.urdf_parser.check_self_collision(q_urdf)
            
            q0 = (q_radians[0] + np.pi) % (2 * np.pi) - np.pi
            q1 = (q_radians[1] + np.pi) % (2 * np.pi) - np.pi
            q2 = (q_radians[2] + np.pi) % (2 * np.pi) - np.pi if len(q_radians) > 2 else 0.0
            voxel = [round(float(q0), 3), round(float(q1), 3), round(float(q2), 3)]
            
            if is_self_collision:
                forbidden_voxels.append(voxel)
                self_collision_voxels.append(voxel)
            else:
                is_obs_collision = False
                obstacles_tuples = [(obs.center, obs.radius) for obs in self.collider.spherical_obstacles]
                if self.collider.urdf_parser is not None:
                    is_obs_collision = self.collider.urdf_parser.check_obstacle_collision(q_urdf, obstacles_tuples)
                
                if is_obs_collision:
                    forbidden_voxels.append(voxel)
                    obstacle_voxels.append(voxel)
                
        self.get_logger().info(f"Finished processing C-Space. Found {len(forbidden_voxels)} forbidden states.")
        return {
            "forbidden_voxels": forbidden_voxels,
            "self_collision_voxels": self_collision_voxels,
            "obstacle_voxels": obstacle_voxels,
            "step_rad": float(self.grid.step_rad)
        }

    def _save_cspace_cache(self, cspace_data: dict) -> bool:
        """
        Serialize C-space data dictionary and save it to the persistent cache file.

        :param cspace_data: Dictionary containing forbidden, self_collision and obstacle voxels.
        :return: True if cache saved successfully, False otherwise.
        """
        serialized_data = json.dumps(cspace_data)
        try:
            with open(self.cache_filepath, 'w') as f:
                f.write(serialized_data)
            self.get_logger().info(f"C-Space successfully saved to cache: {self.cache_filepath}")
            
            msg = String()
            msg.data = serialized_data
            self.cached_voxels_msg = msg
            self.cache_dirty = True
            return True
        except Exception as e:
            self.get_logger().error(f"Error writing cache to disk: {e}")
            return False

    def generate_cspace_callback(self, request, response):
        """
        Service callback for /generate_cspace to compute and save the cache in the background.

        :param request: The Trigger service request.
        :param response: The Trigger service response.
        :return: The populated service response.
        """
        self.get_logger().info("Starting C-Space cache generation...")
        
        cspace_data = self._compute_cspace_voxels()
        success = self._save_cspace_cache(cspace_data)
        
        if success:
            response.success = True
            response.message = f"C-Space generated successfully. Saved in {self.cache_filepath}."
        else:
            response.success = False
            response.message = f"Failed to save C-Space cache to disk: {self.cache_filepath}"
            
        return response

    def publish_voxels(self):
        """
        Publish voxels to the topic only when there are active subscribers.

        Republish is triggered by:
        1. Any change in subscriber count (increase OR decrease-then-increase).
        2. Cache dirty flag (new cache loaded or obstacles changed).
        3. Periodic fallback every ``_republish_interval_s`` seconds — covers the
           race condition where React StrictMode's rapid unsubscribe/resubscribe
           cycle happens faster than the 500 ms timer, so the timer never observes
           sub_count == 0 and never resets last_sub_count.
        """
        sub_count = self.publisher_.get_subscription_count()
        msg_is_set = self.cached_voxels_msg is not None

        should_publish = False
        if msg_is_set:
            if sub_count > 0:
                now_s = self.get_clock().now().nanoseconds / 1e9
                # Trigger 1: subscriber count changed in any direction
                if sub_count != self.last_sub_count:
                    should_publish = True
                    self.get_logger().debug(
                        f"[voxelizer] Sub count changed {self.last_sub_count} → {sub_count}, republishing."
                    )
                # Trigger 2: cache was refreshed
                if self.cache_dirty:
                    should_publish = True
                # Trigger 3: periodic fallback to handle page-reload race conditions
                if (now_s - self.last_publish_time) >= self._republish_interval_s:
                    should_publish = True
                self.last_sub_count = sub_count
            else:
                # No subscribers — reset so the very next subscriber gets fresh data
                self.last_sub_count = 0

        if should_publish:
            self.publisher_.publish(self.cached_voxels_msg)
            self.cache_dirty = False
            self.last_publish_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().debug(
                f"Published C-space voxels to {sub_count} subscribers "
                f"(size: {len(self.cached_voxels_msg.data)} chars)"
            )
        elif not msg_is_set:
            if not getattr(self, 'warned_no_cache', False):
                self.get_logger().warn("publish_voxels: self.cached_voxels_msg is None, cannot publish.")
                self.warned_no_cache = True

    def _load_obstacles_from_urdf(self, urdf_path: str) -> list:
        """
        Load obstacles from URDF file and add them to the collider.
        
        :param urdf_path: Absolute path to the URDF file.
        :return: List of obstacle spheres (center, radius).
        """
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            return self._load_obstacles_from_xml_root(root)
        except Exception as e:
            self.get_logger().error(f"Failed to parse XML from URDF: {e}")
            return []

    def _load_obstacles_from_xml_root(self, root: ET.Element) -> list:
        """
        Load obstacles from an ElementTree XML root element.
        """
        # Parse joints to get child link origins relative to world
        link_positions = {'world': (0.0, 0.0, 0.0)}
        
        for joint in root.findall('joint'):
            parent_el = joint.find('parent')
            child_el = joint.find('child')
            
            if parent_el is None or child_el is None:
                continue
            
            parent = parent_el.get('link')
            child = child_el.get('link')
            origin = joint.find('origin')
            xyz_str = origin.get('xyz') if origin is not None else "0 0 0"
            xyz = [float(x) for x in xyz_str.split()]
            
            # If parent link position is known, accumulate
            if parent in link_positions:
                p_pos = link_positions[parent]
                link_positions[child] = (
                    p_pos[0] + xyz[0],
                    p_pos[1] + xyz[1],
                    p_pos[2] + xyz[2]
                )
            else:
                link_positions[child] = tuple(xyz)
                
        obstacles = []
        # Parse links for collision spheres
        for link in root.findall('link'):
            link_name = link.get('name')
            link_pos = link_positions.get(link_name, (0.0, 0.0, 0.0))
            
            for collision in link.findall('collision'):
                origin = collision.find('origin')
                geometry = collision.find('geometry')
                if geometry is not None:
                    sphere = geometry.find('sphere')
                    if sphere is not None:
                        radius = float(sphere.get('radius'))
                        xyz_str = origin.get('xyz') if origin is not None else "0 0 0"
                        offset = [float(x) for x in xyz_str.split()]
                        
                        # Absolute center position
                        abs_center = (
                            link_pos[0] + offset[0],
                            link_pos[1] + offset[1],
                            link_pos[2] + offset[2]
                        )
                        obstacles.append((abs_center, radius))
        return obstacles

    def web_command_callback(self, msg: String):
        """
        Processes commands received from the web dashboard.
        """
        try:
            data = json.loads(msg.data)
            action = data.get("action")
            if action == "change_cspace":
                obstacle_type = data.get("obstacle_type")
                step_size = float(data.get("step_size_deg"))
                
                self.get_logger().info(f"Changing C-space dynamically to: {obstacle_type} at {step_size}deg")
                
                # 1. Update grid step size
                self.grid = GridDiscretizer(step_size_deg=step_size, num_dof=self.kinematics.get_dof())
                
                # 2. Update obstacles
                self.collider.spherical_obstacles = []
                obstacles_hash = "no_obstacles"
                obstacles_urdf_content = '<?xml version="1.0"?><robot name="obstacles"><link name="root"/></robot>'
                if obstacle_type != "no_obstacles":
                    try:
                        pkg_share = get_package_share_directory('community_robot_arm')
                        obstacles_urdf = os.path.join(pkg_share, 'urdf', 'spherized', 'obstacles', f"{obstacle_type}_spherized.urdf")
                        if os.path.exists(obstacles_urdf):
                            with open(obstacles_urdf, 'r') as infp:
                                obstacles_urdf_content = infp.read()
                            obstacles = self._load_obstacles_from_urdf(obstacles_urdf)
                            for center, radius in obstacles:
                                self.collider.add_obstacle(center, radius)
                            
                            import hashlib
                            hash_md5 = hashlib.md5()
                            with open(obstacles_urdf, "rb") as f:
                                for chunk in iter(lambda: f.read(4096), b""):
                                    hash_md5.update(chunk)
                            obstacles_hash = hash_md5.hexdigest()[:8]
                        else:
                            self.get_logger().error(f"Obstacle URDF not found: {obstacles_urdf}")
                    except Exception as e:
                        self.get_logger().error(f"Failed to load obstacles dynamically: {e}")

                try:
                    desc_msg = String()
                    desc_msg.data = obstacles_urdf_content
                    self.obstacles_desc_pub.publish(desc_msg)
                    self.get_logger().info(f"Published dynamically reloaded obstacles description to RViz2 ({obstacle_type})")
                except Exception as e:
                    self.get_logger().error(f"Failed to publish obstacles description: {e}")
                
                # 3. Reload cache
                self._setup_cache(step_size, self.collider.urdf_parser.min_dist, obstacles_hash, self.cache_dir)
                
                # 4. Force republishing
                self.publish_voxels()

            elif action == "move_obstacle":
                obstacle_type = data.get("obstacle_type", "box_obstacle")
                pos_xyz = data.get("position_xyz", [0.3, 0.0, 0.15])
                step_size = float(data.get("step_size_deg", 15.0))
                
                self.get_logger().info(f"Moving obstacle '{obstacle_type}' center to target position {pos_xyz} at {step_size}deg")
                
                self.grid = GridDiscretizer(step_size_deg=step_size, num_dof=self.kinematics.get_dof())
                obstacles_urdf_content = '<?xml version="1.0"?><robot name="obstacles"><link name="root"/></robot>'
                if obstacle_type != "no_obstacles":
                    try:
                        pkg_share = get_package_share_directory('community_robot_arm')
                        obstacles_urdf = os.path.join(pkg_share, 'urdf', 'spherized', 'obstacles', f"{obstacle_type}_spherized.urdf")
                        if os.path.exists(obstacles_urdf):
                            tree = ET.parse(obstacles_urdf)
                            root = tree.getroot()
                            
                            # 1. Parse original obstacle spheres to compute current center of mass / centroid
                            orig_spheres = self._load_obstacles_from_xml_root(root)
                            if orig_spheres:
                                centers = np.array([c for c, r in orig_spheres])
                                orig_centroid = np.mean(centers, axis=0)
                            else:
                                orig_centroid = np.array([0.0, 0.0, 0.0])
                                
                            # 2. Compute shift required to move centroid to target pos_xyz
                            target_pos = np.array(pos_xyz, dtype=float)
                            shift = target_pos - orig_centroid
                            
                            # 3. Update URDF joint origins for RViz visualization
                            for joint in root.findall('joint'):
                                origin = joint.find('origin')
                                if origin is None:
                                    origin = ET.SubElement(joint, 'origin')
                                    origin.set('rpy', '0 0 0')
                                    current_xyz = np.array([0.0, 0.0, 0.0])
                                else:
                                    xyz_str = origin.get('xyz', '0 0 0')
                                    current_xyz = np.array([float(x) for x in xyz_str.split()])
                                    
                                new_xyz = current_xyz + shift
                                origin.set('xyz', f"{new_xyz[0]:.6f} {new_xyz[1]:.6f} {new_xyz[2]:.6f}")
                            
                            obstacles_urdf_content = ET.tostring(root, encoding='utf-8').decode('utf-8')
                            
                            # 4. Update collision spheres in FoamCollider
                            self.collider.spherical_obstacles = []
                            for center, radius in orig_spheres:
                                new_center = (center[0] + shift[0], center[1] + shift[1], center[2] + shift[2])
                                self.collider.add_obstacle(new_center, radius)
                    except Exception as e:
                        self.get_logger().error(f"Failed to move obstacle URDF dynamically: {e}")

                try:
                    desc_msg = String()
                    desc_msg.data = obstacles_urdf_content
                    self.obstacles_desc_pub.publish(desc_msg)
                except Exception as e:
                    self.get_logger().error(f"Failed to publish updated obstacle URDF: {e}")
                
                # Compute voxels in ephemeral mode (RAM only)
                cspace_data = self._compute_cspace_voxels()
                msg = String()
                msg.data = json.dumps(cspace_data)
                self.cached_voxels_msg = msg
                self.cache_dirty = True
                self.publish_voxels()

        except Exception as e:
            self.get_logger().error(f"Failed to process web command in voxelizer: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CSpaceVoxelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
