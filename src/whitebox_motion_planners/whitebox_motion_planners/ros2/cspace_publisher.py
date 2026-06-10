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
        
        # 1. Initialize parameters
        (robot_type, step_size, use_horizontal, use_obstacles, 
         thinning_dist, robot_urdf, obstacles_urdf, cache_dir) = self._init_parameters()
        
        # 2. Get kinematics model
        self.kinematics = get_kinematics(robot_type, use_horizontal_constraint=use_horizontal)
        
        # 3. Setup collider and load obstacles
        obstacles_hash = self._setup_collider_and_obstacles(
            robot_type, use_obstacles, thinning_dist, robot_urdf, obstacles_urdf
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
        # Optional override path to the robot URDF file
        self.declare_parameter('robot_urdf_path', '')
        # Optional override path to the obstacles URDF file
        self.declare_parameter('obstacles_urdf_path', '')
        # Optional override directory path to save/load persistent cache
        self.declare_parameter('cache_dir', '')
        
        return (
            self.get_parameter('robot_type').value,
            self.get_parameter('step_size_deg').value,
            self.get_parameter('use_horizontal_constraint').value,
            self.get_parameter('use_obstacles').value,
            self.get_parameter('sphere_thinning_dist').value,
            self.get_parameter('robot_urdf_path').value,
            self.get_parameter('obstacles_urdf_path').value,
            self.get_parameter('cache_dir').value
        )

    def _setup_collider_and_obstacles(
        self, 
        robot_type, 
        use_obstacles, 
        thinning_dist, 
        robot_urdf, 
        obstacles_urdf
    ) -> str:
        """
        Configure the robot collider and optionally load environment obstacles.
        
        :param robot_type: Type of the robot.
        :param use_obstacles: Flag to load environment obstacles from URDF.
        :param thinning_dist: Safety distance to shrink the collision spheres.
        :param robot_urdf: User-defined path to robot URDF.
        :param obstacles_urdf: User-defined path to obstacles URDF.
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
        
        # Timer to publish voxels when there are active subscribers (e.g. web dashboard)
        self.timer = self.create_timer(2.0, self.publish_voxels)

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
            self.get_logger().info(f"CSpace Voxelizer: Loading cache from: {self.cache_filepath}")
            try:
                with open(self.cache_filepath, 'r') as f:
                    cached_json = f.read()
                
                # Validate JSON syntax and structure
                raw_data = json.loads(cached_json)
                
                if isinstance(raw_data, dict) and "self_collision_voxels" in raw_data and "obstacle_voxels" in raw_data:
                    msg = String()
                    msg.data = cached_json
                    self.cached_voxels_msg = msg
                else:
                    # Old cache format! Auto-regenerate using the new Rust solver
                    self.get_logger().info("Old cache format detected (missing segregated layers). Auto-regenerating in segregated dictionary format...")
                    cspace_data = self._compute_cspace_voxels()
                    self._save_cspace_cache(cspace_data)
                
                self.cache_dirty = True
                self.get_logger().info("C-Space loaded from cache successfully!")
                
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
            
            is_self_collision = False
            if self.collider.urdf_parser is not None:
                is_self_collision = self.collider.urdf_parser.check_self_collision(q_radians)
            
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
                    is_obs_collision = self.collider.urdf_parser.check_obstacle_collision(q_radians, obstacles_tuples)
                
                if is_obs_collision:
                    forbidden_voxels.append(voxel)
                    obstacle_voxels.append(voxel)
                
        self.get_logger().info(f"Finished processing C-Space. Found {len(forbidden_voxels)} forbidden states.")
        return {
            "forbidden_voxels": forbidden_voxels,
            "self_collision_voxels": self_collision_voxels,
            "obstacle_voxels": obstacle_voxels
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
        """
        sub_count = self.publisher_.get_subscription_count()
        msg_is_set = self.cached_voxels_msg is not None
        
        should_publish = False
        if msg_is_set:
            if sub_count > 0:
                if sub_count > self.last_sub_count:
                    should_publish = True
                if self.cache_dirty:
                    should_publish = True
                self.last_sub_count = sub_count
            else:
                self.last_sub_count = 0
                
        if should_publish:
            self.publisher_.publish(self.cached_voxels_msg)
            self.cache_dirty = False
            self.get_logger().info(f"Published C-space voxels to {sub_count} subscribers (size: {len(self.cached_voxels_msg.data)} chars)")
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
        except Exception as e:
            self.get_logger().error(f"Failed to parse XML from URDF: {e}")
            return []
        
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
                if obstacle_type != "no_obstacles":
                    try:
                        pkg_share = get_package_share_directory('community_robot_arm')
                        obstacles_urdf = os.path.join(pkg_share, 'urdf', 'spherized', 'obstacles', f"{obstacle_type}_spherized.urdf")
                        if os.path.exists(obstacles_urdf):
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
                
                # 3. Reload cache
                self._setup_cache(step_size, self.collider.urdf_parser.min_dist, obstacles_hash, self.cache_dir)
                
                # 4. Force republishing
                self.publish_voxels()
        except Exception as e:
            self.get_logger().error(f"Failed to process web command in voxelizer: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CSpaceVoxelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
