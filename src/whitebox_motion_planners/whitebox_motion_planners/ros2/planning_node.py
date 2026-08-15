import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from std_srvs.srv import Trigger
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import math
import time
import os
import json
import xml.etree.ElementTree as ET
import numpy as np
from ament_index_python.packages import get_package_share_directory

from ..kinematics import get_kinematics, TrajectoryGenerator, CommunityArmIKSolver
from ..collision.foam_collider import FoamCollider
from ..collision.grid_discretizer import GridDiscretizer
from ..planners.planner_factory import PlannerFactory


class TopologicalPlannerNode(Node):
    """
    Main ROS 2 node for planning on the Toroidal Manifold T^n.
    Uses Dependency Injection to handle Kinematics, Collision, and Search.
    """
    def __init__(self):
        super().__init__('topological_planner_node')
        self.get_logger().info("Initializing White-Box Planner Node...")

        # 1. ROS 2 Parameters (Truth comes from config/planner_params.yaml)
        self.declare_parameter('robot_type', 'community_arm')
        self.declare_parameter('goal', [0.0, 0.0, 0.0])
        self.declare_parameter('start', [0.0, 0.0, 0.0])
        self.declare_parameter('use_static_start', False)
        self.declare_parameter('angles_in_degrees', False)
        self.declare_parameter('step_size_deg', 10.0)
        self.declare_parameter('planner_type', 'astar')
        self.declare_parameter('heuristic_type', 'L2')
        self.declare_parameter('use_horizontal_constraint', False)

        # RRT Parameters
        self.declare_parameter('rrt.max_samples', 10000)
        self.declare_parameter('rrt.step_size', 0.15)
        self.declare_parameter('rrt.goal_bias', 0.05)
        self.declare_parameter('rrt.goal_tolerance', 0.2)
        
        self.declare_parameter('obstacles_urdf_path', '')
        self.declare_parameter('sphere_thinning_dist', 0.015)
        self.declare_parameter('cache_dir', '')
        self.declare_parameter('singularity_threshold', 0.0)
        self.declare_parameter('animation_rate_hz', 50.0)

        # Link lengths for kinematics
        self.declare_parameter('link_lengths.base_height', 0.130)
        self.declare_parameter('link_lengths.lower_shank', 0.140)
        self.declare_parameter('link_lengths.upper_shank', 0.140)
        self.declare_parameter('link_lengths.gripper_dx', 0.05467)
        self.declare_parameter('link_lengths.gripper_dz', 0.0)
        self.declare_parameter('link_lengths.gripper_k_elbow', 0.0)

        # Configurable joint mapping to global world frame (defaults to neutral 0.0 offsets, 1 direction)
        self.declare_parameter('joint_offsets.base_yaw', 0.0)
        self.declare_parameter('joint_offsets.shoulder_pitch', 0.0)
        self.declare_parameter('joint_offsets.elbow_pitch', 0.0)
        self.declare_parameter('joint_directions.base_yaw', 1)
        self.declare_parameter('joint_directions.shoulder_pitch', 1)
        self.declare_parameter('joint_directions.elbow_pitch', 1)

        # Parse joint offsets (convert degrees to radians)
        self.base_yaw_offset = math.radians(self.get_parameter('joint_offsets.base_yaw').value)
        self.shoulder_pitch_offset = math.radians(self.get_parameter('joint_offsets.shoulder_pitch').value)
        self.elbow_pitch_offset = math.radians(self.get_parameter('joint_offsets.elbow_pitch').value)
        
        # Parse direction multipliers
        self.base_yaw_dir = float(self.get_parameter('joint_directions.base_yaw').value)
        self.shoulder_pitch_dir = float(self.get_parameter('joint_directions.shoulder_pitch').value)
        self.elbow_pitch_dir = float(self.get_parameter('joint_directions.elbow_pitch').value)
        
        # Internal State
        self.is_animating = False
        self._pending_origin_update = False  # True when set_origin triggered the trajectory
        self.animation_path = []
        self.animation_index = 0
        self.animation_timer = None
        self.last_gui_msg = None        # Full GUI JointState as template
        self.final_planned_q = None     # Holds final position after animation
        self.trail_points = []          # Accumulated end-effector positions for RViz trail
        self.show_path_trail = True     # Toggle to show/hide end-effector trail in RViz
        self.active_waypoints = None    # Active sequential waypoints in memory
 
        # 2. Mathematical Components (White-Box)
        robot_type = self.get_parameter('robot_type').value
        use_horizontal = self.get_parameter('use_horizontal_constraint').value
        link_lengths = {
            'base_height': self.get_parameter('link_lengths.base_height').value,
            'lower_shank': self.get_parameter('link_lengths.lower_shank').value,
            'upper_shank': self.get_parameter('link_lengths.upper_shank').value,
            'gripper_dx': self.get_parameter('link_lengths.gripper_dx').value,
            'gripper_dz': self.get_parameter('link_lengths.gripper_dz').value,
            'gripper_k_elbow': self.get_parameter('link_lengths.gripper_k_elbow').value
        }

        # Get the Kinematics from the robot type and pass the link lengths and use horizontal constraint parameter
        self.kinematics = get_kinematics(
            robot_type,
            use_horizontal_constraint=use_horizontal,
            link_lengths=link_lengths)
        self.ik_solver = CommunityArmIKSolver(self.kinematics)
  
        # If using static start, initialize the robot's initial override to the start position
        use_static = self.get_parameter('use_static_start').value
        if use_static:
            start_list = self.get_parameter('start').get_parameter_value().double_array_value
            if self.get_parameter('angles_in_degrees').value:
                start_list = [math.radians(x) for x in start_list]
            self.final_planned_q = tuple(start_list[:self.kinematics.get_dof()])
        
        self.grid = GridDiscretizer(
            step_size_deg=self.get_parameter('step_size_deg').value,
            num_dof=self.kinematics.get_dof()
        )
        
        # Spherized URDF path for real robot collision checking
        urdf_path = None
        if robot_type == 'community_arm':
            try:
                pkg_share = get_package_share_directory('community_robot_arm')
                urdf_path = os.path.join(pkg_share, 'urdf', 'spherized', 'community_robot_arm_slim_spherized.urdf')
                if not os.path.exists(urdf_path):
                    urdf_path = None
            except Exception as e:
                self.get_logger().warn(f"Could not resolve URDF path: {e}")
                
        self.collider = FoamCollider(
            urdf_path=urdf_path,
            sphere_thinning_dist=self.get_parameter('sphere_thinning_dist').value
        )
        # Pass offsets and directions to collider
        self.collider.set_joint_transforms(
            offset_base_yaw=self.base_yaw_offset,
            offset_shoulder_pitch=self.shoulder_pitch_offset,
            offset_elbow_pitch=self.elbow_pitch_offset,
            dir_base_yaw=self.base_yaw_dir,
            dir_shoulder_pitch=self.shoulder_pitch_dir,
            dir_elbow_pitch=self.elbow_pitch_dir
        )
        self.collider.singularity_threshold = self.get_parameter('singularity_threshold').value
        # Declare and read parameter use_obstacles
        self.declare_parameter('use_obstacles', True)
        use_obstacles = self.get_parameter('use_obstacles').value
        obstacles_urdf_path = self.get_parameter('obstacles_urdf_path').value
        
        if use_obstacles:
            try:
                pkg_share = get_package_share_directory('community_robot_arm')
                if not obstacles_urdf_path:
                    obstacles_urdf_path = os.path.join(pkg_share, 'urdf', 'spherized', 'obstacles', 'box_obstacle_spherized.urdf')
                self.get_logger().info(f"Loading environment obstacles from: {obstacles_urdf_path}")
                if os.path.exists(obstacles_urdf_path):
                    obstacles = self._load_obstacles_from_urdf(obstacles_urdf_path)
                    for center, radius in obstacles:
                        self.collider.add_obstacle(center, radius)
                        self.get_logger().info(f"Added obstacle sphere: center={center}, radius={radius:.3f}")
                else:
                    self.get_logger().warn(f"Obstacles URDF file not found at: {obstacles_urdf_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load obstacles: {e}")

        # Try to load C-Space cache to enable O(1) set-based collision checks during path planning
        try:
            obstacles_hash = "no_obstacles"
            if use_obstacles and obstacles_urdf_path and os.path.exists(obstacles_urdf_path):
                if "no_obstacles" in os.path.basename(obstacles_urdf_path):
                    obstacles_hash = "no_obstacles"
                else:
                    import hashlib
                    hash_md5 = hashlib.md5()
                    with open(obstacles_urdf_path, "rb") as f:
                        for chunk in iter(lambda: f.read(4096), b""):
                            hash_md5.update(chunk)
                    obstacles_hash = hash_md5.hexdigest()[:8]
            
            step_size = self.get_parameter('step_size_deg').value
            thinning_dist = self.get_parameter('sphere_thinning_dist').value
            src_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            cache_dir = self.get_parameter('cache_dir').value
            if not cache_dir:
                if os.path.exists('/home/ros_ws/cspace_cache'):
                    cache_dir = '/home/ros_ws/cspace_cache'
                else:
                    if os.path.exists('/home/ros_ws/src/whitebox_motion_planners'):
                        cache_dir = '/home/ros_ws/src/whitebox_motion_planners/cspace_cache'
                    else:
                        cache_dir = os.path.join(src_dir, 'cspace_cache')
            sing_thresh = self.collider.singularity_threshold
            if sing_thresh > 0.0:
                cache_filename = f"cspace_cache_{step_size}deg_{thinning_dist}m_{obstacles_hash}_singularity{sing_thresh}.json"
            else:
                cache_filename = f"cspace_cache_{step_size}deg_{thinning_dist}m_{obstacles_hash}.json"
            cache_filepath = os.path.join(cache_dir, cache_filename)
            
            if os.path.exists(cache_filepath):
                self.get_logger().info(f"Loading C-Space cache for planner from: {cache_filepath}")
                with open(cache_filepath, 'r') as f:
                    cache_data = json.load(f)
                
                if isinstance(cache_data, dict):
                    forbidden_list = cache_data.get('forbidden_voxels', [])
                else:
                    forbidden_list = cache_data
                
                # Convert radians coordinates list to a set of discrete tuples for O(1) lookup
                forbidden_set = set()
                for voxel in forbidden_list:
                    # voxel is [q0_rad, q1_rad, q2_rad]
                    # Map continuous coordinates back to discrete indices in the grid
                    q_discrete = self.grid.discretize(tuple(voxel))
                    forbidden_set.add(q_discrete)
                
                # Inject cache into FoamCollider
                self.collider.set_cspace_cache(forbidden_set, self.grid)
                self.get_logger().info(f"Loaded {len(forbidden_set)} forbidden voxels into collider cache!")
            else:
                self.get_logger().warn("No C-Space cache file found. Planning will fallback to real-time collision checking.")
        except Exception as e:
            self.get_logger().error(f"Failed to load C-Space cache for planning node: {e}")

        # --- 2. Planner via Factory ---
        # We read the algorithm and metric from parameters
        planner_type = self.get_parameter('planner_type').value
        heuristic_type = self.get_parameter('heuristic_type').value

        self.planner = PlannerFactory.create_planner(
            planner_type=planner_type,
            space=self.grid,
            collider=self.collider,
            kinematics=self.kinematics,
            heuristic_type=heuristic_type,
            max_samples=self.get_parameter('rrt.max_samples').value,
            step_size=self.get_parameter('rrt.step_size').value,
            goal_bias=self.get_parameter('rrt.goal_bias').value,
            goal_tolerance=self.get_parameter('rrt.goal_tolerance').value
        )

        # --- 3. ROS 2 Interface (SENSE - THINK - ACT) ---
        
        # [SENSE]: Monitor the robot's current state via the GUI feedback.
        # This allows the planner to perceive the starting configuration (Point A).
        self.current_q = None      # Robot's actual World position (used as planning start)
        self.last_gui_q = None     # Last GUI command received (used for manual change detection)
        self.master_joint_names = None
        self.planner_lock_until = 0.0  # Timestamp until which joint_callback ignores GUI overrides
        self.joint_sub = (
            self.create_subscription(
                JointState, 
                '/gui_master_states', 
                self.joint_callback, 
                10
            )
        )

        # [ACT]: Execute the planned trajectory by publishing to the robot's controllers.
        # This sends the "Think" results back to the simulation/hardware.
        self.joint_pub = (
            self.create_publisher(
                JointState, 
                '/master_states', 
                10
            )
        )

        # [STATUS]: Publish status messages (success/failure) for the dashboard
        self.status_pub = (
            self.create_publisher(
                String,
                '/planner_status',
                10
            )
        )

        # [VISUALIZATION]: Publisher for thinned collision spheres in RViz2
        self.marker_pub = (
            self.create_publisher(
                MarkerArray,
                '/robot_collision_markers',
                10
            )
        )

        # Publisher for the yellow trajectory trail
        self.trail_pub = (
            self.create_publisher(
                MarkerArray,
                '/trajectory_trail',
                10
            )
        )

        from rclpy.qos import QoSProfile, DurabilityPolicy
        desc_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.obstacles_desc_pub = self.create_publisher(
            String,
            '/obstacles_description',
            desc_qos
        )

        # [CONFIG]: Latched topic that publishes the start configuration once at startup.
        # Any dashboard subscriber connecting at any time will receive this message automatically,
        # eliminating the need for service call retries.
        config_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.start_config_pub = self.create_publisher(
            String,
            '/planner_start_config',
            config_qos
        )

        # [INTERACTION]: Human-in-the-loop trigger via RViz2 "Publish Point" tool.
        # This converts a 3D Cartesian click into a goal-oriented planning event.
        self.click_sub = (
            self.create_subscription(
                PointStamped, 
                '/clicked_point', 
                self.click_callback, 
                10
            )
        )

        # [INTERACTION]: Service call trigger (Terminal command).
        # Provides programmatic control to invoke the planner without user interaction.
        self.srv = self.create_service(
            Trigger, '/execute_plan', self.service_callback
        )

        # [INTERACTION]: Web command interface via JSON messages over WebSockets.
        self.web_cmd_sub = self.create_subscription(
            String,
            '/web_commands',
            self.web_command_callback,
            10
        )

        # Synchronize C-Space forbidden voxels computed on-the-fly by voxelizer
        self.cspace_voxels_sub = self.create_subscription(
            String,
            '/cspace_voxels',
            self.cspace_voxels_callback,
            10
        )

        # Dynamic & Static TF broadcasters for live obstacle visualization
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.active_obstacle_transforms = []
        self.tf_timer = self.create_timer(0.1, self._timer_publish_obstacle_tfs)

        self.get_logger().info("Node ready. Waiting for trigger...")
        self.get_logger().info("  Option 1: Click in RViz2 using 'Publish Point' tool")
        self.get_logger().info("  Option 2: ros2 service call /execute_plan std_srvs/srv/Trigger")
        self.get_logger().info("  Option 3: Publish JSON commands to /web_commands (dashboard interface)")

        # Publish start config to latched topic so the dashboard can read it at any time
        start_list = self.get_parameter('start').get_parameter_value().double_array_value
        angles_in_degrees = self.get_parameter('angles_in_degrees').value
        if angles_in_degrees:
            start_deg = list(start_list)
        else:
            start_deg = [math.degrees(x) for x in start_list]
        config_msg = String()
        config_msg.data = json.dumps({'start': start_deg})
        self.start_config_pub.publish(config_msg)
        self.get_logger().info(f"Published start config to /planner_start_config: {start_deg}")


    def world_to_urdf(self, q_world: tuple) -> tuple:
        """
        Converts world-frame coordinates (yaw_w, pitch1_w, pitch2_w) in radians
        to URDF joint values (base_yaw_joint, shoulder_pitch_joint, elbow_pitch_joint) in radians
        using configurable offset parameters.
        """
        if len(q_world) == 2:
            yaw_w, pitch1_w = q_world
            pitch2_w = 0.0
        else:
            yaw_w, pitch1_w, pitch2_w = q_world

        base_yaw = self.base_yaw_offset + self.base_yaw_dir * yaw_w
        shoulder_pitch = self.shoulder_pitch_offset + self.shoulder_pitch_dir * pitch1_w
        # Coupled: elbow URDF = -shoulder_urdf - q3_relative
        # This makes q3_world a RELATIVE angle (upper shank relative to lower shank)
        q3_relative = self.elbow_pitch_offset + self.elbow_pitch_dir * pitch2_w
        elbow_pitch = -shoulder_pitch - q3_relative

        return (base_yaw, shoulder_pitch, elbow_pitch)

    def wrap_to_pi(self, val: float) -> float:
        a = (val + math.pi) % (2 * math.pi)
        if a <= 1e-9:
            a += 2 * math.pi
        return a - math.pi

    def urdf_to_world(self, q_urdf: tuple) -> tuple:
        """
        Converts URDF joint values (base_yaw_joint, shoulder_pitch_joint, elbow_pitch_joint) in radians
        to world-frame coordinates (yaw_w, pitch1_w, pitch2_w) in radians
        using configurable offset parameters.
        """
        if len(q_urdf) == 2:
            base_yaw, shoulder_pitch = q_urdf
            elbow_pitch = 0.0
        else:
            base_yaw, shoulder_pitch, elbow_pitch = q_urdf

        yaw_w = self.wrap_to_pi((base_yaw - self.base_yaw_offset) / self.base_yaw_dir)
        pitch1_w = self.wrap_to_pi((shoulder_pitch - self.shoulder_pitch_offset) / self.shoulder_pitch_dir)
        # Inverse of coupled conversion: q3_relative = -elbow_pitch - shoulder_pitch
        q3_relative = -elbow_pitch - shoulder_pitch
        pitch2_w = self.wrap_to_pi((q3_relative - self.elbow_pitch_offset) / self.elbow_pitch_dir)

        return (yaw_w, pitch1_w, pitch2_w)

    def publish_status(self, success: bool, msg: str, path: list = None) -> tuple:
        """
        Publishes planning status (success and details message) as a JSON string to /planner_status
        and returns the (success, msg) tuple for convenience.
        """
        status_msg = String()
        data = {"success": success, "message": msg}
        if success and path is not None:
            # The path returned by the planner is already in radians (world coordinates)
            path_rads = [list(wp) for wp in path]
            data["path"] = path_rads
            try:
                data["manipulability"] = [self.collider.compute_manipulability(tuple(wp)) for wp in path_rads]
            except Exception as e:
                self.get_logger().warn(f"Failed to compute path manipulability: {e}")
        status_msg.data = json.dumps(data)
        self.status_pub.publish(status_msg)
        return (success, msg)

    def joint_callback(self, msg: JointState):
        """
        Captures the current robot configuration from the GUI sliders.
        If the user moves a slider manually (or clicks Center), it releases
        the planned position override.

        Args:
            msg (JointState): The current robot configuration from the GUI sliders.
            
        Returns:
            None
        """
        # Extract master joints from msg (msg contains URDF coordinates for master joints)
        joint_map = {}
        for name, pos in zip(msg.name, msg.position):
            joint_map[name] = pos
            
        q_urdf = (
            joint_map.get('base_yaw_joint', 0.0),
            joint_map.get('shoulder_pitch_joint', 0.0),
            joint_map.get('elbow_pitch_joint', 0.0)
        )
        
        new_q_world = self.urdf_to_world(q_urdf)

        # Detect manual change by comparing INCOMING GUI command against the LAST GUI command.
        # This avoids false positives when the GUI passively re-sends its old position
        # (e.g., after animation ends and robot is at goal but GUI is still at start).
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_gui_q is not None and now > self.planner_lock_until:
            manual_change = any(
                abs(a - b) > 0.01
                for a, b in zip(new_q_world, self.last_gui_q)
            )
            if manual_change:
                self.final_planned_q = None
                self.current_q = new_q_world  # Follow new GUI command
                xyz = self.kinematics.compute_forward_kinematics_gripper(new_q_world)
                q_deg = [round(math.degrees(x), 1) for x in new_q_world]
                xyz_cm = [round(v * 100.0, 1) for v in xyz]
                self.get_logger().info(f"Joint control moved → q={q_deg}°  |  Calculated FK XYZ={xyz_cm} cm")

        # Track last GUI command (separate from robot actual position)
        self.last_gui_q = new_q_world
        self.last_gui_msg = msg
        self.master_joint_names = list(msg.name)

        # Initialize current_q on first message
        if self.current_q is None:
            self.current_q = new_q_world

        # Publish to /master_states (always in URDF coordinates)
        if not self.is_animating:
            q_to_publish = self.final_planned_q if self.final_planned_q is not None else self.current_q
            out = self._build_full_msg(q_to_publish)
            self.joint_pub.publish(out)
            self.publish_collision_markers(q_to_publish)

    def click_callback(self, msg: PointStamped):
        """
        Triggered when the user clicks in RViz2 with the 'Publish Point' tool.
        The click coordinates are ignored; the click itself is the trigger.

        Args:
            msg (PointStamped): The clicked point in RViz2.
            
        Returns:
            None
        """
        
        self.get_logger().info(f"RViz click detected at ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f}). Starting plan...")
        self.execute_plan()

    def service_callback(self, request, response):
        """
        Triggered when the user calls the /execute_plan service from the terminal.
        
        Args:
            request: Service request.
            response: Service response.
            
        Returns:
            tuple: (success: bool, message: str)
        """
        success, message = self.execute_plan()
        response.success = success
        response.message = message
        return response

    def web_command_callback(self, msg: String):
        """
        Receives JSON commands from the web dashboard.
        """
        try:
            data = json.loads(msg.data)
            action = data.get("action")
            
            # 1. Update parameters dynamically if present
            if "planner_type" in data:
                self.set_parameters([rclpy.Parameter("planner_type", rclpy.Parameter.Type.STRING, data["planner_type"])])
                self.get_logger().info(f"Updated planner_type via web to: {data['planner_type']}")
            if "heuristic_type" in data:
                self.set_parameters([rclpy.Parameter("heuristic_type", rclpy.Parameter.Type.STRING, data["heuristic_type"])])
                self.get_logger().info(f"Updated heuristic_type via web to: {data['heuristic_type']}")
            
            if action == "plan":
                self.active_waypoints = None
                
                if "goal" in data:
                    goal_val = [float(x) for x in data["goal"]]
                    if self.get_parameter('angles_in_degrees').value:
                        goal_val = [math.degrees(x) for x in goal_val]
                    self.set_parameters([rclpy.Parameter("goal", rclpy.Parameter.Type.DOUBLE_ARRAY, goal_val)])
                
                self.execute_plan()

            elif action == "set_origin":
                if "xyz" in data:
                    xyz = data["xyz"]
                    q_goal = self.ik_solver.compute_ik(xyz[0], xyz[1], xyz[2], current_q=self.current_q)
                    if q_goal is None:
                        msg = f"Cannot set origin: XYZ {xyz} m is out of reach."
                        self.get_logger().error(msg)
                        self.publish_status(False, msg)
                        return
                elif "q" in data:
                    q_vals = data["q"]
                    angles_in_degrees = self.get_parameter('angles_in_degrees').value
                    if angles_in_degrees:
                        q_goal = tuple(math.radians(x) for x in q_vals)
                    else:
                        q_goal = tuple(float(x) for x in q_vals)
                    xyz = None
                else:
                    return

                angles_in_degrees = self.get_parameter('angles_in_degrees').value
                start_param = [math.degrees(x) for x in q_goal] if angles_in_degrees else list(q_goal)

                self.set_parameters([rclpy.Parameter("start", rclpy.Parameter.Type.DOUBLE_ARRAY, start_param)])

                q_start = self.current_q if self.current_q is not None else q_goal

                self.final_planned_q = None
                self.current_q = q_goal
                self.last_gui_q = q_goal
                # Lock GUI slider overrides for 2 seconds after setting Cartesian origin
                self.planner_lock_until = self.get_clock().now().nanoseconds * 1e-9 + 2.0

                self.get_logger().info(
                    f"Updated start origin → "
                    f"XYZ={[round(v, 3) for v in xyz] if xyz is not None else 'N/A'} m  |  "
                    f"q={[round(v, 1) for v in start_param]}°"
                )

                if any(abs(a - b) > 1e-3 for a, b in zip(q_start, q_goal)):
                    self.start_animation([q_start, q_goal])
                else:
                    out = self._build_full_msg(q_goal)
                    self.joint_pub.publish(out)
                    self.publish_collision_markers(q_goal)

            elif action == "plan_cartesian":
                self.active_waypoints = None
                start_xyz = data.get("start_xyz", None)
                goal_xyz = data.get("goal_xyz", [0.15, 0.05, 0.20])

                if start_xyz is not None:
                    q_start = self.ik_solver.compute_ik(start_xyz[0], start_xyz[1], start_xyz[2], current_q=self.current_q)
                else:
                    q_start = self.current_q

                q_goal = self.ik_solver.compute_ik(goal_xyz[0], goal_xyz[1], goal_xyz[2], current_q=q_start)

                if q_start is None:
                    msg = f"Punto Inicial {start_xyz} fuera del alcance del brazo."
                    self.get_logger().error(msg)
                    self.publish_status(False, msg)
                    return
                if q_goal is None:
                    msg = f"Punto Objetivo {goal_xyz} fuera del alcance del brazo."
                    self.get_logger().error(msg)
                    self.publish_status(False, msg)
                    return

                angles_in_degrees = self.get_parameter('angles_in_degrees').value
                start_param = [math.degrees(x) for x in q_start] if angles_in_degrees else list(q_start)
                goal_param = [math.degrees(x) for x in q_goal] if angles_in_degrees else list(q_goal)

                self.set_parameters([
                    rclpy.Parameter("start", rclpy.Parameter.Type.DOUBLE_ARRAY, start_param),
                    rclpy.Parameter("goal", rclpy.Parameter.Type.DOUBLE_ARRAY, goal_param)
                ])

                self.final_planned_q = None
                self.current_q = q_start
                self.last_gui_q = q_start
                if not self.is_animating:
                    out = self._build_full_msg(q_start)
                    self.joint_pub.publish(out)
                    self.publish_collision_markers(q_start)

                start_deg = [round(math.degrees(x), 1) for x in q_start]
                goal_deg = [round(math.degrees(x), 1) for x in q_goal]
                self.get_logger().info(f"Cartesian IK resolved: Start q={start_deg}°, Goal q={goal_deg}°")

                self.execute_plan()
                
            elif action == "plan_sequential":
                raw_waypoints = data.get("waypoints", [])
                if len(raw_waypoints) >= 2:
                    resolved_waypoints = []
                    last_q = self.current_q
                    for pt in raw_waypoints:
                        if len(pt) == 3 and all(isinstance(v, (int, float)) for v in pt):
                            # Check if waypoint is Cartesian XYZ (values in meters e.g. -0.2 to 0.5) or Joint angles
                            # Cartesian coordinates typically have magnitude < 1.0 meter
                            if max(abs(v) for v in pt) <= 1.0:
                                q_sol = self.ik_solver.compute_ik(pt[0], pt[1], pt[2], current_q=last_q)
                                if q_sol is None:
                                    msg = f"Waypoint {pt} m fuera de alcance por IK."
                                    self.get_logger().error(msg)
                                    self.publish_status(False, msg)
                                    return
                                resolved_waypoints.append(q_sol)
                                last_q = q_sol
                            else:
                                resolved_waypoints.append(pt)
                                last_q = tuple(math.radians(v) for v in pt) if self.get_parameter('angles_in_degrees').value else tuple(pt)
                        else:
                            resolved_waypoints.append(pt)

                    if self.get_parameter('angles_in_degrees').value:
                        waypoints = [[math.degrees(coord) if abs(coord) <= 2*math.pi else coord for coord in pt] for pt in resolved_waypoints]
                    else:
                        waypoints = resolved_waypoints

                    self.active_waypoints = waypoints
                    self.execute_plan()

                    
            elif action == "change_cspace":
                obstacle_type = data.get("obstacle_type")
                step_size = float(data.get("step_size_deg"))
                
                self.get_logger().info(f"Changing planner C-space dynamically to: {obstacle_type} at {step_size}deg")
                
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
                                hash_md5.update(f.read())
                            obstacles_hash = hash_md5.hexdigest()[:8]
                    except Exception as e:
                        self.get_logger().error(f"Failed to load obstacles dynamically: {e}")

                try:
                    desc_msg = String()
                    desc_msg.data = obstacles_urdf_content
                    self.obstacles_desc_pub.publish(desc_msg)
                    self.publish_obstacle_tfs(obstacles_urdf_content)
                    self.get_logger().info(f"Published dynamically reloaded obstacles description and TFs to RViz2 ({obstacle_type})")
                except Exception as e:
                    self.get_logger().error(f"Failed to publish obstacles description: {e}")
                
                # 3. Reload cache
                cache_dir = self.get_parameter('cache_dir').value
                if not cache_dir:
                    if os.path.exists('/home/ros_ws/cspace_cache'):
                        cache_dir = '/home/ros_ws/cspace_cache'
                    else:
                        try:
                            pkg_share_wb = get_package_share_directory('whitebox_motion_planners')
                            cache_dir = os.path.join(pkg_share_wb, 'cspace_cache')
                        except Exception:
                            src_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
                            cache_dir = os.path.join(src_dir, 'cspace_cache')
                
                thinning_dist = self.get_parameter('sphere_thinning_dist').value
                sing_thresh = self.collider.singularity_threshold
                if sing_thresh > 0.0:
                    cache_filename = f"cspace_cache_{step_size}deg_{thinning_dist}m_{obstacles_hash}_singularity{sing_thresh}.json"
                else:
                    cache_filename = f"cspace_cache_{step_size}deg_{thinning_dist}m_{obstacles_hash}.json"
                cache_filepath = os.path.join(cache_dir, cache_filename)
                
                if os.path.exists(cache_filepath):
                    self.get_logger().info(f"Loading C-space cache from: {cache_filepath}")
                    try:
                        with open(cache_filepath, 'r') as f:
                            cache_data = json.load(f)
                        
                        if isinstance(cache_data, dict):
                            forbidden_list = cache_data.get('forbidden_voxels', [])
                        else:
                            forbidden_list = cache_data
                            
                        forbidden_set = set()
                        for voxel in forbidden_list:
                            q_discrete = self.grid.discretize(tuple(voxel))
                            forbidden_set.add(q_discrete)
                            
                        self.collider.set_cspace_cache(forbidden_set, self.grid)
                        self.get_logger().info(f"Loaded {len(forbidden_set)} forbidden voxels into planner!")
                    except Exception as e:
                        self.get_logger().error(f"Failed to load cache: {e}")
                        self.collider.set_cspace_cache(None, self.grid)
                else:
                    self.get_logger().warn(f"No C-space cache found at {cache_filepath}. Planning will run in real-time mode.")
                    self.collider.set_cspace_cache(None, self.grid)
                
                if self.current_q is not None:
                    self.publish_collision_markers(self.current_q)
            
            elif action in ["move_obstacle", "preview_obstacle"]:
                obstacle_type = data.get("obstacle_type", "box_obstacle")
                pos_xyz = data.get("position_xyz", [0.3, 0.0, 0.15])
                
                obstacles_urdf_content = '<?xml version="1.0"?><robot name="obstacles"><link name="root"/></robot>'
                if obstacle_type != "no_obstacles":
                    try:
                        pkg_share = get_package_share_directory('community_robot_arm')
                        obstacles_urdf = os.path.join(pkg_share, 'urdf', 'spherized', 'obstacles', f"{obstacle_type}_spherized.urdf")
                        if os.path.exists(obstacles_urdf):
                            tree = ET.parse(obstacles_urdf)
                            root = tree.getroot()
                            
                            orig_spheres = self._load_obstacles_from_xml_root(root)
                            if orig_spheres:
                                centers = np.array([c for c, r in orig_spheres])
                                orig_centroid = np.mean(centers, axis=0)
                            else:
                                orig_centroid = np.array([0.0, 0.0, 0.0])
                                
                            target_pos = np.array(pos_xyz, dtype=float)
                            shift = target_pos - orig_centroid
                            
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
                            
                            if action == "move_obstacle":
                                self.collider.spherical_obstacles = []
                                for center, radius in orig_spheres:
                                    new_center = (center[0] + shift[0], center[1] + shift[1], center[2] + shift[2])
                                    self.collider.add_obstacle(new_center, radius)
                    except Exception as e:
                        self.get_logger().error(f"Failed to move obstacle in planner: {e}")

                try:
                    desc_msg = String()
                    desc_msg.data = obstacles_urdf_content
                    self.obstacles_desc_pub.publish(desc_msg)
                    self.publish_obstacle_tfs(obstacles_urdf_content)
                except Exception as e:
                    self.get_logger().error(f"Failed to publish updated obstacle URDF in planner: {e}")
                
                if action == "move_obstacle":
                    self.collider.set_cspace_cache(None, self.grid)
                    if self.current_q is not None:
                        self.publish_collision_markers(self.current_q)

            elif action == "clear_trail":
                self.trail_points = []
                clear_trail_marker = Marker()
                clear_trail_marker.header.frame_id = 'world'
                clear_trail_marker.header.stamp = self.get_clock().now().to_msg()
                clear_trail_marker.ns = 'trajectory_trail'
                clear_trail_marker.id = 0
                clear_trail_marker.action = Marker.DELETE
                
                marker_array = MarkerArray()
                marker_array.markers.append(clear_trail_marker)
                self.trail_pub.publish(marker_array)
                self.get_logger().info("Cleared trajectory trail in RViz!")
                
            elif action == "go_to_position":
                # Explicit move command from the dashboard (e.g. Reset to Home).
                # Works even when moving to the same position the GUI was already at.
                angles_in_degrees = self.get_parameter('angles_in_degrees').value
                q_vals = data.get("q", [])
                if len(q_vals) >= self.kinematics.get_dof():
                    if angles_in_degrees:
                        q_world = tuple(math.radians(x) for x in q_vals[:self.kinematics.get_dof()])
                    else:
                        q_world = tuple(float(x) for x in q_vals[:self.kinematics.get_dof()])
                    self.final_planned_q = None
                    self.current_q = q_world
                    self.last_gui_q = q_world  # Sync GUI tracking to avoid re-trigger
                    self.planner_lock_until = self.get_clock().now().nanoseconds * 1e-9 + 2.0
                    if not self.is_animating:
                        out = self._build_full_msg(q_world)
                        self.joint_pub.publish(out)
                        self.publish_collision_markers(q_world)
                    self.get_logger().info(f"Explicit go_to_position: {tuple(round(math.degrees(x),1) for x in q_world)}°")

            elif action == "toggle_trail":
                show_val = bool(data.get("show", True))
                self.show_path_trail = show_val
                self.get_logger().info(f"Updated show_path_trail via web to: {show_val}")
                if not show_val:
                    self.trail_points = []
                    clear_trail_marker = Marker()
                    clear_trail_marker.header.frame_id = 'world'
                    clear_trail_marker.header.stamp = self.get_clock().now().to_msg()
                    clear_trail_marker.ns = 'trajectory_trail'
                    clear_trail_marker.id = 0
                    clear_trail_marker.action = Marker.DELETE
                    
                    marker_array = MarkerArray()
                    marker_array.markers.append(clear_trail_marker)
                    self.trail_pub.publish(marker_array)
                    
        except Exception as e:
            self.get_logger().error(f"Failed to process web command in planner: {e}")

    def cspace_voxels_callback(self, msg: String):
        """
        Callback triggered whenever /cspace_voxels publishes forbidden voxels
        (e.g., when dynamic obstacle is moved and recomputed on-the-fly by voxelizer).
        Synchronizes planner cache so A* and RRT plan in ~5ms instead of 60 seconds.
        """
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                forbidden_list = data.get('forbidden_voxels', [])
            else:
                forbidden_list = data

            if forbidden_list:
                old_count = len(self.collider.forbidden_set) if self.collider.forbidden_set else 0
                forbidden_set = set()
                for voxel in forbidden_list:
                    q_discrete = self.grid.discretize(tuple(voxel))
                    forbidden_set.add(q_discrete)
                self.collider.set_cspace_cache(forbidden_set, self.grid)
                if old_count != len(forbidden_set):
                    self.get_logger().info(f"Planner C-space cache updated: {len(forbidden_set)} voxels")
        except Exception as e:
            self.get_logger().error(f"Failed to update planner C-space cache from /cspace_voxels: {e}")

    def execute_plan(self) -> tuple:
        """
        Core planning logic. Reads Point A, plans to Point B, and starts animation.

        Returns:
            tuple: (success: bool, message: str)
        """
        if self.is_animating:
            msg = "Already animating a trajectory. Please wait."
            self.get_logger().warn(msg)
            return self.publish_status(False, msg)

        if self.current_q is None:
            msg = "No joint state received yet. Is the robot visualization running?"
            self.get_logger().error(msg)
            return self.publish_status(False, msg)

        # Clear previous final position so we plan from actual GUI state
        self.final_planned_q = None

        # Recreate planner dynamically based on current parameters
        planner_type = self.get_parameter('planner_type').value
        heuristic_type = self.get_parameter('heuristic_type').value
        self.planner = PlannerFactory.create_planner(
            planner_type=planner_type,
            space=self.grid,
            collider=self.collider,
            kinematics=self.kinematics,
            heuristic_type=heuristic_type,
            max_samples=self.get_parameter('rrt.max_samples').value,
            step_size=self.get_parameter('rrt.step_size').value,
            goal_bias=self.get_parameter('rrt.goal_bias').value,
            goal_tolerance=self.get_parameter('rrt.goal_tolerance').value
        )

        angles_in_degrees = self.get_parameter('angles_in_degrees').value

        waypoints_list = self.active_waypoints if (self.active_waypoints and len(self.active_waypoints) >= 2) else []

        if len(waypoints_list) >= 2:
            self.get_logger().info(f"Loaded {len(waypoints_list)} sequential waypoints. Planning sequential trajectory...")
            path_segments = []
            for i, pt in enumerate(waypoints_list):
                if angles_in_degrees:
                    pt_rad = tuple(math.radians(x) for x in pt[:self.kinematics.get_dof()])
                else:
                    pt_rad = tuple(pt[:self.kinematics.get_dof()])
                path_segments.append(pt_rad)

            full_path = []
            for i in range(len(path_segments) - 1):
                p_start = path_segments[i]
                p_goal = path_segments[i+1]

                if not self.collider.is_state_valid(p_start, self.kinematics):
                    xyz_cm = [round(v * 100.0, 1) for v in self.kinematics.compute_forward_kinematics_gripper(p_start)]
                    deg = [round(math.degrees(x), 1) for x in p_start]
                    msg = f"COLLISION ERROR: Waypoint #{i+1} XYZ={xyz_cm} cm ({deg}°) is in collision."
                    self.get_logger().error(msg)
                    return self.publish_status(False, msg)
                if not self.collider.is_state_valid(p_goal, self.kinematics):
                    xyz_cm = [round(v * 100.0, 1) for v in self.kinematics.compute_forward_kinematics_gripper(p_goal)]
                    deg = [round(math.degrees(x), 1) for x in p_goal]
                    msg = f"COLLISION ERROR: Waypoint #{i+2} XYZ={xyz_cm} cm ({deg}°) is in collision."
                    self.get_logger().error(msg)
                    return self.publish_status(False, msg)

                start_discrete = self.grid.discretize(p_start)
                goal_discrete = self.grid.discretize(p_goal)

                self.get_logger().info(f"Planning segment {i+1}/{len(path_segments)-1}: {waypoints_list[i]} -> {waypoints_list[i+1]}")
                segment = self.planner.plan(start_discrete, goal_discrete)
                if not segment:
                    xyz_start = [round(v * 100.0, 1) for v in self.kinematics.compute_forward_kinematics_gripper(p_start)]
                    xyz_goal = [round(v * 100.0, 1) for v in self.kinematics.compute_forward_kinematics_gripper(p_goal)]
                    msg = f"PLANNING ERROR: No collision-free path found for segment {i+1}: XYZ={xyz_start} cm -> XYZ={xyz_goal} cm."
                    self.get_logger().error(msg)
                    return self.publish_status(False, msg)

                if i > 0:
                    full_path.extend(segment[1:])
                else:
                    full_path.extend(segment)

            msg = f"Sequential path found! {len(full_path)} waypoints. Starting animation..."
            self.get_logger().info(msg)
            self.start_animation(full_path)
            return self.publish_status(True, msg, full_path)

        # Fallback to single start/goal from parameters
        use_static_start = self.get_parameter('use_static_start').value
        
        # Determine start configuration (Point A)
        if use_static_start:
            start_list = self.get_parameter('start').get_parameter_value().double_array_value
            if angles_in_degrees:
                start_list = [math.radians(x) for x in start_list]
            start_q = tuple(start_list[:self.kinematics.get_dof()])
            self.get_logger().info(f"Using start configuration from parameters: {start_q}")
        else:
            # Use current GUI position (already stored in World coordinates)
            dof = self.kinematics.get_dof()
            if self.current_q is not None:
                start_q = self.current_q[:dof]
            else:
                start_q = tuple([0.0]*dof)
                self.get_logger().warn("No GUI position received yet, defaulting start to (0,0,...,0)")

        # Read goal from ROS parameter
        goal_list = self.get_parameter('goal').get_parameter_value().double_array_value
        if angles_in_degrees:
            goal_list = [math.radians(x) for x in goal_list]
        goal_q = tuple(goal_list[:self.kinematics.get_dof()])


        self.get_logger().info(f"Point A (current): {tuple(round(x, 3) for x in start_q)}")
        self.get_logger().info(f"Point B (goal):    {tuple(round(x, 3) for x in goal_q)}")

        # Discretize both endpoints onto the toroidal grid
        start_discrete = self.grid.discretize(start_q)
        goal_discrete = self.grid.discretize(goal_q)

        # Check if start or goal is in collision
        if not self.collider.is_state_valid(start_q, self.kinematics):
            start_deg = tuple(round(math.degrees(x), 1) for x in start_q)
            start_xyz = [round(v * 100.0, 1) for v in self.kinematics.compute_forward_kinematics_gripper(start_q)]
            msg = f"COLLISION ERROR: Start configuration (Point A) XYZ={start_xyz} cm ({start_deg}°) is in collision."
            self.get_logger().error(msg)
            return self.publish_status(False, msg)
            
        if not self.collider.is_state_valid(goal_q, self.kinematics):
            goal_deg = tuple(round(math.degrees(x), 1) for x in goal_q)
            goal_xyz = [round(v * 100.0, 1) for v in self.kinematics.compute_forward_kinematics_gripper(goal_q)]
            msg = f"COLLISION ERROR: Goal configuration (Point B) XYZ={goal_xyz} cm ({goal_deg}°) is in collision."
            self.get_logger().error(msg)
            return self.publish_status(False, msg)

        self.get_logger().info(f"Computing {self.get_parameter('planner_type').value.upper()} on toroidal manifold T^n...")
        path = self.planner.plan(start_discrete, goal_discrete)

        if not path:
            msg = "PLANNING ERROR: No collision-free path found in C_free."
            self.get_logger().error(msg)
            return self.publish_status(False, msg)

        msg = f"Path found! {len(path)} waypoints. Starting animation..."
        self.get_logger().info(msg)

        # Log planned waypoints in degrees for debugging
        path_rad = path
        for i, wp_rad in enumerate(path_rad):
            wp_deg = tuple(round(math.degrees(x), 1) for x in wp_rad)
            if i < 5 or i >= len(path_rad) - 3:
                self.get_logger().info(f"  Waypoint [{i}]: {wp_deg}°")
            elif i == 5:
                self.get_logger().info(f"  ... ({len(path_rad) - 8} waypoints omitted) ...")

        self.start_animation(path)
        return self.publish_status(True, msg, path)


    def _build_full_msg(self, q_planned: tuple) -> JointState:
        """
        Takes the last full GUI message (all XX joints) and overrides
        only the 3 master joints with the planned values (converted to URDF coordinates).
        This ensures parallelogram_kinematics.py receives ALL joint data.

        Args:
            q_planned (tuple): The planned configuration (2-DOF or 3-DOF) in World coordinates.

        Returns:
            JointState: The full joint state message.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # 
        if self.last_gui_msg is not None:
            # Clone all names and positions from the GUI
            names = list(self.last_gui_msg.name)
            positions = list(self.last_gui_msg.position)

            # Reconstruct full 3D master configuration if constrained
            if getattr(self.kinematics, 'use_horizontal_constraint', False):
                q_full = (q_planned[0], q_planned[1], 0.0)
            else:
                q_full = q_planned

            # Convert q_full (World coordinates) to URDF coordinates
            q_urdf = self.world_to_urdf(q_full)

            # Override master joints
            master_map = {
                'base_yaw_joint': q_urdf[0],
                'shoulder_pitch_joint': q_urdf[1],
                'elbow_pitch_joint': q_urdf[2],
            }
            for i, name in enumerate(names):
                if name in master_map:
                    val = master_map[name]
                    normalized_val = (val + math.pi) % (2 * math.pi) - math.pi
                    positions[i] = float(normalized_val)

            msg.name = names
            msg.position = positions
        else:
            # Fallback if no GUI message received yet
            msg.name = ['base_yaw_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint']
            q_urdf = self.world_to_urdf(q_planned)
            msg.position = [float(val) for val in q_urdf[:3]]

        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)
        return msg

    def start_animation(self, path: list):
        """
        Begins stepping through the planned path using high-frequency
        interpolated quintic splines, publishing to /master_states.
        """
        self.is_animating = True
        self.animation_path = path
        
        # Instantiate the trajectory generator (max_vel = 1.0 rad/s, max_acc = 1.0 rad/s^2)
        self.trajectory = TrajectoryGenerator(path, max_vel=1.0, max_acc=1.0)
        self.animation_start_time = self.get_clock().now()
        self.trail_points = []  # Clear previous trail for new trajectory
        if path and len(path) > 0:
            try:
                q_start = tuple(path[0])
                if getattr(self.collider, 'urdf_parser', None) is not None:
                    q_urdf = self.world_to_urdf(q_start)
                    pos = self.collider.urdf_parser.get_end_effector_position(q_urdf)
                else:
                    pos = self.kinematics.compute_forward_kinematics(q_start)[-1]
                if pos is not None:
                    pt = Point()
                    pt.x, pt.y, pt.z = float(pos[0]), float(pos[1]), float(pos[2])
                    self.trail_points.append(pt)
            except Exception as e:
                self.get_logger().warn(f"Failed to pre-populate start trail point: {e}")

        self.publish_status(True, "Executing planned trajectory...", path)

        # Publish waypoints at dynamic rate (animation_rate_hz)
        rate_hz = self.get_parameter('animation_rate_hz').value
        period = 1.0 / max(1.0, rate_hz)  # avoid division by zero or negative rate
        self.animation_timer = self.create_timer(period, self.animation_step)

    def animation_step(self):
        """
        Timer callback: evaluates the trajectory spline at the current elapsed time.
        """
        elapsed = (self.get_clock().now() - self.animation_start_time).nanoseconds / 1e9
        
        if elapsed >= self.trajectory.total_duration:
            self.animation_timer.cancel()
            self.is_animating = False
            # Set the exact goal configuration at the end
            q_final = tuple(self.trajectory.waypoints[-1])
            out = self._build_full_msg(q_final)
            self.joint_pub.publish(out)
            self.publish_collision_markers(q_final)
            
            self.final_planned_q = q_final
            self.current_q = q_final

            # If this trajectory was triggered by set_origin, commit goal as the new start
            if self._pending_origin_update:
                self._pending_origin_update = False
                angles_in_degrees = self.get_parameter('angles_in_degrees').value
                new_start = [math.degrees(x) for x in q_final] if angles_in_degrees else list(q_final)
                self.set_parameters([rclpy.Parameter("start", rclpy.Parameter.Type.DOUBLE_ARRAY, new_start)])
                self.last_gui_q = q_final
                self.get_logger().info(f"Origin locked → q={[round(v,1) for v in new_start]}°")
            
            self.get_logger().info("Trajectory execution complete ✅")
            self.publish_status(True, "Trajectory execution complete ✅")
            return

        # Interpolate position
        q_interp, qd_interp, qdd_interp = self.trajectory.evaluate(elapsed)
        q_tuple = tuple(q_interp)
        
        out = self._build_full_msg(q_tuple)
        self.joint_pub.publish(out)
        self.publish_collision_markers(q_tuple)

    def publish_collision_markers(self, q: tuple):
        """
        Publishes the thinned collision spheres and the end-effector trajectory trail as a MarkerArray for RViz2.
        """
        marker_array = MarkerArray()

        # 1. Trajectory Trail (Yellow LINE_STRIP)
        if self.show_path_trail and self.is_animating:
            try:
                # Try to get the exact end-effector position from URDF transforms
                end_effector_pos = None
                if getattr(self.collider, 'urdf_parser', None) is not None:
                    if len(q) == 2:
                        q1, q2 = q
                        q3 = 0.0
                    else:
                        q1, q2, q3 = q
                    
                    # Convert World coordinates to URDF coordinates and get exact EE center
                    q_urdf = self.world_to_urdf((q1, q2, q3))
                    end_effector_pos = self.collider.urdf_parser.get_end_effector_position(q_urdf)
                
                # Fallback to simplified serial forward kinematics if URDF not loaded
                if end_effector_pos is None:
                    fk_positions = self.kinematics.compute_forward_kinematics(q)
                    end_effector_pos = fk_positions[-1]

                # Accumulate the point (only if it has moved to avoid infinite duplicates when static)
                trail_pt = Point()
                trail_pt.x = float(end_effector_pos[0])
                trail_pt.y = float(end_effector_pos[1])
                trail_pt.z = float(end_effector_pos[2])
                
                is_duplicate = False
                if self.trail_points:
                    last_pt = self.trail_points[-1]
                    dist = math.sqrt((trail_pt.x - last_pt.x)**2 + 
                                     (trail_pt.y - last_pt.y)**2 + 
                                     (trail_pt.z - last_pt.z)**2)
                    if dist < 0.005:  # Less than 5mm change
                        is_duplicate = True
                        
                if not is_duplicate:
                    self.trail_points.append(trail_pt)

                # Publish the trail as a LINE_STRIP (needs at least 2 points)
                if len(self.trail_points) >= 2:
                    trail_marker = Marker()
                    trail_marker.header.frame_id = 'world'
                    trail_marker.header.stamp = self.get_clock().now().to_msg()
                    trail_marker.ns = 'trajectory_trail'
                    trail_marker.id = 0
                    trail_marker.type = Marker.LINE_STRIP
                    trail_marker.action = Marker.ADD
                    trail_marker.pose.orientation.w = 1.0
                    trail_marker.scale.x = 0.005  # Line width (5mm)
                    # Yellow color
                    trail_marker.color.r = 1.0
                    trail_marker.color.g = 1.0
                    trail_marker.color.b = 0.0
                    trail_marker.color.a = 1.0
                    trail_marker.points = list(self.trail_points)
                    
                    trail_marker_array = MarkerArray()
                    trail_marker_array.markers.append(trail_marker)
                    self.trail_pub.publish(trail_marker_array)
            except Exception as e:
                self.get_logger().error(f"Failed to publish trajectory trail: {e}")

        # 2. Collision Spheres
        if getattr(self.collider, 'urdf_parser', None) is not None:
            try:
                # Convert World coordinates to URDF coordinates for collision checking/visualization
                q_urdf = self.world_to_urdf(q)
                centers, radii = self.collider.urdf_parser.get_transformed_spheres(q_urdf)
                
                # Clear only the collision sphere markers (not the trail)
                clear_marker = Marker()
                clear_marker.ns = 'thinned_collision_spheres'
                clear_marker.id = 9999
                clear_marker.action = Marker.DELETEALL
                marker_array.markers.append(clear_marker)
                
                for i in range(len(centers)):
                    marker = Marker()
                    marker.header.frame_id = 'world'
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = 'thinned_collision_spheres'
                    marker.id = i
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    
                    # Position
                    marker.pose.position.x = float(centers[i][0])
                    marker.pose.position.y = float(centers[i][1])
                    marker.pose.position.z = float(centers[i][2])
                    marker.pose.orientation.w = 1.0
                    
                    # Scale (diameter)
                    diameter = float(radii[i] * 2.0)
                    marker.scale.x = diameter
                    marker.scale.y = diameter
                    marker.scale.z = diameter
                    
                    # Color (Translucent Green to represent safety spheres)
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 0.4
                    
                    marker_array.markers.append(marker)
            except Exception as e:
                self.get_logger().error(f"Failed to publish collision markers: {e}")

        # 3. Dynamic Obstacle Spheres
        if hasattr(self, 'collider'):
            try:
                clear_obs_marker = Marker()
                clear_obs_marker.header.frame_id = 'world'
                clear_obs_marker.header.stamp = self.get_clock().now().to_msg()
                clear_obs_marker.ns = 'obstacle_spheres'
                clear_obs_marker.id = 9999
                clear_obs_marker.action = Marker.DELETEALL
                marker_array.markers.append(clear_obs_marker)

                if self.collider.spherical_obstacles:
                    for idx, sphere in enumerate(self.collider.spherical_obstacles):
                        center = sphere.center
                        radius = sphere.radius
                        obs_marker = Marker()
                        obs_marker.header.frame_id = 'world'
                        obs_marker.header.stamp = self.get_clock().now().to_msg()
                        obs_marker.ns = 'obstacle_spheres'
                        obs_marker.id = idx
                        obs_marker.type = Marker.SPHERE
                        obs_marker.action = Marker.ADD
                        obs_marker.pose.position.x = float(center[0])
                        obs_marker.pose.position.y = float(center[1])
                        obs_marker.pose.position.z = float(center[2])
                        obs_marker.pose.orientation.w = 1.0
                        obs_marker.scale.x = float(2 * radius)
                        obs_marker.scale.y = float(2 * radius)
                        obs_marker.scale.z = float(2 * radius)
                        obs_marker.color.r = 1.0
                        obs_marker.color.g = 0.2
                        obs_marker.color.b = 0.2
                        obs_marker.color.a = 0.5
                        marker_array.markers.append(obs_marker)
            except Exception as e:
                self.get_logger().error(f"Failed to publish dynamic obstacle markers: {e}")

        if marker_array.markers:
            try:
                self.marker_pub.publish(marker_array)
            except Exception as e:
                self.get_logger().error(f"Failed to publish marker array: {e}")

    def _load_obstacles_from_urdf(self, urdf_path: str) -> list:
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            return self._load_obstacles_from_xml_root(root)
        except Exception as e:
            self.get_logger().error(f"Failed to parse XML from URDF: {e}")
            return []

    def _load_obstacles_from_xml_root(self, root: ET.Element) -> list:
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

    def _timer_publish_obstacle_tfs(self):
        if hasattr(self, 'active_obstacle_transforms') and self.active_obstacle_transforms:
            now_stamp = self.get_clock().now().to_msg()
            for t in self.active_obstacle_transforms:
                t.header.stamp = now_stamp
            self.tf_broadcaster.sendTransform(self.active_obstacle_transforms)

    def publish_obstacle_tfs(self, urdf_content: str):
        try:
            if not hasattr(self, 'tf_broadcaster'):
                self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
            if not hasattr(self, 'tf_static_broadcaster'):
                self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
                
            root = ET.fromstring(urdf_content)
            transforms = []
            
            for joint in root.findall('joint'):
                if joint.get('type') == 'fixed':
                    parent = joint.find('parent')
                    child = joint.find('child')
                    if parent is not None and child is not None:
                        parent_link = parent.get('link')
                        child_link = child.get('link')
                        
                        origin = joint.find('origin')
                        xyz = [0.0, 0.0, 0.0]
                        rpy = [0.0, 0.0, 0.0]
                        if origin is not None:
                            xyz_str = origin.get('xyz')
                            rpy_str = origin.get('rpy')
                            if xyz_str:
                                xyz = [float(x) for x in xyz_str.split()]
                            if rpy_str:
                                rpy = [float(x) for x in rpy_str.split()]
                                
                        t = TransformStamped()
                        t.header.stamp = self.get_clock().now().to_msg()
                        t.header.frame_id = parent_link
                        t.child_frame_id = child_link
                        t.transform.translation.x = xyz[0]
                        t.transform.translation.y = xyz[1]
                        t.transform.translation.z = xyz[2]
                        
                        # Convert RPY to Quaternion
                        cr = math.cos(rpy[0] * 0.5)
                        sr = math.sin(rpy[0] * 0.5)
                        cp = math.cos(rpy[1] * 0.5)
                        sp = math.sin(rpy[1] * 0.5)
                        cy = math.cos(rpy[2] * 0.5)
                        sy = math.sin(rpy[2] * 0.5)
                        
                        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
                        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
                        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
                        t.transform.rotation.z = cr * cp * sy - sr * sp * cy
                        
                        transforms.append(t)
                        
            if transforms:
                self.active_obstacle_transforms = transforms
                self.tf_broadcaster.sendTransform(transforms)
                self.tf_static_broadcaster.sendTransform(transforms)
        except Exception as e:
            self.get_logger().error(f"Failed to publish dynamic obstacle TFs: {e}")


def main(args=None):
    # Initialize the ROS 2 Python client library and communications infrastructure.
    # This parses CLI arguments and prepares the node to interact with the ROS network.
    rclpy.init(args=args)

    # A node is the fundamental unit of computation in ROS 2.
    # It encapsulates a set of functionalities (like publishers, subscribers, services).
    node = TopologicalPlannerNode()

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
