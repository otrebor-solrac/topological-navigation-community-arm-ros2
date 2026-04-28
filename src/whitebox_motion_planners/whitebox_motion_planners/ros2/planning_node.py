import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger
import math
import time

from ..kinematics.community_arm import CommunityArmKinematics
from ..collision.foam_collider import FoamCollider
from ..collision.grid_discretizer import GridDiscretizer
from ..planners.planner_factory import PlannerFactory

class TopologicalPlannerNode(Node):
    """
    Main ROS 2 node for planning on the Toroidal Manifold T^n.
    Uses Dependency Injection to handle Kinematics, Collision, and Search.
    """
    def __init__(self):
        super().__init__('whitebox_planner')
        self.get_logger().info("Initializing White-Box Planner Node...")

        # 1. ROS 2 Parameters (Truth comes from config/planner_params.yaml)
        self.declare_parameter('goal', [0.0, 0.0, 0.0])
        self.declare_parameter('step_size_deg', 10.0)
        self.declare_parameter('link_radius', 0.04)
        self.declare_parameter('planner_type', 'astar')
        self.declare_parameter('heuristic_type', 'L2')
        self.declare_parameter('use_horizontal_constraint', False)
        
        # Internal State
        self.is_animating = False
        self.animation_path = []
        self.animation_index = 0
        self.animation_timer = None
        self.last_gui_msg = None        # Full GUI JointState as template
        self.final_planned_q = None     # Holds final position after animation

        # 2. Mathematical Components (White-Box)
        use_horizontal = self.get_parameter('use_horizontal_constraint').value
        self.kinematics = CommunityArmKinematics(use_horizontal_constraint=use_horizontal)
        
        self.grid = GridDiscretizer(
            step_size_deg=self.get_parameter('step_size_deg').value,
            num_dof=self.kinematics.get_dof()
        )
        self.collider = FoamCollider(
            link_radius=self.get_parameter('link_radius').value
        )
        
        # To handle environment collisions, we can model the lab's obstacles 
        # (e.g., tables, walls, or equipment) as a set of spherical primitives.
        # Example: 
        #   self.collider.add_obstacle(center=(0.5, 0.0, 0.2), radius=0.1)
        # For this initial demo, we start with an empty (collision-free) space.


        # --- 2. Planner via Factory ---
        # We read the algorithm and metric from parameters
        planner_type = self.get_parameter('planner_type').value
        heuristic_type = self.get_parameter('heuristic_type').value

        self.planner = PlannerFactory.create_planner(
            planner_type=planner_type,
            space=self.grid,
            collider=self.collider,
            kinematics=self.kinematics,
            heuristic_type=heuristic_type
        )

        # --- 3. ROS 2 Interface (SENSE - THINK - ACT) ---
        
        # [SENSE]: Monitor the robot's current state via the GUI feedback.
        # This allows the planner to perceive the starting configuration (Point A).
        self.current_q = None
        self.master_joint_names = None
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

        self.get_logger().info("Node ready. Waiting for trigger...")
        self.get_logger().info("  Option 1: Click in RViz2 using 'Publish Point' tool")
        self.get_logger().info("  Option 2: ros2 service call /execute_plan std_srvs/srv/Trigger")


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
        if self.current_q is not None:
            # Check if any of the 3 master joints changed significantly in the GUI
            # (threshold of 0.01 rad to ignore noise)
            master_indices = []
            master_names = (
                ['revolute_1_0', 'revolute_9_0'] 
                if self.kinematics.use_horizontal_constraint 
                else ['revolute_1_0', 'revolute_9_0', 'revolute_10_0']
            )
            for name in master_names:
                if name in msg.name:
                    master_indices.append(list(msg.name).index(name))
            
            # Detect manual change
            manual_change = False
            for idx in master_indices:
                if abs(msg.position[idx] - self.current_q[idx]) > 0.01:
                    manual_change = True
                    break
            
            if manual_change:
                # Release override
                self.final_planned_q = None 

        # Update internal state
        self.last_gui_msg = msg
        self.master_joint_names = list(msg.name)
        self.current_q = tuple(msg.position)

        # Publish the current state or the final planned state if animating
        if not self.is_animating:
            if self.final_planned_q is not None:
                out = self._build_full_msg(self.final_planned_q)
                self.joint_pub.publish(out)
            else:
                self.joint_pub.publish(msg)

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

    def execute_plan(self) -> tuple:
        """
        Core planning logic. Reads Point A, plans to Point B, and starts animation.

        Returns:
            tuple: (success: bool, message: str)
        """
        if self.is_animating:
            msg = "Already animating a trajectory. Please wait."
            self.get_logger().warn(msg)
            return (False, msg)

        if self.current_q is None:
            msg = "No joint state received yet. Is the robot visualization running?"
            self.get_logger().error(msg)
            return (False, msg)

        # Clear previous final position so we plan from actual GUI state
        self.final_planned_q = None

        # Read only the master joints we care about
        if self.master_joint_names and self.current_q:
            joint_map = dict(zip(self.master_joint_names, self.current_q))
            if self.kinematics.use_horizontal_constraint:
                start_q = (
                    joint_map.get('revolute_1_0', 0.0),
                    joint_map.get('revolute_9_0', 0.0)
                )
            else:
                start_q = (
                    joint_map.get('revolute_1_0', 0.0),
                    joint_map.get('revolute_9_0', 0.0),
                    joint_map.get('revolute_10_0', 0.0)
                )
        else:
            dof = self.kinematics.get_dof()
            start_q = self.current_q[:dof] if self.current_q else tuple([0.0]*dof)

        # Read goal from ROS parameter
        goal_list = self.get_parameter('goal').get_parameter_value().double_array_value
        goal_q = tuple(goal_list[:self.kinematics.get_dof()])

        self.get_logger().info(f"Point A (current): {tuple(round(x, 3) for x in start_q)}")
        self.get_logger().info(f"Point B (goal):    {tuple(round(x, 3) for x in goal_q)}")

        # Discretize both endpoints onto the toroidal grid
        start_discrete = self.grid.discretize(start_q)
        goal_discrete = self.grid.discretize(goal_q)

        self.get_logger().info(f"Computing {self.get_parameter('planner_type').value.upper()} on toroidal manifold T^n...")
        path = self.planner.plan(start_discrete, goal_discrete)

        if not path:
            msg = "TOPOLOGICAL ERROR: No collision-free path found in C_free."
            self.get_logger().error(msg)
            return (False, msg)

        msg = f"Path found! {len(path)} waypoints. Starting animation..."
        self.get_logger().info(msg)
        self.start_animation(path)
        return (True, msg)


    def _build_full_msg(self, q_planned: tuple) -> JointState:
        """
        Takes the last full GUI message (all XX joints) and overrides
        only the 3 master joints with the planned values.
        This ensures parallelogram_kinematics.py receives ALL joint data.

        Args:
            q_planned (tuple): The planned configuration (2-DOF or 3-DOF).

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
            if self.kinematics.use_horizontal_constraint:
                q_full = (q_planned[0], q_planned[1], 0.0)
            else:
                q_full = q_planned

            # Override master joints
            master_map = {
                'revolute_1_0': q_full[0],
                'revolute_9_0': q_full[1],
                'revolute_10_0': q_full[2],
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
            msg.name = ['revolute_1_0', 'revolute_9_0', 'revolute_10_0']
            msg.position = [float(val) for val in q_3dof[:3]]

        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)
        return msg

    def start_animation(self, path: list):
        """
        Begins stepping through the planned path, publishing each waypoint
        to /master_states at a fixed rate so the robot animates in RViz.
        """
        self.is_animating = True
        self.animation_path = path
        self.animation_index = 0

        # Publish a waypoint every 150ms (smooth but visible)
        self.animation_timer = self.create_timer(0.15, self.animation_step)

    def animation_step(self):
        """
        Timer callback: publishes the next waypoint in the planned path.
        """
        if self.animation_index >= len(self.animation_path):
            self.animation_timer.cancel()
            self.is_animating = False
            # Store the final position so the robot stays there
            self.final_planned_q = self.animation_path[-1]
            self.get_logger().info("Trajectory execution complete ✅")
            return

        q = self.animation_path[self.animation_index]
        out = self._build_full_msg(q)
        self.joint_pub.publish(out)
        self.animation_index += 1


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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
