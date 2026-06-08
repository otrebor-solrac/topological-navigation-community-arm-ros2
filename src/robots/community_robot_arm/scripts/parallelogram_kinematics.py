#!/usr/bin/env python3
"""
Parallelogram Kinematic Node
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ParallelogramKinematics(Node):

    def __init__(self):
        # Initialize the node
        super().__init__('parallelogram_kinematics')

        # Create a publisher for the joint states
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        # Create a subscription to the master states
        self.sub = self.create_subscription(
            JointState, '/master_states', self.cb, 10)
        
        # Log that the node has started
        self.get_logger().info('Parallelogram kinematics node started')

    def cb(self, msg: JointState):
        joints = dict(zip(msg.name, msg.position))
        
        # 1. Define the primary master motor states
        q1 = joints.get('base_yaw_joint', 0.0)
        q2 = joints.get('shoulder_pitch_joint', 0.0)   # Motor 2 (Lower Shank)
        q3 = joints.get('elbow_pitch_joint', 0.0)      # Motor 3 (Lever / Palanca)
        
        # 2. Solve the parallelogram linkage constraint (dependent joints)
        joints['revolute_16_0'] = -q3 - q2
        joints['revolute_12_0'] = q2 + q3

        # 3. Lower shanks/linkages (imitating lower_shank pitch)
        # The URDF has curved linkages hanging from the main_body.
        # They must copy the lower_shank pitch angle (q2).
        joints['revolute_32_0'] = q2   # Axis [0, 1, 0]
        joints['revolute_31_0'] = -q2  # Axis [0, -1, 0]

        # 4. Triplates hanging from the lower linkages
        # They must remain level by compensating for their parent rotation (q2)
        joints['revolute_13_0'] = -q2  # Axis [0, 1, 0] (inverse to revolute_32)
        joints['revolute_18_0'] = q2   # Axis [0, -1, 0] (inverse to revolute_31)

        # 5. Upper linkages hanging from the triplate towards the end-effector
        # Invert signs to match upper_shank rotation directions
        joints['revolute_15_0'] = -q3   # Axis [0, -1, 0]
        joints['revolute_19_0'] = q3    # Axis [0, 1, 0]


        
        out = JointState()
        out.header   = msg.header
        out.name     = list(joints.keys())
        out.position = list(joints.values())
        out.velocity = [0.0] * len(out.name)
        out.effort   = [0.0] * len(out.name)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ParallelogramKinematics()
    try:
        # Main loop of the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()