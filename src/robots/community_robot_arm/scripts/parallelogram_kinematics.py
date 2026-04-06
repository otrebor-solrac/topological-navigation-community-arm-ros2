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
        
        # 1. Definimos los Motores Principales (Maestros)
        q2 = joints.get('Revolute 9_0', 0.0)   # Motor 2 (Lower Shank)
        q3 = joints.get('Revolute 11_0', 0.0)  # Motor 3 (Lever / Palanca)
        
        # 2. Resolvemos el Paralelogramo (Articulaciones Esclavas dependientes)
        # Angulo Relativo = Angulo_Hijo - Angulo_Padre
        
        # Rev 16 (Codo): Une Upper_Shank (q3) con Lower_Shank (q2)
        joints['Revolute 16_0'] = -q3 - q2
        
        # Rev 12 (Base de la Biela): Une Pleuel (q2) con Lever (q3)
        joints['Revolute 12_0'] = q2 + q3

        # Registro en CSV para análisis de trayectoria (q2=Rev9, q3=Rev11)
        # csv_path = '/tmp/trajectory_analysis.csv'
        # import os
        # if not os.path.exists(csv_path):
        #     with open(csv_path, 'w') as f:
        #         f.write("timestamp_ns,rev9_q2,rev11_q3,rev16_upper,rev12_pleuel\n")
        
        # with open(csv_path, 'a') as f:
        #     f.write(f"{self.get_clock().now().nanoseconds},{q2:.6f},{q3:.6f},{joints['Revolute 16_0']:.6f},{joints['Revolute 12_0']:.6f}\n")
        
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