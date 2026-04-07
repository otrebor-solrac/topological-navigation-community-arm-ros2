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
        q2 = joints.get('revolute_9_0', 0.0)   # Motor 2 (Lower Shank)
        q3 = joints.get('revolute_10_0', 0.0)  # Motor 3 (Lever / Palanca)
        
        # 2. Resolvemos el Paralelogramo (Articulaciones dependientes)
        joints['revolute_16_0'] = -q3 - q2
        joints['revolute_12_0'] = q2 + q3

        # 3. Triplates (7 y 17) - Reparenteados al brazo
        # Los mantenemos nivelados compensando la rotación del Lower Shank (q2)
        joints['revolute_7_0'] = -q2
        joints['revolute_17_0'] = q2  # Signo opuesto si el eje es invertido en URDF

        # 4. Bielas curvas (Paralelogramos superior e inferior)
        # Inferiores (van a la base, dependen del Joint 2):
        # revolute_13_0 (eje 0 1 0) -> Sigue a q2
        joints['revolute_13_0'] = q2
        # revolute_18_0 (eje 0 -1 0) -> Sigue a q2 pero con eje invertido
        joints['revolute_18_0'] = -q2

        # Superiores (van al efector final, dependen del Joint 3):
        # El upper_shank tiene un ángulo absoluto de -q3 (debido a la geometría).
        # revolute_15_0 (eje 0 -1 0) -> Sigue a -q3 con eje invertido -> q3
        joints['revolute_15_0'] = q3
        # revolute_19_0 (eje 0 1 0) -> Sigue a -q3
        joints['revolute_19_0'] = -q3


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