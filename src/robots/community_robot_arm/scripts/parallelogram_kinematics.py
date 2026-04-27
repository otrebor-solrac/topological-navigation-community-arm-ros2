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
        q1 = joints.get('revolute_1_0', 0.0)
        q2 = joints.get('revolute_9_0', 0.0)   # Motor 2 (Lower Shank)
        q3 = joints.get('revolute_10_0', 0.0)  # Motor 3 (Lever / Palanca)
        
        # 2. Resolvemos el Paralelogramo (Articulaciones dependientes)
        joints['revolute_16_0'] = -q3 - q2
        joints['revolute_12_0'] = q2 + q3

        # 3. Bielas Inferiores (Nuevos "Shanks" falsos anclados al cuerpo principal)
        # El usuario modificó el URDF para que las bielas curvas cuelguen de main_body
        # Deben imitar exactamente el ángulo de lower_shank (q2)
        joints['revolute_32_0'] = q2   # Eje 0 1 0
        joints['revolute_31_0'] = -q2  # Eje 0 -1 0

        # 4. Triplates colgando de las bielas inferiores
        # Deben mantenerse nivelados compensando la rotación de sus padres (q2)
        joints['revolute_13_0'] = -q2  # Eje 0 1 0 (inverso a revolute_32)
        joints['revolute_18_0'] = q2   # Eje 0 -1 0 (inverso a revolute_31)

        # 5. Bielas Superiores (colgando del triplate hacia el efector final)
        # Invertimos el signo para que giren en la misma dirección que el upper_shank
        joints['revolute_15_0'] = -q3   # Eje 0 -1 0
        joints['revolute_19_0'] = q3    # Eje 0 1 0


        
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