import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
from ..collision.foam_collider import FoamCollider
from ..collision.grid_discretizer import GridDiscretizer
from ..kinematics.community_arm import CommunityArmKinematics

class CSpaceVoxelPublisher(Node):
    def __init__(self):
        super().__init__('cspace_voxel_publisher')
        
        # Parámetros (Ajustar resolución para no saturar la web)
        self.declare_parameter('step_size_deg', 15.0) 
        self.declare_parameter('link_radius', 0.04)
        self.declare_parameter('use_horizontal_constraint', True)

        # Componentes White-Box
        step_size = self.get_parameter('step_size_deg').value
        use_horizontal = self.get_parameter('use_horizontal_constraint').value
        
        self.kinematics = CommunityArmKinematics(use_horizontal_constraint=use_horizontal)
        self.collider = FoamCollider(link_radius=self.get_parameter('link_radius').value)
        self.grid = GridDiscretizer(step_size_deg=step_size, num_dof=self.kinematics.get_dof())

        # Publicador de String (JSON) para Rosbridge
        self.publisher_ = self.create_publisher(String, '/cspace_voxels', 10)
        
        # Timer para calcular y enviar (solo una vez o bajo demanda)
        self.timer = self.create_timer(2.0, self.publish_voxels)
        self.has_published = False

        self.get_logger().info(f"C-Space Voxelizer iniciado (Resolución: {step_size} deg)")

    def publish_voxels(self):
        if self.has_published:
            return

        self.get_logger().info("Calculando obstáculos en el C-Space T^n... (esto puede tardar unos segundos)")
        
        forbidden_voxels = []
        
        # Recorrer todo el toro discretizado
        # Si es 2D (constrained), th3 será 0. Si es 3D, recorrerá th3.
        dof = self.kinematics.get_dof()
        
        # Generar todos los estados posibles en la rejilla
        # Para simplificar y no matar el navegador, enviamos solo los centros de colisión
        states = self.grid.get_all_states()
        
        for q_discrete in states:
            # Convertir índice de rejilla a ángulos reales
            q_radians = self.grid.get_radians(q_discrete)
            
            if not self.collider.is_state_valid(q_radians, self.kinematics):
                # Es un obstáculo en el C-Space
                forbidden_voxels.append([
                    round(float(q_radians[0]), 3),
                    round(float(q_radians[1]), 3),
                    round(float(q_radians[2]), 3) if len(q_radians) > 2 else 0.0
                ])

        # Enviar como JSON
        msg = String()
        msg.data = json.dumps(forbidden_voxels)
        self.publisher_.publish(msg)
        
        self.get_logger().info(f"¡C-Space mapeado! {len(forbidden_voxels)} voxels prohibidos enviados al Dashboard.")
        self.has_published = True

def main(args=None):
    rclpy.init(args=args)
    node = CSpaceVoxelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
