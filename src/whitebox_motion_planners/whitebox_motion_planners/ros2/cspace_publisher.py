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
        
        # Parameters (Adjust resolution to avoid saturating the web)
        self.declare_parameter('robot_type', 'community_arm')
        self.declare_parameter('step_size_deg', 15.0) 
        self.declare_parameter('use_horizontal_constraint', True)
        self.declare_parameter('use_obstacles', True)
        self.declare_parameter('sphere_thinning_dist', 0.015)

        # Componentes White-Box
        robot_type = self.get_parameter('robot_type').value
        step_size = self.get_parameter('step_size_deg').value
        use_horizontal = self.get_parameter('use_horizontal_constraint').value
        use_obstacles = self.get_parameter('use_obstacles').value
        
        self.kinematics = get_kinematics(robot_type, use_horizontal_constraint=use_horizontal)
        
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
                
        thinning_dist = self.get_parameter('sphere_thinning_dist').value
        self.collider = FoamCollider(
            urdf_path=urdf_path,
            sphere_thinning_dist=thinning_dist
        )
        
        obstacles_hash = "no_obstacles"
        if use_obstacles:
            try:
                # Absolute path to obstacles URDF file
                pkg_share = get_package_share_directory('community_robot_arm')
                urdf_path = os.path.join(pkg_share, 'urdf', 'obstacles', 'box_obstacle_spherized.urdf')
                
                self.get_logger().info(f"CSpace Voxelizer: Loading environment obstacles from: {urdf_path}")
                
                if os.path.exists(urdf_path):
                    obstacles = self._load_obstacles_from_urdf(urdf_path)
                    
                    # Add obstacles to collider
                    for center, radius in obstacles:
                        self.collider.add_obstacle(center, radius)
                        self.get_logger().info(f"CSpace Voxelizer: Added obstacle sphere: center={center}, radius={radius:.3f}")
                        
                    # Calculate short MD5 hash of obstacles URDF to auto-invalidate cache on changes
                    import hashlib
                    hash_md5 = hashlib.md5()
                    with open(urdf_path, "rb") as f:
                        for chunk in iter(lambda: f.read(4096), b""):
                            hash_md5.update(chunk)
                    obstacles_hash = hash_md5.hexdigest()[:8]
                else:
                    self.get_logger().warn(f"CSpace Voxelizer: Obstacles URDF file not found at: {urdf_path}")
            
            except Exception as e:
                self.get_logger().error(f"CSpace Voxelizer: Failed to load obstacles: {e}")

        self.grid = GridDiscretizer(step_size_deg=step_size, num_dof=self.kinematics.get_dof())

        # Resolve persistent source directory to write/load cache files (Docker-compatible volume)
        if os.path.exists('/home/ros_ws/src/whitebox_motion_planners'):
            self.cache_dir = '/home/ros_ws/src/whitebox_motion_planners/cspace_cache'
        else:
            src_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            self.cache_dir = os.path.join(src_dir, 'cspace_cache')
        os.makedirs(self.cache_dir, exist_ok=True)
        
        self.cache_filename = f"cspace_cache_{step_size}deg_{thinning_dist}m_{obstacles_hash}.json"
        self.cache_filepath = os.path.join(self.cache_dir, self.cache_filename)
        self.get_logger().info(f"Cache filepath: {self.cache_filepath}")

        # Publicador de String (JSON) para Rosbridge
        self.publisher_ = self.create_publisher(String, '/cspace_voxels', 10)
        
        # Timer para publicar voxels cuando haya suscriptores (ej: dashboard web)
        self.timer = self.create_timer(2.0, self.publish_voxels)
        self.cached_voxels_msg = None

        # === Intentar cargar la caché al arranque ===
        self._load_cspace_if_exists()

        # === Registrar servicio para generar la caché bajo demanda ===
        self.srv = self.create_service(Trigger, 'generate_cspace', self.generate_cspace_callback)

        self.get_logger().info(f"C-Space Voxelizer iniciado (Resolución: {step_size} deg)")

    def _load_cspace_if_exists(self):
        """
        Carga la caché del C-Space desde disco si existe.
        """
        if os.path.exists(self.cache_filepath):
            self.get_logger().info(f"CSpace Voxelizer: Cargando caché desde: {self.cache_filepath}")
            try:
                with open(self.cache_filepath, 'r') as f:
                    cached_json = f.read()
                json.loads(cached_json)  # Validar sintaxis JSON
                
                msg = String()
                msg.data = cached_json
                self.cached_voxels_msg = msg
                self.get_logger().info("¡C-Space cargado desde caché exitosamente!")
                return True
            except Exception as e:
                self.get_logger().error(f"Error cargando caché: {e}.")
                self.cached_voxels_msg = None
        else:
            self.get_logger().warn("No se encontró archivo de caché C-Space.")
            self.get_logger().info("El planificador usará colisiones en tiempo real y el dashboard no mostrará obstáculos.")
            self.get_logger().info("Puedes generar la caché llamando al servicio: ros2 service call /generate_cspace std_srvs/srv/Trigger")
        return False

    def generate_cspace_callback(self, request, response):
        """
        Callback del servicio /generate_cspace para calcular y guardar la caché en segundo plano.
        """
        self.get_logger().info("Iniciando generación de caché del C-Space...")
        
        forbidden_voxels = []
        states = self.grid.get_all_states()
        total_states = len(states)
        
        self.get_logger().info(f"Calculando {total_states} estados del C-Space...")
        
        for idx, q_discrete in enumerate(states):
            if idx % 5000 == 0 and idx > 0:
                self.get_logger().info(f"Progreso: {idx}/{total_states} estados procesados ({(idx/total_states)*100:.1f}%)...")
                
            q_radians = self.grid.get_radians(q_discrete)
            
            if not self.collider.is_state_valid(q_radians, self.kinematics):
                q0 = (q_radians[0] + np.pi) % (2 * np.pi) - np.pi
                q1 = (q_radians[1] + np.pi) % (2 * np.pi) - np.pi
                q2 = (q_radians[2] + np.pi) % (2 * np.pi) - np.pi if len(q_radians) > 2 else 0.0
                
                forbidden_voxels.append([
                    round(float(q0), 3),
                    round(float(q1), 3),
                    round(float(q2), 3)
                ])
                
        # Guardar el C-Space calculado en disco
        serialized_data = json.dumps(forbidden_voxels)
        try:
            with open(self.cache_filepath, 'w') as f:
                f.write(serialized_data)
            self.get_logger().info(f"C-Space guardado en caché exitosamente: {self.cache_filepath}")
            
            msg = String()
            msg.data = serialized_data
            self.cached_voxels_msg = msg
            
            response.success = True
            response.message = f"C-Space generado exitosamente. {len(forbidden_voxels)} voxels guardados en {self.cache_filepath}."
        except Exception as e:
            err_msg = f"Error al escribir la caché en disco: {e}"
            self.get_logger().error(err_msg)
            response.success = False
            response.message = err_msg
            
        return response

    def publish_voxels(self):
        """Publica los voxels al tópico solo cuando hay suscriptores."""
        if self.cached_voxels_msg is not None and self.publisher_.get_subscription_count() > 0:
            self.publisher_.publish(self.cached_voxels_msg)

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

def main(args=None):
    rclpy.init(args=args)
    node = CSpaceVoxelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
