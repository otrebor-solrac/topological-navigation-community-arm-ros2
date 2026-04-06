import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

from .colisiones import ValidadorColisiones
from .generador_reticula import ReticulaToroidal
from .a_star import AStarPlanner

class TopologicalPlannerNode(Node):
    """
    Punto de convergencia entre las matemáticas abstractas de T^n 
    y la simulación física a nivel de control (ROS2 Middleware).
    """
    def __init__(self):
        super().__init__('topological_planner_node')
        self.get_logger().info("Inicializando Cerebro Matemático (A* en T^n)...")
        
        # 1. Configuración del Núcleo Matemático (Voxel Grid y Colisiones)
        # Usamos 8 grados para mantener el cómputo de A* ágil pero preciso.
        self.grid = ReticulaToroidal(step_size_deg=8.0, num_dof=4)  
        self.collider = ValidadorColisiones(radio_eslabon=0.04)
        
        # Añadiremos un obstáculo esférico topológico enfrente del brazo
        # para probar la evasión determinista de A*.
        self.collider.add_obstaculo(centro=(0.15, 0.0, 0.15), radio=0.08)
        
        # 2. Configuración del Planificador Determinista
        self.astar = AStarPlanner(heuristic_type='L1')
        
        # 3. Configuración ROS2 (Hardware Interface / Gazebo)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        
    def plan_and_execute(self, start_q: tuple, goal_q: tuple):
        self.get_logger().info(f"Discretizando objetivo {goal_q}...")
        
        # Anclar la petición continua a los nodos strictos del Grid
        start_discrete = self.grid.discretizar(start_q)
        goal_discrete = self.grid.discretizar(goal_q)
        
        self.get_logger().info("Traduciendo Toro a grafo y evaluando colisiones... calculando A*")
        
        # Llamar a la lógica matemática Pura de tu Capítulo 2
        ruta_topologica = self.astar.plan(
            start_q=start_discrete,
            goal_q=goal_discrete,
            is_valid_fn=self.collider.is_valid_config,
            get_neighbors_fn=self.grid.get_vecinos_topologicos
        )
        
        if not ruta_topologica:
            self.get_logger().error("¡ERROR TOPOLÓGICO! No se halló un camino en C_free. El objetivo es inalcanzable sin colisión.")
            return
            
        self.get_logger().info(f"¡Camino topológico encontrado! Consta de {len(ruta_topologica)} saltos en el Voxel Grid.")
        self.send_trajectory(ruta_topologica)
        
    def send_trajectory(self, path: list):
        """ Convierte la lista abstracta de tuplas topológicas en comandos servomotores físicos """
        self.get_logger().info("Conectando con puente controlador de Gazebo...")
        self.action_client.wait_for_server()
        
        msg = FollowJointTrajectory.Goal()
        msg.trajectory = JointTrajectory()
        msg.trajectory.joint_names = self.joint_names
        
        time_from_start = 0.0
        time_per_step = 0.15  # Segundos asignados a recorrer cada salto de la retícula
        
        for q in path:
            point = JointTrajectoryPoint()
            point.positions = list(q)
            # Aceleración a cero para prevenir vibración caótica por discretización rígida (Falta Bézier)
            point.velocities = [0.0, 0.0, 0.0, 0.0]
            
            time_from_start += time_per_step
            sec = int(time_from_start)
            nanosec = int((time_from_start - sec) * 1e9)
            point.time_from_start.sec = sec
            point.time_from_start.nanosec = nanosec
            
            msg.trajectory.points.append(point)
            
        self.action_client.send_goal_async(msg)
        self.get_logger().info("Trayectoria convertida al espacio G y enviada a la ejecución física ✅.")

def main(args=None):
    rclpy.init(args=args)
    node = TopologicalPlannerNode()
    
    # ESTADO INICIAL FÍSICO (El robot empieza descansando apuntando al cielo generalmente)
    start_angles = (0.0, 0.0, 0.0, 0.0)
    
    # OBJETIVO MATEMÁTICO: Doblarse cruzando a través del obstáculo invisible (x=0.15, z=0.15)
    # A* debería re-rutear el brazo para "evitar" tocar esa esfera en medio del vuelo.
    goal_angles = (math.pi/4, math.pi/4, -math.pi/4, 0.0) 
    
    node.plan_and_execute(start_angles, goal_angles)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
