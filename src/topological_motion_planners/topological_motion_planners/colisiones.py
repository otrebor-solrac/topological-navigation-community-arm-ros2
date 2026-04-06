import numpy as np
import math
from typing import List, Tuple

class EsferaTopologica:
    """ Representa una bola abierta B(p, epsilon) en R^3 """
    def __init__(self, centro: np.ndarray, radio: float):
        self.centro = centro
        self.radio = radio

class OpenManipulatorKinematics:
    """
    Cinemática Directa simplificada del OpenManipulator-X para calcular 
    la posición de sus articulaciones en R^3.
    """
    def __init__(self):
        # Longitudes aproximadas de los eslabones (en metros)
        self.l1 = 0.077
        self.l2 = 0.130
        self.l3 = 0.124
        self.l4 = 0.126

    def compute_joint_positions(self, q: tuple) -> List[np.ndarray]:
        """
        Retorna las posiciones espaciales [P0, P1, P2, P3, P4] 
        de los ejes de rotación dados los ángulos articulares q en T^n.
        """
        q1, q2, q3, q4 = q
        
        # Geometría base en el origen
        p0 = np.array([0.0, 0.0, 0.0])
        
        # Link 1 (Rotación Z)
        p1 = p0 + np.array([0.0, 0.0, self.l1])
        
        # Link 2 (Elevación Y)
        r2 = self.l2
        p2 = p1 + np.array([
            r2 * math.cos(q1) * math.cos(q2),
            r2 * math.sin(q1) * math.cos(q2),
            r2 * math.sin(q2)
        ])
        
        # Link 3 (Codo Y)
        r3 = self.l3
        q23 = q2 + q3
        p3 = p2 + np.array([
            r3 * math.cos(q1) * math.cos(q23),
            r3 * math.sin(q1) * math.cos(q23),
            r3 * math.sin(q23)
        ])
        
        # Link 4 (Muñeca Y)
        r4 = self.l4
        q234 = q2 + q3 + q4
        p4 = p3 + np.array([
            r4 * math.cos(q1) * math.cos(q234),
            r4 * math.sin(q1) * math.cos(q234),
            r4 * math.sin(q234)
        ])
        
        return [p0, p1, p2, p3, p4]

class ValidadorColisiones:
    """
    Implementa la evaluación de C_free usando recubrimientos de bolas abiertas
    para evitar el costoso choque de polígonos (AABB).
    """
    def __init__(self, radio_eslabon: float = 0.035, puntos_interpolacion: int = 4):
        self.cinematica = OpenManipulatorKinematics()
        self.radio_eslabon = radio_eslabon
        self.puntos_inter = puntos_interpolacion
        
        # Lista de obstáculos (Esferas) en el espacio R^3
        self.obstaculos_esfericos: List[EsferaTopologica] = []
        
    def add_obstaculo(self, centro: tuple, radio: float):
        self.obstaculos_esfericos.append(EsferaTopologica(np.array(centro), radio))

    def _generar_recubrimiento_bolas(self, posiciones_articulaciones: List[np.ndarray]) -> List[EsferaTopologica]:
        """
        Interpola esferas topológicas a lo largo de los eslabones del robot
        para generar un volumen sólido de colisión en base a la línea central.
        """
        esferas_robot = []
        for i in range(len(posiciones_articulaciones) - 1):
            p_inicio = posiciones_articulaciones[i]
            p_fin = posiciones_articulaciones[i+1]
            
            # Interpolación lineal de centros de esfera
            for j in range(self.puntos_inter + 1):
                t = j / float(self.puntos_inter)
                centro_inter = p_inicio + t * (p_fin - p_inicio)
                esferas_robot.append(EsferaTopologica(centro_inter, self.radio_eslabon))
                
        return esferas_robot

    def is_valid_config(self, q: tuple) -> bool:
        """
        Toma una configuración q en T^n y retorna True si pertenece a C_free.
        Retorna False si cae dentro de C_obs (colisión matemática detectada).
        """
        # 1. Transformamos q (Ángulos) en posiciones R^3 (Cinemática Directa)
        posiciones = self.cinematica.compute_joint_positions(q)
        
        # 2. Generamos el recubrimiento de bolas sobre esas posiciones
        bolas_robot = self._generar_recubrimiento_bolas(posiciones)
        
        # 3. Evaluamos la distancia euclidiana L2 contra los obstáculos de entorno
        for bola_r in bolas_robot:
            for obs in self.obstaculos_esfericos:
                dist_l2 = np.linalg.norm(bola_r.centro - obs.centro)
                # Si la distancia entre centros es menor a la suma de radios, hay intersección
                if dist_l2 <= (bola_r.radio + obs.radio):
                    return False # Colisión matemáticamente confirmada (C_obs)
                    
        return True # Es seguro (C_free)
