import math
import numpy as np
from typing import List, Tuple
from .topological_math import TorusTopology

class ReticulaToroidal:
    """
    Representa el espacio de configuraciones discretizado (Voxel Grid)
    topológico en T^n. 
    Traduce el infinito continuo en nodos de salto discretos controlados,
    cuidando meticulosamente los "bordes" del Toro.
    """
    
    def __init__(self, step_size_deg: float = 5.0, num_dof: int = 4):
        """
        Args:
            step_size_deg: Resolución del grid en grados. 
                           Un paso más pequeño da caminos más suaves pero exponencia 
                           la complejidad computacional O(b^d).
            num_dof: Dimensiones de la variedad T^n (Ej. 4 motores libres).
        """
        self.step_rad = math.radians(step_size_deg)
        self.num_dof = num_dof

    def discretizar(self, q_continuous: tuple) -> tuple:
        """
        "Caja Negra" que toma un valor físico contínuo de los sensores
        y lo "ancla" a la coordenada matemática de la retícula más cercana.
        """
        q_discrete = []
        for ang_rad in q_continuous:
            # 1. Asegurar mapeo en [0, 2pi)
            norm_ang = TorusTopology.normalize_angle(ang_rad)
            
            # 2. Encontrar el "escalón" más cercano
            steps = round(norm_ang / self.step_rad)
            snapped_ang = steps * self.step_rad
            
            # 3. Aplicar topología cociente final por si el redondeo 
            # saltó de 359° al escalón "360°", lo devuelve al nodo 0°.
            snapped_ang = TorusTopology.normalize_angle(snapped_ang)  
            
            # Redondeo final para evitar flotantes feos (1e-16) como llaves de diccionario
            q_discrete.append(round(snapped_ang, 4))
            
        return tuple(q_discrete)

    def get_vecinos_topologicos(self, q_node: tuple) -> List[tuple]:
        """
        Genera los nodos adyacentes en la retícula. 
        Implementa vecindad ortogonal estricta (Von Neumann) justificada por la 
        métrica de Manhattan L1 que formulaste para A*.
        """
        vecinos = []
        
        # Iteramos sobre cada eje dimensional del Toro (cada articulación)
        for i in range(self.num_dof):
            # Dos direcciones: Adelante y Atrás
            for direccion in [-1, 1]:
                
                # Copiamos la tupla actual a una lista mutable
                q_vecino = list(q_node)
                
                # Desplazamos un solo motor
                nuevo_ang = q_vecino[i] + (direccion * self.step_rad)
                
                # IMPORTANTE: Cruce estricto de los bordes del Toro $T^n$.
                # Sin esto, el algoritmo A* moriría tratando de ir más allá del borde cartesiano.
                q_vecino[i] = round(TorusTopology.normalize_angle(nuevo_ang), 4)
                
                vecinos.append(tuple(q_vecino))
                
        return vecinos
