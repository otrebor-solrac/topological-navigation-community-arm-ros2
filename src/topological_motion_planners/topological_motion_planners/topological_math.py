import math
import numpy as np

class TorusTopology:
    """
    Clase que implementa las propiedades matemáticas del Toro T^n
    descritas en el Marco Teórico, resolviendo las topologías cociente.
    """
    
    @staticmethod
    def wrap_around_dist(theta1: float, theta2: float) -> float:
        """
        Calcula la distancia geodésica más corta en S^1 (Circunferencia). 
        Implementa la aritmética modular: min(|a-b|, 2pi - |a-b|)
        """
        diff = abs(theta1 - theta2)
        return min(diff, 2 * math.pi - diff)

    @staticmethod
    def heuristic_L1(q1: np.ndarray, q2: np.ndarray) -> float:
        """
        Métrica L1 (Manhattan) evaluada en T^n. Utilizada por A* 
        para una expansión de nodos altamente direccionada.
        Q1 y Q2 son vectores (arrays) de ángulos en radianes.
        """
        dist_sum = 0.0
        for x, y in zip(q1, q2):
            dist_sum += TorusTopology.wrap_around_dist(x, y)
        return dist_sum

    @staticmethod
    def heuristic_L2(q1: np.ndarray, q2: np.ndarray) -> float:
        """
        Métrica L2 (Euclidiana) evaluada en T^n. 
        Para comparar numéricamente su ineficiencia frente a L1 en retículas.
        """
        dist_sum = 0.0
        for x, y in zip(q1, q2):
            dist_sum += TorusTopology.wrap_around_dist(x, y) ** 2
        return math.sqrt(dist_sum)

    @staticmethod
    def normalize_angle(theta: float) -> float:
        """
        Asegura que cualquier ángulo dado se mapee dentro de [0, 2pi).
        Es vital para que los algoritmos de suma mantengan la compacidad.
        """
        return theta % (2 * math.pi)
