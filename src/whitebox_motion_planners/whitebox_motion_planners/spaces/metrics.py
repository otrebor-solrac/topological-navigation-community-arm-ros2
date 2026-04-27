import math
import numpy as np
from .topological_math import TorusTopology

class Metrics:
    """Collection of metrics evaluable on the toroidal manifold."""
    
    @staticmethod
    def heuristic_L1(q1: np.ndarray, q2: np.ndarray) -> float:
        """
        L1 metric (Manhattan) evaluated on T^n.
        """
        dist_sum = 0.0
        for x, y in zip(q1, q2):
            dist_sum += TorusTopology.wrap_around_dist(x, y)
        return dist_sum

    @staticmethod
    def heuristic_L2(q1: np.ndarray, q2: np.ndarray) -> float:
        """
        L2 metric (Euclidean) evaluated on T^n.
        """
        dist_sum = 0.0
        for x, y in zip(q1, q2):
            dist_sum += TorusTopology.wrap_around_dist(x, y) ** 2
        return math.sqrt(dist_sum)
