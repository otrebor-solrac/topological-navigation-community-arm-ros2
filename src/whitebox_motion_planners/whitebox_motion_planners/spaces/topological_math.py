import math
import numpy as np

class TorusTopology:
    """
    Utility class for pure mathematical operations on the n-dimensional Torus T^n.
    """
    
    @staticmethod
    def wrap_around_dist(theta1: float, theta2: float) -> float:
        """
        Calculates the shortest geodesic distance on S^1 (Circle).
        Implements modular arithmetic: min(|a-b|, 2pi - |a-b|).
        """
        diff = abs(theta1 - theta2)
        return min(diff, 2 * math.pi - diff)

    @staticmethod
    def normalize_angle(theta: float) -> float:
        """
        Ensures that any given angle maps within [0, 2pi).
        Vital for sum algorithms to maintain compactness.
        """
        return theta % (2 * math.pi)
