import math
import numpy as np
from typing import List
from ..core.interfaces import BaseKinematics

class OpenManipulatorKinematics(BaseKinematics):
    """
    Simplified Forward Kinematics of the OpenManipulator-X to calculate 
    the position of its joints in R^3. (Legacy Model)
    """
    def __init__(self):
        # Approximate link lengths (in meters)
        self.l1 = 0.077
        self.l2 = 0.130
        self.l3 = 0.124
        self.l4 = 0.126

    def get_dof(self) -> int:
        return 4

    def compute_forward_kinematics(self, q: tuple) -> List[np.ndarray]:
        """
        Returns the spatial positions [P0, P1, P2, P3, P4] 
        of the rotation axes given the joint angles q in T^n.
        """
        q1, q2, q3, q4 = q
        
        # Geometría base en el origen
        p0 = np.array([0.0, 0.0, 0.0])
        p1 = p0 + np.array([0.0, 0.0, self.l1])
        
        r2 = self.l2
        p2 = p1 + np.array([
            r2 * math.cos(q1) * math.cos(q2),
            r2 * math.sin(q1) * math.cos(q2),
            r2 * math.sin(q2)
        ])
        
        r3 = self.l3
        q23 = q2 + q3
        p3 = p2 + np.array([
            r3 * math.cos(q1) * math.cos(q23),
            r3 * math.sin(q1) * math.cos(q23),
            r3 * math.sin(q23)
        ])
        
        r4 = self.l4
        q234 = q2 + q3 + q4
        p4 = p3 + np.array([
            r4 * math.cos(q1) * math.cos(q234),
            r4 * math.sin(q1) * math.cos(q234),
            r4 * math.sin(q234)
        ])
        
        return [p0, p1, p2, p3, p4]
