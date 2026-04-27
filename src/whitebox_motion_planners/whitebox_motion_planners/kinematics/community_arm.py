import numpy as np
from typing import List
from ..core.interfaces import BaseKinematics

class CommunityArmKinematics(BaseKinematics):
    """
    Forward Kinematics for the Community Robot Arm (T^3).

    Simplified 3-DOF serial model:
        q1 = revolute_1_0  → Base rotation (yaw, around Z)
        q2 = revolute_9_0  → Shoulder (lower shank pitch)
        q3 = revolute_10_0 → Elbow (lever/palanca pitch)

    Link lengths are approximate values derived from the URDF geometry.
    These will be refined once precise DH parameters are extracted.
    """

    # Approximate link lengths [meters] from URDF visual inspection
    BASE_HEIGHT = 0.065   # Height from root to shoulder axis
    LOWER_SHANK = 0.140   # Lower shank length (140mm)
    UPPER_SHANK = 0.140   # Upper shank length (140mm)

    def __init__(self):
        pass

    def get_dof(self) -> int:
        """
        Returns the number of independent Degrees of Freedom (DOF).

        The Community Arm is treated as a 3-DOF system (T^3) because it has
        three master joints that define its configuration:

        1. Base Rotation (revolute_1_0)
        2. Shoulder Pitch (revolute_9_0)
        3. Elbow/Lever Pitch (revolute_10_0)
        
        All other moving joints are kinematically dependent on these three
        via the parallelogram linkage logic.
        """
        return 3

    def compute_forward_kinematics(self, q: tuple) -> List[np.ndarray]:
        """
        Computes joint positions in R^3 for the 3-DOF serial approximation.

        Args:
            q: Tuple of 3 joint angles (q1, q2, q3) in radians.

        Returns:
            List of 4 position vectors [p_base, p_shoulder, p_elbow, p_end].
        """
        q1, q2, q3 = q
        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2), np.sin(q2)
        c23, s23 = np.cos(q2 + q3), np.sin(q2 + q3)

        # P0: Base (origin)
        p0 = np.array([0.0, 0.0, 0.0])

        # P1: Shoulder joint (fixed elevation)
        p1 = np.array([0.0, 0.0, self.BASE_HEIGHT])

        # P2: Elbow joint (Spherical projection of Lower Shank)
        # r = link_length * cos(pitch), z = link_length * sin(pitch)
        r2 = self.LOWER_SHANK * c2
        p2 = p1 + np.array([
            r2 * c1, # X: projection on XY plane * cos(yaw)
            r2 * s1, # Y: projection on XY plane * sin(yaw)
            self.LOWER_SHANK * s2  # Z: vertical elevation
        ])

        # P3: End effector (Spherical projection of Upper Shank)
        r3 = self.UPPER_SHANK * c23
        p3 = p2 + np.array([
            r3 * c1, # X
            r3 * s1, # Y
            self.UPPER_SHANK * s23 # Z
        ])

        return [p0, p1, p2, p3]
