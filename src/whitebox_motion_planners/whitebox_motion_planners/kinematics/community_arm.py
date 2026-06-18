import numpy as np
from typing import List
from ..core.interfaces import BaseKinematics

class CommunityArmKinematics(BaseKinematics):
    """
    Forward Kinematics for the Community Robot Arm (T^3).

    Simplified 3-DOF serial model:
        q1 = base_yaw_joint  → Base rotation (yaw, around Z)
        q2 = shoulder_pitch_joint  → Shoulder (lower shank pitch)
        q3 = elbow_pitch_joint → Elbow (lever/palanca pitch)

    Link lengths are approximate values derived from the URDF geometry.
    These will be refined once precise DH parameters are extracted.
    """

    # Approximate link lengths [meters] from URDF visual inspection
    BASE_HEIGHT = 0.065   # Height from root to shoulder axis
    LOWER_SHANK = 0.140   # Lower shank length (140mm)
    UPPER_SHANK = 0.140   # Upper shank length (140mm)

    def __init__(self, use_horizontal_constraint: bool = False, link_lengths: dict = None):
        self.use_horizontal_constraint = use_horizontal_constraint
        if link_lengths is not None:
            self.base_height = link_lengths.get('base_height', self.BASE_HEIGHT)
            self.lower_shank = link_lengths.get('lower_shank', self.LOWER_SHANK)
            self.upper_shank = link_lengths.get('upper_shank', self.UPPER_SHANK)
        else:
            self.base_height = self.BASE_HEIGHT
            self.lower_shank = self.LOWER_SHANK
            self.upper_shank = self.UPPER_SHANK

    def get_dof(self) -> int:
        """
        Returns the number of independent Degrees of Freedom (DOF).

        The Community Arm is normally treated as a 3-DOF system (T^3):
        1. Base Rotation (base_yaw_joint)
        2. Shoulder Pitch (shoulder_pitch_joint)
        3. Elbow/Lever Pitch (elbow_pitch_joint)
        
        If use_horizontal_constraint is True, the Elbow (q3) becomes dependent 
        on the Shoulder (q2), reducing the planning space to 2-DOF.
        """
        return 2 if self.use_horizontal_constraint else 3

    def compute_forward_kinematics(self, q: tuple) -> List[np.ndarray]:
        """
        Computes joint positions in R^3 for the serial approximation.

        Args:
            q: Tuple of joint angles. (q1, q2) if horizontal, (q1, q2, q3) otherwise.

        Returns:
            List of 4 position vectors [p_base, p_shoulder, p_elbow, p_end].
        """
        if self.use_horizontal_constraint:
            q1, q2 = q
            q3 = 0.0  # Parallelogram keeps it horizontal if lever is at 0
        else:
            q1, q2, q3 = q
        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2), np.sin(q2)
        c23, s23 = np.cos(q2 + q3), np.sin(q2 + q3)

        # P0: Base (origin)
        p0 = np.array([0.0, 0.0, 0.0])

        # P1: Shoulder joint (fixed elevation)
        p1 = np.array([0.0, 0.0, self.base_height])

        # P2: Elbow joint (Spherical projection of Lower Shank)
        # r = link_length * cos(pitch), z = link_length * sin(pitch)
        r2 = self.lower_shank * c2
        p2 = p1 + np.array([
            r2 * c1, # X: projection on XY plane * cos(yaw)
            r2 * s1, # Y: projection on XY plane * sin(yaw)
            self.lower_shank * s2  # Z: vertical elevation
        ])

        # P3: End effector (Spherical projection of Upper Shank)
        r3 = self.upper_shank * c23
        p3 = p2 + np.array([
            r3 * c1, # X
            r3 * s1, # Y
            self.upper_shank * s23 # Z
        ])

        return [p0, p1, p2, p3]
