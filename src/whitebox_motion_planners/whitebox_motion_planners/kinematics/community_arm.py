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

    def __init__(self, use_horizontal_constraint: bool = False, link_lengths: dict = None):
        self.use_horizontal_constraint = use_horizontal_constraint
        if link_lengths is None:
            raise ValueError("link_lengths dictionary must be provided to CommunityArmKinematics")
        self.base_height = link_lengths['base_height']
        self.lower_shank = link_lengths['lower_shank']
        self.upper_shank = link_lengths['upper_shank']
        self.gripper_dx = link_lengths['gripper_dx']
        self.gripper_dz = link_lengths['gripper_dz']
        self.gripper_k_elbow = link_lengths['gripper_k_elbow']

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

    def compute_forward_kinematics_gripper(self, q: tuple, dx: float = None, dz: float = None, k_elbow: float = None) -> np.ndarray:
        """
        Computes the Cartesian 3D position of the gripper TCP in meters.

        Incorporates:
        1. World-frame shoulder shift: the planning frame defines q2=90° as horizontal,
           but the kinematics model defines q2=0° as horizontal. We subtract pi/2 to align.
        2. Parallelogram kinematics (KEY CORRECTION): the upper shank (forearm) maintains
           its absolute world-frame angle based on q3 ALONE, independent of q2. This is
           because the physical parallelogram linkage drives the forearm from the base,
           not from the end of the shoulder link.
             WRONG (serial):    r = a2*cos(q2) + a3*cos(q2+q3)
             CORRECT (parallel):r = a2*cos(q2) + a3*cos(q3)
         3. Gripper mounting offset (dx, dz): calibrated from RViz2 measurements.
            dx = -0.019 m (longitudinal), dz = -0.015 m (transverse).
         4. Elbow correction (k_elbow=0.080 m/rad): residual empirical correction for 
            gripper assembly geometry when q3 != 0. Calibrated from RViz2.
        """
        if self.use_horizontal_constraint:
            q1, q2 = q
            q3 = 0.0
        else:
            q1, q2, q3 = q
            
        # Shift q2 by -90 degrees (-pi/2 radians) to align world frame (90 deg is horizontal)
        # with the kinematics model frame (0 deg is horizontal).
        q2_kin = q2 - np.pi / 2.0
            
        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2_kin), np.sin(q2_kin)   # lower shank: follows shoulder (q2)
        c3, s3 = np.cos(q3), np.sin(q3)             # upper shank: follows elbow (q3) ONLY
        
        # Use instance variables if parameters are not provided explicitly
        dx = dx if dx is not None else self.gripper_dx
        dz = dz if dz is not None else self.gripper_dz
        k_elbow = k_elbow if k_elbow is not None else self.gripper_k_elbow
        
        # Effective gripper offset: dx grows with elbow angle due to the parallelogram 
        # geometry. sin(|q3|) provides smooth, symmetric correction.
        dx_eff = dx + k_elbow * np.sin(abs(q3))
        
        # Parallelogram model: upper shank and gripper offset project using q3 alone
        r_offset = dx_eff * c3 - dz * s3
        z_offset = dx_eff * s3 + dz * c3
        
        r = self.lower_shank * c2 + self.upper_shank * c3 + r_offset
        x = r * c1
        y = r * s1
        z = self.base_height + self.lower_shank * s2 + self.upper_shank * s3 + z_offset
        
        return np.array([x, y, z])



    def compute_jacobian(self, q: tuple) -> np.ndarray:
        """
        Computes the geometric Jacobian matrix J(q) for the end-effector.
        J is a 3xN matrix (3x2 if use_horizontal_constraint is True, 3x3 otherwise).
        """
        if self.use_horizontal_constraint:
            q1, q2 = q
            q3 = 0.0
        else:
            q1, q2, q3 = q
            
        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2), np.sin(q2)
        c23, s23 = np.cos(q2 + q3), np.sin(q2 + q3)
        
        l1 = self.lower_shank
        l2 = self.upper_shank
        
        # Radius of the end-effector projected on XY plane
        R = l1 * c2 + l2 * c23
        
        # Column 1: derivatives w.r.t q1 (yaw)
        col1 = np.array([
            -R * s1,
            R * c1,
            0.0
        ])
        
        # Column 2: derivatives w.r.t q2 (shoulder pitch)
        col2 = np.array([
            -(l1 * s2 + l2 * s23) * c1,
            -(l1 * s2 + l2 * s23) * s1,
            l1 * c2 + l2 * c23
        ])
        
        if self.use_horizontal_constraint:
            return np.column_stack((col1, col2))
            
        # Column 3: derivatives w.r.t q3 (elbow pitch)
        col3 = np.array([
            -l2 * s23 * c1,
            -l2 * s23 * s1,
            l2 * c23
        ])
        
        return np.column_stack((col1, col2, col3))

    def compute_manipulability(self, q: tuple) -> float:
        """
        Computes the Yoshikawa measure of manipulability:
        w = sqrt(det(J * J^T)) if task space dim <= joint space dim
        w = sqrt(det(J^T * J)) if task space dim > joint space dim
        """
        J = self.compute_jacobian(q)
        m, n = J.shape
        if m <= n:
            return float(np.sqrt(max(0.0, np.linalg.det(J @ J.T))))
        else:
            return float(np.sqrt(max(0.0, np.linalg.det(J.T @ J))))
