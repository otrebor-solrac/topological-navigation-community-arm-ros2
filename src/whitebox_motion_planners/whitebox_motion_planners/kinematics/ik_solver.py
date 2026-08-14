import numpy as np
import math
from typing import Optional, Tuple
from .community_arm import CommunityArmKinematics

class CommunityArmIKSolver:
    """
    Closed-form Analytical Inverse Kinematics (IK) for the 3-DOF Community Robot Arm.
    
    Given Cartesian end-effector position (x, y, z) in meters, computes joint angles
    q = (q1, q2, q3) in radians such that FK(q) == (x, y, z).
    """

    def __init__(self, kinematics: CommunityArmKinematics):
        self.kinematics = kinematics
        self.base_height = kinematics.base_height
        self.lower_shank = kinematics.lower_shank
        self.upper_shank = kinematics.upper_shank
        self.gripper_dx = kinematics.gripper_dx
        self.gripper_dz = kinematics.gripper_dz
        self.gripper_k_elbow = kinematics.gripper_k_elbow
        
        # Link 1 length
        self.L1 = self.lower_shank
        # Link 2 length (since gripper is perfectly horizontal, we don't extend L2)
        self.L2 = self.upper_shank

    def is_reachable(self, x: float, y: float, z: float) -> bool:
        """
        Check if target Cartesian coordinate (x, y, z) is reachable by the arm.
        """
        r_xy = math.sqrt(x**2 + y**2)
        
        # Because the gripper is perfectly horizontal, we subtract its offset to find the wrist target
        r_wrist = abs(r_xy - self.gripper_dx)
        z_wrist = z - self.gripper_dz
        
        z_prime = z_wrist - self.base_height
        d = math.sqrt(r_wrist**2 + z_prime**2)
        
        max_reach = self.L1 + self.L2
        min_reach = abs(self.L1 - self.L2)
        
        return min_reach <= d <= max_reach

    def is_valid_solution(self, q: Tuple[float, float, float]) -> bool:
        """
        Enforce physical joint limits on q = (q1, q2, q3):
        q2 (shoulder pitch): must be >= 0 (arm above ground level)
        """
        q1, q2, q3 = q
        # Shoulder pitch q2 must be in [0, pi] (positive elevation above ground)
        if q2 < -0.05 or q2 > math.pi + 0.05:
            return False
        return True

    def compute_ik(
        self, 
        x: float, 
        y: float, 
        z: float, 
        elbow_up: Optional[bool] = None,
        current_q: Optional[Tuple[float, float, float]] = None
    ) -> Optional[Tuple[float, float, float]]:
        """
        Computes analytical inverse kinematics for target (x, y, z).

        Args:
            x, y, z: Target Cartesian position in world frame (meters).
            elbow_up: If True, selects elbow-up solution; if False, elbow-down.
                     If None, automatically picks the solution closest to current_q
                     (or defaults to elbow-up if current_q is None).
            current_q: Optional reference joint configuration (q1, q2, q3) in radians
                       used to select the nearest continuous branch solution.

        Returns:
            Tuple (q1, q2, q3) in radians, or None if target is unreachable.
        """
        if not self.is_reachable(x, y, z):
            return None

        if elbow_up is None:
            sol_up = self._compute_ik_branch(x, y, z, elbow_up=True, current_q=current_q)
            sol_down = self._compute_ik_branch(x, y, z, elbow_up=False, current_q=current_q)

            if sol_up is not None and not self.is_valid_solution(sol_up):
                sol_up = None
            if sol_down is not None and not self.is_valid_solution(sol_down):
                sol_down = None

            if sol_up is None:
                return sol_down
            if sol_down is None:
                return sol_up

            if current_q is not None:
                def dist(sol):
                    d0 = (sol[0] - current_q[0] + math.pi) % (2.0 * math.pi) - math.pi
                    d1 = (sol[1] - current_q[1] + math.pi) % (2.0 * math.pi) - math.pi
                    d2 = (sol[2] - current_q[2] + math.pi) % (2.0 * math.pi) - math.pi
                    return d0**2 + d1**2 + d2**2

                return sol_up if dist(sol_up) <= dist(sol_down) else sol_down
            else:
                return sol_up
        else:
            sol = self._compute_ik_branch(x, y, z, elbow_up=elbow_up, current_q=current_q)
            return sol if (sol is not None and self.is_valid_solution(sol)) else None

    def _compute_ik_branch(
        self, 
        x: float, 
        y: float, 
        z: float, 
        elbow_up: bool,
        current_q: Optional[Tuple[float, float, float]] = None
    ) -> Optional[Tuple[float, float, float]]:
        r_xy = math.sqrt(x**2 + y**2)
        if r_xy < 1e-6 and current_q is not None:
            q1 = current_q[0]
        else:
            q1 = math.atan2(y, x)

        # Because the gripper is perfectly horizontal, we subtract its offset to find the wrist target
        r_wrist = r_xy - self.gripper_dx
        if r_wrist < 0:
            r_wrist = -r_wrist
            q1 = (q1 + math.pi + math.pi) % (2.0 * math.pi) - math.pi

        z_wrist = z - self.gripper_dz

        z_prime = z_wrist - self.base_height

        L1 = self.L1
        L2 = self.L2

        cos_delta = (r_wrist**2 + z_prime**2 - L1**2 - L2**2) / (2.0 * L1 * L2)
        cos_delta = max(-1.0, min(1.0, cos_delta))
        
        sin_delta_mag = math.sqrt(1.0 - cos_delta**2)
        delta_theta = math.atan2(-sin_delta_mag if elbow_up else sin_delta_mag, cos_delta)

        A = L1 + L2 * cos_delta
        B = L2 * math.sin(delta_theta)

        num_theta2 = A * z_prime - B * r_wrist
        den_theta2 = A * r_wrist + B * z_prime
        
        theta2 = math.atan2(num_theta2, den_theta2)
        q2 = theta2 + math.pi / 2.0

        # Relative elbow bend q3
        q3 = -delta_theta

        # Normalize angles appropriately for T^3 Community Arm:
        # q1 (base yaw): [-pi, pi]
        # q2 (shoulder): [0, pi]
        # q3 (elbow): [0, 2*pi]
        q1 = (q1 + math.pi) % (2.0 * math.pi) - math.pi
        q2 = (q2 + math.pi) % (2.0 * math.pi) - math.pi
        q3 = (q3 + math.pi) % (2.0 * math.pi) - math.pi

        return (q1, q2, q3)


