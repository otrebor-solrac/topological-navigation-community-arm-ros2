import numpy as np
from typing import List, Tuple
from ..core.interfaces import BaseCollider, BaseKinematics
from .urdf_collision_parser import UrdfCollisionParser

class TopologicalSphere:
    """ 
    Represents an open ball B(p, epsilon) in R^3.
    """
    def __init__(self, center: np.ndarray, radius: float):
        self.center = center
        self.radius = radius

class FoamCollider(BaseCollider):
    """
    Implements C_free evaluation using open ball coverings 
    (Fast Open Approximation of Manifolds).
    """
    def __init__(
        self, 
        interpolation_points: int = 4, 
        urdf_path: str = None, 
        sphere_thinning_dist: float = 0.015
    ):
        """
        Initialize the foam collider.

        :param interpolation_points: The number of interpolation points to use for covering the links.
        :param urdf_path: Optional path to the spherized URDF file for the robot.
        :param sphere_thinning_dist: The distance threshold in meters for URDF sphere thinning.
        """
        self.interpolation_points = interpolation_points
        self.spherical_obstacles: List[TopologicalSphere] = []
        
        self.urdf_parser = None
        if urdf_path is not None:
            self.urdf_parser = UrdfCollisionParser(urdf_path, min_dist=sphere_thinning_dist)
            
        self.forbidden_set = None
        self.grid_discretizer = None
        self.singularity_threshold = 0.0

        # Default joint offset and direction configurations (relative to world axes)
        self.offset_base_yaw = 0.0
        self.offset_shoulder_pitch = 0.0
        self.offset_elbow_pitch = 0.0
        self.dir_base_yaw = 1.0
        self.dir_shoulder_pitch = 1.0
        self.dir_elbow_pitch = 1.0

    def set_cspace_cache(self, forbidden_set: set, grid_discretizer):
        """
        Injects a precomputed set of forbidden voxels (discrete coordinates)
        to enable O(1) set-based collision checks during planning.
        """
        self.forbidden_set = forbidden_set
        self.grid_discretizer = grid_discretizer
        
    def add_obstacle(self, center: tuple, radius: float):
        """
        Adds a spherical obstacle to the environment.
        """
        self.spherical_obstacles.append(TopologicalSphere(np.array(center), radius))

    def _generate_sphere_covering(self, joint_positions: List[np.ndarray]) -> List[List[TopologicalSphere]]:
        """
        Generates a list of sphere lists covering the robot's links, grouped by link.
        """
        link_spheres = []
        for i in range(len(joint_positions) - 1):
            p_start = joint_positions[i]
            p_end = joint_positions[i+1]
            
            spheres_for_link = []
            for j in range(self.interpolation_points + 1):
                t = j / float(self.interpolation_points)
                center_inter = p_start + t * (p_end - p_start)
                spheres_for_link.append(TopologicalSphere(center_inter, self.link_radius))
                
            link_spheres.append(spheres_for_link)
            
        return link_spheres

    def set_joint_transforms(
        self,
        offset_base_yaw: float = 0.559643,
        offset_shoulder_pitch: float = 1.57079632679,
        offset_elbow_pitch: float = 0.0,
        dir_base_yaw: float = -1.0,
        dir_shoulder_pitch: float = -1.0,
        dir_elbow_pitch: float = 1.0
    ):
        """
        Sets the joint offsets and directions dynamically.
        """
        self.offset_base_yaw = offset_base_yaw
        self.offset_shoulder_pitch = offset_shoulder_pitch
        self.offset_elbow_pitch = offset_elbow_pitch
        self.dir_base_yaw = dir_base_yaw
        self.dir_shoulder_pitch = dir_shoulder_pitch
        self.dir_elbow_pitch = dir_elbow_pitch

        if self.urdf_parser is not None:
            self.urdf_parser.update_home_pose(
                self.offset_base_yaw,
                self.offset_shoulder_pitch,
                self.offset_elbow_pitch
            )

    def compute_manipulability(self, q: tuple, epsilon: float = 1e-5) -> float:
        """
        Computes the manipulability index using the numerical Jacobian of the URDF parser
        if available, otherwise raises NotImplementedError.
        """
        if self.urdf_parser is not None:
            # Helper to get EE position from q_world
            def get_ee(q_w):
                yaw_w, pitch1_w, pitch2_w = q_w[:3] if len(q_w) >= 3 else (q_w[0], q_w[1], 0.0)
                base_yaw = self.offset_base_yaw + self.dir_base_yaw * yaw_w
                shoulder_pitch = self.offset_shoulder_pitch + self.dir_shoulder_pitch * pitch1_w
                # Coupled: elbow_pitch = -shoulder_pitch - q3_relative
                q3_relative = self.offset_elbow_pitch + self.dir_elbow_pitch * pitch2_w
                elbow_pitch = -shoulder_pitch - q3_relative
                q_urdf = (base_yaw, shoulder_pitch, elbow_pitch)
                return self.urdf_parser.get_end_effector_position(q_urdf)

            J = []
            dof = len(q)
            for i in range(dof):
                q_plus = list(q)
                q_plus[i] += epsilon
                p_plus = get_ee(q_plus)
                
                q_minus = list(q)
                q_minus[i] -= epsilon
                p_minus = get_ee(q_minus)
                
                if p_plus is None or p_minus is None:
                    return 0.0
                    
                col = (p_plus - p_minus) / (2.0 * epsilon)
                J.append(col)
                
            J = np.column_stack(J)
            m, n = J.shape
            if m <= n:
                w = float(np.sqrt(max(0.0, np.linalg.det(J @ J.T))))
            else:
                w = float(np.sqrt(max(0.0, np.linalg.det(J.T @ J))))
            return w
        else:
            raise NotImplementedError("URDF parser not initialized")

    def is_state_valid(self, q: tuple, kinematics: BaseKinematics) -> bool:
        """
        Determines if state q is safe (C_free) using the injected kinematic model.
        Checks both self-collisions and external environment obstacles.
        """
        # 0. Fast cache lookup (O(1) set search)
        if self.forbidden_set is not None and self.grid_discretizer is not None:
            q_discrete = self.grid_discretizer.discretize(q)
            if q_discrete in self.forbidden_set:
                return False
            return True

        # 1. Singularity / Manipulability check
        if getattr(self, 'singularity_threshold', 0.0) > 0.0:
            try:
                if self.urdf_parser is not None:
                    w = self.compute_manipulability(q)
                else:
                    w = kinematics.compute_manipulability(q)
                if w < self.singularity_threshold:
                    return False
            except (NotImplementedError, AttributeError):
                pass

        if self.urdf_parser is not None:
            # Convert q from World coordinates to URDF coordinates using configured parameters
            yaw_w, pitch1_w, pitch2_w = q[:3] if len(q) >= 3 else (q[0], q[1], 0.0)
            base_yaw = self.offset_base_yaw + self.dir_base_yaw * yaw_w
            shoulder_pitch = self.offset_shoulder_pitch + self.dir_shoulder_pitch * pitch1_w
            # Coupled: elbow_pitch = -shoulder_pitch - q3_relative
            q3_relative = self.offset_elbow_pitch + self.dir_elbow_pitch * pitch2_w
            elbow_pitch = -shoulder_pitch - q3_relative
            q_urdf = (base_yaw, shoulder_pitch, elbow_pitch)

            # 1. Self-Collision Detection
            if self.urdf_parser.check_self_collision(q_urdf):
                return False
                
            # 2. External Collision Detection
            obstacles_tuples = [(obs.center, obs.radius) for obs in self.spherical_obstacles]
            if self.urdf_parser.check_obstacle_collision(q_urdf, obstacles_tuples):
                return False
                
            return True
            
        # Fallback to interpolated capsule model
        positions = kinematics.compute_forward_kinematics(q)
        link_spheres = self._generate_sphere_covering(positions)
        
        # 1. Self-Collision Detection (Non-adjacent links: |i - j| >= 2)
        num_links = len(link_spheres)
        for i in range(num_links):
            for j in range(i + 2, num_links):
                for sphere_i in link_spheres[i]:
                    for sphere_j in link_spheres[j]:
                        dist_l2 = np.linalg.norm(sphere_i.center - sphere_j.center)
                        if dist_l2 <= (sphere_i.radius + sphere_j.radius):
                            # Self-collision detected (C_obs)
                            return False
        
        # 2. External Collision Detection (Against environment obstacles)
        for link in link_spheres:
            for ball_r in link:
                for obs in self.spherical_obstacles:
                    dist_l2 = np.linalg.norm(ball_r.center - obs.center)
                    if dist_l2 <= (ball_r.radius + obs.radius):
                        # Collision detected (C_obs)
                        return False 
        
        # Safe (C_free)            
        return True 
