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

        if self.urdf_parser is not None:
            # 1. Self-Collision Detection
            if self.urdf_parser.check_self_collision(q):
                return False
                
            # 2. External Collision Detection
            obstacles_tuples = [(obs.center, obs.radius) for obs in self.spherical_obstacles]
            if self.urdf_parser.check_obstacle_collision(q, obstacles_tuples):
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
