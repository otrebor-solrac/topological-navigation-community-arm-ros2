import numpy as np
from typing import List, Tuple
from ..core.interfaces import BaseCollider, BaseKinematics

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
    def __init__(self, link_radius: float = 0.04, interpolation_points: int = 4):
        """
        Initialize the foam collider.

        Args:
            link_radius: The radius of the robot's links.
            interpolation_points: The number of interpolation points to use for covering the links.
        """

        self.link_radius = link_radius
        self.interpolation_points = interpolation_points
        self.spherical_obstacles: List[TopologicalSphere] = []
        
    def add_obstacle(self, center: tuple, radius: float):
        """
        Adds a spherical obstacle to the environment.
        """
        self.spherical_obstacles.append(TopologicalSphere(np.array(center), radius))

    def _generate_sphere_covering(self, joint_positions: List[np.ndarray]) -> List[TopologicalSphere]:
        """
        Generates a set of spheres that cover the robot's links.
        """
        robot_spheres = []
        for i in range(len(joint_positions) - 1):
            p_start = joint_positions[i]
            p_end = joint_positions[i+1]
            
            for j in range(self.interpolation_points + 1):
                t = j / float(self.interpolation_points)
                center_inter = p_start + t * (p_end - p_start)
                robot_spheres.append(TopologicalSphere(center_inter, self.link_radius))
                
        return robot_spheres

    def is_state_valid(self, q: tuple, kinematics: BaseKinematics) -> bool:
        """
        Determines if state q is safe (C_free) using the injected kinematic model.
        """
        positions = kinematics.compute_forward_kinematics(q)
        robot_balls = self._generate_sphere_covering(positions)
        
        for ball_r in robot_balls:
            for obs in self.spherical_obstacles:
                dist_l2 = np.linalg.norm(ball_r.center - obs.center)
                if dist_l2 <= (ball_r.radius + obs.radius):
                    # Collision detected (C_obs)
                    return False 
        
        # Safe (C_free)            
        return True 
