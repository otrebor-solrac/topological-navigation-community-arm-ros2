from abc import ABC, abstractmethod
from typing import List
import numpy as np

class BaseKinematics(ABC):
    """
    Contract that every robotic arm in the laboratory must fulfill.
    """
    
    @abstractmethod
    def get_dof(self) -> int:
        """
        Returns the degrees of freedom (n in T^n).
        """
        pass

    @abstractmethod
    def compute_forward_kinematics(self, q: tuple) -> List[np.ndarray]:
        """
        Calculates the positions of the links in R^3 given a state in T^n.
        """
        pass

    def compute_jacobian(self, q: tuple) -> np.ndarray:
        """
        Computes the geometric Jacobian matrix J(q) for the end-effector.
        """
        raise NotImplementedError("compute_jacobian is not implemented for this robot kinematics.")

    def compute_manipulability(self, q: tuple) -> float:
        """
        Computes the Yoshikawa measure of manipulability.
        """
        raise NotImplementedError("compute_manipulability is not implemented for this robot kinematics.")

class BaseCollider(ABC):
    """
    Contract for any collision detection engine (FOAM, AABB, etc).
    """
    
    @abstractmethod
    def is_state_valid(self, q: tuple, kinematics: BaseKinematics) -> bool:
        """
        Determines if a configuration q belongs to C_free or C_obs.
        """
        pass

class BasePlanner(ABC):
    """
    Master contract for A*, RRT, PRM, etc.
    """
    
    def __init__(self, space, collider: BaseCollider, kinematics: BaseKinematics):
        """
        Dependency injection.
        
        Args:
            space: Object that defines the topological space.
            collider: Object that defines the collisions.
            kinematics: Object that defines the robot kinematics.
        """
        self.space = space
        self.collider = collider
        self.kinematics = kinematics

    @abstractmethod
    def plan(self, start_q: tuple, goal_q: tuple) -> List[tuple]:
        """
        Searches for a collision-free path between start and goal.
        """
        pass
