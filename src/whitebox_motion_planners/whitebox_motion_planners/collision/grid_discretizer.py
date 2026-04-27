import math
from typing import List
from ..spaces.topological_math import TorusTopology

class GridDiscretizer:
    """
    Topological grid on T^n using integer indexing to avoid floating point errors.
    """

    def __init__(self, step_size_deg: float = 5.0, num_dof: int = 4):
        """
        Initialize the grid discretizer.

        Args:
            step_size_deg: The size of each grid step in degrees.
            num_dof: The number of degrees of freedom.
        """

        self.step_rad = math.radians(step_size_deg)
        self.num_dof = num_dof

    def discretize(self, q_continuous: tuple) -> tuple:
        """ 
        Maps continuous angles to integer grid indices. 

        Args:
            q_continuous: Tuple of continuous angles in radians.

        Returns:
            Tuple of integer grid indices.
        """

        indices = []
        for ang_rad in q_continuous:
            norm_ang = TorusTopology.normalize_angle(ang_rad)
            idx = int(round(norm_ang / self.step_rad))
            indices.append(idx)

        return tuple(indices)

    def to_radians(self, q_indices: tuple) -> tuple:
        """
        Maps integer grid indices back to continuous radians.

        Args:
            q_indices: Tuple of integer grid indices.

        Returns:
            Tuple of continuous angles in radians.
        """

        angles = []
        for idx in q_indices:
            ang = idx * self.step_rad
            angles.append(TorusTopology.normalize_angle(ang))

        return tuple(angles)

    def get_neighbors(self, q_indices: tuple) -> List[tuple]:
        """
        Returns adjacent integer indices in the grid.

        Args:
            q_indices: Tuple of integer grid indices.

        Returns:
            List of tuples of integer grid indices.
        """
        
        neighbors = []
        # Calculate number of steps in a full 2pi circle
        steps_per_circle = int(round(2 * math.pi / self.step_rad))
        
        for i in range(self.num_dof):
            for direction in [-1, 1]:
                neighbor = list(q_indices)
                neighbor[i] = (neighbor[i] + direction) % steps_per_circle
                neighbors.append(tuple(neighbor))
                
        return neighbors
