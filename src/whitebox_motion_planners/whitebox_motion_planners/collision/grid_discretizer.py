import math
import itertools
from typing import List
from ..spaces.topological_math import TorusTopology

class GridDiscretizer:
    """
    Topological grid on T^n using integer indexing to avoid floating point errors.
    """

    def __init__(self, step_size_deg: float = 5.0, num_dof: int = 4):
        """
        Initialize the grid discretizer.

        :param step_size_deg: The size of each grid step in degrees.
        :param num_dof: The number of degrees of freedom.
        """

        self.step_rad = math.radians(step_size_deg)
        self.num_dof = num_dof
        self.steps_per_circle = int(round(2 * math.pi / self.step_rad))

        # Precompute neighbor offsets (avoids regenerating itertools.product per call)
        self._l1_offsets = []
        for i in range(self.num_dof):
            for d in [-1, 1]:
                offset = [0] * self.num_dof
                offset[i] = d
                self._l1_offsets.append(tuple(offset))

        self._l2_offsets = [
            o for o in itertools.product([-1, 0, 1], repeat=self.num_dof)
            if any(d != 0 for d in o)
        ]

        # Precompute radian lookup table for fast index→radian conversion
        self._radian_table = [
            TorusTopology.normalize_angle(i * self.step_rad)
            for i in range(self.steps_per_circle)
        ]

    def discretize(self, q_continuous: tuple) -> tuple:
        """ 
        Maps continuous angles to integer grid indices. 
        E.g (12,6,0) -> (180,90,0)deg with a step of 15 deg

        :param q_continuous: Tuple of continuous angles in radians.
        :return: Tuple of integer grid indices.
        """

        indices = []
        for ang_rad in q_continuous:
            # Wrap to [-pi, pi]
            norm_ang = TorusTopology.normalize_angle(ang_rad)
            idx = int(round(norm_ang / self.step_rad))
            indices.append(idx % self.steps_per_circle)

        return tuple(indices)

    def get_radians(self, q_indices: tuple) -> tuple:
        """
        Maps integer grid indices back to continuous radians.
        Uses precomputed lookup table for O(1) per dimension.

        :param q_indices: Tuple of integer grid indices.
        :return: Tuple of continuous angles in radians.
        """
        return tuple(self._radian_table[idx % self.steps_per_circle] for idx in q_indices)

    def get_neighbors(self, q_indices: tuple, metric_type: str = 'L1') -> List[tuple]:
        """
        Returns adjacent integer indices in the grid.
        Uses precomputed offset tables for fast neighbor generation.

        :param q_indices: Tuple of integer grid indices.
        :param metric_type: The metric type to use ('L1' or 'L2').
        :return: List of tuples of integer grid indices.
        """
        n = self.steps_per_circle
        if metric_type == 'L1':
            return [
                tuple((q_indices[i] + offset[i]) % n for i in range(self.num_dof))
                for offset in self._l1_offsets
            ]
        elif metric_type == 'L2':
            return [
                tuple((q_indices[i] + offset[i]) % n for i in range(self.num_dof))
                for offset in self._l2_offsets
            ]
        else:
            raise ValueError(f"Unsupported metric_type: {metric_type}. Must be 'L1' or 'L2'.")

    def get_all_states(self) -> List[tuple]:
        """
        Generates all possible discrete states in the T^n grid.
        WARNING: Grows exponentially with num_dof.
        
        :return: List of tuples of all possible integer grid indices.
        """
        ranges = [range(self.steps_per_circle) for _ in range(self.num_dof)]
        return list(itertools.product(*ranges))
