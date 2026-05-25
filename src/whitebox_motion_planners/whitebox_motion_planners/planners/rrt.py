"""
Rapidly-exploring Random Tree (RRT) on the Toroidal Manifold T^n.

This is a sampling-based planner that explores the configuration space
by growing a tree from the start towards random samples. Unlike A*,
RRT does not require a grid discretization — it works in continuous
radian space and only uses the grid to snap the final path for
consistency with the animation system.

Mathematical Foundation:
    - Sampling uses the Haar measure on T^n (uniform on [0, 2π)^n)
    - Distance uses the geodesic metric on S^1 per dimension
    - Steering respects the circular topology (wraps around)
"""

import math
import numpy as np
from typing import List, Optional, Tuple
from ..core.interfaces import BasePlanner
from ..spaces.topological_math import TorusTopology
from ..spaces.metrics import Metrics


class RRTPlanner(BasePlanner):
    """
    Rapidly-exploring Random Tree algorithm on T^n.

    The tree grows from start_q towards random samples in configuration
    space. Each new sample is checked for collisions using FOAM spheres.
    When a node lands within `goal_tolerance` of the goal, the path is
    reconstructed and returned.

    Args:
        space: GridDiscretizer — used to snap final path to grid indices.
        collider: BaseCollider — collision checker (FOAM).
        kinematics: BaseKinematics — forward kinematics model.
        max_samples: Maximum number of random samples before giving up.
        step_size: Maximum extension distance per step (radians).
        goal_bias: Probability [0,1] of sampling the goal directly.
        goal_tolerance: Distance threshold to consider goal reached.
    """

    def __init__(self, space, collider, kinematics,
                 max_samples: int = 5000,
                 step_size: float = 0.15,
                 goal_bias: float = 0.10,
                 goal_tolerance: float = 0.2):
        super().__init__(space, collider, kinematics)
        self.max_samples = max_samples
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.goal_tolerance = goal_tolerance

        # Tree storage: list of nodes (radian tuples)
        # Parent map: node_index -> parent_index
        self.nodes: List[tuple] = []
        self.parent: dict = {}

    # ------------------------------------------------------------------
    # Public API (implements BasePlanner.plan)
    # ------------------------------------------------------------------

    def plan(self, start_q: tuple, goal_q: tuple) -> List[tuple]:
        """
        Plans a path from start_q to goal_q on T^n.

        Both start_q and goal_q are integer grid indices (from GridDiscretizer).
        Returns a list of radian tuples representing the path.
        """
        # Convert grid indices to continuous radians for RRT
        start_rad = self.space.get_radians(start_q)
        goal_rad = self.space.get_radians(goal_q)

        # Initialize tree with start node
        self.nodes = [start_rad]
        self.parent = {0: None}

        n_dof = len(start_rad)

        for i in range(self.max_samples):
            # 1. SAMPLE: Haar measure on T^n (or bias towards goal)
            q_rand = self._sample(goal_rad, n_dof)

            # 2. NEAREST: Find closest node in tree using toroidal metric
            nearest_idx = self._nearest(q_rand)
            q_nearest = self.nodes[nearest_idx]

            # 3. STEER: Extend towards sample by step_size
            q_new = self._steer(q_nearest, q_rand)

            # 4. COLLISION CHECK: Validate the new configuration
            if not self.collider.is_state_valid(q_new, self.kinematics):
                continue

            # 5. ADD to tree
            new_idx = len(self.nodes)
            self.nodes.append(q_new)
            self.parent[new_idx] = nearest_idx

            # 6. CHECK GOAL: Are we close enough?
            dist_to_goal = Metrics.heuristic_L2(
                np.array(q_new), np.array(goal_rad)
            )
            if dist_to_goal < self.goal_tolerance:
                # Add exact goal as final node
                goal_idx = len(self.nodes)
                self.nodes.append(goal_rad)
                self.parent[goal_idx] = new_idx

                # Reconstruct path in radians
                return self._reconstruct_path(goal_idx)

        # Failed to find a path
        return []

    # ------------------------------------------------------------------
    # Private Methods
    # ------------------------------------------------------------------

    def _sample(self, goal_rad: tuple, n_dof: int) -> tuple:
        """
        Samples a random configuration on T^n using the Haar measure.

        With probability `goal_bias`, returns the goal directly.
        Otherwise, samples uniformly from [0, 2π)^n.

        The Haar measure on T^n = S^1 × S^1 × ... × S^1 is simply
        the product of uniform measures on each circle factor.
        """
        if np.random.random() < self.goal_bias:
            return goal_rad

        # Uniform sampling on each S^1 factor
        return tuple(np.random.uniform(0, 2 * math.pi) for _ in range(n_dof))

    def _nearest(self, q_target: tuple) -> int:
        """
        Finds the index of the nearest node in the tree to q_target,
        using the toroidal L2 metric (geodesic distance on T^n).
        """
        target_arr = np.array(q_target)
        best_idx = 0
        best_dist = float('inf')

        for idx, node in enumerate(self.nodes):
            d = Metrics.heuristic_L2(np.array(node), target_arr)
            if d < best_dist:
                best_dist = d
                best_idx = idx

        return best_idx

    def _steer(self, q_from: tuple, q_to: tuple) -> tuple:
        """
        Extends from q_from towards q_to by at most `step_size` radians.

        The extension respects the circular topology of each S^1 factor:
        it always takes the shortest arc direction (clockwise or
        counter-clockwise) and wraps the result into [0, 2π).
        """
        result = []
        for theta_from, theta_to in zip(q_from, q_to):
            # Compute shortest signed arc on S^1
            diff = theta_to - theta_from
            # Wrap to [-π, π]
            diff = (diff + math.pi) % (2 * math.pi) - math.pi

            # Limit step size
            if abs(diff) > self.step_size:
                diff = self.step_size * (1.0 if diff > 0 else -1.0)

            new_angle = TorusTopology.normalize_angle(theta_from + diff)
            result.append(new_angle)

        return tuple(result)

    def _reconstruct_path(self, goal_idx: int) -> List[tuple]:
        """
        Walks the parent pointers from goal back to root,
        returning the path as a list of radian tuples.
        """
        path = []
        idx = goal_idx
        while idx is not None:
            path.append(self.nodes[idx])
            idx = self.parent[idx]
        path.reverse()
        return path
