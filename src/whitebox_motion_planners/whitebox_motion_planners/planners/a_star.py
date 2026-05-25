import heapq
import numpy as np
from typing import List, Tuple, Dict
from ..core.interfaces import BasePlanner
from ..spaces.metrics import Metrics

class AStarPlanner(BasePlanner):
    """
    Deterministic A* algorithm operating on the Torus T^n.
    Inherits from BasePlanner.
    """
    
    def __init__(self, space, collider, kinematics, heuristic_type: str = 'L1'):
        super().__init__(space, collider, kinematics)
        if heuristic_type == 'L1':
            self.heuristic = Metrics.heuristic_L1
        elif heuristic_type == 'L2':
            self.heuristic = Metrics.heuristic_L2
        else:
            raise ValueError("Unsupported metric. Use 'L1' or 'L2'.")
            
    def plan(self, start_q: tuple, goal_q: tuple) -> List[tuple]:
        """
        Plans a path in the grid.
        start_q and goal_q are expected to be integer indices tuples.
        Returns a list of radian tuples.
        """
        open_set = []
        heapq.heappush(open_set, (0.0, start_q))
        
        came_from: Dict[tuple, tuple] = {}
        g_score = {start_q: 0.0}
        
        # Pre-convert goal to radians for heuristic calculations
        goal_rad = self.space.get_radians(goal_q)
        goal_array = np.array(goal_rad)
        
        start_rad = self.space.get_radians(start_q)
        start_array = np.array(start_rad)
        
        f_score = {start_q: self.heuristic(start_array, goal_array)}
        closed_set = set()

        while open_set:
            current_f, current_q = heapq.heappop(open_set)
            
            if current_q == goal_q:
                # Return the path converted to radians
                indices_path = self._reconstruct_path(came_from, current_q)
                return [self.space.get_radians(idx) for idx in indices_path]
                
            closed_set.add(current_q)
            current_rad = self.space.get_radians(current_q)
            current_array = np.array(current_rad)
            
            for neighbor_q in self.space.get_neighbors(current_q):
                if neighbor_q in closed_set:
                    continue
                    
                neighbor_rad = self.space.get_radians(neighbor_q)
                neighbor_array = np.array(neighbor_rad)

                if not self.collider.is_state_valid(neighbor_rad, self.kinematics):
                    continue
                
                step_cost = Metrics.heuristic_L2(current_array, neighbor_array)
                tentative_g_score = g_score[current_q] + step_cost
                
                if tentative_g_score < g_score.get(neighbor_q, float('inf')):
                    came_from[neighbor_q] = current_q
                    g_score[neighbor_q] = tentative_g_score
                    f_score[neighbor_q] = tentative_g_score + self.heuristic(neighbor_array, goal_array)
                    heapq.heappush(open_set, (f_score[neighbor_q], neighbor_q))
                    
        return []

    def _reconstruct_path(self, came_from: Dict[tuple, tuple], current: tuple) -> List[tuple]:
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path
