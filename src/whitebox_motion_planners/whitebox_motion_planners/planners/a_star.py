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
        self.heuristic_type = heuristic_type
        if heuristic_type == 'L1':
            self.heuristic = Metrics.heuristic_L1
        elif heuristic_type == 'L2':
            self.heuristic = Metrics.heuristic_L2
        else:
            raise ValueError("Unsupported metric. Use 'L1' or 'L2'.")
            
    def plan(self, start_q: tuple, goal_q: tuple) -> List[tuple]:
        """
        Plans a path in the grid.
        """
        return self._plan_internal(start_q, goal_q)

    def _plan_internal(self, start_q: tuple, goal_q: tuple) -> List[tuple]:
        """
        Plans a path in the grid.
        :param start_q: Starting configuration as integer indices tuple.
        :param goal_q: Goal configuration as integer indices tuple.
        :return: A list of radian tuples representing the path.
        """
        # Initialize the priority queue (open set) as a min-heap to keep track of nodes to be evaluated
        open_set = []
        # Insert the start node with an initial priority/f_score of 0.0 to bootstrap the loop
        heapq.heappush(open_set, (0.0, start_q))
        
        # Dictionary to store the navigation history (key: current node, value: parent node) for path reconstruction
        came_from: Dict[tuple, tuple] = {}
        # Map to store the exact cost of the shortest path from the start node to any visited node
        g_score = {start_q: 0.0}
        
        # Pre-convert goal coordinates to radians and wrap into a NumPy array for fast metric calculations
        goal_rad = self.space.get_radians(goal_q)
        goal_array = np.array(goal_rad)

        # Pre-convert start coordinates to radians and wrap into a NumPy array for consistency        
        start_rad = self.space.get_radians(start_q)
        start_array = np.array(start_rad)
        
        # Map to store the total estimated cost (g_score + heuristic) from start to goal through each node
        f_score = {start_q: self.heuristic(start_array, goal_array)}

        # Set to store already fully evaluated nodes to prevent reprocessing and infinite loops
        closed_set = set()

        while open_set:
            # Get the node with the lowest f_score
            current_f, current_q = heapq.heappop(open_set)
            
            if current_q == goal_q:
                # Return the path converted to radians
                indices_path = self._reconstruct_path(came_from, current_q)
                return [self.space.get_radians(idx) for idx in indices_path]
                
            closed_set.add(current_q)
            current_rad = self.space.get_radians(current_q)
            current_array = np.array(current_rad)
            
            for neighbor_q in self.space.get_neighbors(current_q, metric_type=self.heuristic_type):
                if neighbor_q in closed_set:
                    continue
                    
                # Si tenemos la caché del C-space discretizada en un set, hacemos búsqueda O(1) con los índices enteros.
                # Esto evita convertir a radianes, instanciar arrays de numpy y volver a discretizar en el colisionador.
                if self.collider.forbidden_set is not None:
                    if neighbor_q in self.collider.forbidden_set:
                        continue
                    neighbor_rad = self.space.get_radians(neighbor_q)
                else:
                    neighbor_rad = self.space.get_radians(neighbor_q)
                    if not self.collider.is_state_valid(neighbor_rad, self.kinematics):
                        continue
                
                neighbor_array = np.array(neighbor_rad)
                step_cost = self.heuristic(current_array, neighbor_array)
                tentative_g_score = g_score[current_q] + step_cost
                
                # Check if this new path to the neighbor is better (cheaper) than any previously found path
                if tentative_g_score < g_score.get(neighbor_q, float('inf')):
                    
                    # Record the current node as the best parent/predecessor for this neighbor
                    came_from[neighbor_q] = current_q
                    
                    # Update the neighbor's exact cost from the start node with the new lower score
                    g_score[neighbor_q] = tentative_g_score
                    
                    # Calculate and update the total estimated cost (f = g + h) for this neighbor
                    f_score[neighbor_q] = tentative_g_score + self.heuristic(neighbor_array, goal_array)
                    
                    # Push the neighbor into the priority queue with its new f_score for future evaluation
                    heapq.heappush(open_set, (f_score[neighbor_q], neighbor_q))
                    
        return []

    def _reconstruct_path(self, came_from: Dict[tuple, tuple], current: tuple) -> List[tuple]:
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path
