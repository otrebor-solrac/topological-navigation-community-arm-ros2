import heapq
import numpy as np
from typing import List, Callable, Tuple, Dict
from .topological_math import TorusTopology

class AStarPlanner:
    """
    Algoritmo A* determinista operando sobre el Toro T^n. 
    Usa la métrica L1 o L2 con "wrap-around" para guiar la búsqueda sin saltos topológicos.
    """
    
    def __init__(self, heuristic_type: str = 'L1'):
        if heuristic_type == 'L1':
            self.heuristic = TorusTopology.heuristic_L1
        elif heuristic_type == 'L2':
            self.heuristic = TorusTopology.heuristic_L2
        else:
            raise ValueError("Métrica no soportada. Usar 'L1' o 'L2'.")
            
    def plan(self, start_q: tuple, goal_q: tuple, is_valid_fn: Callable, get_neighbors_fn: Callable) -> List[tuple]:
        """
        Busca el camino más corto esquivando C_obs.
        Trabaja asumiendo que las coordenadas articulares vienen discretizadas en 'tuplas'
        para poder ser usadas como llaves (keys) en diccionarios.
        
        - is_valid_fn(q): Verifica si q está en C_free.
        - get_neighbors_fn(q): Retorna los vecinos continuos en el Toro.
        """
        
        # Cola de prioridad (Open Set) almacena: (f_score, config_q)
        open_set = []
        heapq.heappush(open_set, (0.0, start_q))
        
        # Diccionarios de rastreo
        came_from: Dict[tuple, tuple] = {}
        
        # Costo real g(n) desde el inicio
        g_score = {start_q: 0.0}
        
        # Costo estimado f(n) = g(n) + h(n)
        goal_array = np.array(goal_q)
        start_array = np.array(start_q)
        f_score = {start_q: self.heuristic(start_array, goal_array)}
        
        # Para evitar expansiones redundantes
        closed_set = set()

        while open_set:
            # Extraemos el nodo con menor f(n)
            current_f, current_q = heapq.heappop(open_set)
            
            # Condición de salida: Hemos llegado al objetivo con cierta tolerancia
            # Como trabajamos en un Voxel Grid discreto, si la tupla es idéntica, llegamos.
            if current_q == goal_q:
                return self._reconstruct_path(came_from, current_q)
                
            closed_set.add(current_q)
            current_array = np.array(current_q)
            
            # Expandir frontera de vecindad
            for neighbor_q in get_neighbors_fn(current_q):
                if neighbor_q in closed_set:
                    continue
                    
                # Evaluar si choca matemáticamente con C_obs
                if not is_valid_fn(neighbor_q):
                    continue
                
                neighbor_array = np.array(neighbor_q)
                
                # Para adyacencias en el Grid, la distancia de paso g(n) 
                # suele ser la distancia euclidiana L2 del paso
                step_cost = TorusTopology.heuristic_L2(current_array, neighbor_array)
                tentative_g_score = g_score[current_q] + step_cost
                
                if tentative_g_score < g_score.get(neighbor_q, float('inf')):
                    # Este camino al vecino es mejor (o es la primera vez que lo vemos)
                    came_from[neighbor_q] = current_q
                    g_score[neighbor_q] = tentative_g_score
                    
                    # f(n) = g(n) + h(n)
                    f_score[neighbor_q] = tentative_g_score + self.heuristic(neighbor_array, goal_array)
                    
                    # Añadir al Open Set si no estaba
                    # (Si ya estaba, la tupla se duplicará pero con mejor costo, 
                    # el set cerrado prevendrá que se expanda mal).
                    heapq.heappush(open_set, (f_score[neighbor_q], neighbor_q))
                    
        # Retorna lista vacía si no hay solución matemática posible
        return []

    def _reconstruct_path(self, came_from: Dict[tuple, tuple], current: tuple) -> List[tuple]:
        """ Reconstruye el linaje óptimo desde el objetivo hacia el inicio. """
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path
