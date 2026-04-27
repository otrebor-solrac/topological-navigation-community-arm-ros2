from ..core.interfaces import BasePlanner, BaseKinematics, BaseCollider
from .a_star import AStarPlanner
from .rrt import RRTPlanner

class PlannerFactory:
    """
    Factory to instantiate the planner required by ROS 2.
    """
    
    @staticmethod
    def create_planner(
        planner_type: str, 
        space, 
        collider: BaseCollider, 
        kinematics: BaseKinematics, 
        **kwargs
    ) -> BasePlanner:
        """
        Creates a planner instance based on the specified type.

        Args:
            planner_type: Type of planner to create (e.g., "astar", "rrt").
            space: The configuration space (e.g., GridDiscretizer).
            collider: The collision checker (e.g., FoamCollider).
            kinematics: The forward kinematics model (e.g., CommunityArmKinematics).
            **kwargs: Additional arguments for the planner.

        Returns:
            An instance of the specified planner.

        Raises:
            ValueError: If the specified planner type is not found.
        """
        planner_type = planner_type.lower()
        
        # A* Planner
        if planner_type == "astar":
            return AStarPlanner(
                space, 
                collider, 
                kinematics, 
                heuristic_type=kwargs.get('heuristic_type', 'L1')
            )
        
        # RRT Planner
        elif planner_type == "rrt":
            return RRTPlanner(
                space, 
                collider, 
                kinematics,
                max_samples=kwargs.get('max_samples', 1000),
                step_size=kwargs.get('step_size', 0.1)
            )
        
        # Unknown Planner
        else:
            raise ValueError(f"Unknown planner: {planner_type}")
