class TopologicalError(Exception):
    """Base class for topological exceptions."""
    pass

class TargetInCollisionError(TopologicalError):
    """Exception raised when the goal is inside C_obs."""
    pass

class StartInCollisionError(TopologicalError):
    """Exception raised when the start is inside C_obs."""
    pass

class PathNotFoundError(TopologicalError):
    """Exception raised when no continuous path exists in C_free."""
    pass
