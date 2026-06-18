from .community_arm import CommunityArmKinematics
from .open_manipulator import OpenManipulatorKinematics

def get_kinematics(robot_type: str, use_horizontal_constraint: bool = False, link_lengths: dict = None):
    """
    Kinematics Factory to instantiate robot models dynamically 
    based on parameter settings.

    :param robot_type: The type of robot to instantiate.
    :param use_horizontal_constraint: Whether to use the horizontal 
                                        constraint for the end-effector.
    :param link_lengths: Dictionary containing physical measurements of the arm links.
    :return: Instance of the appropriate kinematics class.
    :raises: ValueError if the robot type is unknown.
    """

    if robot_type == "community_arm":
        return CommunityArmKinematics(
            use_horizontal_constraint=use_horizontal_constraint,
            link_lengths=link_lengths
        )
    
    elif robot_type == "open_manipulator":
        return OpenManipulatorKinematics()
    
    else:
        raise ValueError(f"Unknown robot type: {robot_type}")
