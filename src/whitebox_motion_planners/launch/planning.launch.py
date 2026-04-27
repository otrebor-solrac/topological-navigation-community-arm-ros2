import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for the White-Box Planner.
    
    This launch file will:
    1. Launch the robot and RViz.
    2. Launch the White-Box Planner node.
    3. Load the planner parameters from config/planner_params.yaml.
    
    """

    # 1. Get the launch directory of the robot
    robot_launch_dir = os.path.join(
        get_package_share_directory('community_robot_arm'), 'launch')
    
    # 2. Get the config file path
    config = os.path.join(
        get_package_share_directory('whitebox_motion_planners'),
        'config',
        'planner_params.yaml'
    )

    # 3. Create the planner node
    planner_node = Node(
        package='whitebox_motion_planners',
        executable='planificador',
        name='whitebox_planner',
        output='screen',
        parameters=[config]
    )
    
    # 4. Final Orchestration
    return LaunchDescription([
        # 4.1 Robot Layer: Reuse the existing display launch for RViz and TF.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_launch_dir, 'display.launch.py')
            ),
            # Force the use of the spherized URDF model for thesis compatibility
            launch_arguments={'spherized': 'true'}.items()
        ),
        
        # 4.2 Planning Layer: Launch the White-Box Planner agent with YAML params.
        planner_node
    ])
