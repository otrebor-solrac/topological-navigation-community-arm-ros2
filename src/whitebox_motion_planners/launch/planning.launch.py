"""
Launch file for the White-Box Motion Planning Stack.

This launch file orchestrates the execution of the entire simulation and planning environment:
1. Robot Layer: Includes display.launch.py to launch RViz and joint state processing.
2. Obstacles Layer: Publishes the selected obstacle model and static TFs.
3. Planning Layer: Launches the main planning_node (planner) with YAML parameters.
4. Communication Layer: Starts the Rosbridge WebSocket server for the Web Dashboard.
5. Visualization Layer: Runs the C-Space voxelizer node to populate C-Space state validities.
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def get_default_obstacle_type():
    """
    Parse the default obstacle type from the planner_params.yaml file.
    """
    try:
        # TODO: What if whitebox_motion_planners is not the unique package?
        # TODO: Code repeated in launch_setup function
        pkg_share = get_package_share_directory('whitebox_motion_planners')
        config_path = os.path.join(pkg_share, 'config', 'planner_params.yaml')
        
        with open(config_path, 'r') as f:
            yaml_data = yaml.safe_load(f)
            global_params = yaml_data.get('/**', {}).get('ros__parameters', {})
            return global_params.get('obstacle_type', 'box_obstacle')
    
    except Exception as e:
        print(f"Warning: Could not parse default obstacle_type from YAML: {e}")
        return 'box_obstacle'

def launch_setup(context, *args, **kwargs):
    """
    Setup the launch environment dynamically evaluating configurations.
    """
    # 1. Get packages share directories
    # TODO: This path only works for community_robot_arm, we need to adjust for more robots.
    pkg_share_community_arm = get_package_share_directory('community_robot_arm')
    
    # TODO: What if whitebox_motion_planners is not the unique package?
    pkg_share_whitebox = get_package_share_directory('whitebox_motion_planners')

    # 2. Get the config file path
    config = os.path.join(pkg_share_whitebox, 'config', 'planner_params.yaml')

    # 3. Retrieve obstacle name from LaunchConfiguration 
    # (variable defined in terminal, e.g. ros2 launch ... obstacle_type:=box_obstacle)
    obstacle_type = LaunchConfiguration('obstacle_type').perform(context)
    obstacles_urdf_file = os.path.join(
        pkg_share_community_arm, 'urdf', 'spherized', 'obstacles', f"{obstacle_type}_spherized.urdf"
    )
    
    print(f"LOADING SPHERIZED OBSTACLES FROM: {obstacles_urdf_file}")

    if os.path.exists(obstacles_urdf_file):
        with open(obstacles_urdf_file, 'r') as infp:
            obstacles_desc = infp.read()
    else:
        print(f"Warning: Spherized Obstacles URDF file not found at: {obstacles_urdf_file}")
        obstacles_desc = ""

    # Obstacles robot state publisher
    obstacles_publisher = Node(
        name='obstacles_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': obstacles_desc}],
        remappings=[('/robot_description', '/obstacles_description')],
        condition=IfCondition(LaunchConfiguration('use_obstacles'))
    )

    # Static transform publisher linking root to world
    # Published unconditionally so that RViz2 retains the root->world TF frame
    # when obstacles are toggled/loaded dynamically later via the web dashboard.
    static_tf_publisher = Node(
        name='root_to_world_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'root', 'world']
    )

    # Main planner node, where all the computations are done
    planner_node = Node(
        name='whitebox_planner',
        package='whitebox_motion_planners',
        executable='planner',
        output='screen',
        parameters=[config, {
            'obstacles_urdf_path': obstacles_urdf_file,
            'obstacle_type': obstacle_type,
            'use_obstacles': LaunchConfiguration('use_obstacles')
        }]
    )

    # Rosbridge Server (for the web dashboard)
    # TODO: Should the port be a parameter in YAML file?
    rosbridge_node = Node(
        name='rosbridge_websocket',        
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}]
    )

    # C-Space voxelizer node
    voxelizer_node = Node(
        name='cspace_voxelizer',
        package='whitebox_motion_planners',
        executable='voxelizer',
        output='screen',
        parameters=[config, {
            'obstacles_urdf_path': obstacles_urdf_file,
            'obstacle_type': obstacle_type,
            'use_obstacles': LaunchConfiguration('use_obstacles')
        }]
    )

    # Include the robot display launch
    robot_launch_dir = os.path.join(pkg_share_community_arm, 'launch')
    # TODO: If new robot is used we need to create a folder and file with the 
    # same names. It is needed to document the behaviour.
    robot_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_launch_dir, 'display.launch.py')
        ),
        # Force the use of the spherized URDF model for collision / link display in RViz
        launch_arguments={'spherized': 'true'}.items()
    )

    return [
        robot_include,
        obstacles_publisher,
        static_tf_publisher,
        planner_node,
        rosbridge_node,
        voxelizer_node
    ]

def generate_launch_description():
    """
    Returns a LaunchDescription object containing all the nodes to be launched.
    """
    
    return LaunchDescription([
        # Declare launch argument to toggle obstacles
        # TODO: As we are using the GUI for obstacle selection,Is this chunk of code still needed?
        DeclareLaunchArgument(
            'use_obstacles',
            default_value='false',
            description='Whether to load and visualize environment obstacles'
        ),
        # Declare launch argument to choose which obstacles file to load
        DeclareLaunchArgument(
            'obstacle_type',
            default_value = get_default_obstacle_type(),
            description = 'Obstacle type to load (box_obstacle, narrow_passage, u_obstacle, toroidal_wall)'
        ),
        # OpaqueFunction executes the setup logic dynamically
        OpaqueFunction(function=launch_setup)
    ])
