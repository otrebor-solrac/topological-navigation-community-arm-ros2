"""
Launch file for the White-Box Motion Planning Stack.

This launch file orchestrates the execution of the entire simulation and planning environment:
1. Robot Layer: Includes display.launch.py to launch RViz and joint state processing.
2. Obstacles Layer: Publishes the box obstacle model and static TFs.
3. Planning Layer: Launches the main planning_node (planificador) with YAML parameters.
4. Communication Layer: Starts the Rosbridge WebSocket server for the Web Dashboard.
5. Visualization Layer: Runs the C-Space voxelizer node to populate C-Space state validities.
"""



import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
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

    # Load obstacles URDF content
    pkg_share_community_arm = get_package_share_directory('community_robot_arm')
    obstacles_urdf_file = os.path.join(
        pkg_share_community_arm, 'urdf', 'obstacles', 'box_obstacle_spherized.urdf'
    )
    
    if os.path.exists(obstacles_urdf_file):
        with open(obstacles_urdf_file, 'r') as infp:
            obstacles_desc = infp.read()
    else:
        print("Obstacles URDF file not added.")
        obstacles_desc = ""

    # Obstacles robot state publisher:
    # Publishes the obstacles URDF description model under the '/obstacles_description' topic.
    # Enables RViz to visualize the static box obstacle. Only run if 'use_obstacles' is true.
    obstacles_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='obstacles_state_publisher',
        output='screen',
        parameters=[{'robot_description': obstacles_desc}],
        remappings=[('/robot_description', '/obstacles_description')],
        condition=IfCondition(LaunchConfiguration('use_obstacles'))
    )

    # Static transform publisher linking root to world:
    # Joins the coordinate frames of the robot ('root') and the environment obstacles ('world') at the same origin.
    # This resolves transform (TF) errors in RViz and allows both the robot and obstacles to render in the same coordinate space.
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='root_to_world_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'root', 'world'],
        condition=IfCondition(LaunchConfiguration('use_obstacles'))
    )

    # Create the planner node:
    # Launches the main motion planning node (planning_node.py) which handles start/goal conversion,
    # collision checks, pathfinding, and publishing path trails. Parameters are loaded from config/planner_params.yaml.
    planner_node = Node(
        package='whitebox_motion_planners',
        executable='planificador',
        name='whitebox_planner',
        output='screen',
        parameters=[config]
    )

    # Rosbridge Server (for the Web Dashboard):
    # Launches a WebSocket server on port 9090 to enable bi-directional communications
    # between ROS 2 topics and the Web Dashboard front-end (Three.js).
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}]
    )

    # C-Space Voxelizer (for the Web Dashboard):
    # Launches the C-Space voxelizer node (cspace_publisher.py). It discretizes the joint space grid,
    # checks states against registered obstacle spheres, and publishes C-Space voxels for dashboard visualization.
    voxelizer_node = Node(
        package='whitebox_motion_planners',
        executable='voxelizer',
        name='cspace_voxelizer',
        output='screen',
        parameters=[config]
    )
    
    # Final Orchestration   
    return LaunchDescription([
        # Launch Arguments
        # Declare launch argument to toggle between the spherized collision 
        # model and the raw URDF model.
        DeclareLaunchArgument(
            'use_obstacles',
            default_value='true',
            description='Whether to load and visualize environment obstacles'
        ),

        # Robot Layer: Reuse the existing display launch for RViz and TF.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_launch_dir, 'display.launch.py')
            ),
            # Force the use of the spherized URDF model.
            launch_arguments={'spherized': 'true'}.items()
        ),
        
        # Obstacles Layer
        # Publishes the obstacles URDF description model under the '/obstacles_description' topic.
        # Enables RViz to visualize the static box obstacle. Only run if 'use_obstacles' is true.
        obstacles_publisher,
        
        # Joins the coordinate frames of the robot ('root') and the environment obstacles ('world') at the same origin.
        # This resolves transform (TF) errors in RViz and allows both the robot and obstacles to render in the same coordinate space.
        static_tf_publisher,
        
        # Planning Layer: Launch the White-Box Planner agent with YAML params.
        # Launches the main motion planning node (planning_node.py) which handles start/goal conversion,
        # collision checks, pathfinding, and publishing path trails. Parameters are loaded from config/planner_params.yaml.
        planner_node,

        # Communication Layer: Open WebSockets bridge.
        # Launches a WebSocket server on port 9090 to enable bi-directional communications
        # between ROS 2 topics and the Web Dashboard front-end (Three.js).
        rosbridge_node,

        # Visualization Layer: Calculate C-Space obstacles.
        # Launches the C-Space voxelizer node (cspace_publisher.py). It discretizes the joint space grid,
        # checks states against registered obstacle spheres, and publishes C-Space voxels for dashboard visualization.
        voxelizer_node
    ])
