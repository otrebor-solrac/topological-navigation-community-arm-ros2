import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def get_default_obstacle_type():
    """
    Parse the default obstacle type from the planner_params.yaml file.
    """
    try:
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
    pkg_share = get_package_share_directory('community_robot_arm')
    
    # 1. Get obstacle type parameter
    obstacle_type = LaunchConfiguration('obstacle_type').perform(context)
    obstacles_urdf_file = os.path.join(pkg_share, 'urdf', 'spherized', 'obstacles', f"{obstacle_type}_spherized.urdf")
    
    print(f"LOADING SPHERIZED OBSTACLE IN RVIZ ONLY: {obstacles_urdf_file}")

    if os.path.exists(obstacles_urdf_file):
        with open(obstacles_urdf_file, 'r') as f:
            obstacles_desc = f.read()
    else:
        print(f"Warning: Spherized Obstacles URDF file not found: {obstacles_urdf_file}")
        obstacles_desc = ""

    # 2. Get robot URDF
    robot_urdf_file = os.path.join(pkg_share, 'urdf', 'spherized', 'community_robot_arm_slim_spherized.urdf')
    with open(robot_urdf_file, 'r') as f:
        robot_desc = f.read()

    # Robot State Publisher
    robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Obstacles State Publisher
    obstacles_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='obstacles_state_publisher',
        output='screen',
        parameters=[{'robot_description': obstacles_desc}],
        remappings=[('/robot_description', '/obstacles_description')]
    )

    # Static link transform between root (robot base) and world (obstacles)
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='root_to_world_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'root', 'world']
    )

    # Interactive GUI for joints
    joint_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Run parallelogram kinematics script
    script_path = os.path.join(pkg_share, 'scripts', 'parallelogram_kinematics.py')
    kinematics_process = ExecuteProcess(
        cmd=['python3', script_path],
        output='screen'
    )

    # RViz config
    rviz_config = os.path.join(pkg_share, 'rviz', 'spherized.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return [
        robot_publisher,
        obstacles_publisher,
        static_tf_publisher,
        joint_gui,
        kinematics_process,
        rviz
    ]

def generate_launch_description():
    default_obstacle = get_default_obstacle_type()
    return LaunchDescription([
        DeclareLaunchArgument(
            'obstacle_type',
            default_value=default_obstacle,
            description='Obstacle type to load (box_obstacle, narrow_passage, u_obstacle, toroidal_wall)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
