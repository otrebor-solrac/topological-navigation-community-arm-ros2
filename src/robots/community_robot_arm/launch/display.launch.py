import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('community_robot_arm')
    
    # Obtenemos el valor del argumento
    use_spherized = LaunchConfiguration('spherized').perform(context).lower() == 'true'
    
    if use_spherized:
        urdf_file = os.path.join(pkg_share, 'urdf', 'spherized', 'community_robot_arm_slim_spherized-v2.urdf')
        print("LOADING SPHERIZED MODEL (ROS 2 COMPATIBLE)...")
    else:
        urdf_file = os.path.join(pkg_share, 'urdf', 'raw', 'community_robot_arm_slim.urdf')
        print("LOADING NORMAL MODEL...")

    if not os.path.exists(urdf_file):
        # Fallback al full si el slim no existe
        urdf_file = os.path.join(pkg_share, 'urdf', 'raw', 'community_robot_arm_full.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Seleccionar archivo RViz
    if use_spherized:
        rviz_config = os.path.join(pkg_share, 'rviz', 'spherized.rviz')
    else:
        rviz_config = os.path.join(pkg_share, 'rviz', 'display.rviz')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            remappings=[('/joint_states', LaunchConfiguration('gui_topic'))]
        ),
        ExecuteProcess(
            cmd=['python3', '/home/ros_ws/src/robots/community_robot_arm/scripts/parallelogram_kinematics.py'],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'spherized',
            default_value='true',
            description='Usa "true" para ver las esferas'
        ),
        DeclareLaunchArgument(
            'gui_topic',
            default_value='/gui_master_states',
            description='Topic where the GUI publishes master joint states'
        ),
        OpaqueFunction(function=launch_setup)
    ])
