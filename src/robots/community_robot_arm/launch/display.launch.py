import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the share directory of the package community_robot_arm previously built
    pkg_share = get_package_share_directory('community_robot_arm')

    # Declare the launch argument mode
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='full',
        description='Modo de carga: full o slim'
    )

    # Se resuelve en runtime, pero para simplificar usamos full por defecto
    # Para cambiar: ros2 launch community_robot_arm display.launch.py mode:=slim
    mode = LaunchConfiguration('mode')

    # Intentar cargar slim primero, si no existe, cargar full
    slim_path = os.path.join(pkg_share, 'urdf', 'community_robot_arm_slim.urdf')
    full_path = os.path.join(pkg_share, 'urdf', 'community_robot_arm_full.urdf')

    # Por defecto cargamos full
    urdf_file = full_path
    if os.path.exists(slim_path):
        # Si existe slim, lo usamos por defecto
        urdf_file = slim_path

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        mode_arg,
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
            remappings=[('/joint_states', '/master_states')]
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
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')]
        )
    ])
