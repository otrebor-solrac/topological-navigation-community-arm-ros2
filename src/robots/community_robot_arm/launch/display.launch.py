"""
Launch file for the Community Robot Arm.

This launch file will:
1. Launch the robot and RViz.
2. Launch the White-Box Planner node.
3. Load the planner parameters from config/planner_params.yaml.
"""

import os
import yaml
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# To add new urdf file, add the file to the urdf folder and update this function
def _select_urdf_file(pkg_share, use_spherized):
    """
    Select the URDF file based on the use_spherized flag.

    :param pkg_share: The package share directory
    :param use_spherized: Boolean flag to select the URDF file
    :return: URDF file path
    """
    if use_spherized:
        urdf_file = os.path.join(pkg_share, 'urdf', 'spherized', 'community_robot_arm_slim_spherized.urdf')
        print("LOADING SPHERIZED MODEL (ROS 2 COMPATIBLE)...")
    else:
        urdf_file = os.path.join(pkg_share, 'urdf', 'raw', 'community_robot_arm_slim.urdf')
        print("LOADING NORMAL MODEL...")

    return urdf_file

def _select_rviz_config(pkg_share, use_spherized):
    """
    Select the RViz configuration file based on the use_spherized flag.

    :param pkg_share: The package share directory
    :param use_spherized: Boolean flag to select the RViz configuration file
    :return: RViz configuration file path
    """
    
    if use_spherized:
        return os.path.join(pkg_share, 'rviz', 'spherized.rviz')
    else:
        return os.path.join(pkg_share, 'rviz', 'display.rviz')

def _get_zeros_params(pkg_share_wb):
    """
    Load start positions from whitebox_motion_planners waypoints.yaml
    to initialize the GUI sliders.

    :param pkg_share_wb: The package share directory
    :return: Dictionary of start positions
    """

    yaml_path = os.path.join(pkg_share_wb, 'config', 'waypoints.yaml')
    try:
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
            waypoints = data.get('waypoints', [])
        
        if waypoints:
            # Take the first waypoint as the starting position
            start = waypoints[0]
            # Convert degrees to radians
            start_rad = [float(x) * math.pi / 180.0 for x in start]
            
            return {
                'zeros.base_yaw_joint': start_rad[0],
                'zeros.shoulder_pitch_joint': start_rad[1],
                'zeros.elbow_pitch_joint': start_rad[2],
            }
    except Exception as e:
        print(f"Could not load start zeros from waypoints.yaml: {e}")
    return {}

def launch_setup(context, *args, **kwargs):
    """
    Initialize the launch setup with the given context.

    :param context: The launch context
    :param args: The launch arguments
    :param kwargs: The launch keyword arguments
    :return: List of nodes to be launched
    """
    pkg_share = get_package_share_directory('community_robot_arm')
    
    # Get the value of the argument
    use_spherized = LaunchConfiguration('spherized').perform(context).lower() == 'true'
    urdf_file = _select_urdf_file(pkg_share, use_spherized)

    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Select RViz configuration file
    rviz_config = _select_rviz_config(pkg_share, use_spherized)
    if not os.path.exists(rviz_config):
        raise FileNotFoundError(f"RViz configuration file not found: {rviz_config}")

    # Load start positions from whitebox_motion_planners config to initialize the GUI sliders
    wb_share = get_package_share_directory('whitebox_motion_planners')
    zeros_params = _get_zeros_params(wb_share)

    # Dynamic path for parallelogram kinematics script
    script_path = os.path.join(pkg_share, 'scripts', 'parallelogram_kinematics.py')

    return [
        # robot_state_publisher: Computes and publishes the 3D transforms (TF) of all robot links
        # based on the URDF model and the current joint positions published to /joint_states.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        # joint_state_publisher_gui: Spawns the interactive GUI sliders allowing manual control
        # of the active (master) joint positions. Initialized with values from waypoints.yaml.
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            remappings=[('/joint_states', LaunchConfiguration('gui_topic'))],
            parameters=[zeros_params] if zeros_params else []
        ),
        # ExecuteProcess: Runs the parallelogram kinematics Python script as a system process
        # (which initializes a ROS 2 node). It translates active/master joint states to
        # passive dependent links to maintain physical parallelogram constraints.
        ExecuteProcess(
            cmd=['python3', script_path],
            output='screen'
        ),
        # rviz2: Launches the ROS 3D visualizer using the selected configuration file to render
        # the robot model, obstacles, and path markers.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ]

def generate_launch_description():
    """
    Returns a LaunchDescription object containing all the nodes to be launched.
    """
    
    return LaunchDescription([
        # Declare launch argument to toggle between the spherized collision model and the raw URDF model.
        DeclareLaunchArgument(
            'spherized',
            default_value='true',
            description='Set to "true" to visualize the spherized robot collision primitives in RViz'
        ),
        # Declare launch argument to specify the topic where the joint state publisher GUI publishes raw states.
        # This prevents the GUI from publishing directly to '/joint_states', allowing the kinematics node to intercept 
        # these raw states, solve the parallelogram linkage loop constraints, and publish the final synchronized joints.
        DeclareLaunchArgument(
            'gui_topic',
            default_value='/gui_master_states',
            description='Topic where the GUI publishes raw joint states before parallel kinematics validation'
        ),
        # OpaqueFunction executes the setup logic to dynamically resolve paths and configure node parameters before execution.
        OpaqueFunction(function=launch_setup)
    ])
