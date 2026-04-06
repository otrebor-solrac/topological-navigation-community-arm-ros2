# BASE IMAGE: We use the full desktop version of ROS2 Humble.
# Includes graphical libraries (X11/OpenGL) necessary to run RViz2.
FROM osrf/ros:humble-desktop-full

# Update repositories and install key tools for the manipulator.
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-trajectory-controller \
    ros-humble-moveit \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    git \
    python3-pip \
    && pip3 install pudb \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace directory
ENV ROS_WS=/home/ros_ws
WORKDIR $ROS_WS

# The simulation_entrypoint.sh is now handled via docker-compose volumes
# to allow real-time edits without rebuilding the image.
ENTRYPOINT ["/home/ros_ws/simulation_entrypoint.sh"]
CMD ["bash"]
