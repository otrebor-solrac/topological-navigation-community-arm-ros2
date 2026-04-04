#!/bin/bash
set -e

echo "🧹 Cleaning previous environment..."
docker compose down

echo "🚀 Starting Docker environment..."
docker compose up -d

echo "⏳ Waiting for compilation to finish..."
# Wait for the 'install/setup.bash' file to exist (signal that compilation is finished)
until docker exec ros2_thesis_env [ -f /home/ros_ws/install/setup.bash ]
do
     echo "...still building..."
     sleep 2
done

echo "🤖 Launching Gazebo + RViz..."
# This triggers the ROS2 Launch system. Instead of starting 10 different nodes manually, this single Python script orchestrates the entire "bringup."
docker exec -it ros2_thesis_env bash -c "source install/setup.bash && ros2 launch open_manipulator_x_bringup gazebo.launch.py start_rviz:=true verbose:=true"