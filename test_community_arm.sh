#!/bin/zsh
echo "🤖 Rebuilding and Launching INSIDE the container..."
# 1. Source de ROS oficial
# 2. Limpieza
# 3. Construccion
# 4. Source de nuestro paquete
# 5. Launch
docker exec -it ros2_thesis_env bash -c "source /opt/ros/humble/setup.bash && cd /home/ros_ws && rm -rf build/community_robot_arm install/community_robot_arm && colcon build --packages-select community_robot_arm && source install/setup.bash && ros2 launch community_robot_arm display.launch.py"
