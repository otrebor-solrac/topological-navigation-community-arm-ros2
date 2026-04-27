# --- Configurations ---
CONTAINER_NAME = ros2_thesis_env
WS_PATH = /home/ros_ws
PKGS = community_robot_arm whitebox_motion_planners

# --- Default Goal ---
.DEFAULT_GOAL := help

# --- Targets ---

.PHONY: build run shell debug-env clean help

help:
	@echo "🦾 White-Box Motion Planning - Command Center"
	@echo "--------------------------------------------"
	@echo "make build     - Compile the ROS 2 packages"
	@echo "make run       - Build and launch the full project (Master Launch)"
	@echo "make shell     - Open an interactive terminal inside the Docker container"
	@echo "make debug-env - Launch ONLY the robot and RViz (ready for manual planner debug)"
	@echo "make clean     - Delete build, install, and log directories"
	@echo "--------------------------------------------"

# 1. Compile the project
build:
	docker compose up -d
	docker exec -it $(CONTAINER_NAME) bash -c "source /opt/ros/humble/setup.bash && cd $(WS_PATH) && colcon build --symlink-install --packages-select $(PKGS)"

# 2. Build and Launch everything
run: build
	docker exec -it $(CONTAINER_NAME) bash -c "source /opt/ros/humble/setup.bash && cd $(WS_PATH) && source install/setup.bash && ros2 launch whitebox_motion_planners planning.launch.py"

# 3. Enter the container shell
shell:
	docker exec -it $(CONTAINER_NAME) bash -c "source /opt/ros/humble/setup.bash && source $(WS_PATH)/install/setup.bash && exec bash"

# 4. Environment for Debugging (No planner)
debug-env: build
	docker exec -it $(CONTAINER_NAME) bash -c "source /opt/ros/humble/setup.bash && cd $(WS_PATH) && source install/setup.bash && ros2 launch community_robot_arm display.launch.py spherized:=true"

# 5. Clean Workspace
clean:
	docker exec -it $(CONTAINER_NAME) bash -c "cd $(WS_PATH) && rm -rf build/ install/ log/"
