# --- Configurations ---
CONTAINER_NAME = ros2_thesis_env
WS_PATH = /home/ros_ws
# Solo compilamos los paquetes críticos del robot y el planificador
PKGS = community_robot_arm whitebox_motion_planners

# Obstáculo por defecto (leído de planner_params.yaml o sobreescrito por el usuario)
OBSTACLE ?= $(shell python3 -c "import yaml; print(yaml.safe_load(open('src/whitebox_motion_planners/config/planner_params.yaml'))['/**']['ros__parameters'].get('obstacle_type', 'box_obstacle'))" 2>/dev/null || echo "box_obstacle")

# --- Default Goal ---
.DEFAULT_GOAL := help

# --- Targets ---

.PHONY: build run build-cpu run-cpu restart shell debug-env visualize-obstacles clean help

help:
	@echo "🦾 White-Box Motion Planning - Command Center"
	@echo "--------------------------------------------"
	@echo "make build               - Compile the ROS 2 packages (GPU)"
	@echo "make run                 - Build and launch the full project (GPU)"
	@echo "make build-cpu           - Compile the ROS 2 packages (CPU)"
	@echo "make run-cpu             - Build and launch the full project (CPU)"
	@echo "make restart             - Quick restart of all Docker containers"
	@echo "make shell               - Open an interactive terminal inside the Docker container"
	@echo "make debug-env           - Launch ONLY the robot and RViz (ready for manual planner debug)"
	@echo "make visualize-obstacles - Launch robot and obstacles in RViz to see positioning"
	@echo "make clean               - Delete build, install, and log directories"
	@echo "--------------------------------------------"

# 1. Compile the project (GPU)
build:
	docker compose up -d
	docker exec -it $(CONTAINER_NAME) bash -c "source /opt/ros/humble/setup.bash && cd $(WS_PATH) && colcon build --symlink-install --packages-select $(PKGS)"

# 1.1 Compile the project (CPU)
build-cpu:
	docker compose -f docker-compose.cpu.yml up -d
	docker exec -it $(CONTAINER_NAME) bash -c "source /opt/ros/humble/setup.bash && cd $(WS_PATH) && colcon build --symlink-install --packages-select $(PKGS)"

# 2. Build and Launch everything (GPU)
run: build
	docker exec -it $(CONTAINER_NAME) bash -c "source /opt/ros/humble/setup.bash && cd $(WS_PATH) && source install/setup.bash && ros2 launch whitebox_motion_planners planning.launch.py"

# 2.1 Build and Launch everything (CPU)
run-cpu: build-cpu
	docker exec -it $(CONTAINER_NAME) bash -c "source /opt/ros/humble/setup.bash && cd $(WS_PATH) && source install/setup.bash && ros2 launch whitebox_motion_planners planning.launch.py"

# 3. Quick restart
restart:
	docker compose down && docker compose up -d

# 3. Enter the container shell
shell:
	docker exec -it $(CONTAINER_NAME) bash -c "source /opt/ros/humble/setup.bash && source $(WS_PATH)/install/setup.bash && exec bash"

# 4. Environment for Debugging (No planner)
debug-env: build
	docker exec -it $(CONTAINER_NAME) bash -c "source /opt/ros/humble/setup.bash && cd $(WS_PATH) && source install/setup.bash && ros2 launch community_robot_arm display.launch.py spherized:=true"

# 5. Visualize robot and obstacles in RViz
visualize-obstacles: build
	docker exec -it $(CONTAINER_NAME) bash -c "source /opt/ros/humble/setup.bash && cd $(WS_PATH) && source install/setup.bash && ros2 launch community_robot_arm visualize_obstacles.launch.py obstacle_type:=$(OBSTACLE)"

# 6. Clean Workspace
clean:
	docker exec -it $(CONTAINER_NAME) bash -c "cd $(WS_PATH) && rm -rf build/ install/ log/"

# 7. Generate all C-space caches in batch
generate-caches:
	docker exec -it $(CONTAINER_NAME) bash -c "PYTHONPATH=$(WS_PATH)/src/whitebox_motion_planners python3 $(WS_PATH)/src/tools/cspace_solver/generate_all_caches.py"
