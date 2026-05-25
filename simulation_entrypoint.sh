#!/bin/bash
set -e

# === 1. Source System ROS2 Humble (Underlay) ===
source /opt/ros/humble/setup.bash

# === 2. Build the Workspace (Only if needed) ===
# This ensures that your custom packages are compiled on startup.
# if [ ! -d "build" ]; then
#     echo "🔨 Initial build of the workspace..."
#     colcon build --symlink-install
# fi

# === 3. Source Local Workspace (Overlay) ===
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

# === 4. Execute the Passed Command (The One Truth) ===
# This executes whatever is passed by Docker Compose 'command' 
# or Dockerfile 'CMD', after the environment is set up.
exec "$@"