#!/bin/bash
# Este script te abre una terminal dentro del contenedor con ROS 2 configurado.
# Úsalo para compilar o correr tus scripts de Python.

if [ ! "$(docker ps -q -f name=ros2_thesis_env)" ]; then
    echo "Error: El contenedor 'ros2_thesis_env' no está corriendo."
    echo "Por favor inicia la simulación primero."
    exit 1
fi

# Entramos al contenedor, cargamos las variables de entorno, y abrimos bash
docker exec -it ros2_thesis_env bash -c "source /opt/ros/humble/setup.bash && source /home/ros_ws/install/setup.bash && exec bash"
