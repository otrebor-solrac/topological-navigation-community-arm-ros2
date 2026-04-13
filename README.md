# Topological Navigation: Community Robot Arm (ROS 2)

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![ROS2](https://img.shields.io/badge/ros2-humble-green.svg)

A ROS 2-based framework for topological motion planning and trajectory optimization applied to the **Community Robot Arm**. This project implements a **"white-box" approach** to model the manipulator's configuration space as a toroidal manifold ($T^n$), featuring a comparative analysis between **A*** (deterministic search) and **RRT** (stochastic sampling) algorithms.

## 📖 Overview

This repository transitions from traditional "black-box" simulation to a mathematically rigorous framework focusing on the topological structure of the robot's motion. 

**Key Technical Concepts:**
- **Toroidal Modeling:** Representation of the $n$-joint manipulator as a Torus ($T^n$).
- **Voxel-based Discretization:** Mapping continuous manifolds into discrete search grids.
- **Ball Covering Collision Detection:** Optimized $L_2$ distance-based safety volumes.
- **Institutional Collaboration:** Developed as a joint research project between **UnADM** and **TESH**.

## 🛠️ Installation & Setup

### Requirements
- **Docker** & **NVIDIA Container Toolkit** (for hardware acceleration).
- **ROS 2 Humble** (Running inside the container).

### Environment Configuration
Run the following commands to configure your local environment for GPU-accelerated simulation:

```bash
# Install NVIDIA Container Toolkit
sudo apt-get install -y nvidia-container-toolkit

# Configure Docker runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

## 🚀 Quick Start (Simulation)

The included scripts serve different developmental stages:

### 1. Environment Setup & Reference Launch
Use this to perform a clean start of the Docker environment and launch the reference **Open Manipulator X** simulation.
```bash
# Cleans previous containers, starts docker-compose, and launches generic Gazebo
chmod +x start_manual.sh
./start_manual.sh
```

### 2. Thesis Hardware Validation (Community Arm)
Use this to specifically rebuild and visualize the **Community Robot Arm** package described in the thesis.
```bash
# Performs a targeted 'colcon build' and launches the Custom Arm URDF
chmod +x test_community_arm.sh
./test_community_arm.sh
```

### 3. Developer Shell (Interactive)
Access the running container's shell to run manual commands (`colcon`, `ros2 node`, etc.).
```bash
# Enters the container with ROS 2 environment sourced
chmod +x dev_shell.sh
./dev_shell.sh
```

## ⚙️ Hardware Reference

This project is built upon the **Community Robot Arm** open-source hardware:
- 🐙 **GitHub Repository:** [20sffactory/community_robot_arm](https://github.com/20sffactory/community_robot_arm)
- 📐 **GrabCAD (CAD Files):** [Robot Arm Community Version](https://grabcad.com/library/robot-arm-community-version-cad-3d-printed-robotic-arm-1)

## 📐 CAD & URDF Pipeline

The **Community Robot Arm** models are managed through a custom pipeline that bridges high-fidelity CAD with simulation-ready URDFs:

1.  **Source:** Designed in **Onshape** with complex mechanical constraints (revolute joints, parallel linkages).
2.  **Export:** Raw models are exported to the `robot-test/` directory. *(Note: Raw source files in `robot-test/` are not included in this repository; only the final generated models are provided).*
3.  **Post-Processing:** A custom Python script sanitizes the export, fixes Onshape-specific asymmetry bugs, converts ASCII STLs to Binary, and renames meshes semantically.

### Generate Optimized Models
```bash
# Generate the 'Slim' version (optimized for planning, removes hardware visuals)
python3 src/robots/community_robot_arm/scripts/create_urdf.py --mode slim

# Generate the 'Full' version (includes NEMA motors, gears, and screws)
python3 src/robots/community_robot_arm/scripts/create_urdf.py --mode full
```

---

## 👨‍🔬 Author
**Roberto Carlos Vazquez Nava**  
*UnADM / TESH - Mechatronics Engineering*
