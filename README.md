# Topological Navigation: Community Robot Arm (ROS 2)

![Community Robot Arm](docs/images/Robot-Arm.png)

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

---

# 📂 Project Structure

This section describes the organization of the **topological-navigation-community-arm-ros2** workspace.

```text
ROS2/ (Workspace Root)
│
├── src/
│   ├── robots/
│   │   └── community_robot_arm/      <-- ROBOT DEFINITION PACKAGE
│   │       ├── urdf/                 <-- [CAD] Cleaned-up Robot Models (URDF)
│   │       ├── meshes/               <-- Optimized STL/DAE files
│   │       ├── launch/               <-- Visualizers and State Publishers
│   │       └── scripts/              <-- Parallel Kinematics Solvers
│   │
│   └── whitebox_motion_planners/     <-- CORE ALGORITHMIC PACKAGE
│       ├── whitebox_motion_planners/
│       │   ├── core/                 <-- [OOP] Interfaces & Exceptions
│       │   ├── kinematics/           <-- [KINEMATICS] Robot Models
│       │   ├── collision/            <-- [FOAM] Grid & Spherical Colliders
│       │   ├── planners/             <-- [ALGO] A*, RRT, and Factory
│       │   └── ros2/                 <-- ROS 2 Nodes and Integrations
│       ├── config/                   <-- [YAML] Single Source of Truth
│       ├── launch/                   <-- Master Orchestration
│       ├── setup.py
│       └── README.md             <-- Package documentation and architecture guide
│
├── Tesis/                            <-- ACADEMIC DOCUMENTATION (LaTeX)
│   ├── Capítulos/                    <-- 1-Intro, 2-Marco-Teórico, 3-Metodología...
│   ├── Imagenes/                     <-- TikZ, PGF, and external figures
│   └── main.tex                      <-- Thesis entry point
│
├── Makefile                          <-- [EXEC] Unified Command Center (build, run, shell)
├── docker-compose.yml                <-- [INFRA] Environment Orchestration
├── Dockerfile                        <-- [IMAGE] ROS 2 Humble base image
└── README.md                         <-- This document
```

## Component Descriptions

### 🤖 Robots & Hardware (`src/robots/`)
Contains the physical representation of the manipulators. The `community_robot_arm` is the main focus, featuring a "Slim" URDF purged of non-functional hardware (screws, caps) to optimize collision checking.

### 🌀 Planners (`src/whitebox_motion_planners/`)
The "White-Box" implementation of the planning pipeline. 
- **Topological Math**: Handles the "Wrap-around" distance calculations to treat joints as $S^1$ components of a Torus $T^n$.
- **Collision Engine**: Uses the FOAM (Fast Open Approximation of Manifolds) approach, representing obstacles as unions of open balls.

### 🎓 Thesis (`Tesis/`)
The complete academic work. It is structured to maintain a direct link between mathematical definitions (Chapter 2) and their implementation in the source code.

### 🐳 Infrastructure
The project is fully containerized to ensure reproducibility. 
- **Docker Compose**: Bridges the host's X11 socket for RViz2 hardware acceleration.
- **Makefile**: Provides a unified interface for the developer (`make run`, `make build`, `make shell`).
