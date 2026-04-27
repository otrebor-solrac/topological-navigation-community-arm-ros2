# White-Box Motion Planners

This package provides a transparent, "white-box" implementation of motion planning algorithms on topological manifolds, specifically tailored for robotic arms modeled on the n-dimensional Torus ($T^n$). It is designed as a modular framework for both academic research (Thesis) and laboratory education.

## 📦 Package Metadata & Configuration

The package follows the standard ROS 2 `ament_python` build system.

### `package.xml`
- **Identity**: Defines the package name as `whitebox_motion_planners`.
- **Dependencies**: Lists the required ROS 2 and system dependencies (e.g., `rclpy`, `std_msgs`, `control_msgs`).
- **Export**: Specifies the build type as `ament_python` to ensure compatibility with `colcon build`.

### `setup.py` & `setup.cfg`
- **Discovery**: Automatically finds all submodules within the `whitebox_motion_planners` directory.
- **Entry Points**: Defines the console script `planificador`, which maps to the main execution loop in `ros2/planning_node.py`.
- **Installation**: Configures where scripts and metadata are placed in the `install/` directory of the workspace.

---

## 🏗️ System Architectured

The package is structured to ensure scalability.

### 1. `core/` (The Contract)
- **`interfaces.py`**: Contains Abstract Base Classes (ABCs) like `BasePlanner`, `BaseKinematics`, and `BaseCollider`. Any new algorithm or robot must implement these interfaces.
- **`exceptions.py`**: Custom error handling for planning scenarios (e.g., `PathNotFoundError`).

### 2. `spaces/` (Topological Logic)
- **`topological_math.py`**: Pure mathematical functions for $T^n$ geometry, such as `normalize_angle` and `wrap_around_dist` (modular arithmetic).
- **`metrics.py`**: Implementations of distance heuristics (L1 Manhattan, L2 Euclidean) adapted for wrap-around manifolds.

### 3. `kinematics/` (Robot Models)
- **`community_arm.py`**: Dedicated class for the thesis robot using Denavit-Hartenberg (DH) parameters.
- **`open_manipulator.py`**: Legacy model used for initial validation and testing.

### 4. `collision/` (Environment Awareness)
- **`foam_collider.py`**: Implements the **Fast Open Approximation of Manifolds (FOAM)**, using spherical coverings ($B_r$) to detect collisions.
- **`grid_discretizer.py`**: Handles the discretization of the continuous configuration space into a voxel grid for deterministic search.

### 5. `planners/` (Algorithms)
- **`a_star.py`**: Deterministic search implementation on $T^n$.
- **`rrt.py`**: Stochastic sampling-based search (Rapidly-exploring Random Trees).
- **`planner_factory.py`**: A factory class that instantiates the desired algorithm at runtime based on parameters.

### 6. `ros2/` (Middleware Interface)
- **`planning_node.py`**: The primary ROS 2 Node. It orchestrates the planning process by communicating with Gazebo/RViz and injecting the required Kinematics/Collider/Planner components.

---

## 🛠️ Usage

To build the package:
```bash
colcon build --packages-select whitebox_motion_planners
source install/setup.bash
```

To run the planner:
```bash
ros2 run whitebox_motion_planners planificador
```

## 🎓 Academic Purpose
This framework was developed to avoid "Black-Box" planning solutions. Every step of the planning process—from the topological metric calculation to the collision primitive check—is exposed and documented for educational transparency.
