# Thesis Guide and Implementation Roadmap

This document contains the academic outline for the thesis project and the technical roadmap for its completion.

## Thesis Structure Overview

### CHAPTER 1: INTRODUCTION
*   **1.1. Problem Statement:** The lack of transparency ("Black-Boxes") in industrial robotics education.
*   **1.2. Objectives:** Developing a "White-Box" ROS 2 platform for A*/RRT comparison.
*   **1.3. Justification:** Importance of algorithmic transparency in engineering education.
*   **1.4. Scope & Limitations:** Offline planning in static maps.

### CHAPTER 2: MATHEMATICAL FOUNDATIONS
*   **2.1. Kinematics and Spaces:** SE(3), DH Matrices, Workspace vs. Configuration Space.
*   **2.2. Topological Manifolds:** Modeling the robot as a Torus ($T^n$).
*   **2.3. Pathplanning Algorithms:** A* (Heuristics $L_1/L_2$) and RRT (Probabilistic Completeness).
*   **2.4. Optimization:** Smooth trajectories using Bézier curves.

### CHAPTER 3: METHODOLOGY
*   **3.1. System Architecture:** Sense-Plan-Act framework.
*   **3.2. Simulation Environment:** ROS 2 Humble, Gazebo, and RViz2.
*   **3.3. Test Scenarios:** Free space, "Bug Trap" (U-shape), and narrow corridors.

### CHAPTER 4: IMPLEMENTATION
*   **4.1. Package Structure:** Nodes and topics in ROS 2.
*   **4.2. Voxel Grid:** Discretization of the continuous toroidal manifold.
*   **4.3. Algorithm Realization:** Python implementation of topological A* and RRT.

---

## 🚀 IMPLEMENTATION ROADMAP

### PHASE 1: ROS 2 Package Core
- [x] Technical environment (Docker, NVIDIA Toolkit).
- [ ] Create `planificacion_didactica_pkg` structure.
- [ ] Implement central node `simulador_nodo.py`.

### PHASE 2: C-Space Mapping
- [ ] Script `cinematica.py` (DH Matrices).
- [x] Mathematical visualization of $T^2$ (LATEX/PGF).
- [ ] Collision detection via **Open Ball Covering** ($L_2$ distance logic).

### PHASE 3: Deterministic Búsqueda (A*)
- [ ] Voxel Grid generator for Toroidal structure.
- [ ] Implementation of `a_star.py` with Manhattan $L_1$ metrics.

### PHASE 4: Stochastic Sampling (RRT)
- [ ] Implementation of `rrt.py` with modular arithmetic.
- [ ] RViz2 Marker visualization for graph/tree expansion.

### PHASE 5: Validation and Results
- [ ] Design `.sdf` worlds in Gazebo (Bug Trap, Corridors).
- [ ] Benchmark script for computation time and path length.
- [ ] Thesis final draft consolidation.
