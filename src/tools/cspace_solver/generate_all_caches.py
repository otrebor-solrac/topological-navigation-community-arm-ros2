#!/usr/bin/env python3
import os
import sys
import json
import yaml
import hashlib
import subprocess
import xml.etree.ElementTree as ET

# Add package directory to PYTHONPATH
# Script is at src/tools/cspace_solver/generate_all_caches.py
workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
sys.path.append(os.path.join(workspace_dir, 'src/whitebox_motion_planners'))

from whitebox_motion_planners.collision.urdf_collision_parser import UrdfCollisionParser
from whitebox_motion_planners.collision.grid_discretizer import GridDiscretizer

def load_obstacles_from_urdf(urdf_path):
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
    except Exception as e:
        print(f"Failed to parse XML from URDF: {e}")
        return []
    
    link_positions = {'world': (0.0, 0.0, 0.0)}
    for joint in root.findall('joint'):
        parent_el = joint.find('parent')
        child_el = joint.find('child')
        if parent_el is None or child_el is None:
            continue
        parent = parent_el.get('link')
        child = child_el.get('link')
        origin = joint.find('origin')
        xyz_str = origin.get('xyz') if origin is not None else "0 0 0"
        xyz = [float(x) for x in xyz_str.split()]
        
        if parent in link_positions:
            p_pos = link_positions[parent]
            link_positions[child] = (
                p_pos[0] + xyz[0],
                p_pos[1] + xyz[1],
                p_pos[2] + xyz[2]
            )
        else:
            link_positions[child] = tuple(xyz)
            
    obstacles = []
    for link in root.findall('link'):
        link_name = link.get('name')
        link_pos = link_positions.get(link_name, (0.0, 0.0, 0.0))
        for collision in link.findall('collision'):
            origin = collision.find('origin')
            geometry = collision.find('geometry')
            if geometry is not None:
                sphere = geometry.find('sphere')
                if sphere is not None:
                    radius = float(sphere.get('radius'))
                    xyz_str = origin.get('xyz') if origin is not None else "0 0 0"
                    offset = [float(x) for x in xyz_str.split()]
                    abs_center = (
                        link_pos[0] + offset[0],
                        link_pos[1] + offset[1],
                        link_pos[2] + offset[2]
                    )
                    obstacles.append((abs_center, radius))
    return obstacles

def compute_file_hash(file_path):
    hash_md5 = hashlib.md5()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()[:8]

def main():
    config_path = os.path.join(os.path.dirname(__file__), 'cspace_generation.yaml')
    if not os.path.exists(config_path):
        print(f"Error: Config file not found at {config_path}")
        sys.exit(1)
        
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
        
    paths_cfg = config.get('paths', {})
    robot_urdf = os.path.abspath(os.path.join(workspace_dir, paths_cfg.get('robot_urdf', '')))
    rust_binary = os.path.abspath(os.path.join(workspace_dir, paths_cfg.get('rust_binary', '')))
    cache_dir = os.path.abspath(os.path.join(workspace_dir, paths_cfg.get('cache_dir', 'cspace_cache')))
    obstacles_dir = os.path.abspath(os.path.join(workspace_dir, paths_cfg.get('obstacles_dir', '')))
    
    # Load joint offsets and directions from planner_params.yaml
    params_path = os.path.abspath(os.path.join(workspace_dir, 'src/whitebox_motion_planners/config/planner_params.yaml'))
    if os.path.exists(params_path):
        with open(params_path, 'r') as f:
            params_data = yaml.safe_load(f)
        
        # ROS 2 params are typically structured under /**/ros__parameters
        ros_params = params_data.get('/**', {}).get('ros__parameters', {})
        offsets_deg = ros_params.get('joint_offsets', {})
        directions = ros_params.get('joint_directions', {})
        
        import math
        offset_base_yaw = math.radians(offsets_deg.get('base_yaw', 0.0))
        offset_shoulder_pitch = math.radians(offsets_deg.get('shoulder_pitch', 0.0))
        offset_elbow_pitch = math.radians(offsets_deg.get('elbow_pitch', 0.0))
        
        dir_base_yaw = float(directions.get('base_yaw', 1.0))
        dir_shoulder_pitch = float(directions.get('shoulder_pitch', 1.0))
        dir_elbow_pitch = float(directions.get('elbow_pitch', 1.0))
    else:
        print(f"Warning: planner_params.yaml not found at {params_path}, using defaults (offset=0, dir=1)")
        offset_base_yaw = 0.0
        offset_shoulder_pitch = 0.0
        offset_elbow_pitch = 0.0
        dir_base_yaw = 1.0
        dir_shoulder_pitch = 1.0
        dir_elbow_pitch = 1.0
    
    scenarios = config.get('scenarios', [])
    
    os.makedirs(cache_dir, exist_ok=True)
    
    if not os.path.exists(robot_urdf):
        print(f"Error: Robot URDF not found at {robot_urdf}")
        sys.exit(1)
    if not os.path.exists(rust_binary):
        print(f"Error: Rust solver binary not found at {rust_binary}")
        sys.exit(1)
        
    for scenario in scenarios:
        name = scenario.get('name')
        obs_type = scenario.get('obstacle_type')
        resolutions = scenario.get('resolutions', [15.0])
        thinning_dist = scenario.get('thinning', 0.015)
        
        print(f"\n==================================================")
        print(f"Scenario: {name} (obstacle={obs_type}, thinning={thinning_dist}m)")
        print(f"==================================================")
        
        # Initialize parser for robot spheres
        print(f"Initializing collider parser with thinning_dist={thinning_dist}m...")
        parser = UrdfCollisionParser(robot_urdf, min_dist=thinning_dist)
        
        # Load obstacles URDF if applicable
        obstacles = []
        obstacles_hash = "no_obstacles"
        if obs_type != "no_obstacles":
            obs_urdf = os.path.join(obstacles_dir, f"{obs_type}_spherized.urdf")
            if not os.path.exists(obs_urdf):
                print(f"Warning: Obstacle URDF not found at {obs_urdf}, skipping this scenario.")
                continue
            obstacles = load_obstacles_from_urdf(obs_urdf)
            obstacles_hash = compute_file_hash(obs_urdf)
            print(f"Loaded {len(obstacles)} obstacle spheres (MD5 hash: {obstacles_hash})")
            
        for step_size in resolutions:
            print(f"\nEvaluating step size: {step_size}deg...")
            grid = GridDiscretizer(step_size_deg=step_size, num_dof=3)
            
            cache_file = os.path.join(cache_dir, f"cspace_cache_{step_size}deg_{thinning_dist}m_{obstacles_hash}.json")
            
            # Format payload for Rust solver
            input_data = {
                'joints': [
                    {
                        'name': jinfo['name'],
                        'parent': jinfo['parent'],
                        'child': child,
                        'static_t': jinfo['static_T'].tolist(),
                        'axis': [float(x) for x in jinfo['axis']]
                    }
                    for child, jinfo in parser.joints.items()
                ],
                'root_link': parser.root_link,
                'thinned_spheres': [
                    {
                        'link': s['link'],
                        'local_c': s['local_center'].tolist(),
                        'radius': float(s['radius'])
                    }
                    for s in parser.thinned_spheres
                ],
                'active_pairs': list(parser.active_checking_pairs),
                'obstacles': [
                    {
                        'center': [float(c) for c in center],
                        'radius': float(radius)
                    }
                    for center, radius in obstacles
                ],
                'steps_per_circle': int(grid.steps_per_circle),
                'num_dof': int(grid.num_dof),
                'step_rad': float(grid.step_rad),
                'offset_base_yaw': float(offset_base_yaw),
                'offset_shoulder_pitch': float(offset_shoulder_pitch),
                'offset_elbow_pitch': float(offset_elbow_pitch),
                'dir_base_yaw': float(dir_base_yaw),
                'dir_shoulder_pitch': float(dir_shoulder_pitch),
                'dir_elbow_pitch': float(dir_elbow_pitch),
            }
            
            print(f"Launching Rust parallel solver for {grid.steps_per_circle}^3 = {grid.steps_per_circle**3} configurations...")
            try:
                process = subprocess.Popen(
                    [rust_binary],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                stdout, stderr = process.communicate(input=json.dumps(input_data))
                if process.returncode != 0:
                    print(f"Error running Rust solver: {stderr}")
                    continue
                    
                output = json.loads(stdout)
                forbidden_voxels = output.get('forbidden_voxels', [])
                
                # Save cache file
                with open(cache_file, 'w') as out_f:
                    json.dump(output, out_f)
                    
                print(f"Saved cache file: {os.path.basename(cache_file)}")
                print(f"Forbidden voxels: {len(forbidden_voxels)} / {grid.steps_per_circle**3}")
            except Exception as e:
                print(f"Exception during generation: {e}")
                
    print("\nBatch generation complete!")

if __name__ == '__main__':
    main()
