import xml.etree.ElementTree as ET
import numpy as np
import os
from typing import List, Tuple, Dict, Set

def rpy_to_rotation_matrix(r: float, p: float, y: float) -> np.ndarray:
    c_r, s_r = np.cos(r), np.sin(r)
    c_p, s_p = np.cos(p), np.sin(p)
    c_y, s_y = np.cos(y), np.sin(y)
    
    R_x = np.array([
        [1, 0, 0],
        [0, c_r, -s_r],
        [0, s_r, c_r]
    ])
    R_y = np.array([
        [c_p, 0, s_p],
        [0, 1, 0],
        [-s_p, 0, c_p]
    ])
    R_z = np.array([
        [c_y, -s_y, 0],
        [s_y, c_y, 0],
        [0, 0, 1]
    ])
    return R_z @ R_y @ R_x

def rotation_matrix_axis_angle(axis: np.ndarray, theta: float) -> np.ndarray:
    axis = np.array(axis, dtype=float)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-6:
        return np.eye(3)
    axis = axis / axis_norm
    
    a_x, a_y, a_z = axis
    skew = np.array([
        [0.0, -a_z, a_y],
        [a_z, 0.0, -a_x],
        [-a_y, a_x, 0.0]
    ])
    R = np.eye(3) + np.sin(theta) * skew + (1.0 - np.cos(theta)) * (skew @ skew)
    return R

def make_homogeneous_matrix(xyz: List[float], rpy: List[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_to_rotation_matrix(*rpy)
    T[:3, 3] = xyz
    return T

class UrdfCollisionParser:
    def __init__(self, urdf_path: str, min_dist: float = 0.015):
        self.urdf_path = urdf_path
        self.min_dist = min_dist
        
        self.joints = {}
        self.links_with_spheres = {}
        self.root_link = 'root'
        
        # 1. Parse URDF joints and link spheres
        self._parse_urdf()
        
        # 2. Compute thinned spheres and Allowed Collision Matrix (ACM)
        self.thinned_spheres = []
        self.allowed_pairs = set()
        self._initialize_thinned_spheres()
        
        # 3. Preclassify Base vs Moving groups and precompute active checking pairs
        self.base_indices = []
        self.moving_indices = []
        base_keywords = ['basering', 'socket', 'leg', 'main_body', 'stepper_motor', 'rotategear', 'stabilizer', 'limit_switch', 'endstop']
        moving_keywords = ['shank', 'gripper', 'finger', 'wire', 'pleuel', 'triplate', 'manipulator']
        
        for i, s in enumerate(self.thinned_spheres):
            lname = s['link'].lower()
            if any(k in lname for k in base_keywords):
                self.base_indices.append(i)
            elif any(k in lname for k in moving_keywords):
                self.moving_indices.append(i)
                
        self.active_checking_pairs = []
        for i in self.base_indices:
            for j in self.moving_indices:
                if (i, j) not in self.allowed_pairs and (j, i) not in self.allowed_pairs:
                    self.active_checking_pairs.append((i, j))
                    
        # 4. Memoization Cache for kinematics transformations
        self._cache = {}
        
    def _parse_urdf(self):
        if not os.path.exists(self.urdf_path):
            raise FileNotFoundError(f"URDF file not found at: {self.urdf_path}")
            
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()
        
        # Parse joints
        for joint in root.findall('joint'):
            name = joint.get('name')
            jtype = joint.get('type')
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            
            origin = joint.find('origin')
            xyz_str = origin.get('xyz') if origin is not None else "0 0 0"
            rpy_str = origin.get('rpy') if origin is not None else "0 0 0"
            xyz = [float(x) for x in xyz_str.split()]
            rpy = [float(x) for x in rpy_str.split()]
            
            axis_el = joint.find('axis')
            if axis_el is not None:
                axis_str = axis_el.get('xyz')
                axis = [float(x) for x in axis_str.split()]
            else:
                axis = [1.0, 0.0, 0.0]
                
            self.joints[child] = {
                'name': name,
                'type': jtype,
                'parent': parent,
                'static_T': make_homogeneous_matrix(xyz, rpy),
                'axis': axis
            }
            
        # Determine root of the tree
        all_parents = set(jinfo['parent'] for jinfo in self.joints.values())
        all_children = set(self.joints.keys())
        roots = all_parents - all_children
        if roots:
            self.root_link = list(roots)[0]
            
        # Parse links and their collision spheres
        for link in root.findall('link'):
            lname = link.get('name')
            spheres = []
            for col in link.findall('collision'):
                origin = col.find('origin')
                xyz_str = origin.get('xyz') if origin is not None else "0 0 0"
                xyz = [float(x) for x in xyz_str.split()]
                
                geom = col.find('geometry')
                if geom is not None:
                    sphere = geom.find('sphere')
                    if sphere is not None:
                        r = float(sphere.get('radius'))
                        spheres.append((np.array(xyz), r))
            if spheres:
                self.links_with_spheres[lname] = spheres

    def compute_transforms(self, q1: float, q2: float, q3: float) -> Dict[str, np.ndarray]:
        # Solve parallelogram kinematic coupling
        joint_angles = {
            'revolute_1_0': q1,
            'revolute_9_0': q2,
            'revolute_10_0': q3,
            'revolute_16_0': -q3 - q2,
            'revolute_12_0': q2 + q3,
            'revolute_32_0': q2,
            'revolute_31_0': -q2,
            'revolute_13_0': -q2,
            'revolute_18_0': q2,
            'revolute_15_0': -q3,
            'revolute_19_0': q3,
        }
        
        transforms = {self.root_link: np.eye(4)}
        
        # Build children mapping
        children = {}
        for child, jinfo in self.joints.items():
            parent = jinfo['parent']
            if parent not in children:
                children[parent] = []
            children[parent].append(child)
            
        queue = [self.root_link]
        while queue:
            parent = queue.pop(0)
            p_T = transforms[parent]
            
            if parent in children:
                for child in children[parent]:
                    jinfo = self.joints[child]
                    static_T = jinfo['static_T']
                    jtype = jinfo['type']
                    jname = jinfo['name']
                    
                    if jtype in ['revolute', 'continuous']:
                        angle = joint_angles.get(jname, 0.0)
                        R_joint = rotation_matrix_axis_angle(jinfo['axis'], angle)
                        T_joint = np.eye(4)
                        T_joint[:3, :3] = R_joint
                        transforms[child] = p_T @ static_T @ T_joint
                    else:
                        transforms[child] = p_T @ static_T
                        
                    queue.append(child)
                    
        return transforms

    def _initialize_thinned_spheres(self):
        # 1. Compute positions of all spheres at home pose q = (0, 0, 0)
        tfs = self.compute_transforms(0.0, 0.0, 0.0)
        
        all_spheres = []
        for lname, spheres in self.links_with_spheres.items():
            if lname not in tfs:
                continue
            T = tfs[lname]
            for local_c, r in spheres:
                c_h = np.ones(4)
                c_h[:3] = local_c
                world_c = (T @ c_h)[:3]
                all_spheres.append((world_c, r, lname, local_c))
                
        # 2. Thinning algorithm
        thinned = []
        # Sort by radius descending to prioritize larger spheres
        for world_c, r, lname, local_c in sorted(all_spheres, key=lambda x: x[1], reverse=True):
            too_close = False
            for tw_c, tr, _, _ in thinned:
                if np.linalg.norm(world_c - tw_c) < self.min_dist:
                    too_close = True
                    break
            if not too_close:
                thinned.append((world_c, r, lname, local_c))
                
        # Save thinned spheres representation
        self.thinned_spheres = [
            {'link': lname, 'local_center': local_c, 'radius': r}
            for _, r, lname, local_c in thinned
        ]
        
        # 3. Compute Allowed Collision Matrix (ACM) for the thinned spheres at home pose
        num_spheres = len(thinned)
        for i in range(num_spheres):
            wc_i, r_i, ln_i, _ = thinned[i]
            for j in range(i + 1, num_spheres):
                wc_j, r_j, ln_j, _ = thinned[j]
                
                # If they belong to the same link or are adjacent in the tree, allow collision
                same_or_adj = False
                if ln_i == ln_j:
                    same_or_adj = True
                else:
                    # Check if one is parent of the other
                    if ln_i in self.joints and self.joints[ln_i]['parent'] == ln_j:
                        same_or_adj = True
                    elif ln_j in self.joints and self.joints[ln_j]['parent'] == ln_i:
                        same_or_adj = True
                        
                # Also check distance: if they overlap at home pose, allow it
                dist = np.linalg.norm(wc_i - wc_j)
                if same_or_adj or dist <= (r_i + r_j + 0.005):
                    self.allowed_pairs.add((i, j))

    def get_transformed_spheres(self, q: tuple) -> Tuple[np.ndarray, np.ndarray]:
        """
        Computes the world centers and radii of the thinned spheres for a configuration q.
        Returns:
            centers: numpy array of shape (N, 3)
            radii: numpy array of shape (N,)
        """
        q_key = tuple(q)
        if q_key in self._cache:
            return self._cache[q_key]
            
        q1, q2, q3 = q_key[0], q_key[1], q_key[2]
        tfs = self.compute_transforms(q1, q2, q3)
        
        centers = []
        radii = []
        for s in self.thinned_spheres:
            lname = s['link']
            T = tfs.get(lname, np.eye(4))
            c_h = np.ones(4)
            c_h[:3] = s['local_center']
            world_c = (T @ c_h)[:3]
            centers.append(world_c)
            radii.append(s['radius'])
            
        res = (np.array(centers), np.array(radii))
        self._cache[q_key] = res
        return res

    def check_self_collision(self, q: tuple) -> bool:
        """
        Checks if the robot collides with itself at configuration q.
        To avoid false positives from the parallel link mechanism, we only check
        collisions between the Base group and the Moving Arm group.
        """
        centers, radii = self.get_transformed_spheres(q)
        
        for i, j in self.active_checking_pairs:
            c_i = centers[i]
            c_j = centers[j]
            r_sum = radii[i] + radii[j]
            
            # Fast raw arithmetic to avoid numpy overhead
            dx = c_i[0] - c_j[0]
            dy = c_i[1] - c_j[1]
            dz = c_i[2] - c_j[2]
            dist_sq = dx*dx + dy*dy + dz*dz
            if dist_sq <= r_sum*r_sum:
                return True
        return False

    def check_obstacle_collision(self, q: tuple, obstacles: List[Tuple[np.ndarray, float]]) -> bool:
        """
        Checks if the robot collides with any external obstacles at configuration q.
        """
        if not obstacles:
            return False
            
        centers, radii = self.get_transformed_spheres(q)
        
        # Vectorized check for external obstacles
        obs_centers = np.array([o[0] for o in obstacles]) # (M, 3)
        obs_radii = np.array([o[1] for o in obstacles]) # (M,)
        
        # centers: (N, 3)
        # obs_centers: (M, 3)
        # diff shape: (N, M, 3)
        diff = centers[:, None, :] - obs_centers[None, :, :]
        dist_sq = np.sum(diff**2, axis=-1) # (N, M)
        
        radii_sum = radii[:, None] + obs_radii[None, :] # (N, M)
        
        if np.any(dist_sq <= radii_sum**2):
            return True
            
        return False
