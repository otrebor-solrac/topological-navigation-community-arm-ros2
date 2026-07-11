import xml.etree.ElementTree as ET
import numpy as np
import os
from typing import List, Tuple, Dict, Set

def rpy_to_rotation_matrix(r: float, p: float, y: float) -> np.ndarray:
    """
    This function is a helper function to create a 3x3 
    rotation matrix from a 3x1 rotation vector.
    The rotation matrix is computed using the Z-Y-X 
    Euler angles convention (yaw, pitch, roll).
    
    :param r: roll angle in radians
    :param p: pitch angle in radians
    :param y: yaw angle in radians
    :return: 3x3 rotation matrix
    """

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
    """
    This function is a helper function to create a 4x4 
    homogeneous matrix from a 3x1 translation vector and a 3x1 rotation vector.

    :param xyz: 3x1 translation vector
    :param rpy: 3x1 rotation vector (roll, pitch, yaw)
    :return: 4x4 homogeneous transformation matrix
    """
    
    T = np.eye(4)
    T[:3, :3] = rpy_to_rotation_matrix(*rpy)
    T[:3, 3] = xyz
    return T

class UrdfCollisionParser:
    def __init__(
        self, 
        urdf_path: str, 
        min_dist: float = 0.015,
        acm_margin: float = 0.005,
        offset_base_yaw: float = 0.559643,
        offset_shoulder_pitch: float = 1.57079632679,
        offset_elbow_pitch: float = 0.0
    ):

        self.urdf_path = urdf_path
        self.min_dist = min_dist
        self.acm_margin = acm_margin
        self.offset_base_yaw = offset_base_yaw
        self.offset_shoulder_pitch = offset_shoulder_pitch
        self.offset_elbow_pitch = offset_elbow_pitch
        
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
        # Base Group: Represents static mount components (base, legs, socket) and the closed
        # parallel linkage mechanism (lower_shank, pleuel, triplate). Grouping them here prevents
        # false-positive self-collisions between adjacent/parallel parts of the linkage.
        base_keywords = [
            'main_body', 'stepper_motor', 'stabilizer', 'limit_switch', 'endstop',
            'lower_shank', 'pleuel', 'triplate', 'basering', 'socket', 'leg', 
            'gear_body', 'lever', 'rotategear'
        ]
        # Moving Arm Group: Represents dynamic segments of the arm that travel through space (upper arm,
        # gripper body, claws, and fingers). These are checked for collisions against the Base Group.
        moving_keywords = ['upper_shank', 'gripper', 'finger', 'manipulator','pleuel']
        
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
                # static_T: 4x4 homogeneous transformation matrix representing the fixed translation (xyz) 
                # and rotation (rpy) of the child frame relative to the parent frame when joint angle is 0.
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
                        # (x, y, z) coordinates of the sphere center in the link frame
                        # r: radius of the sphere
                        spheres.append((np.array(xyz), r))
            if spheres:
                self.links_with_spheres[lname] = spheres

    def compute_transforms(
        self,
        q1: float,
        q2: float,
        q3: float
    ) -> Dict[str, np.ndarray]:
        # Solve parallelogram kinematic coupling.
        # Note: These equations are copied from parallelogram_kinematics.py.
        # We must explicitly resolve the passive joint angles of the closed-loop
        # parallel linkage mechanism here because compute_transforms() performs 
        # forward kinematics. Without these equations, the position transforms 
        # of child links (like triplates and upper linkages) would not align, 
        # resulting in incorrect sphere coordinates and broken collision checks.
        joint_angles = {
            # Primary active joint states (master motor inputs)
            'base_yaw_joint': q1,
            'shoulder_pitch_joint': q2,
            'elbow_pitch_joint': q3,
            
            # Parallelogram linkage loop closure constraints
            'revolute_16_0': -q3 - q2,
            'revolute_12_0': q2 + q3,
            
            # Lower linkages (rotating with lower_shank pitch)
            'revolute_32_0': q2,
            'revolute_31_0': -q2,
            
            # Triplate orientation compensation (keeping triplates level relative to world frame)
            'revolute_13_0': -q2,
            'revolute_18_0': q2,
            
            # Upper linkages (driving the end-effector pitch matching elbow rotation)
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
            
        # Perform a Breadth-First Search (BFS) traversal starting from the root link 
        # to propagate coordinate frames down the kinematic tree hierarchy.
        queue = [self.root_link]
        while queue:
            parent = queue.pop(0)
            p_T = transforms[parent] # World transform matrix of the parent link
            
            # Check if this parent link has any child links connected via joints
            if parent in children:
                for child in children[parent]:
                    jinfo = self.joints[child]
                    static_T = jinfo['static_T'] # Static CAD translation/rotation offset
                    jtype = jinfo['type']
                    jname = jinfo['name']
                    
                    if jtype in ['revolute', 'continuous']:
                        # For active/passive moving joints, compute the variable rotation around the joint axis
                        angle = joint_angles.get(jname, 0.0)
                        R_joint = rotation_matrix_axis_angle(jinfo['axis'], angle)
                        T_joint = np.eye(4)
                        T_joint[:3, :3] = R_joint
                        
                        # Child transform = Parent transform * Static offset * Joint rotation
                        transforms[child] = p_T @ static_T @ T_joint
                    else:
                        # For fixed joints, simply apply the static offset
                        transforms[child] = p_T @ static_T
                        
                    # Queue the child link to propagate transforms to its descendants
                    queue.append(child)
                    
        return transforms

    def _initialize_thinned_spheres(self):
        # 1. Compute positions of all spheres at home pose using offsets
        tfs = self.compute_transforms(self.offset_base_yaw, self.offset_shoulder_pitch, self.offset_elbow_pitch)
        
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
            for tw_c, tr, tlname, _ in thinned:
                # Only thin out spheres if they belong to the SAME link and are too close.
                # Comparing across different links would cause adjacent links to erase each other's spheres.
                if tlname == lname and np.linalg.norm(world_c - tw_c) < self.min_dist:
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
                if same_or_adj or dist <= (r_i + r_j + self.acm_margin):
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
        Checks if the robot collides with itself at configuration q, or with the floor.
        To avoid false positives from the parallel link mechanism, we only check
        collisions between the Base group and the Moving Arm group.
        """
        centers, radii = self.get_transformed_spheres(q)
        
        # Check collision with the floor (Z < 0) for moving links
        for i, s in enumerate(self.thinned_spheres):
            lname = s['link'].lower()
            if any(k in lname for k in ['basering', 'leg', 'main_body', 'stepper_motor', 'stabilizer', 'socket']):
                continue
            
            c_z = centers[i][2]
            r = radii[i]
            if c_z - r < 0.0:
                return True

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

    def update_home_pose(self, offset_base_yaw: float, offset_shoulder_pitch: float, offset_elbow_pitch: float):
        self.offset_base_yaw = offset_base_yaw
        self.offset_shoulder_pitch = offset_shoulder_pitch
        self.offset_elbow_pitch = offset_elbow_pitch
        self.allowed_pairs = set()
        self._initialize_thinned_spheres()
        
        # Reclassify Base vs Moving groups
        self.base_indices = []
        self.moving_indices = []
        # Base Group: Represents static mount components (base, legs, socket) and the closed
        # parallel linkage mechanism (lower_shank, pleuel, triplate). Grouping them here prevents
        # false-positive self-collisions between adjacent/parallel parts of the linkage.
        base_keywords = [
            'main_body', 'stepper_motor', 'stabilizer', 'limit_switch', 'endstop',
            'lower_shank', 'pleuel', 'triplate', 'basering', 'socket', 'leg', 
            'gear_body', 'lever', 'rotategear'
        ]
        # Moving Arm Group: Represents dynamic segments of the arm that travel through space (upper arm,
        # gripper body, claws, and fingers). These are checked for collisions against the Base Group.
        moving_keywords = ['upper_shank', 'gripper', 'finger', 'manipulator']
        for i, s in enumerate(self.thinned_spheres):
            lname = s['link'].lower()
            if any(k in lname for k in base_keywords):
                self.base_indices.append(i)
            elif any(k in lname for k in moving_keywords):
                self.moving_indices.append(i)
                
        # Recompute active checking pairs
        self.active_checking_pairs = []
        for i in self.base_indices:
            for j in self.moving_indices:
                if (i, j) not in self.allowed_pairs and (j, i) not in self.allowed_pairs:
                    self.active_checking_pairs.append((i, j))
                    
        # Clear cache since link sphere positions changed
        self._cache = {}

    def get_end_effector_position(self, q_urdf: tuple) -> np.ndarray:
        """
        Computes the exact Cartesian position of the end-effector center (claws) 
        using the URDF tree transforms for the configuration q_urdf.
        """
        q1, q2, q3 = q_urdf[0], q_urdf[1], q_urdf[2]
        tfs = self.compute_transforms(q1, q2, q3)
        
        # We use the gripper base link and apply the exact calibrated physical offset
        # of the gripper fingers (-54.67 mm in local X and -21.70 mm in local Z)
        link_name = 'gripperbase_by_ftobler'
        if link_name in tfs:
            T = tfs[link_name]
            # X is negative because the local frame is oriented towards the base
            # Z is negative because the claws extend downwards
            tcp_local = np.array([0.02, 0.0, -0.0217, 1.0])
            return (T @ tcp_local)[:3]
            
        # # Fallback to standard end-effector link names if base link not found
        # for fallback_name in ['gripperbase_by_ftobler', 'manipulator_dual']:
        #     if fallback_name in tfs:
        #         return tfs[fallback_name][:3, 3]
                
        # Ultimate fallback to root frame origin
        return np.array([0.0, 0.0, 0.0])
