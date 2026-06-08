import xml.etree.ElementTree as ET
import numpy as np
import os

def rpy_to_rotation_matrix(r, p, y):
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

def rotation_matrix_axis_angle(axis, theta):
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

def make_homogeneous_matrix(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_rotation_matrix(*rpy)
    T[:3, 3] = xyz
    return T

class URDFKinematics:
    def __init__(self, urdf_path):
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        
        self.joints = {}
        self.links = set()
        
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
                
            self.links.add(parent)
            self.links.add(child)
            
            self.joints[child] = {
                'name': name,
                'type': jtype,
                'parent': parent,
                'static_T': make_homogeneous_matrix(xyz, rpy),
                'axis': axis
            }
            
    def compute_transforms(self, q1, q2, q3):
        # Parallelogram calculations
        joint_angles = {
            'base_yaw_joint': q1,
            'shoulder_pitch_joint': q2,
            'elbow_pitch_joint': q3,
            'revolute_16_0': -q3 - q2,
            'revolute_12_0': q2 + q3,
            'revolute_32_0': q2,
            'revolute_31_0': -q2,
            'revolute_13_0': -q2,
            'revolute_18_0': q2,
            'revolute_15_0': -q3,
            'revolute_19_0': q3,
        }
        
        # Start from base_link or world
        transforms = {'root': np.eye(4)}
        
        # Queue for BFS traversal
        # Build adjacency list: parent -> children
        children = {}
        for child, jinfo in self.joints.items():
            parent = jinfo['parent']
            if parent not in children:
                children[parent] = []
            children[parent].append(child)
            
        queue = ['root']
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

if __name__ == '__main__':
    urdf = "/home/ros_ws/src/robots/community_robot_arm/urdf/spherized/community_robot_arm_slim_spherized.urdf"
    kin = URDFKinematics(urdf)
    
    # Let's find links that have no parent joint
    all_parents = set(jinfo['parent'] for jinfo in kin.joints.values())
    all_children = set(kin.joints.keys())
    roots = all_parents - all_children
    print("Roots found in URDF tree:", roots)
    
    tfs = kin.compute_transforms(0.0, 0.5, -0.5)
    print(f"Computed {len(tfs)} transforms. Links in transforms: {list(tfs.keys())[:10]}")
    
    # Try print some known links if they exist
    for l in ['lower_shank_140', 'upper_shank_140', 'gripperbase_by_ftobler']:
        if l in tfs:
            print(f"{l} origin:", tfs[l][:3, 3])
        else:
            print(f"{l} not found in computed transforms")
