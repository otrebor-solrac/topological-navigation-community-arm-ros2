import xml.etree.ElementTree as ET
import numpy as np
import os
import sys

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

def make_homogeneous_matrix(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_rotation_matrix(*rpy)
    T[:3, 3] = xyz
    return T

def parse_urdf_spheres(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    # 1. Parse joints to build parent-child relations and static transforms
    parent_map = {}
    active_joints = ['base_yaw_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint']
    
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
        
        T_static = make_homogeneous_matrix(xyz, rpy)
        parent_map[child] = (parent, jtype, name, T_static)
        
    # 2. Parse links to get collision spheres
    link_spheres = {}
    for link in root.findall('link'):
        lname = link.get('name')
        spheres = []
        for col in link.findall('collision'):
            origin = col.find('origin')
            xyz_str = origin.get('xyz') if origin is not None else "0 0 0"
            rpy_str = origin.get('rpy') if origin is not None else "0 0 0"
            xyz = [float(x) for x in xyz_str.split()]
            rpy = [float(x) for x in rpy_str.split()]
            
            geom = col.find('geometry')
            if geom is not None:
                sphere = geom.find('sphere')
                if sphere is not None:
                    r = float(sphere.get('radius'))
                    spheres.append((np.array(xyz), r))
        if spheres:
            link_spheres[lname] = spheres
            
    # 3. Classify links into segments and compute static transforms to active joint child frames
    segment_spheres = {0: [], 1: [], 2: [], 3: []}
    
    segment_roots = {
        1: None, # child of base_yaw_joint
        2: None, # child of shoulder_pitch_joint
        3: None, # child of elbow_pitch_joint
    }
    
    # Find child link of active joints
    for child, (parent, jtype, jname, T_static) in parent_map.items():
        if jname == 'base_yaw_joint':
            segment_roots[1] = child
        elif jname == 'shoulder_pitch_joint':
            segment_roots[2] = child
        elif jname == 'elbow_pitch_joint':
            segment_roots[3] = child
            
    print(f"Segment roots: {segment_roots}")
    
    for lname, spheres in link_spheres.items():
        # Trace path to root
        path = []
        curr = lname
        while curr in parent_map:
            parent, jtype, jname, T_static = parent_map[curr]
            path.append((curr, parent, jtype, jname, T_static))
            curr = parent
            
        # Determine segment
        jnames = [p[3] for p in path]
        if 'elbow_pitch_joint' in jnames:
            seg = 3
            root_link = segment_roots[3]
        elif 'shoulder_pitch_joint' in jnames:
            seg = 2
            root_link = segment_roots[2]
        elif 'base_yaw_joint' in jnames:
            seg = 1
            root_link = segment_roots[1]
        else:
            seg = 0
            root_link = 'world'
            
        # Compute static transform to root_link
        T_accum = np.eye(4)
        curr = lname
        while curr != root_link and curr in parent_map:
            parent, jtype, jname, T_static = parent_map[curr]
            T_accum = T_static @ T_accum
            curr = parent
            
        # Transform spheres to Segment Root Frame
        for local_center, r in spheres:
            c_h = np.ones(4)
            c_h[:3] = local_center
            c_transformed = (T_accum @ c_h)[:3]
            segment_spheres[seg].append((c_transformed, r))
            
    # Print links and sphere counts for each segment
    seg_links = {0: {}, 1: {}, 2: {}, 3: {}}
    for lname, spheres in link_spheres.items():
        # Trace path to root
        path = []
        curr = lname
        while curr in parent_map:
            parent, jtype, jname, T_static = parent_map[curr]
            path.append((curr, parent, jtype, jname, T_static))
            curr = parent
            
        jnames = [p[3] for p in path]
        if 'elbow_pitch_joint' in jnames:
            seg = 3
        elif 'shoulder_pitch_joint' in jnames:
            seg = 2
        elif 'base_yaw_joint' in jnames:
            seg = 1
        else:
            seg = 0
        seg_links[seg][lname] = len(spheres)
        
    for seg, s_list in segment_spheres.items():
        print(f"Segment {seg}: {len(s_list)} spheres total")
        # Print all links in this segment
        sorted_links = sorted(seg_links[seg].items(), key=lambda x: x[1], reverse=True)
        print(f"  Links: {sorted_links}")
        
    def thin_spheres(spheres, min_dist=0.015):
        thinned = []
        # Sort by radius descending to keep larger spheres
        for c, r in sorted(spheres, key=lambda x: x[1], reverse=True):
            too_close = False
            for tc, tr in thinned:
                if np.linalg.norm(c - tc) < min_dist:
                    too_close = True
                    break
            if not too_close:
                thinned.append((c, r))
        return thinned

    thinned_spheres = {}
    for seg, s_list in segment_spheres.items():
        thinned_spheres[seg] = thin_spheres(s_list, min_dist=0.015)
        print(f"Segment {seg} thinned (min_dist=1.5cm): {len(thinned_spheres[seg])} spheres (down from {len(s_list)})")

    # Benchmark vectorized check with thinned spheres
    if len(thinned_spheres[1]) > 0 and len(thinned_spheres[3]) > 0:
        import time
        c1 = np.array([s[0] for s in thinned_spheres[1]])
        r1 = np.array([s[1] for s in thinned_spheres[1]])
        c3 = np.array([s[0] for s in thinned_spheres[3]])
        r3 = np.array([s[1] for s in thinned_spheres[3]])
        
        t0 = time.perf_counter()
        diff = c1[:, None, :] - c3[None, :, :]
        dists_sq = np.sum(diff**2, axis=-1)
        radii_sum = r1[:, None] + r3[None, :]
        collision = np.any(dists_sq <= radii_sum**2)
        t1 = time.perf_counter()
        print(f"Thinned vectorized check took {(t1 - t0)*1000:.3f} ms. Collision: {collision}")
        
    return segment_spheres

if __name__ == '__main__':
    urdf = "/home/ros_ws/src/robots/community_robot_arm/urdf/spherized/community_robot_arm_slim_spherized.urdf"
    parse_urdf_spheres(urdf)
