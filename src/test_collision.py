import os
import math
import sys
import numpy as np

# Add python path to package
sys.path.insert(0, '/home/ros_ws/src/whitebox_motion_planners')

from whitebox_motion_planners.collision.foam_collider import FoamCollider

def get_ee_pos_from_world(q_world, collider, base_yaw_offset, shoulder_pitch_offset, elbow_pitch_offset, base_yaw_dir, shoulder_pitch_dir, elbow_pitch_dir):
    yaw_w, pitch1_w, pitch2_w = q_world[:3] if len(q_world) >= 3 else (q_world[0], q_world[1], 0.0)
    base_yaw = base_yaw_offset + base_yaw_dir * yaw_w
    shoulder_pitch = shoulder_pitch_offset + shoulder_pitch_dir * pitch1_w
    elbow_pitch = elbow_pitch_offset + elbow_pitch_dir * pitch2_w
    q_urdf = (base_yaw, shoulder_pitch, elbow_pitch)
    return collider.urdf_parser.get_end_effector_position(q_urdf)

def compute_numerical_jacobian(q_world, collider, base_yaw_offset, shoulder_pitch_offset, elbow_pitch_offset, base_yaw_dir, shoulder_pitch_dir, elbow_pitch_dir, epsilon=1e-5):
    J = []
    # We perturb q_world components
    dof = len(q_world)
    for i in range(dof):
        q_plus = list(q_world)
        q_plus[i] += epsilon
        p_plus = get_ee_pos_from_world(q_plus, collider, base_yaw_offset, shoulder_pitch_offset, elbow_pitch_offset, base_yaw_dir, shoulder_pitch_dir, elbow_pitch_dir)
        
        q_minus = list(q_world)
        q_minus[i] -= epsilon
        p_minus = get_ee_pos_from_world(q_minus, collider, base_yaw_offset, shoulder_pitch_offset, elbow_pitch_offset, base_yaw_dir, shoulder_pitch_dir, elbow_pitch_dir)
        
        col = (p_plus - p_minus) / (2.0 * epsilon)
        J.append(col)
        
    return np.column_stack(J)

def main():
    urdf_path = "/home/ros_ws/install/community_robot_arm/share/community_robot_arm/urdf/spherized/community_robot_arm_slim_spherized.urdf"
    collider = FoamCollider(
        urdf_path=urdf_path,
        sphere_thinning_dist=0.015
    )
    
    # Offsets and dirs
    base_yaw_offset = math.radians(32.0694)
    shoulder_pitch_offset = math.radians(90.0)
    elbow_pitch_offset = math.radians(0.0)
    base_yaw_dir = -1.0
    shoulder_pitch_dir = -1.0
    elbow_pitch_dir = 1.0
    
    collider.set_joint_transforms(
        offset_base_yaw=base_yaw_offset,
        offset_shoulder_pitch=shoulder_pitch_offset,
        offset_elbow_pitch=elbow_pitch_offset,
        dir_base_yaw=base_yaw_dir,
        dir_shoulder_pitch=shoulder_pitch_dir,
        dir_elbow_pitch=elbow_pitch_dir
    )
    
    # Home position in World coordinates (degrees)
    q_deg = (0.0, 90.0, 0.0)
    q_rad = tuple(math.radians(x) for x in q_deg)
    
    print(f"Testing state in degrees: {q_deg}")
    print(f"Testing state in radians: {q_rad}")
    
    # Compute EE position
    p_ee = get_ee_pos_from_world(q_rad, collider, base_yaw_offset, shoulder_pitch_offset, elbow_pitch_offset, base_yaw_dir, shoulder_pitch_dir, elbow_pitch_dir)
    print(f"EE Position: {p_ee}")
    
    # Compute numerical Jacobian w.r.t q_world
    J = compute_numerical_jacobian(q_rad, collider, base_yaw_offset, shoulder_pitch_offset, elbow_pitch_offset, base_yaw_dir, shoulder_pitch_dir, elbow_pitch_dir)
    print("Numerical Jacobian J:")
    print(J)
    
    # Compute manipulability
    m, n = J.shape
    if m <= n:
        w = float(np.sqrt(max(0.0, np.linalg.det(J @ J.T))))
    else:
        w = float(np.sqrt(max(0.0, np.linalg.det(J.T @ J))))
        
    print(f"Manipulability: {w:.6f}")

if __name__ == '__main__':
    main()
