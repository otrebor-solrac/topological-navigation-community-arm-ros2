#!/usr/bin/env python3
import os
import sys
import numpy as np
import yaml

# Set up paths
WORKSPACE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../'))
sys.path.append(os.path.join(WORKSPACE_DIR, 'src/whitebox_motion_planners'))

# Import the UrdfCollisionParser
from whitebox_motion_planners.collision.urdf_collision_parser import UrdfCollisionParser

def main():
    # 1. Load planner params for offsets
    params_path = os.path.join(WORKSPACE_DIR, 'src/whitebox_motion_planners/config/planner_params.yaml')
    with open(params_path, 'r') as f:
        config = yaml.safe_load(f)
    
    ros_params = config.get('/**', {}).get('ros__parameters', {})
    offsets_deg = ros_params.get('joint_offsets', {})
    
    # Home joint angles (offsets in radians)
    import math
    q1 = math.radians(offsets_deg.get('base_yaw', 32.0694))
    q2 = math.radians(offsets_deg.get('shoulder_pitch', 90.0))
    q3 = math.radians(offsets_deg.get('elbow_pitch', 0.0))
    
    print(f"Plotting robot at Home Pose (radians): q1={q1:.4f}, q2={q2:.4f}, q3={q3:.4f}")
    
    # 2. Initialize the URDF Parser
    urdf_path = os.path.join(WORKSPACE_DIR, 'src/robots/community_robot_arm/urdf/spherized/community_robot_arm_slim_spherized.urdf')
    parser = UrdfCollisionParser(
        urdf_path=urdf_path,
        offset_base_yaw=q1,
        offset_shoulder_pitch=q2,
        offset_elbow_pitch=q3
    )
    
    # 4. Extract all original spheres of the base link in local coordinates
    sphere_coords = []
    sphere_radii = []
    sphere_colors = []
    
    # Get all original (non-thinned) spheres of the base ring link
    base_spheres = parser.links_with_spheres.get('robot_belt_arm_basering', [])
    print(f"Found {len(base_spheres)} original spheres for 'robot_belt_arm_basering'")
    
    for local_c, r in base_spheres:
        # Use raw local coordinates directly as defined in the URDF link
        sphere_coords.append(local_c)
        sphere_radii.append(r)
        sphere_colors.append('#e67e22') # Orange for base frame
            
    sphere_coords = np.array(sphere_coords)
    
    # 5. Set up Matplotlib 3D plotting
    import matplotlib
    has_display = True
    if 'DISPLAY' not in os.environ:
        matplotlib.use('Agg')
        has_display = False
        print("No display detected. Script will save the plot as a PNG image instead of opening a window.")
        
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the spheres as a 3D scatter plot
    # Size represents sphere radius (multiplied for visibility)
    sizes = np.array(sphere_radii) * 3000
    scatter = ax.scatter(
        sphere_coords[:, 0],
        sphere_coords[:, 1],
        sphere_coords[:, 2],
        s=sizes,
        c=sphere_colors,
        alpha=0.4,
        edgecolors='#d35400',
        linewidths=0.5
    )
    
    # Plot the Local Origin (0,0,0) of the base link
    ax.scatter([0], [0], [0], color='red', marker='*', s=300, label='Local Origin (0,0,0)')
    
    # Draw reference axes arrows from origin
    axis_len = 0.05 # 5 cm
    ax.quiver(0, 0, 0, axis_len, 0, 0, color='r', arrow_length_ratio=0.2, label='Local X Axis (Red)')
    ax.quiver(0, 0, 0, 0, axis_len, 0, 0, color='g', arrow_length_ratio=0.2, label='Local Y Axis (Green)')
    ax.quiver(0, 0, 0, 0, 0, axis_len, color='b', arrow_length_ratio=0.2, label='Local Z Axis (Blue)')
    
    # Set labels and title
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_zlabel('Z (meters)')
    ax.set_title('3D Base Ring (robot_belt_arm_basering) Spheres in Local Coordinates')
    
    # Adjust axis scaling to focus tightly on the base ring locally
    ax.set_xlim(-0.15, 0.15)
    ax.set_ylim(-0.15, 0.15)
    ax.set_zlim(-0.15, 0.15)
    
    # Grid and view adjustments
    ax.view_init(elev=20, azim=45)
    
    # Legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#e67e22', markersize=10, label='Base Ring Spheres (robot_belt_arm_basering)'),
        Line2D([0], [0], marker='*', color='r', linestyle='None', markersize=12, label='World Origin (0,0,0)')
    ]
    ax.legend(handles=legend_elements, loc='upper right')
    
    # Save image
    output_img_path = os.path.join(WORKSPACE_DIR, 'docs/collision_spheres_plot.png')
    os.makedirs(os.path.dirname(output_img_path), exist_ok=True)
    plt.savefig(output_img_path, dpi=150, bbox_inches='tight')
    print(f"Successfully saved 3D plot image to: {output_img_path}")
    
    if has_display:
        print("Opening interactive 3D window...")
        plt.show()
    else:
        print("To view the plot, open the image in your workspace: docs/collision_spheres_plot.png")

if __name__ == '__main__':
    main()
