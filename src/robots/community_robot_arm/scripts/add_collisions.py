"""
This script patches an URDF file to add collision geometries and correct mesh paths.

Usage: 
    python3 add_collisions.py input.urdf output.urdf

"""

import xml.etree.ElementTree as ET
import sys
import os
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

def patch_urdf(input_file, output_file):
    """
    Patch an URDF file to add collision geometries and correct mesh paths.
    
    Args:
        input_file (str): Path to the input URDF file.
        output_file (str): Path to the output URDF file.
    """

    logging.info(f"Processing {input_file}...")
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    count_coll = 0
    count_path = 0
    
    # Iterate over all links
    for link in root.findall('link'):
        visual = link.find('visual')
        collision = link.find('collision')
        
        # Add collisions if missing
        if visual is not None and collision is None:
            new_collision = ET.Element('collision')
            origin = visual.find('origin')
            
            # Copy origin from visual to collision
            if origin is not None:
                new_collision.append(ET.fromstring(ET.tostring(origin)))
            
            geometry = visual.find('geometry')
            
            # Copy geometry from visual to collision
            if geometry is not None:
                new_collision.append(ET.fromstring(ET.tostring(geometry)))
            
            link.append(new_collision)
            count_coll += 1

    # Correct mesh paths (package://community_robot_arm/ -> ../../)
    # This is so that FOAM can find the meshes relative to the URDF in urdf/processed/
    for mesh in root.iter('mesh'):
        filename = mesh.get('filename')

        # If filename starts with package://community_robot_arm/, replace it with ../../
        if filename and filename.startswith('package://community_robot_arm/'):
            new_filename = filename.replace('package://community_robot_arm/', '../../')
            mesh.set('filename', new_filename)
            count_path += 1
            
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    logging.info(f"Done! Added {count_coll} collisions and patched {count_path} paths.")
    logging.info(f"Saved to {output_file}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        logging.error("Usage: python3 add_collisions.py input.urdf output.urdf")
    else:
        patch_urdf(sys.argv[1], sys.argv[2])
