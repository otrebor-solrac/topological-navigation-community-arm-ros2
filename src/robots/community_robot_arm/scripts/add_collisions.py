import xml.etree.ElementTree as ET
import sys
import os

def patch_urdf(input_file, output_file):
    print(f"Processing {input_file}...")
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    count_coll = 0
    count_path = 0
    
    for link in root.findall('link'):
        visual = link.find('visual')
        collision = link.find('collision')
        
        # 1. Añadir colisiones si faltan
        if visual is not None and collision is None:
            new_collision = ET.Element('collision')
            origin = visual.find('origin')
            if origin is not None:
                new_collision.append(ET.fromstring(ET.tostring(origin)))
            geometry = visual.find('geometry')
            if geometry is not None:
                new_collision.append(ET.fromstring(ET.tostring(geometry)))
            link.append(new_collision)
            count_coll += 1

    # 2. Corregir rutas de mallas (package://community_robot_arm/ -> ../)
    # Esto es para que FOAM pueda encontrar las mallas relativas al URDF que está en urdf/
    for mesh in root.iter('mesh'):
        filename = mesh.get('filename')
        if filename and filename.startswith('package://community_robot_arm/'):
            new_filename = filename.replace('package://community_robot_arm/', '../')
            mesh.set('filename', new_filename)
            count_path += 1
            
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    print(f"Done! Added {count_coll} collisions and patched {count_path} paths.")
    print(f"Saved to {output_file}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 add_collisions.py input.urdf output.urdf")
    else:
        patch_urdf(sys.argv[1], sys.argv[2])
