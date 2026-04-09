#!/usr/bin/env python3
import os
import shutil
import struct
import argparse
import logging
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

# --- CONFIGURACIÓN ---
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)

class URDFMigrationTool:
    def __init__(self, mode='slim'):
        self.mode = mode
        
        # Rutas dinámicas basadas en la ubicación del script
        self.script_dir = Path(__file__).resolve().parent
        self.ws_dir = self.script_dir.parents[3] # /home/rc/workspace/ROS2
        
        self.pkg_dir = self.script_dir.parent    # .../community_robot_arm
        self.dst_urdf_dir = self.pkg_dir / "urdf"
        self.dst_mesh_dir = self.pkg_dir / "meshes"

        # Ruta del exportador original (Onshape)
        self.onshape_dir = self.ws_dir / "robot-test/complete_robotic_arm_assembly_with_dual_manipulator_140mm_shank"
        self.src_meshes = self.onshape_dir / "meshes"
        self.src_urdf = self.onshape_dir / "urdf/complete_robotic_arm_assembly_with_dual_manipulator_140mm_shank.urdf"
        
        # Filtros y Juntas
        self.hardware_keywords = ['NEMA', 'GT2']
        self.active_joints = ['Revolute 1_0', 'Revolute 9_0', 'Revolute 10_0']
        self.parallelogram_joints = [
            'Revolute 11_0',   # gear_body → lever
            'Revolute 12_0',   # lever → pleuel_140
            'Revolute 13_0',   # pleuel_bend_140 → triplate
            'Revolute 15_0',   # triplate → pleuel_bend_140_Mirrored_1
            'Revolute 16_0',   # lower_shank → upper_shank
            'Revolute 18_0',   # pleuel_bend_140_Mirrored → triplate_dual
            'Revolute 19_0',   # triplate_dual → pleuel_bend_140_1
            'Revolute 21_0',   # upper_shank → manipulator_dual
        ]
        
        # Prefijos de paquete para mallas
        self.old_pkg_prefix = "package://complete_robotic_arm_assembly_with_dual_manipulator_140mm_shank/meshes/"
        self.new_pkg_prefix = "package://community_robot_arm/meshes/"

    def run(self):
        """
        Execute the migration pipeline.
        """
        logger.info(f"Starting URDF migration (Mode: {self.mode.upper()})")
        
        try:
            # 0. Validate all paths exist
            if not self.src_urdf.exists():
                raise FileNotFoundError(f"❌ URDF not found at {self.src_urdf}")
            self.dst_urdf_dir.mkdir(parents=True, exist_ok=True)
            self.dst_mesh_dir.mkdir(parents=True, exist_ok=True)

            # 1. Load the original URDF ONCE
            tree = ET.parse(self.src_urdf)
            root = tree.getroot()

            # 2. Map and copy meshes
            self._process_assets(root)

            # 3. Structural logic (Slim mode)
            if self.mode == 'slim':
                self._apply_slim_logic(root)

            # 4. Actualizar rutas de mallas
            self._update_mesh_paths(root)

            # 5. Guardar resultado
            output_file = self.dst_urdf_dir / f"community_robot_arm_{self.mode}.urdf"
            tree.write(output_file, xml_declaration=True, encoding='utf-8')
            logger.info("Migration process completed successfully.")
            logger.info(f"📄 URDF saved in: {output_file}")
        
        except Exception as e:
            logger.error(f"❌ Error in migration: {e}")
            import traceback
            traceback.print_exc()

    def _ascii_to_binary(self, path):
        try:
            with open(path, 'rb') as f:
                if f.read(5) != b'solid': return False
            with open(path, 'r', errors='ignore') as f:
                content = f.readlines()
            facets = []
            for i, line in enumerate(content):
                if line.strip().startswith('facet normal'):
                    n = [float(x) for x in line.split()[2:5]]
                    v1 = [float(x) for x in content[i+2].split()[1:4]]
                    v2 = [float(x) for x in content[i+3].split()[1:4]]
                    v3 = [float(x) for x in content[i+4].split()[1:4]]
                    facets.append((n, v1, v2, v3))
            if facets:
                with open(path, 'wb') as f:
                    f.write(b'\x00' * 80)
                    f.write(struct.pack('<I', len(facets)))
                    for n, v1, v2, v3 in facets:
                        f.write(struct.pack('<fff', *n))
                        for v in [v1, v2, v3]: f.write(struct.pack('<fff', *v))
                        f.write(struct.pack('<H', 0))
                return True
        except: return False
        return False

    def _apply_slim_logic(self, root):
        parent_map = {
            j.find('child').get('link'): j.find('parent').get('link') 
            for j in root.findall('joint')
        }
        all_links = {l.get('name') for l in root.findall('link')}
        structural = {
            n for n in all_links if not any(kw.lower() in n.lower() for kw in self.hardware_keywords)
        }
        protected = set()
        for name in structural:
            curr = name
            while curr and curr not in protected:
                protected.add(curr)
                curr = parent_map.get(curr)
        to_remove = all_links - protected
        
        for link in root.findall('link'):
            if link.get('name') in to_remove: root.remove(link)
        for joint in root.findall('joint'):
            if joint.find('parent').get('link') in to_remove or \
               joint.find('child').get('link') in to_remove:
                root.remove(joint)

        # Limpiar ROOT
        root_link = root.find(".//link[@name='root']")
        if root_link is not None:
            visuals = root_link.findall('visual')
            if len(visuals) > 0:
                print(f"[*] Root link purgado: {len(visuals)} balines y tornillos eliminados.")
                for v in visuals: root_link.remove(v)

        # Ajuste de Juntas
        for joint in root.findall('joint'):
            name = joint.get('name')
            
            # Parche para error de asimetría en exportación desde OnShape (la 15 es la correcta, la 19 está mal)
            if name == 'revolute_19_0':
                origin = joint.find('origin')
                if origin is not None:
                    # Invertimos la Y (-0.0035) y mantenemos el pitch 1.23027 de revolute_15_0
                    origin.set('xyz', "0.045 -0.0035 0.008")
                    origin.set('rpy', "0 1.23027 -3.14159")

            # Centrar la base del robot al origen (X/Y=centro, Z=piso)
            if joint.find('parent') is not None and joint.find('parent').get('link') == 'root':
                origin = joint.find('origin')
                if origin is not None:
                    origin.set('xyz', "-0.101239 0.049005 0.0677")
                    origin.set('rpy', "0 0 0")

            if name in self.parallelogram_joints or name.capitalize() in self.parallelogram_joints or name.replace("r", "R") in self.parallelogram_joints:
                joint.set('type', 'continuous')
                for m in joint.findall('mimic'): joint.remove(m)

    def _get_mesh_mapping(self, root):
        """
        Get the mesh mapping from the URDF tree.
        
        :param root: The root element of the URDF tree.
        :return: A dictionary mapping the mesh filename to the link name.
        """
        mapping = {}
        for link in root.findall('link'):
            link_name = link.get('name')
            mesh = link.find('.//mesh')
            
            if mesh is not None:
                filename = mesh.get('filename')
                if filename:
                    stl_name = filename.split('/')[-1]
                    if stl_name.endswith('.stl'):
                        # robot_belt_arm main_body -> main_body.stl
                        normalized = link_name.split('_')
                        normalized = "_".join(normalized)
                        normalized = normalized.replace(' ', '_')
                        mapping[stl_name] = f"{normalized}.stl"
        return mapping

    def _process_assets(self, root):
        """
        Rename meshes semantically and copy them to the target directory.
        
        :param root: The root element of the URDF tree.
        """
        logger.info("📦 Renaming meshes semantically...")
        self.mesh_map = self._get_mesh_mapping(root)
        self.dst_mesh_dir.mkdir(parents=True, exist_ok=True)
        stls_onshape = list(self.src_meshes.glob("*.stl"))
        
        converted = 0
        for stl in stls_onshape:
            new_name = self.mesh_map.get(stl.name, stl.name)
            target = self.dst_mesh_dir / new_name
            shutil.copy2(stl, target)
            if self._ascii_to_binary(target): converted += 1
        logger.info(f"✅ {len(stls_onshape)} mallas procesadas.")

    def _update_mesh_paths(self, root):
        old_pkg = 'package://complete_robotic_arm_assembly_with_dual_manipulator_140mm_shank/'
        new_pkg = 'package://community_robot_arm/'
        for mesh in root.findall('.//mesh'):
            fn = mesh.get('filename')
            if fn:
                new_fn = fn.replace(old_pkg, new_pkg)
                new_fn = new_fn.replace('package://meshes/', new_pkg + 'meshes/')
                stl_name = new_fn.split('/')[-1]
                if hasattr(self, 'mesh_map') and stl_name in self.mesh_map:
                    new_fn = new_fn.replace(stl_name, self.mesh_map[stl_name])
                mesh.set('filename', new_fn)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['full', 'slim'], default='slim')
    args = parser.parse_args()
    URDFMigrationTool(mode=args.mode).run()