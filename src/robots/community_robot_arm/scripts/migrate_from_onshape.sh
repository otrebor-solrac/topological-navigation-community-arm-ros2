#!/bin/bash
# ============================================================
# migrate_from_onshape.sh [full|slim]
#
# Migra el URDF exportado de Onshape al paquete ROS 2.
#   full  → Todas las piezas (198 mallas)
#   slim  → Solo piezas estructurales (sin motores, tornillos,
#            rondanas, poleas ni hardware menor)
# ============================================================

set -e

MODE="${1:-full}"

if [ "$MODE" != "full" ] && [ "$MODE" != "slim" ]; then
    echo "❌ Uso: $0 [full|slim]"
    echo "   full  → URDF completo con todas las piezas"
    echo "   slim  → URDF filtrado sin hardware"
    exit 1
fi

# Rutas
ONSHAPE_DIR="/home/rc/workspace/ROS2/robot-test/complete_robotic_arm_assembly_with_dual_manipulator_140mm_shank"
PKG_DIR="/home/rc/workspace/ROS2/src/robots/community_robot_arm"

ONSHAPE_URDF="$ONSHAPE_DIR/urdf/complete_robotic_arm_assembly_with_dual_manipulator_140mm_shank.urdf"
ONSHAPE_MESHES="$ONSHAPE_DIR/meshes"

TARGET_MESHES="$PKG_DIR/meshes"

echo "🔄 Migrando URDF de Onshape (modo: $MODE)..."

# 1. Verificar que el URDF fuente existe
if [ ! -f "$ONSHAPE_URDF" ]; then
    echo "❌ No se encontró el URDF de Onshape en: $ONSHAPE_URDF"
    exit 1
fi

# 2. Crear carpetas
mkdir -p "$PKG_DIR/urdf"
mkdir -p "$TARGET_MESHES"

# 3. Copiar mallas
echo "📦 Copiando mallas..."
cp "$ONSHAPE_MESHES"/*.stl "$TARGET_MESHES/"
TOTAL=$(ls "$TARGET_MESHES"/*.stl 2>/dev/null | wc -l)
echo "   ✅ $TOTAL mallas copiadas"

# 4. Convertir STLs de ASCII a binario
echo "🔧 Convirtiendo STLs a binario..."
python3 << 'PYEOF'
import os, struct

mesh_dir = "/home/rc/workspace/ROS2/src/robots/community_robot_arm/meshes"
converted = 0

for filename in sorted(os.listdir(mesh_dir)):
    if not filename.endswith('.stl'):
        continue
    path = os.path.join(mesh_dir, filename)
    try:
        with open(path, 'rb') as f:
            first_bytes = f.read(6)
        if first_bytes[:5] == b'solid':
            with open(path, 'r', errors='ignore') as f:
                content = f.read()
            if 'endsolid' in content:
                lines = content.split('\n')
                facets = []
                i = 0
                while i < len(lines):
                    line = lines[i].strip()
                    if line.startswith('facet normal'):
                        try:
                            parts = line.split()
                            normal = [float(parts[2]), float(parts[3]), float(parts[4])]
                            v1 = [float(x) for x in lines[i+2].strip().split()[1:4]]
                            v2 = [float(x) for x in lines[i+3].strip().split()[1:4]]
                            v3 = [float(x) for x in lines[i+4].strip().split()[1:4]]
                            facets.append((normal, v1, v2, v3))
                        except:
                            pass
                    i += 1
                if facets:
                    with open(path, 'wb') as f:
                        f.write(b'\x00' * 80)
                        f.write(struct.pack('<I', len(facets)))
                        for normal, v1, v2, v3 in facets:
                            f.write(struct.pack('<fff', *normal))
                            f.write(struct.pack('<fff', *v1))
                            f.write(struct.pack('<fff', *v2))
                            f.write(struct.pack('<fff', *v3))
                            f.write(struct.pack('<H', 0))
                    converted += 1
    except Exception as e:
        print(f"  ⚠️  Error en {filename}: {e}")

print(f"   ✅ {converted} archivos convertidos a binario")
PYEOF

# 5. Generar el URDF
if [ "$MODE" = "full" ]; then
    echo "📄 Generando URDF completo..."
    TARGET_URDF="$PKG_DIR/urdf/community_robot_arm_full.urdf"
    cp "$ONSHAPE_URDF" "$TARGET_URDF"
    sed -i 's|package://complete_robotic_arm_assembly_with_dual_manipulator_140mm_shank/meshes/|package://community_robot_arm/meshes/|g' "$TARGET_URDF"
    echo "   ✅ URDF completo generado"

elif [ "$MODE" = "slim" ]; then
    echo "🔪 Generando URDF slim (filtrando hardware)..."
    TARGET_URDF="$PKG_DIR/urdf/community_robot_arm_slim.urdf"

    python3 << 'PYEOF2'
import xml.etree.ElementTree as ET

src = "/home/rc/workspace/ROS2/robot-test/complete_robotic_arm_assembly_with_dual_manipulator_140mm_shank/urdf/complete_robotic_arm_assembly_with_dual_manipulator_140mm_shank.urdf"
dst = "/home/rc/workspace/ROS2/src/robots/community_robot_arm/urdf/community_robot_arm_slim.urdf"

tree = ET.parse(src)
root = tree.getroot()

# --- Palabras clave de HARDWARE a eliminar ---
hardware_keywords = ['NEMA', 'GT2']

def is_hardware(name):
    return any(kw.lower() in name.lower() for kw in hardware_keywords)

# --- Construir el árbol cinemático ---
# child_to_parent: para cada link hijo, quién es su padre
child_to_parent = {}
for joint in root.findall('joint'):
    p = joint.find('parent').get('link')
    c = joint.find('child').get('link')
    child_to_parent[c] = p

# --- Identificar links ESTRUCTURALES (no-hardware) ---
all_links = set(link.get('name') for link in root.findall('link'))
structural_links = set(name for name in all_links if not is_hardware(name))

# --- Proteger caminos: cada link estructural protege su cadena hasta root ---
protected = set()
for name in structural_links:
    current = name
    while current and current not in protected:
        protected.add(current)
        current = child_to_parent.get(current)

# --- Links a eliminar = todos los que NO están protegidos ---
links_to_remove = all_links - protected

print(f"   🔒 {len(protected)} links protegidos (estructurales + caminos)")
print(f"   🗑️  Eliminando {len(links_to_remove)} links de hardware:")
for name in sorted(links_to_remove)[:10]:
    print(f"      - {name}")
if len(links_to_remove) > 10:
    print(f"      ... y {len(links_to_remove) - 10} más")

# --- Eliminar links ---
for link in root.findall('link'):
    if link.get('name') in links_to_remove:
        root.remove(link)

# --- Eliminar joints que conectan a links eliminados ---
joints_removed = 0
for joint in root.findall('joint'):
    parent = joint.find('parent').get('link')
    child = joint.find('child').get('link')
    if parent in links_to_remove or child in links_to_remove:
        root.remove(joint)
        joints_removed += 1

print(f"   🗑️  Eliminados {joints_removed} joints asociados")

# --- Filtrar tornillos/rondanas del link root ---
# Los tornillos son visuals cuya malla STL se repite 3+ veces
from collections import Counter
for link in root.findall('link'):
    if link.get('name') == 'root':
        visuals = link.findall('visual')
        # Contar cuántas veces se usa cada malla
        mesh_count = Counter()
        for vis in visuals:
            mesh = vis.find('.//mesh')
            if mesh is not None:
                fn = mesh.get('filename', '').split('/')[-1]
                mesh_count[fn] += 1
        
        # Mallas repetidas 3+ veces = hardware (tornillos, rondanas, etc.)
        hardware_meshes = {fn for fn, count in mesh_count.items() if count >= 3}
        
        removed_visuals = 0
        for vis in visuals:
            mesh = vis.find('.//mesh')
            if mesh is not None:
                fn = mesh.get('filename', '').split('/')[-1]
                if fn in hardware_meshes:
                    link.remove(vis)
                    removed_visuals += 1
        
        kept = len(link.findall('visual'))
        print(f"   🔩 Root: eliminadas {removed_visuals} visuals de tornillería ({len(hardware_meshes)} tipos de malla)")
        print(f"   🔩 Root: quedan {kept} visuals estructurales")
        break

# --- 5. Joints Móviles y Mimic (Paralelogramo) ---
active_joints_names = ['Revolute 1_0', 'Revolute 9_0', 'Revolute 10_0']

# Joints del mecanismo de paralelogramo que necesitan moverse libremente
parallelogram_joints = [
    'Revolute 11_0',   # gear_body → lever
    'Revolute 12_0',   # lever → pleuel_140
    'Revolute 13_0',   # pleuel_bend_140 → triplate
    'Revolute 14_0',   # main_body → pleuel_bend_140_Mirrored
    'Revolute 15_0',   # triplate → pleuel_bend_140_Mirrored_1
    'Revolute 16_0',   # lower_shank → upper_shank  ← ya lo tenías
    'Revolute 18_0',   # pleuel_bend_140_Mirrored → triplate_dual
    'Revolute 19_0',   # triplate_dual → pleuel_bend_140_1
    'Revolute 21_0',   # upper_shank → manipulator_dual
]

for joint in root.findall('joint'):
    name = joint.get('name')
    jtype = joint.get('type')

    if name in active_joints_names:
        continue  # estos los mueve el GUI

    if name in parallelogram_joints:
        # Deben ser continuous para que el nodo pueda publicar su valor
        joint.set('type', 'continuous')
        for old_mimic in joint.findall('mimic'):
            joint.remove(old_mimic)

    elif jtype in ['revolute', 'continuous', 'prismatic']:
        joint.set('type', 'fixed')

# --- 6. Actualizar rutas de mallas al paquete local ---
for mesh in root.findall('.//mesh'):
    fn = mesh.get('filename')
    if fn:
        mesh.set('filename', fn.replace(
            'package://complete_robotic_arm_assembly_with_dual_manipulator_140mm_shank/meshes/',
            'package://community_robot_arm/meshes/'
        ))

# --- 7. Resumen y Guardado ---
remaining_links = len(root.findall('link'))
remaining_joints = len(root.findall('joint'))
active_joints = len([j for j in root.findall('joint') if j.get('type') != 'fixed'])
print(f"   ✅ URDF slim: {remaining_links} links, {remaining_joints} joints ({active_joints} activos)")

tree.write(dst, xml_declaration=True, encoding='unicode')
PYEOF2

fi

echo ""
echo "🎉 Migración completada exitosamente! (modo: $MODE)"
echo "   URDF: $TARGET_URDF"
echo "   Meshes: $TARGET_MESHES/"
echo ""
echo "Para lanzar en RViz:"
echo "   ./test_community_arm.sh"
