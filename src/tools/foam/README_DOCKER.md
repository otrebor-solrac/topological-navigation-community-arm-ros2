# Guía Operativa de FOAM (Docker Edition)

Este módulo es una versión depurada y optimizada de la herramienta FOAM, adaptada específicamente para el flujo de trabajo de esta tesis. 

> [!NOTE]
> El código original y la documentación completa de la herramienta pueden encontrarse en el repositorio oficial de CoMMALab: [https://github.com/CoMMALab/foam](https://github.com/CoMMALab/foam). Esta implementación local ha sido simplificada para enfocarse en la compatibilidad con ROS 2 y la esferización jerárquica de manipuladores.

## 1. Inicialización (Build)
Antes de usar la herramienta por primera vez, debes construir la imagen. Asegúrate de estar en `src/tools/foam/`:

```bash
docker build -t foam-v2 -f Dockerfile.light .
```

## 2. Flujo de Datos (Input/Output)

FOAM funciona mapeando tus carpetas locales dentro del contenedor mediante volúmenes (`-v`).

*   **Input (Entrada):** Un archivo URDF exportado (ej. desde Onshape) que referencia mallas STL.
*   **Meshes (Mallas):** La carpeta que contiene los archivos `.stl` referenciados.
*   **Output (Salida):** La carpeta donde quieres que se guarde el nuevo archivo `_spherized.urdf`.

## 3. Cómo Correr el Contenedor (El Comando Maestro)

Para procesar tu robot, usa el siguiente esquema de comando:

```bash
docker run --rm --name foam_run \
  -v $(pwd)/src/robots/community_robot_arm/urdf/processed:/input \
  -v $(pwd)/src/robots/community_robot_arm/meshes:/meshes \
  -v $(pwd)/src/robots/community_robot_arm/urdf/spherized:/output \
  foam-v2 /input/community_robot_arm_with_collisions.urdf \
  --output /output/community_robot_arm_slim_spherized-v2.urdf \
  --ros_package community_robot_arm \
  --depth 2
```

### Desglose del comando:
*   `--rm`: Borra el contenedor al terminar (mantiene limpio tu Docker).
*   `-v ...:/input`: Mapea tu carpeta de URDFs originales.
*   `-v ...:/meshes`: Mapea tu carpeta de mallas (necesario para que FOAM "vea" el volumen de las piezas).
*   `-v ...:/output`: Mapea el destino.
*   `--ros_package`: **[CRÍTICO]** Al poner el nombre de tu paquete ROS, FOAM reescribe las rutas para que RViz las entienda.

## 4. ¿Qué hace FOAM internamente?
1.  **Carga:** Lee el URDF y localiza las mallas en `/meshes`.
2.  **Esferización:** Calcula un conjunto de esferas que cubren el volumen de cada link según el `--depth`.
3.  **Reemplazo:** Quita las etiquetas `<collision>` originales (mallas pesadas) y las reemplaza por decenas de etiquetas `<sphere>` (muy ligeras).
4.  **Corrección de Rutas:** Cambia las rutas de `/visual` a `package://nombre_paquete/meshes/`.
5.  **Guardado:** Exporta el nuevo URDF a la carpeta `/output`.
