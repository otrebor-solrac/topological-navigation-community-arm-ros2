# Plan de Implementación: Brazo Robótico Comunitario (ROS2 + Topología)

Este documento detalla los pasos técnicos para traducir la **Fase 3 (Metodología)** de la tesis a una solución funcional en ROS2 y RViz2.

---

## 🏗️ 1. Fase Actual: Consolidación del URDF (Paso 0)
*Según el cronograma, el URDF es el cimiento para mapear la variedad $T^n$ al espacio $\mathbb{R}^3$.*

- [ ] **Geometrias de Colisión:** Asegurar que cada `link` en el URDF tenga una etiqueta `<collision>` simplificada (preferiblemente cilindros y cajas).
- [ ] **Estructura Cinemática:** Finalizar el árbol de transformaciones (`tf`) para que refleje fielmente los GDL del brazo.
- [ ] **Validación:** El robot debe visualizarse correctamente en RViz2 usando el `joint_state_publisher_gui`.

---

## 🌀 2. Implementación de la Variedad Toroidal (Mayo 18 - Jun 05)
*Objetivo: Programar la lógica de "Gesto de Frontera" (Wrap-around).*

- [ ] **Clase `ToralSpace`:** Crear una clase en Python que gestione las dimensiones del Toro ($T^3$ para 3 articulaciones).
- [ ] **Aritmética Modular:** Implementar la función `distance(q1, q2)` que use el operador `min(|θ1 - θ2|, 2π - |θ1 - θ2|)` para cada eje.
- [ ] **Métricas L1/L2:** Codificar ambas métricas sobre la distancia toroidal para comparar eficiencias después.

---

## 🧊 3. Voxelización y Detección de Colisiones (Junio)
*Objetivo: Crear el mapa $\mathcal{C}_{free}$ usando el recubrimiento de bolas abiertas.*

- [ ] **Muestreo de Bolas:** Definir los centros y radios de las esferas que recubren cada eslabón (Sección 2.2.2).
- [ ] **C-Space Mapping (Voxelización):**
    - Discretizar el espacio articular en una retícula (ej. pasos de 5° o 10°).
    - Para cada celda, ejecutar cinemática directa y verificar si las "bolas" del robot colisionan con el obstáculo.
    - Marcar celdas como `SAFE` (Voxel libre) o `COLLISION` (Voxel prohibido).

---

## 🔍 4. Algoritmos de Búsqueda (Junio - Julio)
*Objetivo: Navegar la retícula toroidal.*

- [ ] **A* Toroidal:**
    - Usar la retícula de voxels como grafo de búsqueda.
    - Implementar la heurística admisible considerando el salto de frontera (0-360°).
- [ ] **RRT Estocástico:**
    - Programar el muestreo aleatorio uniforme sobre $T^n$ respetando la medida de Haar (uniformidad angular).
    - Usar la función de distancia toroidal para encontrar el "nodo más cercano".

---

## 📺 5. Visualización "Caja Blanca" en RViz2 (Julio 20 - Julio 31)
*Objetivo: Evidenciar la matemática en tiempo real.*

- [ ] **Publicador de Marcadores (`visualization_msgs`):**
    - Dibujar las **bolas de colisión** como esferas semitransparentes sobre el brazo.
    - Visualizar los **voxels prohibidos** en el espacio de trabajo.
    - Dibujar el **Árbol de RRT** o la **Ruta de A*** como líneas de colores en el espacio cartesiano.
- [ ] **Integración en Launch:** Unificar todo en un solo `ros2 launch community_robot_arm planning.launch.py`.

---

## 📐 Notas de Seguridad Cinemática (Fase 4: Análisis Diferencial)
- Integración de la **Matriz Jacobiana** para detectar singularidades.
- Penalización de los voxels que se acerquen a un determinante nulo (det(J) ≈ 0) para evitar comportamientos caóticos ("saltos" de articulación).
