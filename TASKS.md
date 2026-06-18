# Plan de Implementación: Brazo Robótico Comunitario (ROS2 + Topología)

Este documento detalla los pasos técnicos para traducir la **Fase 3 (Metodología)** de la tesis a una solución funcional en ROS2 y RViz2.

---

## ✅ 1. Fase 0: Infraestructura y URDF (COMPLETADO)
- [x] **Estructura Cinemática:** Finalizado el árbol de transformaciones (`tf`) con el script de paralelogramo.
- [x] **Validación RViz:** El robot se visualiza completo (corregido el error de fragmentación).
- [x] **Single Source of Truth:** Parámetros centralizados en `planner_params.yaml`.
- [x] **Workflow Profesional:** Makefile implementado con `make run`, `make shell` y `make build`.
- [x] **Symlink Install:** Cambios en Python se ven reflejados al instante en el Docker.

---

## ✅ 2. Implementación de la Variedad Toroidal (COMPLETADO)
- [x] **Clase `TorusTopology`:** Lógica de "Gesto de Frontera" (Wrap-around) funcional en `topological_math.py`.
- [x] **Aritmética Modular:** Función `wrap_around_dist` operativa.
- [x] **Métricas L1/L2:** Implementadas en la clase `Metrics` y seleccionables por YAML.

---

## ✅ 3. Voxelización y Detección de Colisiones (COMPLETADO)
*Objetivo: Crear el mapa $\mathcal{C}_{free}$ usando el recubrimiento de bolas abiertas.*

- [x] **Muestreo de Bolas:** Definido en `FoamCollider` usando esferas.
- [x] **C-Space Mapping (Voxelización):** Discretización funcional en `GridDiscretizer`.
- [x] **Gestión de Obstáculos Externos:** 
    - [x] Permitir cargar obstáculos (centros y radios) desde el `planner_params.yaml`.
    - [x] Integrar obstáculos estáticos en la validación del `FoamCollider`.
- [x] **Optimización de Rendimiento:**
    - [x] Implementar caché de cinemática directa (memoization cache) en `get_transformed_spheres(q)` para evitar repetir cálculos de matrices y trigonometría para estados $q$ ya evaluados.
    - [x] Implementar optimización en `check_self_collision` reduciendo operaciones redundantes y usando aritmética rápida en los ciclos.
    - [x] Diseñar un sistema de caché de C-Space persistente en archivos JSON (`cspace_cache/`) indexado por resolución, thinning, e invalidado mediante MD5 hash de los obstáculos.
    - [x] Conectar la caché con el planificador (`planning_node.py`) para lograr un lookup de colisiones de complejidad O(1) en tiempo de ejecución.

---

## ✅ 4. Algoritmos de Búsqueda (COMPLETADO)
- [x] **A* Toroidal:** Heurística admisible con salto de frontera operativa.
- [x] **RRT Estocástico:** 
    - [x] Muestreo Haar (uniformidad angular) implementado en `rrt.py`.
    - [x] Steering que respeta la topología circular.
- [x] **Refinamiento RRT:**
    - [x] Implementar la lógica básica de crecimiento del árbol (Haar sampling, nearest, steering y reconstrucción de ruta).
    - [x] Implementar la validación de colisiones a lo largo de los segmentos (`_is_edge_valid`) para evitar el efecto túnel.

---

## ✅ 5. Visualización "Caja Blanca" en RViz2 (COMPLETADO)
- [x] **Publicador de Marcadores (`visualization_msgs`):**
    - [x] Dibujar las **bolas de colisión** como esferas semitransparentes sobre el brazo (con carga automática en RViz).
    - [x] Visualizar los **voxels prohibidos** (C-Space) en el Dashboard Web 3D (completado en la interfaz web).
    - [x] Dibujar la **Ruta de A* o RRT** (estela del efector final) como una línea amarilla en RViz2.

---

## ✅ 6. Fase 2: Dashboard Web "White-Box" (Control Remoto) (COMPLETADO)
*Objetivo: Llevar la visualización y control del robot al navegador para una presentación más profesional y accesible.*

- [x] **Backend de Comunicación:**
    - [x] Instalar y configurar `rosbridge_suite` en el Dockerfile.
    - [x] Exponer los puertos necesarios en el `docker-compose.yml` (se configuró en modo host).
    - [x] Actualizar el planificador para aceptar metas secuenciales ($A \to B \to C$) (procesado vía comandos JSON de `/web_commands`).
- [x] **Frontend Web:**
    - [x] Crear la aplicación base en HTML + Three.js (`dashboard/index.html` y `dashboard/app.js`).
    - [x] Integrar `roslibjs` para la comunicación por WebSockets con ROS 2.
    - [x] Implementar visualizador 3D usando `three.js` para renderizar el C-Space (voxels) y la trayectoria del robot en el espacio de configuración.
- [x] **Interfaz de Usuario (UI/UX):**
    - [x] Secuenciador de Puntos (añadir/eliminar objetivos en tiempo real desde la web).
    - [x] Selector dinámico de Algoritmo (RRT / A*) y Métrica (L1 / L2) desde la web.
    - [x] Panel de telemetría (ángulos actuales visualizados en tabla y estado de conexión de ROS).

---

## 📐 Notas de Seguridad Cinemática (COMPLETADO)
- [x] Integración de la **Matriz Jacobiana** para detectar singularidades.
- [x] Penalización de los voxels que se acerquen a un determinante nulo (det(J) ≈ 0).

---

## 📦 7. Generación de C-Space por Lotes (Automatización en Serie) (COMPLETADO)
- [x] **Configuración por Lotes (`cspace_generation.yaml`):** Definir resoluciones, thinning y lista de obstáculos (W-spaces).
- [x] **Script Automatizado (`generate_all_caches.py`):** Procesar cada combinación llamando al solver en Rust en serie.
- [x] **Comando Makefile (`make generate-caches`):** Integrar la ejecución automatizada dentro del Docker.

---

## 🚀 8. Tareas Pendientes para los Objetivos de la Tesis (Pendiente)
*Fase de desarrollo e integración final de los perfiles cinemáticos, la validación geométrica de singularidades y la suite de evaluación comparativa automatizada para los capítulos de la tesis.*

### 📈 8.1. Perfiles de Trayectoria Cinemáticos (Velocidad, Aceleración y Parametrización Temporal)
- [ ] **Generador de Perfiles de Trayectoria (`TrajectoryGenerator`):**
  - Implementar interpolación mediante Splines Quínticas ($C^2$ continuas) o Trapezoidal (LSPB) para convertir la ruta geométrica discreta en una trayectoria de tiempo continuo.
  - Definir y respetar los límites físicos de velocidad ($v_{max}$) y aceleración ($a_{max}$) para cada articulación.
- [ ] **Herramienta de Graficado de Perfiles (`plot_trajectory.py`):**
  - Crear script en Python para graficar Posición ($q_i$), Velocidad ($\dot{q}_i$) y Aceleración ($\ddot{q}_i$) vs Tiempo para cada una de las articulaciones.
  - Guardar automáticamente las curvas en formato PNG en la carpeta de figuras de la tesis.
- [ ] **Simulador de Trayectoria a Alta Frecuencia:**
  - Modificar el publicador de la animación en `planning_node.py` para correr a 50Hz (muestreo cada 20ms) interpolando en tiempo real sobre la spline generada.

### 📐 8.2. Análisis Diferencial y Evasión de Singularidades (Jacobiano)
- [ ] **Cálculo de la Matriz Jacobiana ($J(q)$):**
  - Implementar el cálculo de la matriz Jacobiana geométrica en la clase de cinemática del robot.
  - Calcular el Determinante del Jacobiano ($\det(J)$) o la medida de manipulabilidad de Yoshikawa ($w = \sqrt{\det(J J^T)}$) a lo largo de la trayectoria.
- [ ] **Filtro de Seguridad de Singularidades:**
  - Penalizar estados en el planificador que se acerquen a singularidades ($\det(J) \approx 0$).
  - Graficar el perfil de manipulabilidad a lo largo del tiempo para verificar la estabilidad de la trayectoria.

### 📊 8.3. Suite de Evaluación Comparativa (Benchmarking)
- [ ] **Script de Pruebas Automatizadas (`run_benchmarks.py`):**
  - Diseñar suite que ejecute 100 pruebas aleatorias de planificación en la variedad toroidal $T^n$.
  - Recopilar métricas de comparación para A* vs RRT:
    - Tasa de éxito (%)
    - Tiempo medio de ejecución (ms)
    - Longitud media de la ruta (rad)
    - Número de evaluaciones de colisión (FOAM)
    - Suavidad de la trayectoria (métrica de aceleración acumulada / Jerk)
  - Exportar tablas de resultados en código LaTeX directamente insertables en el Capítulo 4 (Resultados) de la tesis.

### 📐 8.4. Sistema de Referencia de Coordenadas (Absoluto vs Relativo) (COMPLETADO)
- [x] **Resolución del Sistema de Coordenadas:**
  - [x] Resolver e identificar la diferencia entre el sistema absoluto de coordenadas en RViz (marco `world` / `root`) y el sistema relativo de cada articulación y eslabón del robot.
  - [x] Definir de forma clara la correspondencia física y matemática para saber exactamente dónde colocar el robot cuando se especifican configuraciones de juntas angulares (por ejemplo, determinar a qué eje o marco de referencia local se refieren valores como `[30, 60, 120]`).
  - [x] **Alineación de estela en RViz:** Se corrigió el origen de la estela amarilla para rastrear el promedio de los centros de las esferas de colisión de las garras (`gripperfinger_by_ftobler`) en lugar de la junta de la muñeca.

---

## ✨ 9. Mejoras de Calidad y Pulido Final (Opcionales)
*Estas tareas no son requisitos para cumplir los objetivos de la tesis, pero elevan significativamente la calidad del sistema y la presentación ante el jurado. Se recomienda abordarlas después de completar la Sección 8.*

- [ ] **Refinamiento Cinemático (Parámetros DH):**
  - Actualmente las longitudes de los eslabones en `community_arm.py` son aproximaciones (`LOWER_SHANK = 0.140`, `UPPER_SHANK = 0.140`). Esta tarea consiste en medir el hardware físico del TESH y/o extraer las dimensiones exactas de los archivos CAD/URDF para que la cinemática directa sea fiel al 100%. Impacto: las gráficas de posición del efector final serán más precisas para la validación experimental del Capítulo 4.
- [ ] **Suavizado de Trayectoria (`PathSmoother`):**
  - El RRT genera rutas con "zig-zag" porque crece estocásticamente hacia muestras aleatorias. Un algoritmo de suavizado post-procesamiento recorre la ruta y prueba **atajos directos** entre nodos no consecutivos: si el segmento directo está libre de colisiones (reutilizando `_is_edge_valid`), se eliminan los nodos intermedios redundantes. Resultado: rutas visiblemente más cortas, suaves y naturales. Esto mejora tanto la animación en RViz como las métricas de longitud en el benchmarking.
- [ ] **Optimización del Nearest Neighbor (KD-Tree Toroidal):**
  - El `_nearest` actual del RRT es búsqueda lineal $O(N)$ sobre todos los nodos del árbol. Para árboles grandes ($N > 5000$), esto se convierte en el cuello de botella. Se podría implementar un KD-Tree adaptado a la topología circular de $T^n$ (o usar un KD-Tree estándar con distancia geodésica) para reducir la búsqueda a $O(\log N)$. Impacto: tiempos de planificación significativamente menores en el benchmarking.

