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

## 🧊 3. Voxelización y Detección de Colisiones (En Progreso)
*Objetivo: Crear el mapa $\mathcal{C}_{free}$ usando el recubrimiento de bolas abiertas.*

- [x] **Muestreo de Bolas:** Definido en `FoamCollider` usando esferas.
- [x] **C-Space Mapping (Voxelización):** Discretización funcional en `GridDiscretizer`.
- [ ] **Gestión de Obstáculos Externos:** 
    - [ ] Permitir cargar obstáculos (centros y radios) desde el `planner_params.yaml`.
    - [ ] Integrar obstáculos estáticos en la validación del `FoamCollider`.

---

## 🔍 4. Algoritmos de Búsqueda (Casi Completado)
- [x] **A* Toroidal:** Heurística admisible con salto de frontera operativa.
- [x] **RRT Estocástico:** 
    - [x] Muestreo Haar (uniformidad angular) implementado en `rrt.py`.
    - [x] Steering que respeta la topología circular.
- [ ] **Refinamiento RRT:**
    - [ ] Implementar la lógica de crecimiento del árbol (actualmente es un esqueleto funcional).

---

## ✨ 5. Tareas de Excelencia (NUEVAS)
- [ ] **Refinamiento Cinemático:** Revisar matrices DH para que coincidan al 100% con el hardware real.
- [ ] **Suavizado de Trayectoria (`PathSmoother`):** Algoritmo para eliminar el "zig-zag" del RRT mediante atajos (shortcuts).
- [ ] **Benchmarking Suite:** Script para generar comparativas automáticas (A* vs RRT) de tiempo y distancia para los anexos de la tesis.

---

## 📺 6. Visualización "Caja Blanca" en RViz2
- [ ] **Publicador de Marcadores (`visualization_msgs`):**
    - [ ] Dibujar las **bolas de colisión** como esferas semitransparentes sobre el brazo.
    - [ ] Visualizar los **voxels prohibidos** en el espacio de trabajo.
    - [ ] Dibujar el **Árbol de RRT** o la **Ruta de A*** como líneas de colores.

## 🌐 7. Fase 2: Dashboard Web "White-Box" (Control Remoto)
*Objetivo: Llevar la visualización y control del robot al navegador para una presentación más profesional y accesible.*

- [ ] **Backend de Comunicación:**
    - [ ] Instalar y configurar `rosbridge_suite` en el Dockerfile.
    - [ ] Exponer los puertos necesarios (ej. 9090) en el `docker-compose.yml`.
    - [ ] Actualizar el planificador para aceptar metas secuenciales ($A \to B \to C$).
- [ ] **Frontend Web:**
    - [ ] Crear la aplicación base en React/Vite.
    - [ ] Integrar `roslibjs` para la comunicación por WebSockets con ROS 2.
    - [ ] Implementar visualizador 3D usando `three.js` (o similar) para renderizar el URDF y las rutas.
- [ ] **Interfaz de Usuario (UI/UX):**
    - [ ] Secuenciador de Puntos (añadir/eliminar objetivos).
    - [ ] Selector dinámico de Algoritmo (RRT / A*) y Métrica (L1 / L2).
    - [ ] Panel de telemetría (ángulos actuales, estado de colisiones).

---

## 📐 Notas de Seguridad Cinemática
- [ ] Integración de la **Matriz Jacobiana** para detectar singularidades.
- [ ] Penalización de los voxels que se acerquen a un determinante nulo (det(J) ≈ 0).
