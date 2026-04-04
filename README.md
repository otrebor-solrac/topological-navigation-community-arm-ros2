# Instalación del repo y paquete
sudo apt-get install -y nvidia-container-toolkit
# Configuración del runtime de Docker
sudo nvidia-ctk runtime configure --runtime=docker
# Reinicio para aplicar
sudo systemctl restart docker


mi_tesis_ws/ (Tu Workspace)
│
├── src/
│   └── planificacion_didactica_pkg/  <-- TU PAQUETE PRINCIPAL
│       ├── package.xml
│       ├── setup.py
│       │
│       ├── planificacion_didactica/  <-- CÓDIGO FUENTE (PYTHON)
│       │   ├── __init__.py
│       │   ├── cinematica.py       <-- [MATEMÁTICAS] Matrices DH y Transformaciones
│       │   ├── a_star.py           <-- [ALGORITMO 1] Lógica de Grafos y Heurísticas
│       │   ├── rrt.py              <-- [ALGORITMO 2] Muestreo Probabilístico
│       │   ├── suavizado.py        <-- [OPTIMIZACIÓN] Polinomios/Splines
│       │   └── simulador_nodo.py   <-- Conexión con ROS2
│       │
│       ├── notebooks/  <-- EL VALOR AGREGADO (Para los alumnos)
│       │   ├── 1_Entendiendo_Cinematica.ipynb  (Explicación paso a paso con gráficas)
│       │   ├── 2_Comparacion_Astar_vs_RRT.ipynb (Benchmarks y tiempos)
│       │
│       ├── launch/
│       │   ├── simulacion_completa.launch.py (Arranca Gazebo + Tu Código)
│       │
│       └── world/
│           └── entorno_obstaculos.sdf (El mundo de Gazebo con cubos/paredes)


CAPÍTULO 1: INTRODUCCIÓN(Aquí vendes la idea. Corto y directo)1.1. Planteamiento del Problema: La opacidad de las herramientas industriales ("Cajas Negras") en la enseñanza de la robótica.1.2. Objetivos:* 1.2.1. Objetivo General (Desarrollar la plataforma didáctica en ROS2).* 1.2.2. Objetivos Específicos (Implementar A*, Implementar RRT, Suavizar con Bézier, Comparar métricas).1.3. Justificación: Importancia de la transparencia algorítmica y el código abierto para la educación en ingeniería.1.4. Alcances y Limitaciones:* Alcance: Planificación Offline en mapa estático conocido.* Limitación: No se aborda evasión de obstáculos dinámicos en tiempo real.CAPÍTULO 2: FUNDAMENTACIÓN MATEMÁTICA(Este es tu capítulo fuerte. Aquí demuestras que eres matemático. Es el 40% del documento)2.1. Modelado Cinemático y Espacios:* 2.1.1. Cinemática Directa y Matrices de Transformación Homogénea ($SE(3)$).* 2.1.2. Definición del Espacio de Trabajo ($W$) vs. Espacio de Configuración ($C-Space$).* 2.1.3. Topología de los Obstáculos en el $C-Space$ ($C_{obs}$ vs $C_{free}$).2.2. Teoría de Grafos y Búsqueda Determinista (A):** 2.2.1. Definición formal de Grafo $G=(V, E)$.* 2.2.2. Función de Costo $f(n) = g(n) + h(n)$.* 2.2.3. Heurísticas Admisibles: Distancia Euclidiana ($L_2$) vs. Manhattan ($L_1$).2.3. Probabilidad y Muestreo Estocástico (RRT):* 2.3.1. Espacios de Probabilidad y Muestreo Uniforme.* 2.3.2. Diagramas de Voronoi y el sesgo de expansión de RRT.* 2.3.3. Propiedad de Completitud Probabilística ($\lim_{n \to \infty} P(Solution) = 1$).2.4. Optimización Numérica y Suavizado:* 2.4.1. Continuidad Geométrica ($C^0, C^1, C^2$).* 2.4.2. Interpolación mediante Curvas de Bézier y Polinomios de Bernstein.CAPÍTULO 3: METODOLOGÍA Y HERRAMIENTAS(Aquí explicas el "Laboratorio Virtual")3.1. Arquitectura del Sistema:* Esquema "Sense-Plan-Act" (Percibir-Planear-Actuar).3.2. Entorno de Simulación:* Robot Operating System 2 (ROS2 Humble).* Simulador Físico Gazebo y Visualizador Rviz.3.3. Diseño del Robot Manipulador:* Descripción de los grados de libertad (DOF) y restricciones físicas.3.4. Diseño de Escenarios de Prueba:* Escenario A: Entorno libre (Línea base).* Escenario B: "Bug Trap" (Trampa en U - Difícil para A*).* Escenario C: Pasillos estrechos (Difícil para RRT).CAPÍTULO 4: IMPLEMENTACIÓN Y DESARROLLO(Aquí pones tu código Python explicado, no pegado tal cual, sino diagramas de flujo y fragmentos clave)4.1. Estructura del Paquete ROS2: Organización de nodos y tópicos.4.2. Módulo de Discretización (Voxel Grid): Transformación del entorno continuo a discreto para A*.4.3. Implementación del Algoritmo A:** Gestión de la cola de prioridad (Open Set / Closed Set).4.4. Implementación del Algoritmo RRT:* Estrategia de expansión del árbol (step_size y max_iter).4.5. Post-Procesamiento: Algoritmo de suavizado de trayectoria.4.6. Material Didáctico: Descripción de los Jupyter Notebooks interactivos generados.CAPÍTULO 5: RESULTADOS Y DISCUSIÓN(La comparación estadística)5.1. Análisis de Complejidad Computacional:* Gráficas de Tiempo de Cómputo vs. Complejidad del Obstáculo.5.2. Análisis de Calidad de la Trayectoria:* Comparativa de Longitud de Arco (A* vs RRT vs RRT-Suavizado).* Análisis de Suavidad (Derivadas de la posición).5.3. Discusión: ¿Cuándo conviene usar A* y cuándo RRT? (Trade-off entre optimalidad y velocidad).6.1. Conclusiones Generales.6.2. Aportes a la Docencia (El valor de la "Caja Blanca").6.3. Trabajo Futuro (Sugerir implementación en robot real o RRT*).

---

# 🚀 ROADMAP DE IMPLEMENTACIÓN (Siguientes Pasos)
Ya tenemos lo más difícil de la infraestructura: **Docker, ROS2, Gazebo y RViz corriendo de forma estable**. Basado en el `Marco-Teórico.tex` y en la estructura de tu tesis, estos son los pasos prácticos que siguen para materializar tu investigación en código Python:

### FASE 1: Creación del Paquete ROS2 y Nodo Central
Actualmente tienes scripts sueltos en `src/algorithms`. El primer paso es estructurarlos como un paquete formal de ROS2 (`planificacion_didactica_pkg`) tal como lo describes en el Capítulo 4.
- [ ] Crear el esqueleto del paquete en ROS2 con `ros2 pkg create --build-type ament_python planificacion_didactica_pkg`.
- [ ] Construir un Nodo Central (`simulador_nodo.py`) que se subscriba a la odometría del brazo (para saber dónde está) y sea capaz de publicar trayectorias al `/arm_controller/follow_joint_trajectory`. Empezaremos publicando movimientos vacíos o hardcodeados para probar la estructura.

### FASE 2: Traducción del Espacio de Trabajo al Espacio de Configuración ($C-Space$)
La tesis basa toda la planificación en entender el Toro $T^n$. Necesitas las herramientas matemáticas para mapear esto en código.
- [ ] Escribir el script `cinematica.py`. Deberá contener las matrices de Transformación Homogénea de tu brazo.
- [ ] **Aproximación por bolas abiertas (Recubrimientos):** Crear una función matemática que tome el entorno de Gazebo (ej. una caja) y valide si un conjunto de ángulos (puntos en $T^n$) hace colisión. En vez de chocar mallas complejas de Gazebo, vas a construir tu propia función lógica que envuelva el brazo en esferas y compare la métrica euclidiana $L_2$ contra el obstáculo.

### FASE 3: Discretización y Búsqueda Determinista (A*)
Ahora que tu código entiende si una postura es segura ($C_{free}$), empezamos con A*.
- [ ] Crear el generador de **Retícula Toroidal (Voxel Grid)**: Una función que tome el espacio continuo $T^n$ y lo traduzca a un arreglo discreto de "nodos" manejables numéricamente por NumPy (resolviendo la topología cociente "wrap-around", para que de 359° pase a 1°).
- [ ] Desarrollar `a_star.py`.
- [ ] Implementar la función de heurística usando la métrica de Manhattan ($L_1$) ajustada a la aritmética modular, como lo dicta el Marco Teórico.

### FASE 4: Algoritmo Estocástico (RRT) y Visualización Matemática
Como RRT no necesita la retícula de A*, se programa aparte y ayudará a evitar la "maldición de la dimensionalidad".
- [ ] Desarrollar `rrt.py` que genere un muestreo uniforme aleatorio en $C_{free}$ integrando de nuevo tu función de esferas y colisiones.
- [ ] **Conexión visual:** Escribir una función que tome el grafo generado por RRT o A* y lo publique de manera secuencial (Línea de RViz usando `Marker` tipo `LINE_STRIP`) para cumplir con la "validación visual puramente matemática" descrita en la sección 2.3.4 (RViz).

### FASE 5: Diseño de Mundos y Extracción de Resultados
- [ ] Crear los escenarios `.sdf` en Gazebo:
    - Espacio Libre (validar cinemática).
    - Trampa "Bug Trap" en forma de U (Donde verás que A* sufre).
    - Entorno de "Pasillos" (Donde verás que RRT se pierde).
- [ ] Programar un script auxiliar que tome tiempos de cómputo y distancias para graficar los resultados del Capítulo 5.
