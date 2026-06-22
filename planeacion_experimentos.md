# Planeación de Experimentos
## Sistema de Navegación Topológica en $\mathcal{T}^3$ — Robot Community Arm

---

## 1. Objetivo de la Experimentación

Comparar el desempeño de los dos planificadores de movimiento implementados
(**A\*** y **RRT**) bajo distintas condiciones de entorno y resolución del espacio
de configuración, empleando las dos métricas disponibles (**L1** y **L2**), con el
fin de determinar cuál combinación ofrece el mejor equilibrio entre calidad de
trayectoria y tiempo de cómputo.

Cada entorno cuenta con una secuencia de **waypoints diseñados específicamente**
para que la trayectoria deba interactuar con el obstáculo, haciendo la prueba
significativa y comparable entre algoritmos.

---

## 2. Variables del Experimento

### 2.1 Variables independientes (factores)

| Factor | Niveles |
|---|---|
| Algoritmo | A\*, RRT |
| Métrica / heurística | L1 (Manhattan toroidal), L2 (Euclidiana toroidal) |
| Entorno (obstáculo) | sin obstáculos, caja, pasaje estrecho, pared toroidal, obstáculo en U |
| Resolución de la malla | 6°, 8°, 10°, 12°, 15° |

### 2.2 Variables dependientes (métricas de respuesta)

| Métrica | Descripción |
|---|---|
| Tiempo de planificación (s) | Tiempo transcurrido desde la solicitud hasta la primera trayectoria válida |
| Longitud de trayectoria (rad) | Suma de distancias geodésicas entre waypoints consecutivos en $\mathcal{T}^3$ |
| Número de nodos expandidos | Solo para A\*: nodos visitados antes de encontrar la solución |
| Número de muestras generadas | Solo para RRT: iteraciones hasta convergencia |
| Tasa de éxito | Proporción de intentos en que el planificador encontró trayectoria válida |

### 2.3 Variables de control (constantes en todos los experimentos)

- Longitudes de eslabón: `base_height = 0.065 m`, `lower_shank = 0.140 m`, `upper_shank = 0.140 m`
- Radio de esferas para colisión FOAM: `sphere_thinning_dist = 0.015 m`
- Umbral de singularidad (Yoshikawa): `singularity_threshold = 0.0005`
- Número de repeticiones por combinación: **5 ejecuciones**

---

## 3. Combinatoria Completa de Experimentos

La combinatoria total es:

$$
2 \text{ algoritmos} \times 2 \text{ métricas} \times 5 \text{ entornos} \times 5 \text{ resoluciones}
= 100 \text{ combinaciones} \times 5 \text{ repeticiones} = 500 \text{ corridas}
$$

> **Nota:** Para A\*, la métrica actúa como heurística (`heuristic_type`).
> Para RRT, la métrica controla la distancia usada en la búsqueda del nodo más
> cercano (`_nearest`).

---

## 4. Mapas de Espacio de Configuración Disponibles (caché)

Los archivos JSON pre-calculados en `cspace_cache/` cubren las siguientes
combinaciones de resolución × entorno:

| Resolución | sin obstáculos | caja | pasaje estrecho | pared toroidal | obstáculo en U |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **6°** | ✅ (4.8 MB) | ✅ (4.9 MB) | ✅ (5.2 MB) | ✅ (5.0 MB) | ✅ (5.4 MB) |
| **8°** | ✅ (2.0 MB) | ✅ (2.1 MB) | ✅ (2.2 MB) | ✅ (2.1 MB) | ✅ (2.3 MB) |
| **10°** | ✅ (1.0 MB) | ✅ (1.1 MB) | ✅ (1.1 MB) | ✅ (1.1 MB) | ✅ (1.2 MB) |
| **12°** | ✅ (0.6 MB) | ✅ (0.6 MB) | ✅ (0.7 MB) | ✅ (0.6 MB) | ✅ (0.7 MB) |
| **15°** | ✅ (0.3 MB) | ✅ (0.3 MB) | ✅ (0.3 MB) | ✅ (0.3 MB) | ✅ (0.4 MB) |

### Correspondencia hash → obstáculo URDF

Los hashes en los nombres de archivo son los primeros 8 caracteres del SHA-256
del URDF correspondiente. Se deduce la correspondencia por tamaño de archivo:

| Hash | Archivo URDF | Tamaño URDF | Entorno |
|---|---|---|---|
| `no_obstacles` | `no_obstacles_spherized.urdf` | 78 B | E0 — sin obstáculos |
| `240e2caf` | `box_obstacle_spherized.urdf` | 1 084 B | E1 — caja |
| `3dab0f26` | `toroidal_wall_spherized.urdf` | 9 009 B | E3 — pared toroidal |
| `38196ced` | `narrow_passage_spherized.urdf` | 18 480 B | E2 — pasaje estrecho |
| `c8a547e5` | `u_obstacle_spherized.urdf` | 30 902 B | E4 — obstáculo en U |

---

## 5. Entornos de Obstáculos y Waypoints Diseñados

Cada entorno tiene una secuencia de waypoints (en coordenadas mundo, en grados)
elegidos para que la ruta directa entre waypoints consecutivos **pase a través
o muy cerca del obstáculo**, obligando al planificador a rodear o sortear la
región bloqueada del espacio de configuración.

Todos los ángulos están en **coordenadas mundo** (se aplica la transformación
`θ_urdf = offset + dirección × θ_mundo` internamente).

---

### E0 — Sin obstáculos (`no_obstacles_spherized.urdf`)

**Propósito:** Línea base. Mide el rendimiento mínimo sin presencia de obstáculos.
La ruta óptima es la geodésica directa en $\mathcal{T}^3$.

**Posición del obstáculo:** ninguno.

**Waypoints (coordenadas mundo, grados):**

| # | Base (yaw) | Hombro (pitch) | Codo (pitch) | Observación |
|---|---|---|---|---|
| W0 — inicio | 0° | 90° | 0° | Postura vertical |
| W1 | 45° | 110° | 30° | Giro intermedio |
| W2 | 90° | 130° | 60° | Extensión lateral |
| W3 — meta | 180° | 90° | 90° | Postura opuesta |

---

### E1 — Caja (`box_obstacle_spherized.urdf`)

**Propósito:** Obstáculo convexo simple centrado en `(0.30, 0.0, 0.15)` m,
tamaño `0.15 × 0.15 × 0.30` m. La caja bloquea configuraciones donde el
efector final se aproxima al eje X del robot a altura media. Se diseñan
waypoints que crucen este bloqueo.

**Posición del obstáculo (espacio cartesiano):** frente al robot, eje X, altura 0–0.30 m.

**Waypoints (coordenadas mundo, grados):**

| # | Base (yaw) | Hombro (pitch) | Codo (pitch) | Observación |
|---|---|---|---|---|
| W0 — inicio | 0° | 60° | 0° | Brazo extendido, apuntando a la caja |
| W1 | 30° | 90° | 30° | Cruce lateral — la ruta directa W0→W1 pasa por la caja |
| W2 | 60° | 120° | 60° | Evasión hacia arriba |
| W3 | 90° | 90° | 0° | Regreso con brazo extendido al otro lado |
| W4 — meta | 90° | 60° | -30° | Postura final extendida |

---

### E2 — Pasaje estrecho (`narrow_passage_spherized.urdf`)

**Propósito:** Dos paredes paralelas centradas en `x = 0.30` m, una en
`y ≈ +0.18` m y otra en `y ≈ -0.12` m, dejando un corredor angosto de
~0.10 m entre ellas. Desafía especialmente a RRT, que tiene baja probabilidad
de muestrear dentro del pasaje.

**Waypoints (coordenadas mundo, grados):**

| # | Base (yaw) | Hombro (pitch) | Codo (pitch) | Observación |
|---|---|---|---|---|
| W0 — inicio | -45° | 70° | -20° | Brazo a la derecha del pasaje |
| W1 | 0° | 90° | 0° | Entrada al corredor — zona restringida en C-space |
| W2 | 0° | 110° | 20° | Travesía del pasaje, codo elevado |
| W3 | 30° | 130° | 40° | Salida del pasaje hacia la izquierda |
| W4 — meta | 60° | 100° | 60° | Postura final fuera del pasaje |

---

### E3 — Pared toroidal (`toroidal_wall_spherized.urdf`)

**Propósito:** Pared vertical extendida (`0.06 × 0.45 × 0.40` m) centrada en
`(0.35, 0.0, 0.20)` m. Al ser ancha en Y (±0.225 m), bloquea una banda
completa del espacio de configuración. Su topología toroidal implica que el
planificador puede rodear la pared por el "otro lado" del toro $\mathcal{T}^3$.
Esto pone a prueba si los algoritmos explotan el envoltura circular de los ángulos.

**Waypoints (coordenadas mundo, grados):**

| # | Base (yaw) | Hombro (pitch) | Codo (pitch) | Observación |
|---|---|---|---|---|
| W0 — inicio | -30° | 80° | -30° | Frente a la pared, lado negativo Y |
| W1 | 0° | 90° | 0° | Posición de impacto directo contra la pared |
| W2 | 160° | 90° | 0° | Meta al otro lado — requiere dar la vuelta en $\mathcal{T}^3$ |
| W3 | 180° | 80° | 30° | Variación de postura al otro lado de la pared |
| W4 — meta | 200° | 100° | 60° | Postura final |

> El salto W1→W2 (de 0° a 160° en base) es intencional: la ruta directa
> atraviesa la zona bloqueada por la pared; el planificador debe decidir
> si da la vuelta en sentido positivo o negativo en $S^1$.

---

### E4 — Obstáculo en U (`u_obstacle_spherized.urdf`)

**Propósito:** Estructura en U formada por tres paredes: pared frontal en
`x ≈ 0.35` m y dos alas laterales en `y ≈ ±0.10` m, con abertura hacia
el robot (lado negativo X). La meta se coloca **dentro de la concavidad**,
lo que convierte esta prueba en la más exigente para A\*: el algoritmo puede
quedar atrapado en un mínimo local de la heurística dentro del hueco de la U.

**Waypoints (coordenadas mundo, grados):**

| # | Base (yaw) | Hombro (pitch) | Codo (pitch) | Observación |
|---|---|---|---|---|
| W0 — inicio | -60° | 60° | -30° | Fuera de la U, lado izquierdo |
| W1 | -30° | 80° | -10° | Aproximación hacia la apertura de la U |
| W2 | 0° | 100° | 10° | Interior de la U — zona de trampa para A\* |
| W3 | 30° | 110° | 30° | Salida de la U por el único camino libre |
| W4 — meta | 60° | 90° | 60° | Postura final fuera de la concavidad |

> El segmento W1→W2 lleva al robot **dentro** de la concavidad de la U.
> El planificador debe salir por W2→W3 evitando las paredes laterales.
> A\* con heurística L1 tiende a explorar hacia la meta sin considerar la
> concavidad, lo que puede producir más nodos expandidos o fallo total.

---

## 6. Resumen de Waypoints por Entorno

| Entorno | Waypoints | Segmentos | Desafío principal |
|---|---|---|---|
| E0 sin obstáculos | W0→W1→W2→W3 | 3 | línea base |
| E1 caja | W0→W1→W2→W3→W4 | 4 | evasión de obstáculo convexo |
| E2 pasaje estrecho | W0→W1→W2→W3→W4 | 4 | muestreo dentro del corredor (RRT) |
| E3 pared toroidal | W0→W1→W2→W3→W4 | 4 | envoltura toroidal, rodeo en $\mathcal{T}^3$ |
| E4 obstáculo en U | W0→W1→W2→W3→W4 | 4 | trampa de concavidad (A\*) |

Cada **segmento** se planifica de forma independiente: el planificador recibe un
par `(inicio, meta)` y produce una trayectoria. La trayectoria completa es la
concatenación de todos los segmentos.

---

## 7. Parámetros Específicos por Algoritmo

### 7.1 A\* (determinista)

| Parámetro | Valor |
|---|---|
| Heurística | L1 o L2 (factor del experimento) |
| Costo de paso | L2 (distancia geodésica real entre nodos vecinos) |
| Estructura de datos | min-heap (`heapq`) |
| Condición de parada | nodo expandido == nodo meta |

> A\* es determinista: las 5 repeticiones miden variabilidad del tiempo de
> cómputo por interferencia del sistema operativo.

### 7.2 RRT (probabilístico)

| Parámetro | Valor |
|---|---|
| Máx. muestras | 5 000 |
| Tamaño de paso | 0.15 rad |
| Sesgo hacia la meta | 10% |
| Tolerancia de meta | 0.2 rad |
| Métrica de vecindad | L1 o L2 (factor del experimento) |
| Semillas aleatorias | 42, 43, 44, 45, 46 (una por repetición) |

---

## 8. Protocolo de Ejecución

### 8.1 Procedimiento por corrida individual

1. Seleccionar combinación `(algoritmo, métrica, entorno, resolución, repetición)`.
2. Cargar el caché JSON correspondiente desde `cspace_cache/`.
3. Para cada segmento `(Wk → Wk+1)` del entorno:
   - Convertir coordenadas mundo a índices de malla.
   - Ejecutar `planner.plan(start_q, goal_q)`.
   - Registrar tiempo, nodos/muestras, longitud y éxito del segmento.
4. Si algún segmento falla → registrar **fallo parcial** e indicar cuál segmento.
5. Calcular métricas acumuladas de toda la trayectoria.
6. Guardar fila en el CSV de resultados.

### 8.2 Estructura de archivos propuesta

```
experimentos/
├── run_experiments.py       # script principal de automatización
├── config_experimentos.py   # tabla de combinaciones y waypoints por entorno
├── resultados/
│   ├── resultados_raw.csv   # una fila por segmento
│   └── resultados_agg.csv   # medias y desv. estándar por combinación
└── analisis/
    ├── graficas.ipynb        # Jupyter con visualizaciones
    └── tablas_latex.py       # genera tablas para la tesis
```

### 8.3 Formato del archivo CSV de resultados

```
algoritmo, metrica, entorno, resolucion_deg, repeticion, segmento,
inicio_q, meta_q, exito, tiempo_s, longitud_rad,
num_nodos_expandidos, num_muestras_rrt, num_waypoints
```

---

## 9. Análisis Estadístico

| Análisis | Método |
|---|---|
| Comparación A\* vs RRT | Prueba de Wilcoxon (no paramétrica, ya que RRT es estocástico) |
| Efecto de la métrica (L1 vs L2) | Comparación directa (A\* determinista) o Mann-Whitney (RRT) |
| Efecto de la resolución | ANOVA de un factor o prueba de Kruskal-Wallis |
| Efecto del entorno | Comparación por caja con segmentos estandarizados |
| Visualización | Diagramas de caja (boxplots) por factor y por entorno |

---

## 10. Orden de Prioridad para la Ejecución

| Fase | Resoluciones | Cobertura |
|---|---|---|
| Fase piloto | 15° | A\* L1 × E0 sin obstáculos (validar pipeline y waypoints) |
| Fase 2 | 15° | A\* L1 × todos los entornos (validar waypoints diseñados) |
| Fase principal | 10°, 15° | Todas las combinaciones × todos los entornos |
| Fase extendida | 6°, 8°, 12° | Si el tiempo de cómputo lo permite |

---

## 11. Entregables Esperados

- [ ] Confirmar que los waypoints de cada entorno son configuraciones válidas (no en $\mathcal{C}_{obs}$).
- [ ] Script `run_experiments.py` implementado y probado con el protocolo de segmentos.
- [ ] Archivo `resultados_raw.csv` con las corridas realizadas.
- [ ] Notebook `graficas.ipynb` con boxplots y tablas comparativas por entorno.
- [ ] Sección 4.2 de la tesis redactada con tablas y figuras generadas.

---

*Última actualización: junio 2026*
