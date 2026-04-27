# Temario de Estudio: Robótica Topológica (Versión Matemáticas-UnADM)

## 1. Topología General: Del Espacio Métrico al Axioma

*Objetivo: Entender que la métrica es solo una de las muchas formas de generar una topología.*

* **1.1 Espacios Métricos Revisados:**
* Métricas $L_1$ y $L_2$ en $\mathbb{R}^n$ y sus bolas unitarias.
* Equivalencia de métricas topológicas.


* **1.2 La Estructura Topológica:**
* Definición axiomática de $\tau$ (Uniones e intersecciones).
* **Bases de una topología:** Cómo generar una topología a partir de una colección de conjuntos (crucial para entender el Toro).


* **1.3 Propiedades de Separación y Compacidad:**
* **Conjuntos Compactos:** Definición por recubrimientos finitos. Prerrequisito para entender la separación Hausdorff.
* Espacios de Hausdorff ($T_2$): ¿Por qué es vital para distinguir estados del robot?
* Compacidad y el Teorema de Heine-Borel: Relación entre límites de motores y conjuntos cerrados/acotados.



## 2. Construcción de Espacios (El Puente hacia el Toro)

*Objetivo: Aprender las "operaciones" para fabricar el espacio del robot.*

* **2.1 Topología de Subespacio:** Cómo heredar la topología de $\mathbb{R}^n$ a la superficie del robot.
* **2.2 Topología Producto:**
* Definición formal: La base de los cilindros abiertos.
* Construcción de $T^n = S^1 \times \dots \times S^1$.


* **2.3 Topología Cociente (La clave del Toro):**
* Relaciones de equivalencia y clases de equivalencia.
* **Identificación:** Cómo "pegar" el intervalo $[0,1]$ para obtener el círculo $S^1$.
* Construcción del Toro $T^2$ como el cuadrado unitario $I^2$ con bordes identificados.



## 3. Topología de Caminos y Conectividad

*Objetivo: Formalizar el concepto de "trayectoria".*

* **3.1 Conexidad y Conexidad por Caminos:**
* Definición formal de camino $\gamma: [0,1] \to X$.
* Componentes conexas: El espacio libre $\mathcal{C}_{free}$ vs el espacio de colisión $\mathcal{C}_{obs}$.


* **3.2 Homotopía de Caminos:**
* Deformación continua de trayectorias.
* Clases de homotopía: Por qué no puedes "deformar" un camino que rodea un obstáculo en uno que no lo hace.



## 4. Variedades y Geometría en el Toro

*Objetivo: Tratar al Toro como un objeto donde se puede hacer cálculo.*

* **4.1 Variedades Topológicas:** Definición de cartas y homeomorfismos locales a $\mathbb{R}^n$.
* **4.2 Espacios de Recubrimiento (Covering Spaces):**
* El mapeo $p: \mathbb{R}^n \to T^n$ (La función "módulo"). Justificación de por qué el robot "da la vuelta".
* **4.3 Métrica Intrínseca del Toro:**
* La métrica heredada de $\mathbb{R}^3$ vs. la métrica intrínseca (geodésicas).
* Distancia "Wrap-around": Implementación matemática del módulo $2\pi$.



## 5. Aplicación Algorítmica (Discretización)

*Objetivo: Traducir los teoremas a Rust.*

* **5.1 Discretización de Variedades:** Cómo convertir el Toro continuo en un complejo simplicial o un grafo.
* **5.2 Análisis de Algoritmos:**
* **A*:** Heurísticas en espacios no euclidianos (el problema de la distancia en el Toro).
* **RRT:** Muestreo probabilístico en variedades compactas.



## 6. Diferenciales y Singularidades

*Objetivo: El Jacobiano y la suavidad del movimiento.*

* **6.1 El Mapeo Cinemático:** $f: \mathcal{C} \to \mathcal{W}$ como función continua entre variedades.
* **6.2 El Jacobiano:** Análisis de puntos críticos y valores regulares (Singularidades del robot).

---

### Análisis de los "Huecos" (Gaps)

1. **De Espacio Topológico a Cociente:** No puedes entender el Toro sin entender la **Topología Cociente**. Es la operación matemática que "pega" los cables de tus motores para que el robot sepa que después de  sigue .
2. **De Conjuntos a Variedades:** El paso de "conjunto de abiertos" a "variedad" requiere entender que, aunque el Toro es curvo, si lo miras de muy cerca (una carta local), se ve exactamente como . Esto justifica usar algoritmos "planos" localmente.