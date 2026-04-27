# Guion de la Presentación de Avances (Discurso y Glosario)

## 🎙️ Discurso Sugerido por Lámina

### Lámina 1: Portada
> "Buenos días, soy Roberto Carlos Vazquez Nava y hoy les presentaré los avances de mi proyecto terminal: 'Modelado topológico y planificación de trayectorias en variedades toroidales', desarrollado en vinculación entre la UnADM y el TESH."

### Lámina 2: Contenido
> "Abordaremos 5 ejes: el contexto y planteamiento del problema; el panorama actual y la brecha de innovación; los sustentos teóricos alcanzados; el avance técnico; y finalmente, nuestra metodología y conclusiones preliminares."

### Lámina 3: Contexto y motivación
> "En México, la educación robótica suele depender de simuladores de 'caja negra' donde el estudiante opera la máquina sin entender el rigor analítico subyacente. La actual propuesta busca romper este paradigma creando una arquitectura abierta de planificación para el TESH, donde en lugar de usar librerías opacas, implementamos los algoritmos paso a paso para que la matemática sea totalmente transparente y accesible, mediante el uso de herramientas como ROS2, RViz2 y el Community Robot Arm."

### Lámina 4: Planteamiento del Problema
> "Para lograr construir esta arquitectura 'caja blanca', debemos enfrentarnos al problema central que los simuladores comerciales suelen ocultar: la matemática real del movimiento. Físicamente, las articulaciones de un robot no se mueven sobre un plano infinito, sino que sus rotaciones habitan en un espacio topológico cerrado conocido como Toro ($T^n$). Nuestro reto principal es traducir las propiedades de esta 'dona matemática' en un entorno de programación real, para así poder observar y evaluar abiertamente cómo operan las dos grandes filosofías de búsqueda:"
> A*: funciona evaluando paso a paso una cuadrícula para encontrar siempre el camino más corto y óptimo
> RRT: funciona expandiendo ramas de forma aleatoria, priorizando encontrar una salida rápida incluso en los entornos más complejos."

### Lámina 5: Pregunta de Investigación e Hipótesis
> "Ante el hermetismo de los métodos convencionales y la complejidad topológica del Toro, nos planteamos la siguiente pregunta: ¿Cómo podemos modelar este espacio articular mediante una variedad toroidal, usando recubrimientos de esferas, para permitir un análisis comparativo real entre algoritmos? 

> Nuestra hipótesis sostiene que esta formalización matemática permitirá resolver la conectividad del robot de forma eficiente. Anticipamos que, bajo este modelo unificado, A* demostrará ser el estándar de la ruta óptima, mientras que RRT probará su superioridad en velocidad y exploración de entornos densos."



### Lámina 6: Objetivos del Proyecto (y Wrap-around)
> "Para comprobarlo, hemos trazado los siguientes objetivos: El general es construir la plataforma topológica. De forma específica, queremos modelar el espacio, programar los algoritmos desde cero y evaluar su desempeño.


### Lámina 7:Estado del arte: Paradigmas y brecha tecnológica
>Habiendo identificado el problema, es vital mirar qué está pasando en el mundo. La literatura moderna nos dice que la robótica ya no confía en simples nubes de puntos; hoy hablamos de variedades de Riemann y Espacios de Configuración topológicos para asegurar fluidez. Sin embargo, aquí es donde encontramos una brecha crítica: los robots de bajo costo y educativos suelen ser tratados como 'Cajas Negras'. Se les inyecta un software que funciona, pero oculta la matemática. Precisamente aqui es donde nace el proyecto para cerrar ese 'vacío pedagógico', integrando el rigor de la topología avanzada en una plataforma de 'Caja Blanca' que sea transparente, económica y que, gracias a Docker, pueda ser actualiza a librerias mas recientes sin ensuciar o romper el sistema operativo."

### Lámina 8: Robótica Topológica y el Toro $T^n$
> "Entrando a los avances teóricos, nuestra primera piedra angular es justificar geométricamente el mundo natural del robot. Los motores no giran sobre una línea recta, giran en círculos. Topológicamente, cada articulación forma una circunferencia $S^1$. Aplicando el Teorema de Tíconov, sabemos que el producto de estas articulaciones nos genera una variedad compacta llamada Toro de dimensión $n$. ¿Qué significa esto en la práctica? Que al modelarlo mediante la 'topología cociente', pegamos los extremos del mapa. El algoritmo sabe que el grado 359 es equivalente al grado 1, dándonos un espacio finito, cerrado y predecible donde nuestras búsquedas jamás divergirán hacia el infinito."



Cada vez que llegues a un número entero (360), pégalo con el origen (0)

### 🎥 Lámina 9: Visualización de Variedad (VIDEO)
>"Como observan en la demostración visual, mientras el brazo robótico en el espacio de trabajo físico transita por los puntos A, B y C hasta regresar a su origen, su estado articular dibuja simultáneamente una ruta continua sobre la superficie del Toro $T^2$. Esta visualización demuestra que nuestra formulación geométrica no es abstracta, sino que modela el comportamiento físico del robot en un entorno cíclico."

### Lámina 10: Geometría y Recubrimientos
> "Sin embargo, un espacio de navegación está incompleto si no definimos las reglas de seguridad. Una vez que entendemos que el robot vive en un Toro, el siguiente paso crítico es mapear los obstáculos del mundo real hacia este dominio matemático. 

> Para lograrlo con máxima eficiencia, sustituimos las cajas rígidas de la ingeniería clásica por recubrimientos de bolas abiertas. Como muestra la formulación en pantalla, cada zona prohibida se define como una unión de esferas de seguridad. ¿Cuál es el beneficio? Que la detección de colisiones se reduce a un simple cálculo de distancias. Al usar la geometría circular natural del robot, reducimos falsos positivos y garantizamos que el planeador encuentre rutas auténticamente seguras y fluidas."


$C_{obs}$ representa el subespacio prohibido. Son todas aquellas posturas del robot (ángulos) que resultarían en un choque con un objeto físico. Nuestra meta es que el algoritmo navegue por el complemento, es decir, por $\mathcal{C}{free}$ (el espacio libre)."

### Lámina 11: Algoritmos y Discretización 
> "Con nuestro universo matemático ya configurado, llegamos al núcleo de la investigación: la evaluación de eficiencia entre dos filosofías opuestas. 

> Por un lado, implementamos **A***: un método determinista que requiere transformar el Toro en una rejilla de voxeles. Aquí evaluamos si es más eficiente usar la métrica Manhattan para ahorrar tiempo, o la métrica Euclidiana para asegurar la ruta geodésica perfecta.

> Por otro lado, tenemos a **RRT**, cuya fortaleza radica en ignorar las rejillas y navegar directamente sobre el espacio continuo de la variedad toroidal. RRT no busca la perfección, busca la viabilidad mediante una exploración aleatoria inteligente que 'estira' sus ramas hasta encontrar el objetivo. 

> El reto aquí es demostrar cómo cada algoritmo reacciona ante la topología wrap-around: A* debe recalcular sus vecinos en los bordes del mapa, mientras que RRT debe proyectar sus muestras aleatorias a través del Toro para encontrar el camino más corto en una dona que no tiene fin."

### 🎥 Lámina 12: Simulación Algorítmica (VIDEO)
> "Con el espacio toroidal definido y los obstáculos matemáticos insertados, entra la fase algorítmica. Analizaremos dos enfoques radicalmente distintos: el método determinista A*, que nos obliga a particionar nuestro espacio continuo en una 'rejilla' o voxeles; y el método RRT, que se mueve libremente en el espacio continuo iterando al azar. Para evaluar la eficiencia en A*, contrastamos si es mejor usar la métrica Euclidiana tradicional o la métrica Manhattan, que es computacionalmente más barata." 

### Lámina 13: Avance Técnico - Infraestructura en ROS2 y Docker
> "Saliendo de la matemática y entrando al campo de la ingeniería de software, nuestro mayor avance técnico es la infraestructura base. Para evitar el clásico problema de 'en mi computadora sí funciona', hemos empaquetado todo el proyecto en un contenedor Docker; así, cualquier estudiante del TESH puede clonar el entorno y correrlo con un script de arranque. Además, ya logramos digitalizar el gemelo físico del brazo (archivos URDF) dentro del ecosistema ROS2. Hoy, nuestro modelo geométrico en el visualizador RViz2 está 100% configurado y 'escuchando' en los puertos, listo para que le inyectemos las trayectorias dictadas por nuestros algoritmos en Python."

### Lámina 14: Conclusiones preliminares y siguientes pasos
> "Como conclusión principal de esta fase, hemos validado que modelar al robot matemáticamente en su espacio topológico natural (el Toro) nos otorga una enorme ventaja paramétrica: nos garantiza un entorno de simulación continuo, predecible y seguro para la toma de decisiones. 

> Nuestros siguientes pasos se enfocarán en aterrizar toda esta teoría en Python dentro de ROS2. Implementaremos las esferas que evitan colisiones y ejecutaremos la competencia formal entre A* y RRT en el modelo físico. Al finalizar, habremos consolidado una plataforma analítica transparente que esperamos impulse la soberanía tecnológica en el TESH. 

---

## 📖 Glosario Estratégico para el Sínodo


Teorema de Tíconov: Es el teorema que te asegura que "el producto de espacios acotados también es acotado". Garantiza que no importa si el robot tiene 3, 6 o 20 motores, el "mapa de posibilidades" matemático jamás se va a extender hacia el infinito. Estás garantizando que los algoritmos (A y RRT) siempre tendrán una solución real, y que la computadora nunca se quedara en un loop infinito intentando buscar respuestas infinitas.*

Rn/Zn: s la "Dona Infinita" explicada matemáticamente.

### I. Fundamentos (¿Por qué es superior?)
*   **Variedad (Manifold):** Un espacio que localmente parece plano (como una hoja de papel), pero cuya estructura global se cierra sobre sí misma (como una dona o la Tierra). Permite que el robot "dé la vuelta" al mapa de forma continua sin topar con bordes artificiales.
*   **Espacio de Hausdorff:** Garantiza que el robot siempre pueda distinguir entre dos posturas diferentes. Es la base de la estabilidad del control.

### II. Geometría (¿Qué forma tiene?)
*   **Toro ($T^n$):** El resultado de los giros de los motores. Permite el efecto **Wrap-around**: el robot sabe que de $359^\circ$ a $1^\circ$ solo hay $2^\circ$ de distancia, no $358^\circ$.
*   **Topología Cociente:** Es el "pegamento" matemático que une el inicio del giro con el final, convirtiendo una línea en un círculo perfecto y continuo.

### III. Seguridad y Eficiencia
*   **Recubrimiento por Bolas (Ball Covering):** En lugar de usar cajas cuadradas que desperdician espacio, usamos esferas analíticas. Es más preciso y mucho más rápido de calcular para el procesador.
*   **Voxel Topológico:** Divide la 'dona' en pequeños cubitos de búsqueda. Es como convertir un espacio curvo en un tablero de ajedrez gigante para que el algoritmo A* pueda jugar en él.

### IV. Los Algoritmos
*   **L1 (Manhattan) vs L2 (Euclidiana):** Demostramos que son topológicamente equivalentes en el Toro. Usamos L1 para ahorrar CPU sin perder precisión.
*   **Medida de Haar:** Asegura que el algoritmo RRT lance sus puntos de forma "justa" y uniforme en todo el Toro, garantizando que siempre encontrará la salida si existe.
*   **Singularidades (Teorema de Farber):** No son errores de software, son propiedades naturales del espacio. El algoritmo las detecta como "obstáculos matemáticos" y las rodea preventivamente.


RRT significa Rapidly exploring Random Tree (