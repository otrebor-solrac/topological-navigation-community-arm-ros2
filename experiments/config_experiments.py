import numpy as np

# Definición de resoluciones (en grados) y correspondencia de hashes para los archivos de caché
RESOLUTIONS = [6.0, 8.0, 10.0, 12.0, 15.0]

ENVIRONMENT_HASHES = {
    "E0": "no_obstacles",
    "E1": "38196ced",  # box_obstacle_spherized
    "E2": "3dab0f26",  # narrow_passage_spherized
    "E3": "d96224ee",  # toroidal_wall_spherized
    "E4": "c8a547e5",  # u_obstacle_spherized
    "E5": "no_obstacles"
}

# Nombres descriptivos de los entornos
ENVIRONMENT_NAMES = {
    "E0": "Sin obstáculos",
    "E1": "Caja",
    "E2": "Pasaje estrecho",
    "E3": "Pared toroidal",
    "E4": "Obstáculo en U",
    "E5": "Personalizado sin obstáculos"
}

# Secuencias de Waypoints diseñadas específicamente para cada entorno (coordenadas mundo, en grados)
# Formato: [Base (yaw), Hombro (pitch), Codo (pitch)]
# Cada secuencia inicia y termina en el Origen/Home [180.0, 90.0, 0.0] para asegurar consistencia con el Dashboard.
WAYPOINTS = {
    "E0": [ # Sin obstáculos (Línea base)
        [180.0, 90.0, 0.0],     # Origen / Home
        [-90.0, 135.0, 10.0],   # Waypoint #1
        [0.0, 90.0, -20.0],     # Waypoint #2
        [90.0, 120.0, 0.0],     # Waypoint #3
        [180.0, 90.0, 0.0]      # Retorno al Origen
    ],

    "E1": [ # Obstáculo de caja (Box obstacle)
        [180.0, 90.0, 0.0],     # Origen / Home
        [-40.0, 90.0, 0.0],     # Waypoint #1
        [40.0, 90.0, 0.0],      # Waypoint #2
        [180.0, 90.0, 0.0]      # Retorno al Origen
    ],

    "E2": [ # Obstáculo de pasaje estrecho (Narrow passage)
        [180.0, 90.0, 0.0],     # Origen / Home
        [-50.0, 70.0, 0.0],   # Waypoint #1
        [0.0, 90.0, 0.0],       # Waypoint #2
        [0.0, 130.0, 30.0],     # Waypoint #3
        [50.0, 70.0, 0.0],    # Waypoint #5
        [180.0, 90.0, 0.0]      # Retorno al Origen
    ],
    "E3": [ # Obstáculo de pared toroidal (Toroidal wall)
        [180.0, 90.0, 0.0],     # Origen / Home
        [-60.0, 112.0, 0.0],   # Waypoint #1
        [60.0, 112.0, 0.0],    # Waypoint #2
        [0.0, 175.0, 48.0],    # Waypoint #2
        [180.0, 90.0, 0.0]      # Retorno al Origen
    ],

    "E4": [ # Obstáculo en U (U-shaped obstacle)
        [180.0, 90.0, 0.0],     # Origen / Home
        [-60.0, 90.0, 0.0],     # Waypoint #1
        [0.0, 120.0, 0.0],      # Waypoint #2
        [0.0, 140.0, 40.0],     # Waypoint #3
        [60.0, 90.0, 0.0],      # Waypoint #4
        [180.0, 90.0, 0.0]      # Retorno al Origen
    ],

    "E5": [ # Escenario personalizado de validación de paralelogramo
        [180.0, 90.0, 0.0],     # Origen / Home
        [-68.0, 119.0, 0.0],    # Waypoint #1
        [61.0, 119.0, 0.0],     # Waypoint #2
        [61.0, 78.0, 0.0],      # Waypoint #3
        [61.0, 78.0, -17.0],    # Waypoint #4
        [-73.0, 78.0, -17.0]   # Waypoint #5
    ],

}

def get_waypoints_rad(env_key):
    """
    Retorna la lista de waypoints para un entorno en radianes.
    """
    if env_key not in WAYPOINTS:
        raise ValueError(f"Entorno no válido: {env_key}")
    return [list(np.radians(wp)) for wp in WAYPOINTS[env_key]]
