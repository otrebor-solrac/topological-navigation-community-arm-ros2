#!/usr/bin/env python3
import sys
import os
import time
import json
import csv
import argparse
import numpy as np

# Añadir la ruta del código fuente al sys.path para poder importar whitebox_motion_planners
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../src/whitebox_motion_planners"))
sys.path.append(src_path)

from whitebox_motion_planners.collision.grid_discretizer import GridDiscretizer
from whitebox_motion_planners.collision.foam_collider import FoamCollider
from whitebox_motion_planners.kinematics.community_arm import CommunityArmKinematics
from whitebox_motion_planners.kinematics.trajectory import TrajectoryGenerator
from whitebox_motion_planners.planners.planner_factory import PlannerFactory
from whitebox_motion_planners.spaces.metrics import Metrics

import config_experiments as config

def evaluate_trajectory(path, kinematics, dt=0.02):
    """
    Interpola el camino discreto usando splines quánticas y evalúa
    la trayectoria a 50Hz (dt = 0.02s) para calcular la duración,
    el Jerk acumulado y los índices de manipulabilidad de Yoshikawa.
    """
    try:
        # Generar trayectoria de tiempo continuo
        traj = TrajectoryGenerator(path, max_vel=1.0, max_acc=1.0)
        duration = traj.total_duration
        if duration <= 1e-6:
            return 0.0, 0.0, 0.0, 0.0
            
        steps = int(np.ceil(duration / dt))
        jerk_sum = 0.0
        manipulabilities = []
        
        # Evaluar en cada paso temporal
        for step in range(steps):
            t = step * dt
            q, qd, qdd = traj.evaluate(t)
            q_next, qd_next, qdd_next = traj.evaluate(t + dt)
            
            # Calcular Jerk numérico (derivada de la aceleración)
            jerk = (qdd_next - qdd) / dt
            jerk_sum += np.sum(jerk ** 2) * dt
            
            # Calcular manipulabilidad
            w = kinematics.compute_manipulability(tuple(q))
            manipulabilities.append(w)
            
        w_min = min(manipulabilities) if manipulabilities else 0.0
        w_mean = np.mean(manipulabilities) if manipulabilities else 0.0
        
        return duration, jerk_sum, w_min, w_mean
    except Exception as e:
        print(f"Error evaluando la trayectoria temporal: {e}")
        return 0.0, 0.0, 0.0, 0.0

def run_experiment_run(alg, metric, env_key, res, rep, cache_dir, writer):
    """
    Ejecuta una corrida para una combinación específica de factores
    a lo largo de todos sus segmentos de waypoints y escribe los resultados.
    """
    env_hash = config.ENVIRONMENT_HASHES[env_key]
    
    # 1. Cargar caché de C-Space
    cache_filename = f"cspace_cache_{res}deg_0.015m_{env_hash}_singularity0.0005.json"
    cache_filepath = os.path.join(cache_dir, cache_filename)
    
    if not os.path.exists(cache_filepath):
        print(f"Error: No se encontró la caché {cache_filename}")
        return False
        
    with open(cache_filepath, 'r') as f:
        cache_data = json.load(f)
        
    forbidden_list = cache_data.get('forbidden_voxels', [])
    
    # 2. Inicializar componentes de planificación
    grid = GridDiscretizer(step_size_deg=res, num_dof=3)
    
    # Cargar waypoints del entorno (radianes) para limpiar el grid en esas coordenadas
    waypoints_rad = config.get_waypoints_rad(env_key)
    
    # Cargar voxeles prohibidos en el conjunto discretizado
    forbidden_set = set()
    for voxel in forbidden_list:
        q_discrete = grid.discretize(tuple(voxel))
        forbidden_set.add(q_discrete)
        
    # Evitar bloqueos de inicio/meta por redondeo discretizado
    for wp_rad in waypoints_rad:
        wp_discrete = grid.discretize(tuple(wp_rad))
        if wp_discrete in forbidden_set:
            forbidden_set.remove(wp_discrete)
        
    # Inicializar FoamCollider cargando el URDF para evitar crashes de link_radius
    urdf_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../src/robots/community_robot_arm/urdf/spherized/community_robot_arm_slim_spherized.urdf"))
    collider = FoamCollider(urdf_path=urdf_path, sphere_thinning_dist=0.015)
    collider.set_cspace_cache(forbidden_set, grid)
    
    # Configurar transformaciones de juntas predeterminadas para mapeo de URDF/Mundo
    collider.set_joint_transforms(
        offset_base_yaw=0.559643,
        offset_shoulder_pitch=1.5707963,
        offset_elbow_pitch=0.0,
        dir_base_yaw=-1.0,
        dir_shoulder_pitch=-1.0,
        dir_elbow_pitch=1.0
    )
    
    kinematics = CommunityArmKinematics(use_horizontal_constraint=False)
    
    # Crear el planificador vía la fábrica
    planner = PlannerFactory.create_planner(
        planner_type=alg,
        space=grid,
        collider=collider,
        kinematics=kinematics,
        metric_type=metric
    )
    
    success_all_segments = True
    
    # Ejecutar planificación para cada segmento consecutivo
    for i in range(len(waypoints_rad) - 1):
        start_rad = tuple(waypoints_rad[i])
        goal_rad = tuple(waypoints_rad[i+1])
        
        start_discrete = grid.discretize(start_rad)
        goal_discrete = grid.discretize(goal_rad)
        
        # Medir tiempo de planificación de CPU
        t_start = time.perf_counter()
        path = planner.plan(start_discrete, goal_discrete)
        
        # Fallback a nivel de experimento si la discretización bloquea el camino geodésico
        if not path and collider.forbidden_set is not None:
            temp_set = collider.forbidden_set
            collider.forbidden_set = None
            path = planner.plan(start_discrete, goal_discrete)
            collider.forbidden_set = temp_set
            
        t_end = time.perf_counter()
        
        planning_time = t_end - t_start
        exito = path is not None and len(path) > 0
        
        # Inicializar variables de métricas
        path_len = 0.0
        duration = 0.0
        jerk_sum = 0.0
        w_min = 0.0
        w_mean = 0.0
        
        if exito:
            # Calcular longitud geométrica de la ruta
            for j in range(len(path) - 1):
                path_len += Metrics.heuristic_L2(np.array(path[j]), np.array(path[j+1]))
                
            # Evaluar spline y suavidad
            duration, jerk_sum, w_min, w_mean = evaluate_trajectory(path, kinematics)
        else:
            success_all_segments = False
            
        # Escribir fila en CSV
        writer.writerow([
            alg, metric, env_key, res, rep, f"W{i}->W{i+1}",
            list(np.round(np.degrees(start_rad), 2)), list(np.round(np.degrees(goal_rad), 2)),
            int(exito), round(planning_time, 5), round(path_len, 4),
            round(duration, 3), round(jerk_sum, 2), round(w_min, 5), round(w_mean, 5)
        ])
        
    return success_all_segments

def main():
    parser = argparse.ArgumentParser(description="Ejecución de experimentos para la tesis.")
    parser.add_argument("--pilot", action="store_true", help="Ejecutar únicamente la prueba piloto rápida (15deg, A*, E0).")
    parser.add_argument("--all", action="store_true", help="Ejecutar la matriz completa de experimentos (500 corridas).")
    parser.add_argument("--resolution", type=float, choices=config.RESOLUTIONS, help="Filtrar ejecución por una resolución específica.")
    args = parser.parse_args()
    
    cache_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../cspace_cache"))
    resultados_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "resultados"))
    os.makedirs(resultados_dir, exist_ok=True)
    
    csv_filename = "resultados_raw_pilot.csv" if args.pilot else "resultados_raw.csv"
    csv_filepath = os.path.join(resultados_dir, csv_filename)
    
    # Definir combinatoria
    if args.pilot:
        algs = ["astar"]
        metrics = ["L1"]
        envs = ["E0"]
        resolutions = [15.0]
        repetitions = 1
        print("🚀 Iniciando ejecución de la Fase Piloto...")
    else:
        algs = ["astar", "rrt"]
        metrics = ["L1", "L2"]
        envs = ["E0", "E1", "E2", "E3", "E4"]
        resolutions = [args.resolution] if args.resolution else config.RESOLUTIONS
        repetitions = 5
        print(f"🚀 Iniciando ejecución de experimentos (Resoluciones: {resolutions})...")
        
    headers = [
        "algoritmo", "metrica", "entorno", "resolucion_deg", "repeticion", "segmento",
        "inicio_q", "meta_q", "exito", "tiempo_planificacion_s", "longitud_geometrica_rad",
        "duracion_trayectoria_s", "jerk_acumulado", "manipulabilidad_min", "manipulabilidad_mean"
    ]
    
    total_runs = len(algs) * len(metrics) * len(envs) * len(resolutions) * repetitions
    current_run = 0
    successes = 0
    
    with open(csv_filepath, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        
        for alg in algs:
            for metric in metrics:
                for env in envs:
                    for res in resolutions:
                        for rep in range(1, repetitions + 1):
                            current_run += 1
                            print(f"[{current_run}/{total_runs}] Ejecutando: {alg.upper()} | {metric} | {env} | {res}° | Corrida #{rep}...", end="", flush=True)
                            
                            ok = run_experiment_run(alg, metric, env, res, rep, cache_dir, writer)
                            if ok:
                                print(" [OK]")
                                successes += 1
                            else:
                                print(" [FALLO]")
                                
    print("\n🎉 Ejecución finalizada con éxito!")
    print(f"Resultados guardados en: {csv_filepath}")
    print(f"Tasa de éxito general: {successes}/{total_runs} corridas completas ({successes/total_runs*100:.1f}%)")

if __name__ == "__main__":
    main()
