#!/usr/bin/env python3
import sys
import os
import time
import json
import csv
import random
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
    Interpola el camino discreto usando splines quínticas y evalúa
    la trayectoria para obtener duración, Jerk acumulado y manipulabilidad.
    """
    try:
        traj = TrajectoryGenerator(path, max_vel=1.0, max_acc=1.0)
        duration = traj.total_duration
        if duration <= 1e-6:
            return 0.0, 0.0, 0.0, 0.0
            
        steps = int(np.ceil(duration / dt))
        jerk_sum = 0.0
        manipulabilities = []
        
        # Evaluar primer punto fuera del bucle
        q, qd, qdd = traj.evaluate(0.0)
        
        for step in range(steps):
            t_next = (step + 1) * dt
            q_next, qd_next, qdd_next = traj.evaluate(t_next)
            
            jerk = (qdd_next - qdd) / dt
            jerk_sum += np.sum(jerk ** 2) * dt
            
            w = kinematics.compute_manipulability(tuple(q))
            manipulabilities.append(w)
            
            # Reusar estado actual como anterior del siguiente paso
            q, qd, qdd = q_next, qd_next, qdd_next
            
        w_min = min(manipulabilities) if manipulabilities else 0.0
        w_mean = np.mean(manipulabilities) if manipulabilities else 0.0
        
        return duration, jerk_sum, w_min, w_mean
    except Exception as e:
        return 0.0, 0.0, 0.0, 0.0

def generate_random_valid_state(grid, forbidden_set):
    """
    Muestrea un estado aleatorio en la rejilla discreta y verifica
    que no esté en el conjunto de estados prohibidos.
    """
    max_attempts = 1000
    for _ in range(max_attempts):
        q_discrete = (
            random.randint(0, grid.steps_per_circle - 1),
            random.randint(0, grid.steps_per_circle - 1),
            random.randint(0, grid.steps_per_circle - 1)
        )
        if q_discrete not in forbidden_set:
            return q_discrete
    return None

def main():
    # Fijar semilla para reproducibilidad científica
    random.seed(42)
    np.random.seed(42)

    cache_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../cspace_cache"))
    resultados_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "resultados"))
    os.makedirs(resultados_dir, exist_ok=True)
    
    csv_filepath = os.path.join(resultados_dir, "resultados_benchmark.csv")
    
    headers = [
        "algoritmo", "metrica", "entorno", "resolucion_deg", "par_id",
        "inicio_q", "meta_q", "exito", "tiempo_planificacion_s", "longitud_geometrica_rad",
        "duracion_trayectoria_s", "jerk_acumulado", "manipulabilidad_min", "manipulabilidad_mean"
    ]
    
    # Parámetros del benchmark
    algs = ["astar", "rrt"]
    metrics = ["L1", "L2"]
    envs = ["E0", "E1", "E2", "E3", "E4"]
    resoluciones = [15.0]  # Por defecto evaluar a 15.0 grados para mantener el benchmark viable
    num_pairs = 100

    pairs_filepath = os.path.join(resultados_dir, "benchmark_pairs.json")
    env_pairs = {}
    
    if os.path.exists(pairs_filepath):
        print(f"🚀 Cargando pares de benchmark desde: {pairs_filepath}")
        with open(pairs_filepath, 'r') as f_pairs:
            serialized_pairs = json.load(f_pairs)
        # Reconstruir la estructura con tipos correctos (float para resolución y tuplas para coordenadas)
        for env, res_dict in serialized_pairs.items():
            env_pairs[env] = {}
            for res_str, pairs_list in res_dict.items():
                res_val = float(res_str)
                env_pairs[env][res_val] = [(tuple(p[0]), tuple(p[1])) for p in pairs_list]
    else:
        print("🚀 Generando nuevos pares de inicio y meta aleatorios válidos...")
        for env in envs:
            env_pairs[env] = {}
            env_hash = config.ENVIRONMENT_HASHES[env]
            for res in resoluciones:
                cache_filename = f"cspace_cache_{res}deg_0.015m_{env_hash}_singularity0.0005.json"
                cache_filepath = os.path.join(cache_dir, cache_filename)
                
                if not os.path.exists(cache_filepath):
                    print(f"Error: No se encontró la caché {cache_filename} para generar los pares del benchmark.")
                    sys.exit(1)
                    
                with open(cache_filepath, 'r') as f:
                    cache_data = json.load(f)
                forbidden_list = cache_data.get('forbidden_voxels', [])
                
                grid = GridDiscretizer(step_size_deg=res, num_dof=3)
                forbidden_set = set(grid.discretize(tuple(voxel)) for voxel in forbidden_list)
                
                pairs = []
                while len(pairs) < num_pairs:
                    start = generate_random_valid_state(grid, forbidden_set)
                    goal = generate_random_valid_state(grid, forbidden_set)
                    if start is not None and goal is not None and start != goal:
                        pairs.append((start, goal))
                env_pairs[env][res] = pairs
        
        # Guardar los pares generados en el archivo JSON
        serialized_pairs = {}
        for env, res_dict in env_pairs.items():
            serialized_pairs[env] = {}
            for res_val, pairs_list in res_dict.items():
                serialized_pairs[env][str(res_val)] = pairs_list
        with open(pairs_filepath, 'w') as f_pairs:
            json.dump(serialized_pairs, f_pairs, indent=4)
        print(f"🎉 Nuevos pares de benchmark guardados en: {pairs_filepath}")

    print("🚀 Iniciando la suite de benchmarks...")
    total_runs = len(algs) * len(metrics) * len(envs) * len(resoluciones) * num_pairs
    current_run = 0
    successes = 0

    with open(csv_filepath, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        
        for env in envs:
            env_hash = config.ENVIRONMENT_HASHES[env]
            for res in resoluciones:
                # Inicializar componentes comunes de colisión y cinemática
                cache_filename = f"cspace_cache_{res}deg_0.015m_{env_hash}_singularity0.0005.json"
                cache_filepath = os.path.join(cache_dir, cache_filename)
                
                with open(cache_filepath, 'r') as f_cache:
                    cache_data = json.load(f_cache)
                forbidden_list = cache_data.get('forbidden_voxels', [])
                
                grid = GridDiscretizer(step_size_deg=res, num_dof=3)
                forbidden_set = set(grid.discretize(tuple(voxel)) for voxel in forbidden_list)
                
                collider = FoamCollider(sphere_thinning_dist=0.015)
                collider.set_cspace_cache(forbidden_set, grid)
                collider.set_joint_transforms(
                    offset_base_yaw=0.559643,
                    offset_shoulder_pitch=1.5707963,
                    offset_elbow_pitch=0.0,
                    dir_base_yaw=-1.0,
                    dir_shoulder_pitch=-1.0,
                    dir_elbow_pitch=1.0
                )
                link_lengths = {
                    'base_height': 0.130,
                    'lower_shank': 0.140,
                    'upper_shank': 0.140,
                    'gripper_dx': 0.05467,
                    'gripper_dz': -0.0217,
                    'gripper_k_elbow': 0.0
                }
                kinematics = CommunityArmKinematics(use_horizontal_constraint=False, link_lengths=link_lengths)
                pairs = env_pairs[env][res]

                for alg in algs:
                    for metric in metrics:
                        planner = PlannerFactory.create_planner(
                            planner_type=alg,
                            space=grid,
                            collider=collider,
                            kinematics=kinematics,
                            metric_type=metric
                        )
                        
                        if alg == "rrt":
                            planner.max_samples = 10000
                            planner.step_size = 0.15
                            planner.goal_bias = 0.05
                            planner.goal_tolerance = 0.2
                        
                        for idx, (start_discrete, goal_discrete) in enumerate(pairs):
                            current_run += 1
                            if current_run % 50 == 0:
                                print(f"Progreso: [{current_run}/{total_runs}] {alg.upper()} | {metric} | {env} | Par #{idx+1}...")

                            t_start = time.perf_counter()
                            path = planner.plan(start_discrete, goal_discrete)
                            t_end = time.perf_counter()
                            
                            planning_time = t_end - t_start
                            exito = path is not None and len(path) > 0
                            
                            path_len = 0.0
                            duration = 0.0
                            jerk_sum = 0.0
                            w_min = 0.0
                            w_mean = 0.0
                            
                            if exito:
                                successes += 1
                                for j in range(len(path) - 1):
                                    path_len += Metrics.heuristic_L2(np.array(path[j]), np.array(path[j+1]))
                                duration, jerk_sum, w_min, w_mean = evaluate_trajectory(path, kinematics)
                                
                            writer.writerow([
                                alg, metric, env, res, idx + 1,
                                list(start_discrete), list(goal_discrete),
                                int(exito), round(planning_time, 5), round(path_len, 4),
                                round(duration, 3), round(jerk_sum, 2), round(w_min, 5), round(w_mean, 5)
                            ])
                            
    print("\n🎉 Benchmark finalizado con éxito!")
    print(f"Resultados guardados en: {csv_filepath}")
    print(f"Tasa de éxito general: {successes}/{total_runs} planificaciones ({successes/total_runs*100:.1f}%)")

if __name__ == "__main__":
    main()
