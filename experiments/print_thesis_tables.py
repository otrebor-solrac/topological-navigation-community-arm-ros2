import os
import pandas as pd
import numpy as np

raw_path = '/home/rc/workspace/ROS2/experiments/resultados/resultados_raw.csv'
bench_path = '/home/rc/workspace/ROS2/experiments/resultados/resultados_benchmark.csv'

if not os.path.exists(raw_path):
    print(f"Error: No existe {raw_path}")
    exit(1)

df_raw = pd.read_csv(raw_path)
df_6 = df_raw[df_raw['resolucion_deg'] == 6.0].copy()

# Determinar éxito de corrida completa (todos los segmentos exitosos en la misma repetición)
# Agrupamos por algoritmo, metrica, entorno, repeticion
rep_exito = df_6.groupby(['algoritmo', 'metrica', 'entorno', 'repeticion'])['exito'].transform(lambda x: (x == 1).all())
df_6['corrida_exitosa'] = rep_exito

# Sumar tiempos y longitudes por corrida (repetición)
corrida_metrics = df_6.groupby(['algoritmo', 'metrica', 'entorno', 'repeticion', 'corrida_exitosa']).agg({
    'tiempo_planificacion_s': 'sum',
    'longitud_geometrica_rad': 'sum',
    'longitud_continua_rad': 'sum',
    'longitud_cartesiana_m': 'sum',
    'duracion_trayectoria_s': 'sum',
    'jerk_acumulado': 'mean', # Promedio de los segmentos
    'manipulabilidad_min': 'min', # El mínimo absoluto de la corrida
    'manipulabilidad_mean': 'mean' # El promedio de la corrida
}).reset_index()

envs = ["E0", "E1", "E2", "E3", "E4"]
algs = [("astar", "L1"), ("astar", "L2"), ("rrt", "L1"), ("rrt", "L2")]

print("=================================================================")
print("TABLA 4.5: Calidad de Trayectorias a 6.0° (Toroidal / Cartesiana)")
print("=================================================================")
for env in envs:
    print(f"\n--- Entorno: {env} ---")
    for alg, met in algs:
        sub = corrida_metrics[(corrida_metrics['algoritmo'] == alg) & 
                              (corrida_metrics['metrica'] == met) & 
                              (corrida_metrics['entorno'] == env) & 
                              (corrida_metrics['corrida_exitosa'] == True)]
        if len(sub) > 0:
            lt_m = sub['longitud_geometrica_rad'].mean()
            lt_s = sub['longitud_geometrica_rad'].std()
            lc_m = sub['longitud_cartesiana_m'].mean()
            lc_s = sub['longitud_cartesiana_m'].std()
            print(f"  {alg.upper()} {met:<2}: Toroidal = {lt_m:.3f} ± {lt_s:.3f} rad | Cartesiana = {lc_m:.3f} ± {lc_s:.3f} m")
        else:
            print(f"  {alg.upper()} {met:<2}: SIN TRAYECTORIAS EXITOSAS")

print("\n=================================================================")
print("TABLA 4.6: Desempeño Computacional a 6.0° (Éxito / CPU)")
print("=================================================================")
for env in envs:
    print(f"\n--- Entorno: {env} ---")
    for alg, met in algs:
        sub_all = corrida_metrics[(corrida_metrics['algoritmo'] == alg) & 
                                  (corrida_metrics['metrica'] == met) & 
                                  (corrida_metrics['entorno'] == env)]
        sub_ok = sub_all[sub_all['corrida_exitosa'] == True]
        
        success_rate = (sub_all['corrida_exitosa'].mean()) * 100
        
        if len(sub_ok) > 0:
            cpu_m = sub_ok['tiempo_planificacion_s'].mean()
            cpu_s = sub_ok['tiempo_planificacion_s'].std()
            jerk_m = sub_ok['jerk_acumulado'].mean()
            print(f"  {alg.upper()} {met:<2}: Éxito = {success_rate:.1f}% | CPU = {cpu_m:.5f} ± {cpu_s:.5f} s | Jerk = {jerk_m:.1f}")
        else:
            print(f"  {alg.upper()} {met:<2}: Éxito = {success_rate:.1f}% | CPU = N/A | Jerk = N/A")

print("\n=================================================================")
print("TABLA 4.9: Suavidad Dinámica a 6.0° (Jerk Acumulado)")
print("=================================================================")
for env in envs:
    print(f"\n--- Entorno: {env} ---")
    for alg, met in algs:
        sub = corrida_metrics[(corrida_metrics['algoritmo'] == alg) & 
                              (corrida_metrics['metrica'] == met) & 
                              (corrida_metrics['entorno'] == env) & 
                              (corrida_metrics['corrida_exitosa'] == True)]
        if len(sub) > 0:
            jerk_m = sub['jerk_acumulado'].mean()
            jerk_s = sub['jerk_acumulado'].std()
            print(f"  {alg.upper()} {met:<2}: Jerk Acumulado = {jerk_m:.1f} ± {jerk_s:.1f}")
        else:
            print(f"  {alg.upper()} {met:<2}: SIN TRAYECTORIAS EXITOSAS")

print("\n=================================================================")
print("TABLA 4.10: Estabilidad Cinemática a 6.0° (Yoshikawa Min / Med)")
print("=================================================================")
for env in envs:
    print(f"\n--- Entorno: {env} ---")
    for alg, met in algs:
        sub = corrida_metrics[(corrida_metrics['algoritmo'] == alg) & 
                              (corrida_metrics['metrica'] == met) & 
                              (corrida_metrics['entorno'] == env) & 
                              (corrida_metrics['corrida_exitosa'] == True)]
        if len(sub) > 0:
            w_min = sub['manipulabilidad_min'].min()
            w_mean = sub['manipulabilidad_mean'].mean()
            print(f"  {alg.upper()} {met:<2}: w_min = {w_min:.6f} | w_mean = {w_mean:.6f}")
        else:
            print(f"  {alg.upper()} {met:<2}: SIN TRAYECTORIAS EXITOSAS")

# --- BENCHMARK ---
if os.path.exists(bench_path):
    print("\n=================================================================")
    print("BENCHMARK ESTADÍSTICO GLOBAL (100 Pares a 15.0°)")
    print("=================================================================")
    df_bench = pd.read_csv(bench_path)
    df_b_ok = df_bench[df_bench['exito'] == 1]
    
    for alg in ["astar", "rrt"]:
        for met in ["L1", "L2"]:
            sub_all = df_bench[(df_bench['algoritmo'] == alg) & (df_bench['metrica'] == met)]
            sub_ok = df_b_ok[(df_b_ok['algoritmo'] == alg) & (df_b_ok['metrica'] == met)]
            
            success_rate = (sub_all['exito'].mean()) * 100
            
            if len(sub_ok) > 0:
                cpu_m = sub_all['tiempo_planificacion_s'].mean()
                cpu_s = sub_all['tiempo_planificacion_s'].std()
                
                lt_m = sub_ok['longitud_geometrica_rad'].mean()
                lt_s = sub_ok['longitud_geometrica_rad'].std()
                
                jerk_m = sub_ok['jerk_acumulado'].mean()
                jerk_s = sub_ok['jerk_acumulado'].std()
                
                w_min = sub_ok['manipulabilidad_min'].min()
                w_mean = sub_ok['manipulabilidad_mean'].mean()
                
                print(f"{alg.upper()} {met:<2}:")
                print(f"  Tasa de éxito: {success_rate:.1f}%")
                print(f"  Tiempo CPU: {cpu_m:.5f} ± {cpu_s:.5f} s")
                print(f"  Longitud geodésica: {lt_m:.3f} ± {lt_s:.3f} rad")
                print(f"  Jerk acumulado: {jerk_m:.1f} ± {jerk_s:.1f}")
                print(f"  Manipulabilidad (Min / Med): {w_min:.6f} / {w_mean:.6f}\n")
            else:
                print(f"{alg.upper()} {met:<2}: Sin datos exitosos\n")
else:
    print(f"\nAdvertencia: No se encontró {bench_path}")
