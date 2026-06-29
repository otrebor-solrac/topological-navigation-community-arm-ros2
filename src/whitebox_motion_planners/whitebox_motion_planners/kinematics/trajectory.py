import numpy as np
from typing import List, Tuple

class QuinticSplineSegment:
    def __init__(self, q0: np.ndarray, q1: np.ndarray, v0: np.ndarray, v1: np.ndarray, a0: np.ndarray, a1: np.ndarray, T: float):
        self.T = T
        self.q0 = q0
        
        self.c0 = q0
        self.c1 = v0
        self.c2 = 0.5 * a0
        
        if T > 1e-6:
            self.c3 = (20.0 * (q1 - q0) - (8.0 * v1 + 12.0 * v0) * T - (3.0 * a0 - a1) * T**2) / (2.0 * T**3)
            self.c4 = (30.0 * (q0 - q1) + (14.0 * v1 + 16.0 * v0) * T + (3.0 * a0 - 2.0 * a1) * T**2) / (2.0 * T**4)
            self.c5 = (12.0 * (q1 - q0) - 6.0 * (v1 + v0) * T - (a0 - a1) * T**2) / (2.0 * T**5)
        else:
            self.c3 = np.zeros_like(q0)
            self.c4 = np.zeros_like(q0)
            self.c5 = np.zeros_like(q0)

    def evaluate(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        t = max(0.0, min(self.T, t))
        q = self.c0 + self.c1 * t + self.c2 * t**2 + self.c3 * t**3 + self.c4 * t**4 + self.c5 * t**5
        qd = self.c1 + 2.0 * self.c2 * t + 3.0 * self.c3 * t**2 + 4.0 * self.c4 * t**3 + 5.0 * self.c5 * t**4
        qdd = 2.0 * self.c2 + 6.0 * self.c3 * t + 12.0 * self.c4 * t**2 + 20.0 * self.c5 * t**3
        return q, qd, qdd

class TrajectoryGenerator:
    def __init__(self, path: List[tuple], max_vel: float = 1.0, max_acc: float = 1.0):
        if len(path) == 0:
            self.segments = []
            self.times = [0.0]
            self.total_duration = 0.0
            return
            
        self.num_joints = len(path[0])
        
        # Unwrap path waypoints to ensure continuity on the toroidal manifold (T^n)
        unwrapped_waypoints = [np.array(path[0])]
        for i in range(1, len(path)):
            wp_prev = unwrapped_waypoints[-1]
            dq = np.array(path[i]) - np.array(path[i-1])
            # Wrap joint difference to [-pi, pi] to find the shortest path on the torus
            dq_wrapped = (dq + np.pi) % (2 * np.pi) - np.pi
            unwrapped_waypoints.append(wp_prev + dq_wrapped)
            
        self.waypoints = unwrapped_waypoints
        
        self.times = [0.0]
        for i in range(len(self.waypoints) - 1):
            dq = self.waypoints[i+1] - self.waypoints[i]
            dt = np.max(np.abs(dq)) / max_vel
            dt = max(dt, 0.4) 
            self.times.append(self.times[-1] + dt)
            
        self.total_duration = self.times[-1]
        
        velocities = [np.zeros(self.num_joints)]
        for i in range(1, len(self.waypoints) - 1):
            dt_prev = self.times[i] - self.times[i-1]
            dt_next = self.times[i+1] - self.times[i]
            v = (self.waypoints[i+1] - self.waypoints[i-1]) / (dt_prev + dt_next)
            v = np.clip(v, -max_vel, max_vel)
            velocities.append(v)
        if len(self.waypoints) > 1:
            velocities.append(np.zeros(self.num_joints))
            
        accelerations = [np.zeros(self.num_joints)]
        for i in range(1, len(self.waypoints) - 1):
            dt_prev = self.times[i] - self.times[i-1]
            dt_next = self.times[i+1] - self.times[i]
            a = (velocities[i+1] - velocities[i-1]) / (dt_prev + dt_next)
            a = np.clip(a, -max_acc, max_acc)
            accelerations.append(a)
        if len(self.waypoints) > 1:
            accelerations.append(np.zeros(self.num_joints))
            
        self.segments = []
        for i in range(len(self.waypoints) - 1):
            T = self.times[i+1] - self.times[i]
            self.segments.append(
                QuinticSplineSegment(
                    self.waypoints[i],
                    self.waypoints[i+1],
                    velocities[i],
                    velocities[i+1],
                    accelerations[i],
                    accelerations[i+1],
                    T
                )
            )

    def evaluate(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        if len(self.segments) == 0:
            return np.zeros(self.num_joints), np.zeros(self.num_joints), np.zeros(self.num_joints)
            
        t = max(0.0, min(self.total_duration, t))
        
        idx = 0
        for i in range(len(self.segments)):
            if t <= self.times[i+1]:
                idx = i
                break
                
        local_t = t - self.times[idx]
        return self.segments[idx].evaluate(local_t)
