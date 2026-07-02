use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::io::{self, Read};
use std::thread;

type Matrix4 = [[f32; 4]; 4];
type Vector3 = [f32; 3];

#[derive(Deserialize)]
struct JointInput {
    name: String,
    parent: String,
    child: String,
    static_t: Matrix4,
    axis: Vector3,
}

#[derive(Deserialize)]
struct SphereInput {
    link: String,
    local_c: Vector3,
    radius: f32,
}

#[derive(Deserialize)]
struct ObstacleInput {
    center: Vector3,
    radius: f32,
}

#[derive(Deserialize)]
struct InputData {
    joints: Vec<JointInput>,
    root_link: String,
    thinned_spheres: Vec<SphereInput>,
    active_pairs: Vec<(usize, usize)>,
    obstacles: Vec<ObstacleInput>,
    steps_per_circle: usize,
    num_dof: usize,
    step_rad: f32,
    offset_base_yaw: f32,
    offset_shoulder_pitch: f32,
    offset_elbow_pitch: f32,
    dir_base_yaw: f32,
    dir_shoulder_pitch: f32,
    dir_elbow_pitch: f32,
}

#[derive(Serialize)]
struct OutputData {
    forbidden_voxels: Vec<[f32; 3]>,
    self_collision_voxels: Vec<[f32; 3]>,
    obstacle_voxels: Vec<[f32; 3]>,
}

fn mul_m4_m4(a: &Matrix4, b: &Matrix4) -> Matrix4 {
    let mut out = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] + a[i][3] * b[3][j];
        }
    }
    out
}

fn mul_m4_v3(m: &Matrix4, v: &Vector3) -> Vector3 {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2] + m[0][3],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2] + m[1][3],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2] + m[2][3],
    ]
}

fn axis_angle_to_matrix(axis: &Vector3, theta: f32) -> Matrix4 {
    let norm = (axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]).sqrt();
    if norm < 1e-6 {
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
    }
    let u = [axis[0]/norm, axis[1]/norm, axis[2]/norm];
    let s = theta.sin();
    let c = theta.cos();
    let c1 = 1.0 - c;
    
    [
        [c + u[0]*u[0]*c1,       u[0]*u[1]*c1 - u[2]*s, u[0]*u[2]*c1 + u[1]*s, 0.0],
        [u[1]*u[0]*c1 + u[2]*s, c + u[1]*u[1]*c1,       u[1]*u[2]*c1 - u[0]*s, 0.0],
        [u[2]*u[0]*c1 - u[1]*s, u[2]*u[1]*c1 + u[0]*s, c + u[2]*u[2]*c1,       0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn compute_transforms(
    q1: f32, q2: f32, q3: f32,
    joints: &[JointInput],
    root_link: &str,
) -> HashMap<String, Matrix4> {
    let mut transforms = HashMap::new();
    transforms.insert(root_link.to_string(), [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    
    // Joint angles map
    let mut joint_angles = HashMap::new();
    joint_angles.insert("base_yaw_joint".to_string(), q1);
    joint_angles.insert("shoulder_pitch_joint".to_string(), q2);
    joint_angles.insert("elbow_pitch_joint".to_string(), q3);
    // Mimic coupled joints for parallelogram
    joint_angles.insert("revolute_16_0".to_string(), -q3 - q2);
    joint_angles.insert("revolute_12_0".to_string(), q2 + q3);
    joint_angles.insert("revolute_32_0".to_string(), q2);
    joint_angles.insert("revolute_31_0".to_string(), -q2);
    joint_angles.insert("revolute_13_0".to_string(), -q2);
    joint_angles.insert("revolute_18_0".to_string(), q2);
    joint_angles.insert("revolute_15_0".to_string(), -q3);
    joint_angles.insert("revolute_19_0".to_string(), q3);

    // BFS/DFS propagation (loop until no new child can be resolved)
    let mut resolved = true;
    while resolved {
        resolved = false;
        for joint in joints {
            if transforms.contains_key(&joint.parent) && !transforms.contains_key(&joint.child) {
                let p_t = transforms[&joint.parent];
                let angle = joint_angles.get(&joint.name).cloned().unwrap_or(0.0);
                let r_joint = axis_angle_to_matrix(&joint.axis, angle);
                let t_joint = mul_m4_m4(&joint.static_t, &r_joint);
                let child_t = mul_m4_m4(&p_t, &t_joint);
                transforms.insert(joint.child.clone(), child_t);
                resolved = true;
            }
        }
    }
    transforms
}

fn wrap_to_pi(val: f32) -> f32 {
    let mut v = (val + std::f32::consts::PI) % (2.0 * std::f32::consts::PI);
    if v < 0.0 {
        v += 2.0 * std::f32::consts::PI;
    }
    v - std::f32::consts::PI
}

fn main() {
    // Read input data from stdin
    let mut input_buf = String::new();
    if io::stdin().read_to_string(&mut input_buf).is_err() {
        eprintln!("Error reading from stdin");
        std::process::exit(1);
    }
    
    let data: InputData = match serde_json::from_str(&input_buf) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("Error parsing JSON input: {}", e);
            std::process::exit(1);
        }
    };
    
    // Generate all discrete states
    let mut states = Vec::new();
    if data.num_dof == 2 {
        for i in 0..data.steps_per_circle {
            for j in 0..data.steps_per_circle {
                states.push((i, j, 0));
            }
        }
    } else {
        for i in 0..data.steps_per_circle {
            for j in 0..data.steps_per_circle {
                for k in 0..data.steps_per_circle {
                    states.push((i, j, k));
                }
            }
        }
    }
    
    // Determine the number of threads based on CPU cores
    let num_threads = thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(4);
    
    let chunk_size = (states.len() + num_threads - 1) / num_threads;
    
    // Wrap shared data in Arc
    use std::sync::Arc;
    let joints = Arc::new(data.joints);
    let root_link = Arc::new(data.root_link);
    let thinned_spheres = Arc::new(data.thinned_spheres);
    let active_pairs = Arc::new(data.active_pairs);
    let obstacles = Arc::new(data.obstacles);
    let states = Arc::new(states);
    
    let mut handles = Vec::new();
    for thread_idx in 0..num_threads {
        let joints = Arc::clone(&joints);
        let root_link = Arc::clone(&root_link);
        let thinned_spheres = Arc::clone(&thinned_spheres);
        let active_pairs = Arc::clone(&active_pairs);
        let obstacles = Arc::clone(&obstacles);
        let states = Arc::clone(&states);
        let step_rad = data.step_rad;
        let num_dof = data.num_dof;
        
        let offset_base_yaw = data.offset_base_yaw;
        let offset_shoulder_pitch = data.offset_shoulder_pitch;
        let offset_elbow_pitch = data.offset_elbow_pitch;
        let dir_base_yaw = data.dir_base_yaw;
        let dir_shoulder_pitch = data.dir_shoulder_pitch;
        let dir_elbow_pitch = data.dir_elbow_pitch;
        
        let start_idx = thread_idx * chunk_size;
        let end_idx = std::cmp::min(start_idx + chunk_size, states.len());
        
        handles.push(thread::spawn(move || {
            let mut thread_forbidden = Vec::new();
            let mut thread_self = Vec::new();
            let mut thread_obstacle = Vec::new();
            for idx in start_idx..end_idx {
                let (i, j, k) = states[idx];
                let q1_world = (i as f32) * step_rad;
                let q2_world = (j as f32) * step_rad;
                let q3_world = if num_dof > 2 { (k as f32) * step_rad } else { 0.0 };
                
                let q1_urdf = offset_base_yaw + dir_base_yaw * q1_world;
                let q2_urdf = offset_shoulder_pitch + dir_shoulder_pitch * q2_world;
                let q3_urdf = offset_elbow_pitch + dir_elbow_pitch * q3_world;
                
                let tfs = compute_transforms(q1_urdf, q2_urdf, q3_urdf, &joints, &root_link);
                
                // Project robot spheres to world coordinates
                let mut world_spheres = Vec::with_capacity(thinned_spheres.len());
                let mut valid_kinematics = true;
                for s in thinned_spheres.iter() {
                    if let Some(t) = tfs.get(&s.link) {
                        let world_c = mul_m4_v3(t, &s.local_c);
                        world_spheres.push((world_c, s.radius));
                    } else {
                        valid_kinematics = false;
                        break;
                    }
                }
                
                if !valid_kinematics {
                    continue;
                }
                
                let mut is_self_collision = false;
                
                // 1. Floor collision check (Z < 0) for moving links
                for (idx, s) in thinned_spheres.iter().enumerate() {
                    let link_lower = s.link.to_lowercase();
                    let is_base = link_lower.contains("basering")
                        || link_lower.contains("leg")
                        || link_lower.contains("main_body")
                        || link_lower.contains("stepper_motor")
                        || link_lower.contains("stabilizer")
                        || link_lower.contains("socket");
                    if !is_base {
                        let (world_c, radius) = world_spheres[idx];
                        if world_c[2] - radius < 0.0 {
                            is_self_collision = true;
                            break;
                        }
                    }
                }
                
                // 2. Self-collision checks
                if !is_self_collision {
                    for &(pair_i, pair_j) in active_pairs.iter() {
                        if pair_i >= world_spheres.len() || pair_j >= world_spheres.len() {
                            continue;
                        }
                        let (c_i, r_i) = world_spheres[pair_i];
                        let (c_j, r_j) = world_spheres[pair_j];
                        let dx = c_i[0] - c_j[0];
                        let dy = c_i[1] - c_j[1];
                        let dz = c_i[2] - c_j[2];
                        let dist_sq = dx*dx + dy*dy + dz*dz;
                        let limit = r_i + r_j;
                        if dist_sq < limit * limit {
                            is_self_collision = true;
                            break;
                        }
                    }
                }
                
                let voxel = [
                    (wrap_to_pi(q1_world) * 1000.0).round() / 1000.0,
                    (wrap_to_pi(q2_world) * 1000.0).round() / 1000.0,
                    (wrap_to_pi(q3_world) * 1000.0).round() / 1000.0,
                ];

                if is_self_collision {
                    thread_forbidden.push(voxel);
                    thread_self.push(voxel);
                } else {
                    // 2. Obstacle collision checks
                    let mut is_obstacle_collision = false;
                    for obs in obstacles.iter() {
                        for &(c_w, r_w) in world_spheres.iter() {
                            let dx = c_w[0] - obs.center[0];
                            let dy = c_w[1] - obs.center[1];
                            let dz = c_w[2] - obs.center[2];
                            let dist_sq = dx*dx + dy*dy + dz*dz;
                            let limit = r_w + obs.radius;
                            if dist_sq < limit * limit {
                                is_obstacle_collision = true;
                                break;
                            }
                        }
                        if is_obstacle_collision {
                            break;
                        }
                    }
                    if is_obstacle_collision {
                        thread_forbidden.push(voxel);
                        thread_obstacle.push(voxel);
                    }
                }
            }
            (thread_forbidden, thread_self, thread_obstacle)
        }));
    }
    
    let mut all_forbidden = Vec::new();
    let mut all_self = Vec::new();
    let mut all_obstacle = Vec::new();
    for h in handles {
        if let Ok((mut tf, mut ts, mut to)) = h.join() {
            all_forbidden.append(&mut tf);
            all_self.append(&mut ts);
            all_obstacle.append(&mut to);
        }
    }
    
    let output = OutputData {
        forbidden_voxels: all_forbidden,
        self_collision_voxels: all_self,
        obstacle_voxels: all_obstacle,
    };
    
    if let Ok(serialized) = serde_json::to_string(&output) {
        println!("{}", serialized);
    } else {
        eprintln!("Error serializing output JSON");
        std::process::exit(1);
    }
}
