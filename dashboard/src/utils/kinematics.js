import { guiJointPub, jointOffsets, jointDirections } from '../services/ros';

// Bi-directional Kinematics Utility for T^3 Community Robot Arm (Parallelogram mechanism)

export function computeFK(q1_deg, q2_deg, q3_deg) {
    const deg2rad = Math.PI / 180.0;
    const q1_rad = (parseFloat(q1_deg) || 0) * deg2rad;
    const q2_rad = (parseFloat(q2_deg) || 0) * deg2rad;
    const q3_rad = (parseFloat(q3_deg) || 0) * deg2rad;

    const base_height = 0.130;
    const lower_shank = 0.140;
    const upper_shank = 0.140;
    const gripper_dx = 0.05467;
    const gripper_dz = -0.0217;

    const theta2 = q2_rad - Math.PI / 2.0;
    const theta3 = theta2 - q3_rad;

    const c1 = Math.cos(q1_rad);
    const s1 = Math.sin(q1_rad);
    const c2 = Math.cos(theta2);
    const s2 = Math.sin(theta2);
    const c3 = Math.cos(theta3);
    const s3 = Math.sin(theta3);

    const r_offset = gripper_dx;
    const z_offset = gripper_dz;

    const r = lower_shank * c2 + upper_shank * c3 + r_offset;
    const x = r * c1;
    const y = r * s1;
    const z = base_height + lower_shank * s2 + upper_shank * s3 + z_offset;

    return {
        x_cm: (x * 100).toFixed(1),
        y_cm: (y * 100).toFixed(1),
        z_cm: (z * 100).toFixed(1),
        x_m: x,
        y_m: y,
        z_m: z
    };
}

export function computeIK(x_cm, y_cm, z_cm) {
    const x_m = (parseFloat(x_cm) || 0) / 100.0;
    const y_m = (parseFloat(y_cm) || 0) / 100.0;
    const z_m = (parseFloat(z_cm) || 0) / 100.0;

    const base_height = 0.130;
    const L1 = 0.140;
    const L2 = 0.140;
    const gripper_dx = 0.05467;
    const gripper_dz = -0.0217;

    const r_xy = Math.sqrt(x_m * x_m + y_m * y_m);
    let q1_rad = Math.atan2(y_m, x_m);

    let r_wrist = r_xy - gripper_dx;
    if (r_wrist < 0) {
        r_wrist = -r_wrist;
        q1_rad = q1_rad + Math.PI;
    }

    const z_wrist = z_m - gripper_dz;
    const z_prime = z_wrist - base_height;

    let cos_delta = (r_wrist * r_wrist + z_prime * z_prime - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    cos_delta = Math.max(-1.0, Math.min(1.0, cos_delta));

    const sin_delta_mag = Math.sqrt(1.0 - cos_delta * cos_delta);
    // Elbow up branch
    const delta_theta = Math.atan2(-sin_delta_mag, cos_delta);

    const A = L1 + L2 * cos_delta;
    const B = L2 * Math.sin(delta_theta);

    const num_theta2 = A * z_prime - B * r_wrist;
    const den_theta2 = A * r_wrist + B * z_prime;

    const theta2 = Math.atan2(num_theta2, den_theta2);
    const q2_rad = theta2 + Math.PI / 2.0;
    const q3_rad = -delta_theta;

    const rad2deg = 180.0 / Math.PI;

    let q1_deg = Math.round(q1_rad * rad2deg);
    let q2_deg = Math.round(q2_rad * rad2deg);
    let q3_deg = Math.round(q3_rad * rad2deg);

    // Normalize to standard ranges
    q1_deg = ((q1_deg + 180) % 360) - 180;
    q2_deg = ((q2_deg + 180) % 360) - 180;
    q3_deg = ((q3_deg + 180) % 360) - 180;

    return {
        q1: q1_deg,
        q2: q2_deg,
        q3: q3_deg
    };
}

export function publishJoints(q1_deg, q2_deg, q3_deg) {
    const deg2rad = Math.PI / 180.0;
    const q1_rad = parseFloat(q1_deg) * deg2rad;
    const q2_rad = parseFloat(q2_deg) * deg2rad;
    const q3_rad = parseFloat(q3_deg) * deg2rad;

    const offsetBaseYawRad = jointOffsets.base_yaw * Math.PI / 180.0;
    const offsetShoulderPitchRad = jointOffsets.shoulder_pitch * Math.PI / 180.0;
    const offsetElbowPitchRad = jointOffsets.elbow_pitch * Math.PI / 180.0;

    const urdf_q1 = offsetBaseYawRad + jointDirections.base_yaw * q1_rad;
    const urdf_q2 = offsetShoulderPitchRad + jointDirections.shoulder_pitch * q2_rad;
    const q3_relative_rad = offsetElbowPitchRad + jointDirections.elbow_pitch * q3_rad;
    const urdf_q3 = -urdf_q2 - q3_relative_rad;

    guiJointPub.publish({
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
        name: ['base_yaw_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint'],
        position: [urdf_q1, urdf_q2, urdf_q3],
        velocity: [],
        effort: []
    });
}
