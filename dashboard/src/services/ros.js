import * as ROSLIB from 'roslib';

export const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

export const webCmdPub = new ROSLIB.Topic({
    ros: ros,
    name: '/web_commands',
    messageType: 'std_msgs/String'
});

export const jointSub = new ROSLIB.Topic({
    ros: ros,
    name: '/joint_states',
    messageType: 'sensor_msgs/JointState'
});

export const guiJointPub = new ROSLIB.Topic({
    ros: ros,
    name: '/web_gui_master_states',
    messageType: 'sensor_msgs/JointState'
});

export const voxelSub = new ROSLIB.Topic({
    ros: ros,
    name: '/cspace_voxels',
    messageType: 'std_msgs/String'
});

export const statusSub = new ROSLIB.Topic({
    ros: ros,
    name: '/planner_status',
    messageType: 'std_msgs/String'
});

// Joint offset and direction configurations matching planner_params.yaml
// These define the mapping between World frame and URDF frame:
//   q_urdf = offset + direction * q_world
export const jointOffsets = {
    base_yaw: 32.0694,
    shoulder_pitch: 90.0,
    elbow_pitch: 0.0
};

export const jointDirections = {
    base_yaw: -1,
    shoulder_pitch: -1,
    elbow_pitch: 1
};

export function loadParameters(onLoadCallback) {
    if (onLoadCallback) onLoadCallback();
}
