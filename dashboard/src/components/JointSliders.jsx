import React, { useState, useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib';
import { guiJointPub, webCmdPub, jointOffsets, jointDirections } from '../services/ros';

export default function JointSliders({ currentQ, firstPositionReceived }) {
    // Home position in World frame degrees (matches planner_params.yaml start config)
    const DEFAULT_HOME = { q1: 0, q2: 90, q3: 0 };

    const [q1, setQ1] = useState(DEFAULT_HOME.q1);
    const [q2, setQ2] = useState(DEFAULT_HOME.q2);
    const [q3, setQ3] = useState(DEFAULT_HOME.q3);
    const [userDragging, setUserDragging] = useState(false);
    const [homeQ, setHomeQ] = useState(DEFAULT_HOME);
    const initialPublishRef = useRef(false);
    const dragTimeoutRef = useRef(null);
    // Stores the World-frame degrees we commanded the robot to move to.
    // Sliders are frozen until the robot reaches this position (position-based, not time-based).
    const commandedQRef = useRef(null);

    // Publish the home position on first mount so the robot starts at World home
    useEffect(() => {
        if (!initialPublishRef.current && firstPositionReceived) {
            commandedQRef.current = { q1: homeQ.q1, q2: homeQ.q2, q3: homeQ.q3 };
            publishSliderJointState(homeQ.q1, homeQ.q2, homeQ.q3);
            initialPublishRef.current = true;
        }
    }, [firstPositionReceived]);

    // Update sliders when robot moves.
    // - If user is dragging: blocked (userDragging flag).
    // - If a command is pending: blocked until robot arrives at commanded position.
    // - Otherwise: track robot state normally.
    const ARRIVE_TOLERANCE_DEG = 9; // Slightly above grid resolution (8°)
    useEffect(() => {
        if (!currentQ) return;
        const rad2deg = 180.0 / Math.PI;
        const worldQ1 = currentQ[0] * rad2deg;
        const worldQ2 = currentQ[1] * rad2deg;
        const worldQ3 = currentQ[2] * rad2deg;

        if (commandedQRef.current) {
            const cmd = commandedQRef.current;
            const arrived =
                Math.abs(worldQ1 - cmd.q1) < ARRIVE_TOLERANCE_DEG &&
                Math.abs(worldQ2 - cmd.q2) < ARRIVE_TOLERANCE_DEG &&
                Math.abs(worldQ3 - cmd.q3) < ARRIVE_TOLERANCE_DEG;
            if (arrived) {
                commandedQRef.current = null; // Robot reached target, resume tracking
            } else {
                return; // Still moving toward command, keep sliders frozen
            }
        }

        if (!userDragging) {
            setQ1(Math.round(worldQ1));
            setQ2(Math.round(worldQ2));
            setQ3(Math.round(worldQ3));
        }
    }, [currentQ, userDragging]);

    const publishSliderJointState = (sQ1, sQ2, sQ3) => {
        const deg2rad = Math.PI / 180.0;

        // Slider values in World coordinates (radians)
        const q1_rad = sQ1 * deg2rad;
        const q2_rad = sQ2 * deg2rad;
        const q3_rad = sQ3 * deg2rad;

        // Convert World to URDF using loaded parameters
        const offsetBaseYawRad = jointOffsets.base_yaw * Math.PI / 180.0;
        const offsetShoulderPitchRad = jointOffsets.shoulder_pitch * Math.PI / 180.0;
        const offsetElbowPitchRad = jointOffsets.elbow_pitch * Math.PI / 180.0;

        const urdf_q1 = offsetBaseYawRad + jointDirections.base_yaw * q1_rad;
        const urdf_q2 = offsetShoulderPitchRad + jointDirections.shoulder_pitch * q2_rad;
        const urdf_q3 = offsetElbowPitchRad + jointDirections.elbow_pitch * q3_rad;

        guiJointPub.publish({
            header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
            name: ['base_yaw_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint'],
            position: [urdf_q1, urdf_q2, urdf_q3],
            velocity: [],
            effort: []
        });
    };

    const handleSliderChange = (jointIdx, val) => {
        const numVal = parseFloat(val);
        // Clear any pending command since user is taking manual control
        commandedQRef.current = null;
        setUserDragging(true);

        let nextQ1 = q1;
        let nextQ2 = q2;
        let nextQ3 = q3;

        if (jointIdx === 1) {
            nextQ1 = numVal;
            setQ1(numVal);
        } else if (jointIdx === 2) {
            nextQ2 = numVal;
            setQ2(numVal);
        } else if (jointIdx === 3) {
            nextQ3 = numVal;
            setQ3(numVal);
        }

        publishSliderJointState(nextQ1, nextQ2, nextQ3);

        // Reset dragging flag after 500ms of inactivity
        clearTimeout(dragTimeoutRef.current);
        dragTimeoutRef.current = setTimeout(() => {
            setUserDragging(false);
        }, 500);
    };

    const handleSetHome = () => {
        setHomeQ({ q1, q2, q3 });
    };

    const handleReset = () => {
        if (homeQ) {
            setQ1(homeQ.q1);
            setQ2(homeQ.q2);
            setQ3(homeQ.q3);
            commandedQRef.current = { q1: homeQ.q1, q2: homeQ.q2, q3: homeQ.q3 };
            // Send explicit go_to_position command so the planner moves even if
            // the home position equals the last GUI command (avoids false no-change detection)
            webCmdPub.publish({
                data: JSON.stringify({
                    action: 'go_to_position',
                    q: [homeQ.q1, homeQ.q2, homeQ.q3]
                })
            });
            // Also update the joint_state_publisher pipeline
            publishSliderJointState(homeQ.q1, homeQ.q2, homeQ.q3);
        }
    };

    return (
        <div className="card">
            <h2>Joint Control (World Frame)</h2>
            
            <div className="slider-group">
                <div className="slider-header">
                    <span>Base Yaw (θ₁)</span>
                    <span className="value-display">{q1.toFixed(1)}°</span>
                </div>
                <input
                    type="range"
                    min="-180"
                    max="180"
                    value={q1}
                    onChange={(e) => handleSliderChange(1, e.target.value)}
                    className="slider"
                />
            </div>

            <div className="slider-group">
                <div className="slider-header">
                    <span>Shoulder Pitch (θ₂)</span>
                    <span className="value-display">{q2.toFixed(1)}°</span>
                </div>
                <input
                    type="range"
                    min="-180"
                    max="180"
                    value={q2}
                    onChange={(e) => handleSliderChange(2, e.target.value)}
                    className="slider"
                />
            </div>

            <div className="slider-group">
                <div className="slider-header">
                    <span>Elbow Pitch (θ₃)</span>
                    <span className="value-display">{q3.toFixed(1)}°</span>
                </div>
                <input
                    type="range"
                    min="-180"
                    max="180"
                    value={q3}
                    onChange={(e) => handleSliderChange(3, e.target.value)}
                    className="slider"
                />
            </div>

            <div style={{ display: 'flex', gap: '10px', marginTop: '10px' }}>
                <button 
                    onClick={handleSetHome} 
                    className="btn btn-primary" 
                    style={{ flex: 1 }}
                >
                    Set Home
                </button>
                <button 
                    onClick={handleReset} 
                    className="btn btn-secondary" 
                    disabled={!homeQ}
                    style={{ flex: 1 }}
                >
                    Reset to Home
                </button>
            </div>
        </div>
    );
}
