import React, { useState, useEffect, useRef } from 'react';
import { guiJointPub, webCmdPub, jointOffsets, jointDirections } from '../services/ros';

export default function ControlPanel({ currentQ, homeQ, setHomeQ }) {
    // Slider states (in World degrees) — initialized to 0, updated from /joint_states on first message
    const [q1, setQ1] = useState(0);
    const [q2, setQ2] = useState(0);
    const [q3, setQ3] = useState(0);

    const [userDragging, setUserDragging] = useState(false);
    const dragTimeoutRef = useRef(null);
    const commandedQRef = useRef(null);


    // Update sliders when robot moves
    const ARRIVE_TOLERANCE_DEG = 9;
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
                commandedQRef.current = null;
            } else {
                return; // Keep sliders locked
            }
        }

        if (!userDragging) {
            setQ1(Math.round(worldQ1));
            setQ2(Math.round(worldQ2));
            setQ3(Math.round(worldQ3));
        }
    }, [currentQ, userDragging]);

    // Helper to publish sliders to joint_state_publisher (in direct mode)
    const publishSliderJointState = (sQ1, sQ2, sQ3) => {
        const deg2rad = Math.PI / 180.0;
        const val1 = parseFloat(sQ1);
        const val2 = parseFloat(sQ2);
        const val3 = parseFloat(sQ3);

        const q1_rad = (isNaN(val1) ? 0 : val1) * deg2rad;
        const q2_rad = (isNaN(val2) ? 0 : val2) * deg2rad;
        const q3_rad = (isNaN(val3) ? 0 : val3) * deg2rad;

        const offsetBaseYawRad = jointOffsets.base_yaw * Math.PI / 180.0;
        const offsetShoulderPitchRad = jointOffsets.shoulder_pitch * Math.PI / 180.0;
        const offsetElbowPitchRad = jointOffsets.elbow_pitch * Math.PI / 180.0;

        const urdf_q1 = offsetBaseYawRad + jointDirections.base_yaw * q1_rad;
        const urdf_q2 = offsetShoulderPitchRad + jointDirections.shoulder_pitch * q2_rad;
        // Coupled: q3_world is RELATIVE angle (upper shank relative to lower shank)
        const q3_relative_rad = offsetElbowPitchRad + jointDirections.elbow_pitch * q3_rad;
        const urdf_q3 = -urdf_q2 - q3_relative_rad;

        guiJointPub.publish({
            header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
            name: ['base_yaw_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint'],
            position: [urdf_q1, urdf_q2, urdf_q3],
            velocity: [],
            effort: []
        });
    };

    const handleSliderChange = (jointIdx, val) => {
        const floatVal = parseFloat(val);
        const checkedVal = isNaN(floatVal) ? 0 : floatVal;

        let nextQ1 = q1;
        let nextQ2 = q2;
        let nextQ3 = q3;

        if (jointIdx === 1) {
            nextQ1 = checkedVal;
            setQ1(checkedVal);
        } else if (jointIdx === 2) {
            nextQ2 = checkedVal;
            setQ2(checkedVal);
        } else if (jointIdx === 3) {
            nextQ3 = checkedVal;
            setQ3(checkedVal);
        }

        commandedQRef.current = null;
        setUserDragging(true);
        publishSliderJointState(nextQ1, nextQ2, nextQ3);

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
            const val1 = parseFloat(homeQ.q1) || 0;
            const val2 = parseFloat(homeQ.q2) || 0;
            const val3 = parseFloat(homeQ.q3) || 0;
            setQ1(val1);
            setQ2(val2);
            setQ3(val3);

            commandedQRef.current = { q1: val1, q2: val2, q3: val3 };
            webCmdPub.publish({
                data: JSON.stringify({
                    action: 'go_to_position',
                    q: [val1, val2, val3]
                })
            });
            publishSliderJointState(val1, val2, val3);
        }
    };

    return (
        <div className="card">
            <h2 style={{ marginTop: '0px' }}>
                Joint control (world frame)
            </h2>

            {/* Sliders */}
            <div className="slider-group" style={{ marginBottom: '12px' }}>
                <div className="slider-header" style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '4px' }}>
                    <span>Base yaw (θ₁)</span>
                    <input
                        type="number"
                        min="-180"
                        max="180"
                        step="1"
                        value={q1}
                        onChange={(e) => handleSliderChange(1, e.target.value)}
                        className="input-field"
                        style={{ width: '80px', textAlign: 'right', padding: '2px 6px', background: 'var(--card-bg)', color: 'var(--text-main)', border: '1px solid var(--glass-border)', borderRadius: '4px' }}
                    />
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

            <div className="slider-group" style={{ marginBottom: '12px' }}>
                <div className="slider-header" style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '4px' }}>
                    <span>Shoulder pitch (θ₂)</span>
                    <input
                        type="number"
                        min="-180"
                        max="180"
                        step="1"
                        value={q2}
                        onChange={(e) => handleSliderChange(2, e.target.value)}
                        className="input-field"
                        style={{ width: '80px', textAlign: 'right', padding: '2px 6px', background: 'var(--card-bg)', color: 'var(--text-main)', border: '1px solid var(--glass-border)', borderRadius: '4px' }}
                    />
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

            <div className="slider-group" style={{ marginBottom: '16px' }}>
                <div className="slider-header" style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '4px' }}>
                    <span>Elbow pitch (θ₃)</span>
                    <input
                        type="number"
                        min="-180"
                        max="180"
                        step="1"
                        value={q3}
                        onChange={(e) => handleSliderChange(3, e.target.value)}
                        className="input-field"
                        style={{ width: '80px', textAlign: 'right', padding: '2px 6px', background: 'var(--card-bg)', color: 'var(--text-main)', border: '1px solid var(--glass-border)', borderRadius: '4px' }}
                    />
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

            <div style={{ display: 'flex', gap: '10px' }}>
                <button onClick={handleSetHome} className="btn btn-primary" style={{ flex: 1 }}>
                    Set origin
                </button>
                <button onClick={handleReset} className="btn btn-secondary" disabled={!homeQ} style={{ flex: 1 }}>
                    Move to origin
                </button>
            </div>
        </div>
    );
}
