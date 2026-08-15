import React, { useState, useEffect, useRef } from 'react';
import { guiJointPub, webCmdPub, jointOffsets, jointDirections } from '../services/ros';

/* ── SliderRow must be OUTSIDE ControlPanel to prevent unmount on re-render ── */
function SliderRow({ label, value, jointIdx, onChange }) {
    return (
        <div className="slider-group">
            <div className="slider-header">
                <span>{label}</span>
                <input
                    type="number"
                    min="-180" max="180" step="1"
                    value={value}
                    onChange={(e) => onChange(jointIdx, e.target.value)}
                    style={{
                        width: '68px', textAlign: 'right',
                        padding: '3px 6px', fontSize: '0.85em',
                        background: 'var(--input-bg)', color: 'var(--accent-green)',
                        border: '1px solid var(--input-border)', borderRadius: '3px',
                        fontFamily: 'Share Tech Mono, monospace'
                    }}
                />
            </div>
            <input
                type="range" min="-180" max="180"
                value={value}
                onChange={(e) => onChange(jointIdx, e.target.value)}
                className="slider"
            />
        </div>
    );
}

export default function ControlPanel({ currentQ, homeQ, setHomeQ }) {
    const [q1, setQ1] = useState(0);
    const [q2, setQ2] = useState(0);
    const [q3, setQ3] = useState(0);

    const [userDragging, setUserDragging] = useState(false);
    const dragTimeoutRef = useRef(null);
    const commandedQRef = useRef(null);

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
            if (arrived) commandedQRef.current = null;
            else return;
        }

        if (!userDragging) {
            setQ1(Math.round(worldQ1));
            setQ2(Math.round(worldQ2));
            setQ3(Math.round(worldQ3));
        }
    }, [currentQ, userDragging]);

    const publishSliderJointState = (sQ1, sQ2, sQ3) => {
        const deg2rad = Math.PI / 180.0;
        const v1 = isNaN(parseFloat(sQ1)) ? 0 : parseFloat(sQ1);
        const v2 = isNaN(parseFloat(sQ2)) ? 0 : parseFloat(sQ2);
        const v3 = isNaN(parseFloat(sQ3)) ? 0 : parseFloat(sQ3);

        const offBase     = jointOffsets.base_yaw       * deg2rad;
        const offShoulder = jointOffsets.shoulder_pitch * deg2rad;
        const offElbow    = jointOffsets.elbow_pitch    * deg2rad;

        const urdf_q1 = offBase     + jointDirections.base_yaw       * (v1 * deg2rad);
        const urdf_q2 = offShoulder + jointDirections.shoulder_pitch * (v2 * deg2rad);
        const q3_rel  = offElbow    + jointDirections.elbow_pitch    * (v3 * deg2rad);
        const urdf_q3 = -urdf_q2 - q3_rel;

        guiJointPub.publish({
            header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
            name: ['base_yaw_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint'],
            position: [urdf_q1, urdf_q2, urdf_q3],
            velocity: [], effort: []
        });
    };

    const handleSliderChange = (jointIdx, val) => {
        const v = isNaN(parseFloat(val)) ? 0 : parseFloat(val);
        let nq1 = q1, nq2 = q2, nq3 = q3;
        if (jointIdx === 1) { nq1 = v; setQ1(v); }
        else if (jointIdx === 2) { nq2 = v; setQ2(v); }
        else if (jointIdx === 3) { nq3 = v; setQ3(v); }

        commandedQRef.current = null;
        setUserDragging(true);
        publishSliderJointState(nq1, nq2, nq3);
        clearTimeout(dragTimeoutRef.current);
        dragTimeoutRef.current = setTimeout(() => setUserDragging(false), 500);
    };

    const handleSetHome = () => {
        const v1 = parseFloat(q1) || 0;
        const v2 = parseFloat(q2) || 0;
        const v3 = parseFloat(q3) || 0;
        setHomeQ({ q1: v1, q2: v2, q3: v3 });
        webCmdPub.publish({ data: JSON.stringify({ action: 'go_to_position', q: [v1, v2, v3] }) });
    };

    const handleReset = () => {
        if (!homeQ) return;
        const v1 = parseFloat(homeQ.q1) || 0;
        const v2 = parseFloat(homeQ.q2) || 0;
        const v3 = parseFloat(homeQ.q3) || 0;
        setQ1(v1); setQ2(v2); setQ3(v3);
        commandedQRef.current = { q1: v1, q2: v2, q3: v3 };
        webCmdPub.publish({ data: JSON.stringify({ action: 'go_to_position', q: [v1, v2, v3] }) });
        publishSliderJointState(v1, v2, v3);
    };

    return (
        <div className="section">
            <div className="section-header">Joint control — world frame</div>

            <SliderRow label="Base yaw (θ₁)"       value={q1} jointIdx={1} onChange={handleSliderChange} />
            <SliderRow label="Shoulder pitch (θ₂)"  value={q2} jointIdx={2} onChange={handleSliderChange} />
            <SliderRow label="Elbow pitch (θ₃)"    value={q3} jointIdx={3} onChange={handleSliderChange} />

            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '8px', marginTop: '4px' }}>
                <button onClick={handleSetHome} className="btn btn-primary">
                    Set origin
                </button>
                <button onClick={handleReset} className="btn btn-secondary" disabled={!homeQ}>
                    Move to origin
                </button>
            </div>
        </div>
    );
}
