import React, { useState, useEffect } from 'react';
import { webCmdPub } from '../services/ros';

// Helper: Forward Kinematics (FK) for 3-DOF Community Arm
// Matches compute_forward_kinematics_gripper() in community_arm.py exactly.
export const computeFK = (deg1, deg2, deg3) => {
    const rad = Math.PI / 180.0;
    const q1 = deg1 * rad;
    const q2 = deg2 * rad;
    const q3 = deg3 * rad;

    const baseHeight = 0.130;
    const lowerShank = 0.140;
    const upperShank = 0.140;
    const gripperDx = -0.05467;
    const gripperDz = -0.0217;

    const c1 = Math.cos(q1);
    const s1 = Math.sin(q1);

    const theta2 = q2 - Math.PI / 2.0;
    const theta3 = theta2 + (q3 - Math.PI);

    const c2 = Math.cos(theta2);
    const s2 = Math.sin(theta2);
    const c3 = Math.cos(theta3);
    const s3 = Math.sin(theta3);

    const rOffset = gripperDx * c3 - gripperDz * s3;
    const zOffset = gripperDx * s3 + gripperDz * c3;

    const r = lowerShank * c2 + upperShank * c3 + rOffset;
    const x = r * c1;
    const y = r * s1;
    const z = baseHeight + lowerShank * s2 + upperShank * s3 + zOffset;

    return [x, y, z];
};

export default function CartesianControlPanel({ homeQ, currentQ }) {
    const [isExpanded, setIsExpanded] = useState(false);

    // Default start position in cm (-13.3, 0, 19.0)
    const [startX, setStartX] = useState('-13.3');
    const [startY, setStartY] = useState('0');
    const [startZ, setStartZ] = useState('19');

    // Default goal position in cm (13.3, 0, 19.0)
    const [goalX, setGoalX] = useState('13.3');
    const [goalY, setGoalY] = useState('0');
    const [goalZ, setGoalZ] = useState('19');

    // Sync Start position when homeQ is loaded
    useEffect(() => {
        if (homeQ) {
            const [x, y, z] = computeFK(homeQ.q1, homeQ.q2, homeQ.q3);
            setStartX((x * 100).toFixed(1));
            setStartY((y * 100).toFixed(1));
            setStartZ((z * 100).toFixed(1));
        }
    }, [homeQ]);

    const handleUseCurrentPose = () => {
        if (currentQ) {
            const rad2deg = 180.0 / Math.PI;
            const [x, y, z] = computeFK(currentQ[0] * rad2deg, currentQ[1] * rad2deg, currentQ[2] * rad2deg);
            setStartX((x * 100).toFixed(1));
            setStartY((y * 100).toFixed(1));
            setStartZ((z * 100).toFixed(1));
        }
    };

    // Kinematic reachability helper (base_height = 0.130m, L1 = 0.140m, L2_eff = 0.08533m)
    const checkReachability = (xStr, yStr, zStr) => {
        const x = parseFloat(xStr) / 100.0;
        const y = parseFloat(yStr) / 100.0;
        const z = parseFloat(zStr) / 100.0;
        if (isNaN(x) || isNaN(y) || isNaN(z)) return false;

        const baseHeight = 0.130;
        const L1 = 0.140;
        const L2Eff = 0.08533;
        const maxReach = L1 + L2Eff;
        const minReach = Math.abs(L1 - L2Eff);

        const rXY = Math.sqrt(x * x + y * y);
        const zPrime = z - baseHeight;
        const dist = Math.sqrt(rXY * rXY + zPrime * zPrime);

        return dist >= minReach && dist <= maxReach;
    };

    const startReachable = checkReachability(startX, startY, startZ);
    const goalReachable = checkReachability(goalX, goalY, goalZ);
    const canPlan = startReachable && goalReachable;

    const handlePlanCartesian = () => {
        if (!canPlan) return;

        const payload = {
            action: "plan_cartesian",
            start_xyz: [parseFloat(startX) / 100.0, parseFloat(startY) / 100.0, parseFloat(startZ) / 100.0],
            goal_xyz: [parseFloat(goalX) / 100.0, parseFloat(goalY) / 100.0, parseFloat(goalZ) / 100.0]
        };

        webCmdPub.publish({
            data: JSON.stringify(payload)
        });
        console.log("Published plan_cartesian command (m):", payload);
    };

    return (
        <div className="card" style={{ marginTop: '16px' }}>
            <div 
                onClick={() => setIsExpanded(!isExpanded)} 
                style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', cursor: 'pointer', userSelect: 'none' }}
            >
                <h2 style={{ margin: '0px', fontSize: '1.05rem' }}>
                    Cartesian planning (IK)
                </h2>
                <span style={{ fontSize: '0.8rem', color: 'var(--text-muted)', transition: 'transform 0.2s ease', transform: isExpanded ? 'rotate(180deg)' : 'rotate(0deg)' }}>
                    ▼
                </span>
            </div>

            {isExpanded && (
                <div style={{ marginTop: '12px' }}>
                    <p style={{ fontSize: '0.82rem', color: 'var(--text-secondary)', marginBottom: '12px' }}>
                        Input Cartesian coordinates (X, Y, Z in cm). Analytical IK resolves joint targets on T³.
                    </p>

                    {/* Start Position */}
                    <div style={{ marginBottom: '12px', background: 'rgba(255,255,255,0.03)', padding: '10px', borderRadius: '6px', border: '1px solid var(--glass-border)' }}>
                        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '8px' }}>
                            <strong style={{ fontSize: '0.85rem' }}>Start (X₀, Y₀, Z₀)</strong>
                            <span style={{ 
                                fontSize: '0.75rem', 
                                padding: '2px 8px', 
                                borderRadius: '10px', 
                                background: startReachable ? 'rgba(46, 204, 113, 0.2)' : 'rgba(231, 76, 60, 0.2)',
                                color: startReachable ? '#2ecc71' : '#e74c3c',
                                border: `1px solid ${startReachable ? '#2ecc71' : '#e74c3c'}`
                            }}>
                                {startReachable ? 'Reachable' : 'Out of reach'}
                            </span>
                        </div>
                        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '8px' }}>
                            <div>
                                <label style={{ fontSize: '0.75rem' }}>X (cm)</label>
                                <input type="number" step="0.1" value={startX} onChange={(e) => setStartX(e.target.value)} className="input-field" style={{ width: '100%' }} />
                            </div>
                            <div>
                                <label style={{ fontSize: '0.75rem' }}>Y (cm)</label>
                                <input type="number" step="0.1" value={startY} onChange={(e) => setStartY(e.target.value)} className="input-field" style={{ width: '100%' }} />
                            </div>
                            <div>
                                <label style={{ fontSize: '0.75rem' }}>Z (cm)</label>
                                <input type="number" step="0.1" value={startZ} onChange={(e) => setStartZ(e.target.value)} className="input-field" style={{ width: '100%' }} />
                            </div>
                        </div>
                    </div>

                    {/* Goal Position */}
                    <div style={{ marginBottom: '14px', background: 'rgba(255,255,255,0.03)', padding: '10px', borderRadius: '6px', border: '1px solid var(--glass-border)' }}>
                        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '8px' }}>
                            <strong style={{ fontSize: '0.85rem' }}>Goal (X₁, Y₁, Z₁)</strong>
                            <span style={{ 
                                fontSize: '0.75rem', 
                                padding: '2px 8px', 
                                borderRadius: '10px', 
                                background: goalReachable ? 'rgba(46, 204, 113, 0.2)' : 'rgba(231, 76, 60, 0.2)',
                                color: goalReachable ? '#2ecc71' : '#e74c3c',
                                border: `1px solid ${goalReachable ? '#2ecc71' : '#e74c3c'}`
                            }}>
                                {goalReachable ? 'Reachable' : 'Out of reach'}
                            </span>
                        </div>
                        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '8px' }}>
                            <div>
                                <label style={{ fontSize: '0.75rem' }}>X (cm)</label>
                                <input type="number" step="0.1" value={goalX} onChange={(e) => setGoalX(e.target.value)} className="input-field" style={{ width: '100%' }} />
                            </div>
                            <div>
                                <label style={{ fontSize: '0.75rem' }}>Y (cm)</label>
                                <input type="number" step="0.1" value={goalY} onChange={(e) => setGoalY(e.target.value)} className="input-field" style={{ width: '100%' }} />
                            </div>
                            <div>
                                <label style={{ fontSize: '0.75rem' }}>Z (cm)</label>
                                <input type="number" step="0.1" value={goalZ} onChange={(e) => setGoalZ(e.target.value)} className="input-field" style={{ width: '100%' }} />
                            </div>
                        </div>
                    </div>

                    <button 
                        onClick={handlePlanCartesian} 
                        disabled={!canPlan} 
                        className="btn btn-primary" 
                        style={{ width: '100%', opacity: canPlan ? 1 : 0.5, cursor: canPlan ? 'pointer' : 'not-allowed' }}
                    >
                        Solve IK & Plan Trajectory
                    </button>
                </div>
            )}
        </div>
    );
}
