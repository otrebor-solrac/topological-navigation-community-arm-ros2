import React, { useState, useEffect } from 'react';
import * as ROSLIB from 'roslib';
import { webCmdPub, statusSub } from '../services/ros';

export default function PlannerControls() {
    const [goalQ1, setGoalQ1] = useState(72.0);
    const [goalQ2, setGoalQ2] = useState(135.0);
    const [goalQ3, setGoalQ3] = useState(90.0);
    
    const [planner, setPlanner] = useState('astar');
    const [heuristic, setHeuristic] = useState('L1');
    const [status, setStatus] = useState(null);

    useEffect(() => {
        const handleStatus = (msg) => {
            try {
                const data = JSON.parse(msg.data);
                setStatus(data);
            } catch (e) {
                console.error("Failed to parse status message:", e);
            }
        };

        statusSub.subscribe(handleStatus);
        return () => {
            statusSub.unsubscribe(handleStatus);
        };
    }, []);

    const handlePlan = () => {
        const deg2rad = Math.PI / 180.0;
        const payload = {
            action: "plan",
            planner_type: planner,
            heuristic_type: heuristic,
            goal: [goalQ1 * deg2rad, goalQ2 * deg2rad, goalQ3 * deg2rad]
        };

        webCmdPub.publish({
            data: JSON.stringify(payload)
        });
        console.log("Published plan command:", payload);
    };

    return (
        <div className="card">
            <h2>Path planner configuration</h2>
            
            {/* Goal configuration inputs */}
            <div className="form-group">
                <label>Goal configuration (world frame)</label>
                <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '8px' }}>
                    <div>
                        <span style={{ fontSize: '0.85em', color: 'var(--text-muted)' }}>Yaw (θ₁)</span>
                        <input
                            type="number"
                            value={goalQ1}
                            onChange={(e) => setGoalQ1(parseFloat(e.target.value) || 0)}
                            className="input-field"
                        />
                    </div>
                    <div>
                        <span style={{ fontSize: '0.85em', color: 'var(--text-muted)' }}>Shoulder (θ₂)</span>
                        <input
                            type="number"
                            value={goalQ2}
                            onChange={(e) => setGoalQ2(parseFloat(e.target.value) || 0)}
                            className="input-field"
                        />
                    </div>
                    <div>
                        <span style={{ fontSize: '0.85em', color: 'var(--text-muted)' }}>Elbow (θ₃)</span>
                        <input
                            type="number"
                            value={goalQ3}
                            onChange={(e) => setGoalQ3(parseFloat(e.target.value) || 0)}
                            className="input-field"
                        />
                    </div>
                </div>
            </div>

            {/* Planner type select */}
            <div className="form-group">
                <label htmlFor="select-planner">Algorithm</label>
                <select
                    id="select-planner"
                    value={planner}
                    onChange={(e) => setPlanner(e.target.value)}
                    className="select-field"
                >
                    <option value="astar">A* search (optimal)</option>
                    <option value="rrt">Rapidly-exploring random tree (RRT)</option>
                </select>
            </div>

            {/* Heuristic selection (only if astar) */}
            {planner === 'astar' && (
                <div className="form-group" id="heuristic-group">
                    <label htmlFor="select-heuristic">Distance metric / heuristic</label>
                    <select
                        id="select-heuristic"
                        value={heuristic}
                        onChange={(e) => setHeuristic(e.target.value)}
                        className="select-field"
                    >
                        <option value="L1">L1 norm (Manhattan)</option>
                        <option value="L2">L2 norm (Euclidean)</option>
                    </select>
                </div>
            )}

            <button onClick={handlePlan} className="btn btn-primary" style={{ width: '100%', marginBottom: '20px' }}>
                Plan path
            </button>

            {status && (
                <div className={`status-banner ${status.success ? 'status-success' : 'status-error'}`} style={{
                    padding: '12px',
                    borderRadius: '8px',
                    fontSize: '0.9em',
                    lineHeight: '1.4',
                    border: status.success ? '1px solid rgba(0, 255, 127, 0.3)' : '1px solid rgba(255, 64, 64, 0.3)',
                    backgroundColor: status.success ? 'rgba(0, 255, 127, 0.1)' : 'rgba(255, 64, 64, 0.1)',
                    color: status.success ? '#00ff7f' : '#ff4040',
                    marginTop: '10px',
                    wordBreak: 'break-word'
                }}>
                    <strong>{status.success ? '✓ Success: ' : '✗ Error: '}</strong>
                    {status.message}
                </div>
            )}

        </div>
    );
}
