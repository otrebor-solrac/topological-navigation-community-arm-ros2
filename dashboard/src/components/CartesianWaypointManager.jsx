import React, { useState, useEffect } from 'react';
import { webCmdPub, statusSub } from '../services/ros';

export default function CartesianWaypointManager({ cartesianWaypoints, setCartesianWaypoints, homeCartesian }) {
    const [isExpanded, setIsExpanded] = useState(false);
    const [planner, setPlanner] = useState('astar');
    const [heuristic, setHeuristic] = useState('L1');
    const [status, setStatus] = useState(null);

    // Custom input fields for adding a manual Cartesian point in cm
    const [customX, setCustomX] = useState('12');
    const [customY, setCustomY] = useState('0');
    const [customZ, setCustomZ] = useState('14');

    // Listen to planner status messages
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

    const handleAddCustomPoint = () => {
        const x = parseFloat(customX);
        const y = parseFloat(customY);
        const z = parseFloat(customZ);
        if (isNaN(x) || isNaN(y) || isNaN(z)) {
            alert("Please enter valid numerical coordinates for X, Y, Z in cm.");
            return;
        }
        setCartesianWaypoints([...cartesianWaypoints, [x, y, z]]);
    };

    const handleRemoveWaypoint = (idx) => {
        const copy = [...cartesianWaypoints];
        copy.splice(idx, 1);
        setCartesianWaypoints(copy);
    };

    const handleClear = () => {
        setCartesianWaypoints([]);
        setStatus(null);
    };

    const handleExecuteSequence = () => {
        if (cartesianWaypoints.length < 1) {
            alert("At least 1 Cartesian waypoint is required to execute a sequential path starting from origin.");
            return;
        }

        const originXYZ_m = homeCartesian ? homeCartesian : [-0.14, 0.00, 0.21];

        // Convert user waypoints from cm to meters before publishing to ROS 2 backend
        const waypoints_m = [
            originXYZ_m,
            ...cartesianWaypoints.map(pt => [pt[0] / 100.0, pt[1] / 100.0, pt[2] / 100.0])
        ];

        const payload = {
            action: "plan_sequential",
            planner_type: planner,
            heuristic_type: heuristic,
            waypoints: waypoints_m
        };

        webCmdPub.publish({
            data: JSON.stringify(payload)
        });
        console.log("Published Cartesian sequential plan command to ROS 2 backend (m):", payload);
    };

    const originXYZ_m = homeCartesian ? homeCartesian : [-0.14, 0.00, 0.21];
    const originX_cm = (originXYZ_m[0] * 100).toFixed(1);
    const originY_cm = (originXYZ_m[1] * 100).toFixed(1);
    const originZ_cm = (originXYZ_m[2] * 100).toFixed(1);

    return (
        <div className="card" style={{ marginTop: '16px' }}>
            <div 
                onClick={() => setIsExpanded(!isExpanded)} 
                style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', cursor: 'pointer', userSelect: 'none' }}
            >
                <h2 style={{ margin: '0px', fontSize: '1.05rem' }}>
                    Sequential waypoints (Cartesian XYZ)
                </h2>
                <span style={{ fontSize: '0.8rem', color: 'var(--text-muted)', transition: 'transform 0.2s ease', transform: isExpanded ? 'rotate(180deg)' : 'rotate(0deg)' }}>
                    ▼
                </span>
            </div>

            {isExpanded && (
                <div style={{ marginTop: '12px' }}>
                    {/* Add Custom Point Row */}
                    <div style={{ background: 'rgba(255,255,255,0.03)', padding: '8px', borderRadius: '6px', marginBottom: '10px', border: '1px solid var(--glass-border)' }}>
                        <div style={{ fontSize: '0.75rem', color: 'var(--text-muted)', marginBottom: '4px' }}>Custom Point (X, Y, Z in cm)</div>
                        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr auto', gap: '6px', alignItems: 'center' }}>
                            <input type="number" step="0.1" value={customX} onChange={(e) => setCustomX(e.target.value)} placeholder="X (cm)" className="input-field" style={{ fontSize: '0.8rem', padding: '4px' }} />
                            <input type="number" step="0.1" value={customY} onChange={(e) => setCustomY(e.target.value)} placeholder="Y (cm)" className="input-field" style={{ fontSize: '0.8rem', padding: '4px' }} />
                            <input type="number" step="0.1" value={customZ} onChange={(e) => setCustomZ(e.target.value)} placeholder="Z (cm)" className="input-field" style={{ fontSize: '0.8rem', padding: '4px' }} />
                            <button onClick={handleAddCustomPoint} className="btn btn-secondary" style={{ padding: '4px 10px', fontSize: '0.78rem' }}>+ Add</button>
                        </div>
                    </div>

                    <div id="cartesian-waypoint-list" style={{ maxHeight: '200px', overflowY: 'auto', margin: '10px 0' }}>
                        {/* Origin Row */}
                        <div 
                            style={{ 
                                display: 'flex', 
                                justifyContent: 'space-between', 
                                alignItems: 'center',
                                padding: '6px 10px', 
                                background: 'rgba(0, 255, 127, 0.1)', 
                                border: '1px solid rgba(0, 255, 127, 0.3)',
                                borderRadius: '4px',
                                marginBottom: '6px',
                                fontSize: '0.82rem'
                            }}
                        >
                            <span>
                                <strong>Origin (Start):</strong> X={originX_cm}cm, Y={originY_cm}cm, Z={originZ_cm}cm
                            </span>
                            <span style={{ fontSize: '0.75rem', color: '#00ff7f' }}>Fixed</span>
                        </div>

                        {/* Waypoints List */}
                        {cartesianWaypoints.map((pt, idx) => (
                            <div 
                                key={idx} 
                                style={{ 
                                    display: 'flex', 
                                    justifyContent: 'space-between', 
                                    alignItems: 'center',
                                    padding: '6px 10px', 
                                    background: 'var(--card-bg)', 
                                    border: '1px solid var(--glass-border)',
                                    borderRadius: '4px',
                                    marginBottom: '6px',
                                    fontSize: '0.82rem'
                                }}
                            >
                                <span>
                                    <strong>Pt {idx + 1}:</strong> X={pt[0]}cm, Y={pt[1]}cm, Z={pt[2]}cm
                                </span>
                                <button 
                                    onClick={() => handleRemoveWaypoint(idx)} 
                                    style={{ 
                                        background: 'none', 
                                        border: 'none', 
                                        color: '#ff4040', 
                                        cursor: 'pointer',
                                        fontSize: '0.9rem'
                                    }}
                                >
                                    ✕
                                </button>
                            </div>
                        ))}
                    </div>

                    <div style={{ display: 'flex', gap: '8px', marginBottom: '10px' }}>
                        <button 
                            onClick={handleClear} 
                            className="btn btn-secondary" 
                            style={{ flex: 1 }}
                            disabled={cartesianWaypoints.length === 0}
                        >
                            Clear list
                        </button>
                    </div>

                    {/* Algorithm & Heuristic Selectors */}
                    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '8px', marginBottom: '12px' }}>
                        <div>
                            <label style={{ fontSize: '0.75rem', color: 'var(--text-muted)' }}>Algorithm</label>
                            <select 
                                value={planner} 
                                onChange={(e) => setPlanner(e.target.value)}
                                className="input-field"
                                style={{ width: '100%', marginTop: '2px', fontSize: '0.82rem' }}
                            >
                                <option value="astar">A* Search</option>
                                <option value="rrt">RRT (Random Tree)</option>
                            </select>
                        </div>
                        <div>
                            <label style={{ fontSize: '0.75rem', color: 'var(--text-muted)' }}>Heuristic</label>
                            <select 
                                value={heuristic} 
                                onChange={(e) => setHeuristic(e.target.value)}
                                className="input-field"
                                style={{ width: '100%', marginTop: '2px', fontSize: '0.82rem' }}
                            >
                                <option value="L1">L1 (Manhattan)</option>
                                <option value="L2">L2 (Euclidean)</option>
                            </select>
                        </div>
                    </div>

                    <button 
                        onClick={handleExecuteSequence} 
                        className="btn btn-primary" 
                        style={{ width: '100%' }}
                        disabled={cartesianWaypoints.length === 0}
                    >
                        Execute Cartesian Sequence
                    </button>

                    {status && (
                        <div className={`status-banner ${status.success ? 'status-success' : 'status-error'}`} style={{
                            padding: '10px',
                            borderRadius: '8px',
                            fontSize: '0.85em',
                            lineHeight: '1.4',
                            border: status.success ? '1px solid rgba(0, 255, 127, 0.3)' : '1px solid rgba(255, 64, 64, 0.3)',
                            backgroundColor: status.success ? 'rgba(0, 255, 127, 0.08)' : 'rgba(255, 64, 64, 0.08)',
                            color: status.success ? '#00ff7f' : '#ff4040',
                            marginTop: '10px',
                            wordBreak: 'break-word'
                        }}>
                            <strong>{status.success ? '✓ ' : '✗ '}</strong>
                            {status.message}
                        </div>
                    )}
                </div>
            )}
        </div>
    );
}
