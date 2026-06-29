import React, { useState, useEffect } from 'react';
import * as ROSLIB from 'roslib';
import { webCmdPub, statusSub } from '../services/ros';

export default function WaypointManager({ currentQ, homeQ }) {
    const [waypoints, setWaypoints] = useState([]);
    const [planner, setPlanner] = useState('astar');
    const [heuristic, setHeuristic] = useState('L1');
    const [status, setStatus] = useState(null);

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

    const handleAddWaypoint = () => {
        if (!currentQ) {
            alert("No joint state received yet. Is the robot visualization running?");
            return;
        }
        // Save copy of currentQ in radians
        setWaypoints([...waypoints, [...currentQ]]);
    };

    const handleRemoveWaypoint = (idx) => {
        const copy = [...waypoints];
        copy.splice(idx, 1);
        setWaypoints(copy);
    };

    const handleClear = () => {
        setWaypoints([]);
        setStatus(null);
    };

    const handleExecuteSequence = () => {
        if (waypoints.length < 1) {
            alert("At least 1 waypoint is required to execute a sequential path starting from the origin.");
            return;
        }

        const deg2rad = Math.PI / 180.0;
        const originRad = homeQ ? [
            homeQ.q1 * deg2rad,
            homeQ.q2 * deg2rad,
            homeQ.q3 * deg2rad
        ] : [0.0, 90.0 * deg2rad, 0.0];

        const payload = {
            action: "plan_sequential",
            planner_type: planner,
            heuristic_type: heuristic,
            waypoints: [originRad, ...waypoints]
        };

        webCmdPub.publish({
            data: JSON.stringify(payload)
        });
        console.log("Published sequential plan command starting from origin:", payload);
    };

    const rad2deg = 180.0 / Math.PI;

    return (
        <div className="card">
            <h2>Sequential waypoints</h2>
            
            <button 
                onClick={handleAddWaypoint} 
                className="btn btn-primary" 
                style={{ width: '100%', marginBottom: '10px' }}
            >
                Add current pose as waypoint
            </button>

            <div id="waypoint-list" style={{ maxHeight: '200px', overflowY: 'auto', margin: '10px 0' }}>
                {/* Always show origin row — loading from params service if not yet received */}
                {homeQ ? (
                    <div 
                        style={{ 
                            display: 'flex', 
                            justifyContent: 'space-between', 
                            alignItems: 'center', 
                            padding: '6px 4px', 
                            borderBottom: '1px solid var(--glass-border)',
                            color: 'var(--text-muted)',
                            backgroundColor: 'rgba(255, 255, 255, 0.02)',
                            borderRadius: '4px'
                        }}
                    >
                        <span>Start (Origin): [{homeQ.q1.toFixed(1)}°, {homeQ.q2.toFixed(1)}°, {homeQ.q3.toFixed(1)}°]</span>
                        <span style={{ fontSize: '0.8em', fontStyle: 'italic', paddingRight: '6px' }}>Origin</span>
                    </div>
                ) : (
                    <div style={{ color: 'var(--text-muted)', padding: '6px 4px', fontSize: '0.85em', borderBottom: '1px solid var(--glass-border)', fontStyle: 'italic' }}>
                        ⏳ Loading origin from params...
                    </div>
                )}

                {waypoints.length === 0 ? (
                    <div style={{ color: 'var(--text-muted)', textAlign: 'center', padding: '15px 10px', fontSize: '0.85em' }}>
                        No additional waypoints added
                    </div>
                ) : (
                    waypoints.map((wp, idx) => (
                        <div 
                            key={idx}
                            style={{ 
                                display: 'flex', 
                                justifyContent: 'space-between', 
                                alignItems: 'center', 
                                padding: '6px 4px', 
                                borderBottom: '1px solid var(--glass-border)' 
                            }}
                        >
                            <span>Waypoint #{idx + 1}: [{(wp[0]*rad2deg).toFixed(1)}°, {(wp[1]*rad2deg).toFixed(1)}°, {(wp[2]*rad2deg).toFixed(1)}°]</span>
                            <span 
                                onClick={() => handleRemoveWaypoint(idx)} 
                                style={{ color: 'var(--accent-red)', cursor: 'pointer', fontWeight: 'bold', padding: '0 6px', fontSize: '1.1em' }}
                            >
                                &times;
                            </span>
                        </div>
                    ))
                )}
            </div>

            {/* Planner Configuration Dropdowns */}
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '8px', marginBottom: '12px' }}>
                <div className="form-group" style={{ marginBottom: '0px' }}>
                    <label htmlFor="select-planner" style={{ fontSize: '0.8em', marginBottom: '4px', display: 'block' }}>Algorithm</label>
                    <select
                        id="select-planner"
                        value={planner}
                        onChange={(e) => setPlanner(e.target.value)}
                        className="select-field"
                        style={{ padding: '6px' }}
                    >
                        <option value="astar">A* search</option>
                        <option value="rrt">RRT</option>
                    </select>
                </div>

                <div className="form-group" style={{ marginBottom: '0px' }}>
                    <label htmlFor="select-heuristic" style={{ fontSize: '0.8em', marginBottom: '4px', display: 'block' }}>Metric</label>
                    <select
                        id="select-heuristic"
                        value={heuristic}
                        onChange={(e) => setHeuristic(e.target.value)}
                        className="select-field"
                        style={{ padding: '6px' }}
                        disabled={planner !== 'astar'}
                    >
                        <option value="L1">L1 (Manhattan)</option>
                        <option value="L2">L2 (Euclidean)</option>
                    </select>
                </div>
            </div>

            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '8px' }}>
                <button 
                    onClick={handleClear} 
                    className="btn btn-secondary"
                    disabled={waypoints.length === 0}
                >
                    Clear all
                </button>
                <button 
                    onClick={handleExecuteSequence} 
                    className="btn btn-accent"
                    disabled={waypoints.length < 1}
                >
                    Execute sequence
                </button>
            </div>

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
    );
}
