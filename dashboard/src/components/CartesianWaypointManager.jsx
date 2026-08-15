import React, { useState, useEffect } from 'react';
import { webCmdPub, statusSub } from '../services/ros';

export default function CartesianWaypointManager({ cartesianWaypoints, setCartesianWaypoints, homeCartesian }) {
    const [planner, setPlanner] = useState('astar');
    const [heuristic, setHeuristic] = useState('L1');
    const [status, setStatus] = useState(null);
    const [customX, setCustomX] = useState('12');
    const [customY, setCustomY] = useState('0');
    const [customZ, setCustomZ] = useState('14');

    useEffect(() => {
        const handleStatus = (msg) => {
            try { setStatus(JSON.parse(msg.data)); }
            catch (e) { console.error('Failed to parse status:', e); }
        };
        statusSub.subscribe(handleStatus);
        return () => statusSub.unsubscribe(handleStatus);
    }, []);

    const handleAddCustomPoint = () => {
        const x = parseFloat(customX), y = parseFloat(customY), z = parseFloat(customZ);
        if (isNaN(x) || isNaN(y) || isNaN(z)) { alert('Please enter valid X, Y, Z values in cm.'); return; }
        setCartesianWaypoints([...cartesianWaypoints, [x, y, z]]);
    };

    const handleRemoveWaypoint = (idx) => {
        const copy = [...cartesianWaypoints];
        copy.splice(idx, 1);
        setCartesianWaypoints(copy);
    };

    const handleClear = () => { setCartesianWaypoints([]); setStatus(null); };

    const handleExecuteSequence = () => {
        if (cartesianWaypoints.length < 1) {
            alert('At least 1 Cartesian waypoint is required.');
            return;
        }
        const originXYZ_m = homeCartesian ?? [-0.14, 0.00, 0.21];
        const waypoints_m = [originXYZ_m, ...cartesianWaypoints.map(pt => [pt[0] / 100.0, pt[1] / 100.0, pt[2] / 100.0])];
        webCmdPub.publish({
            data: JSON.stringify({
                action: 'plan_sequential',
                planner_type: planner,
                heuristic_type: heuristic,
                waypoints: waypoints_m
            })
        });
    };

    const originXYZ_m  = homeCartesian ?? [-0.14, 0.00, 0.21];
    const originX_cm   = (originXYZ_m[0] * 100).toFixed(1);
    const originY_cm   = (originXYZ_m[1] * 100).toFixed(1);
    const originZ_cm   = (originXYZ_m[2] * 100).toFixed(1);

    return (
        <div className="section">
            <div className="section-header">Sequential waypoints — Cartesian XYZ</div>

            {/* Add point row */}
            <div style={{ marginBottom: '8px' }}>
                <label style={{ fontSize: '0.82em', color: 'var(--text-label)', textTransform: 'uppercase', letterSpacing: '0.5px', display: 'block', marginBottom: '4px' }}>
                    Custom point (X, Y, Z in cm)
                </label>
                <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr auto', gap: '5px', alignItems: 'center' }}>
                    <input type="number" step="0.1" value={customX} onChange={(e) => setCustomX(e.target.value)} className="input-field" style={{ fontSize: '0.82em' }} />
                    <input type="number" step="0.1" value={customY} onChange={(e) => setCustomY(e.target.value)} className="input-field" style={{ fontSize: '0.82em' }} />
                    <input type="number" step="0.1" value={customZ} onChange={(e) => setCustomZ(e.target.value)} className="input-field" style={{ fontSize: '0.82em' }} />
                    <button onClick={handleAddCustomPoint} className="btn btn-secondary" style={{ padding: '6px 10px', fontSize: '0.78em' }}>+ Add</button>
                </div>
            </div>

            {/* Waypoint list */}
            <div id="cartesian-waypoint-list" style={{ marginBottom: '10px' }}>
                {/* Origin row */}
                <div className="waypoint-row waypoint-row-origin">
                    <span>Origin: X={originX_cm} Y={originY_cm} Z={originZ_cm} cm</span>
                    <span style={{ fontSize: '0.72em', opacity: 0.6 }}>start</span>
                </div>

                {cartesianWaypoints.length === 0 ? (
                    <div style={{ color: 'var(--text-muted)', textAlign: 'center', padding: '12px 0', fontSize: '0.8em' }}>
                        No waypoints added
                    </div>
                ) : (
                    cartesianWaypoints.map((pt, idx) => (
                        <div key={idx} className="waypoint-row">
                            <span>#{idx + 1}: X={pt[0]} Y={pt[1]} Z={pt[2]} cm</span>
                            <button className="waypoint-remove" onClick={() => handleRemoveWaypoint(idx)}>✕</button>
                        </div>
                    ))
                )}
            </div>

            {/* Algorithm & heuristic */}
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '8px', marginBottom: '10px' }}>
                <div>
                    <label style={{ fontSize: '0.82em', color: 'var(--text-label)', textTransform: 'uppercase', letterSpacing: '0.5px', display: 'block', marginBottom: '3px' }}>Algorithm</label>
                    <select value={planner} onChange={(e) => setPlanner(e.target.value)} className="select-field">
                        <option value="astar">A* search</option>
                        <option value="rrt">RRT (random tree)</option>
                    </select>
                </div>
                <div>
                    <label style={{ fontSize: '0.82em', color: 'var(--text-label)', textTransform: 'uppercase', letterSpacing: '0.5px', display: 'block', marginBottom: '3px' }}>Heuristic</label>
                    <select value={heuristic} onChange={(e) => setHeuristic(e.target.value)} className="select-field">
                        <option value="L1">L1 (Manhattan)</option>
                        <option value="L2">L2 (Euclidean)</option>
                    </select>
                </div>
            </div>

            {/* Actions */}
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '8px', marginBottom: '10px' }}>
                <button onClick={handleClear} className="btn" disabled={cartesianWaypoints.length === 0}>
                    Clear list
                </button>
                <button onClick={handleExecuteSequence} className="btn btn-primary" disabled={cartesianWaypoints.length === 0}>
                    Execute sequence
                </button>
            </div>

            {status && (
                <div className={`status-banner ${status.success ? 'status-success' : 'status-error'}`}>
                    {status.success ? '✓ ' : '✗ '}{status.message}
                </div>
            )}
        </div>
    );
}
