import React, { useState, useEffect } from 'react';
import { webCmdPub, statusSub } from '../services/ros';

export default function WaypointManager({ currentQ, homeQ, waypoints, setWaypoints }) {
    const [planner, setPlanner] = useState('astar');
    const [heuristic, setHeuristic] = useState('L1');
    const [status, setStatus] = useState(null);

    useEffect(() => {
        const handleStatus = (msg) => {
            try { setStatus(JSON.parse(msg.data)); }
            catch (e) { console.error('Failed to parse status:', e); }
        };
        statusSub.subscribe(handleStatus);
        return () => statusSub.unsubscribe(handleStatus);
    }, []);

    const handleAddWaypoint = () => {
        if (!currentQ) { alert('No joint state received yet.'); return; }
        setWaypoints([...waypoints, [...currentQ]]);
    };

    const handleRemoveWaypoint = (idx) => {
        const copy = [...waypoints];
        copy.splice(idx, 1);
        setWaypoints(copy);
    };

    const handleClear = () => { setWaypoints([]); setStatus(null); };

    const handleExecuteSequence = () => {
        if (waypoints.length < 1) {
            alert('At least 1 waypoint is required to execute a sequential path starting from origin.');
            return;
        }
        const deg2rad = Math.PI / 180.0;
        const originRad = homeQ
            ? [homeQ.q1 * deg2rad, homeQ.q2 * deg2rad, homeQ.q3 * deg2rad]
            : [0.0, 90.0 * deg2rad, 0.0];

        webCmdPub.publish({
            data: JSON.stringify({
                action: 'plan_sequential',
                planner_type: planner,
                heuristic_type: heuristic,
                waypoints: [originRad, ...waypoints]
            })
        });
    };

    const rad2deg = 180.0 / Math.PI;

    return (
        <div className="section">
            <div className="section-header">Sequential waypoints</div>

            <button
                onClick={handleAddWaypoint}
                className="btn btn-primary"
                style={{ width: '100%', marginBottom: '10px' }}
            >
                Add current pose as waypoint
            </button>

            {/* Waypoint list */}
            <div id="waypoint-list" style={{ marginBottom: '10px' }}>
                {homeQ ? (
                    <div className="waypoint-row waypoint-row-origin">
                        <span>Origin: [{homeQ.q1.toFixed(1)}°, {homeQ.q2.toFixed(1)}°, {homeQ.q3.toFixed(1)}°]</span>
                        <span style={{ fontSize: '0.75em', opacity: 0.6 }}>start</span>
                    </div>
                ) : (
                    <div className="waypoint-row" style={{ color: 'var(--text-muted)', fontStyle: 'italic' }}>
                        Loading origin...
                    </div>
                )}

                {waypoints.length === 0 ? (
                    <div style={{ color: 'var(--text-muted)', textAlign: 'center', padding: '12px 0', fontSize: '0.8em' }}>
                        No waypoints added
                    </div>
                ) : (
                    waypoints.map((wp, idx) => (
                        <div key={idx} className="waypoint-row">
                            <span>#{idx + 1}: [{(wp[0]*rad2deg).toFixed(1)}°, {(wp[1]*rad2deg).toFixed(1)}°, {(wp[2]*rad2deg).toFixed(1)}°]</span>
                            <button className="waypoint-remove" onClick={() => handleRemoveWaypoint(idx)}>✕</button>
                        </div>
                    ))
                )}
            </div>

            {/* Algorithm & metric */}
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '8px', marginBottom: '10px' }}>
                <div>
                    <label style={{ fontSize: '0.82em', color: 'var(--text-label)', textTransform: 'uppercase', letterSpacing: '0.5px', display: 'block', marginBottom: '3px' }}>Algorithm</label>
                    <select id="select-planner" value={planner} onChange={(e) => setPlanner(e.target.value)} className="select-field">
                        <option value="astar">A* search</option>
                        <option value="rrt">RRT</option>
                    </select>
                </div>
                <div>
                    <label style={{ fontSize: '0.82em', color: 'var(--text-label)', textTransform: 'uppercase', letterSpacing: '0.5px', display: 'block', marginBottom: '3px' }}>Metric</label>
                    <select id="select-heuristic" value={heuristic} onChange={(e) => setHeuristic(e.target.value)} className="select-field" disabled={planner !== 'astar'}>
                        <option value="L1">L1 (Manhattan)</option>
                        <option value="L2">L2 (Euclidean)</option>
                    </select>
                </div>
            </div>

            {/* Actions */}
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '8px', marginBottom: '10px' }}>
                <button onClick={handleClear} className="btn" disabled={waypoints.length === 0}>
                    Clear all
                </button>
                <button onClick={handleExecuteSequence} className="btn btn-primary" disabled={waypoints.length < 1}>
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
