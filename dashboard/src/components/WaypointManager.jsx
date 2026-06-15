import React, { useState } from 'react';
import * as ROSLIB from 'roslib';
import { webCmdPub } from '../services/ros';

export default function WaypointManager({ currentQ }) {
    const [waypoints, setWaypoints] = useState([]);

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
    };

    const handleExecuteSequence = () => {
        if (waypoints.length < 2) {
            alert("At least 2 waypoints are required to execute a sequential path.");
            return;
        }

        const planner = document.getElementById('select-planner')?.value || 'astar';
        const heuristic = document.getElementById('select-heuristic')?.value || 'L1';

        const payload = {
            action: "plan_sequential",
            planner_type: planner,
            heuristic_type: heuristic,
            waypoints: waypoints
        };

        webCmdPub.publish({
            data: JSON.stringify(payload)
        });
        console.log("Published sequential plan command:", payload);
    };

    const rad2deg = 180.0 / Math.PI;

    return (
        <div className="card">
            <h2>Sequential Waypoints</h2>
            
            <button 
                onClick={handleAddWaypoint} 
                className="btn btn-primary" 
                style={{ width: '100%', marginBottom: '10px' }}
            >
                Add Current Pose as Waypoint
            </button>

            <div id="waypoint-list" style={{ maxHeight: '180px', overflowY: 'auto', margin: '10px 0' }}>
                {waypoints.length === 0 ? (
                    <div style={{ color: 'var(--text-muted)', textAlign: 'center', padding: '10px' }}>
                        No waypoints added
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
                            <span>WP #{idx + 1}: [{(wp[0]*rad2deg).toFixed(1)}°, {(wp[1]*rad2deg).toFixed(1)}°, {(wp[2]*rad2deg).toFixed(1)}°]</span>
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

            {waypoints.length > 0 && (
                <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '8px' }}>
                    <button onClick={handleClear} className="btn btn-secondary">
                        Clear All
                    </button>
                    <button 
                        onClick={handleExecuteSequence} 
                        className="btn btn-accent"
                        disabled={waypoints.length < 2}
                    >
                        Execute Seq
                    </button>
                </div>
            )}
        </div>
    );
}
