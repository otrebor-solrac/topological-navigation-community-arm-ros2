import React, { useState } from 'react';
import { webCmdPub } from '../services/ros';
import { computeFK, computeIK } from '../utils/kinematics';

export default function CartesianOriginPanel({ currentQ, homeCartesian, setHomeCartesian }) {
    const [isExpanded, setIsExpanded] = useState(false);
    const [origX, setOrigX] = useState('-13.3');
    const [origY, setOrigY] = useState('0');
    const [origZ, setOrigZ] = useState('19');

    // Helper: load current robot position into inputs
    const handleSyncCurrentPosition = () => {
        if (!currentQ) return;
        const rad2deg = 180.0 / Math.PI;
        const fk = computeFK(currentQ[0] * rad2deg, currentQ[1] * rad2deg, currentQ[2] * rad2deg);
        setOrigX(fk.x_cm);
        setOrigY(fk.y_cm);
        setOrigZ(fk.z_cm);
    };

    const handleSetOrigin = () => {
        const x_cm = parseFloat(origX);
        const y_cm = parseFloat(origY);
        const z_cm = parseFloat(origZ);
        if (isNaN(x_cm) || isNaN(y_cm) || isNaN(z_cm)) {
            alert("Please enter valid numerical coordinates.");
            return;
        }

        const x = x_cm / 100.0;
        const y = y_cm / 100.0;
        const z = z_cm / 100.0;

        if (setHomeCartesian) {
            setHomeCartesian([x, y, z]);
        }

        // Send raw Cartesian origin command to ROS 2 backend (in meters)
        webCmdPub.publish({
            data: JSON.stringify({
                action: "set_origin",
                xyz: [x, y, z]
            })
        });
        console.log("Sent set_origin action to ROS 2 (m):", [x, y, z]);
    };

    const handleMoveToOrigin = () => {
        const x_cm = parseFloat(origX);
        const y_cm = parseFloat(origY);
        const z_cm = parseFloat(origZ);
        if (isNaN(x_cm) || isNaN(y_cm) || isNaN(z_cm)) {
            alert("Please enter valid numerical coordinates.");
            return;
        }

        const x = x_cm / 100.0;
        const y = y_cm / 100.0;
        const z = z_cm / 100.0;

        webCmdPub.publish({
            data: JSON.stringify({
                action: "set_origin",
                xyz: [x, y, z]
            })
        });
        console.log("Sent move to origin action to ROS 2 (m):", [x, y, z]);
    };

    return (
        <div className="card">
            <div 
                onClick={() => setIsExpanded(!isExpanded)} 
                style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', cursor: 'pointer', userSelect: 'none' }}
            >
                <h2 style={{ margin: '0px', fontSize: '1.05rem' }}>
                    Cartesian origin & controls
                </h2>
                <span style={{ fontSize: '0.8rem', color: 'var(--text-muted)', transition: 'transform 0.2s ease', transform: isExpanded ? 'rotate(180deg)' : 'rotate(0deg)' }}>
                    ▼
                </span>
            </div>

            {isExpanded && (
                <div style={{ marginTop: '12px' }}>
                    <p style={{ fontSize: '0.82rem', color: 'var(--text-secondary)', marginBottom: '12px' }}>
                        Set or move to origin using Cartesian coordinates (X, Y, Z in cm).
                    </p>

                    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '8px', marginBottom: '10px' }}>
                        <div>
                            <label style={{ fontSize: '0.75rem' }}>X (cm)</label>
                            <input 
                                type="number" 
                                step="0.1" 
                                value={origX} 
                                onChange={(e) => setOrigX(e.target.value)} 
                                className="input-field" 
                                style={{ width: '100%' }} 
                            />
                        </div>
                        <div>
                            <label style={{ fontSize: '0.75rem' }}>Y (cm)</label>
                            <input 
                                type="number" 
                                step="0.1" 
                                value={origY} 
                                onChange={(e) => setOrigY(e.target.value)} 
                                className="input-field" 
                                style={{ width: '100%' }} 
                            />
                        </div>
                        <div>
                            <label style={{ fontSize: '0.75rem' }}>Z (cm)</label>
                            <input 
                                type="number" 
                                step="0.1" 
                                value={origZ} 
                                onChange={(e) => setOrigZ(e.target.value)} 
                                className="input-field" 
                                style={{ width: '100%' }} 
                            />
                        </div>
                    </div>

                    <button
                        onClick={handleSyncCurrentPosition}
                        className="btn btn-secondary"
                        style={{ width: '100%', marginBottom: '12px', fontSize: '0.78rem', padding: '4px' }}
                    >
                        📍 Get current position (FK)
                    </button>

                    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '8px' }}>
                        <button 
                            onClick={handleSetOrigin}
                            className="btn btn-secondary"
                            style={{ width: '100%', fontSize: '0.82rem', padding: '6px' }}
                        >
                            Set origin
                        </button>
                        <button 
                            onClick={handleMoveToOrigin}
                            className="btn btn-primary"
                            style={{ width: '100%', fontSize: '0.82rem', padding: '6px' }}
                        >
                            Move to origin
                        </button>
                    </div>
                </div>
            )}
        </div>
    );
}
