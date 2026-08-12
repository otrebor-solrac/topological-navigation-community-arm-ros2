import React, { useState } from 'react';
import { webCmdPub } from '../services/ros';

export default function ObstaclePositioner({ activeObstacle, resolution }) {
    const [isExpanded, setIsExpanded] = useState(false);

    const [posX, setPosX] = useState('0.30');
    const [posY, setPosY] = useState('0.00');
    const [posZ, setPosZ] = useState('0.15');

    const handleMoveObstacle = () => {
        const payload = {
            action: "move_obstacle",
            obstacle_type: activeObstacle,
            position_xyz: [parseFloat(posX), parseFloat(posY), parseFloat(posZ)],
            step_size_deg: parseFloat(resolution)
        };

        webCmdPub.publish({
            data: JSON.stringify(payload)
        });
        console.log("Published move_obstacle command:", payload);
    };

    const isObstacleActive = activeObstacle && activeObstacle !== 'no_obstacles';

    return (
        <div className="card" style={{ marginTop: '16px' }}>
            <div 
                onClick={() => setIsExpanded(!isExpanded)} 
                style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', cursor: 'pointer', userSelect: 'none' }}
            >
                <h2 style={{ margin: '0px', fontSize: '1.05rem' }}>
                    Obstacle positioning
                </h2>
                <span style={{ fontSize: '0.8rem', color: 'var(--text-muted)', transition: 'transform 0.2s ease', transform: isExpanded ? 'rotate(180deg)' : 'rotate(0deg)' }}>
                    ▼
                </span>
            </div>

            {isExpanded && (
                <div style={{ marginTop: '12px' }}>
                    <p style={{ fontSize: '0.82rem', color: 'var(--text-secondary)', marginBottom: '12px' }}>
                        Translate active obstacle link origin in Cartesian world space (X, Y, Z).
                    </p>

                    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '8px', marginBottom: '16px' }}>
                        <div>
                            <label style={{ fontSize: '0.75rem' }}>X_obs (m)</label>
                            <input 
                                type="number" 
                                step="0.02" 
                                value={posX} 
                                onChange={(e) => setPosX(e.target.value)} 
                                className="input-field" 
                                style={{ width: '100%' }} 
                                disabled={!isObstacleActive}
                            />
                        </div>
                        <div>
                            <label style={{ fontSize: '0.75rem' }}>Y_obs (m)</label>
                            <input 
                                type="number" 
                                step="0.02" 
                                value={posY} 
                                onChange={(e) => setPosY(e.target.value)} 
                                className="input-field" 
                                style={{ width: '100%' }} 
                                disabled={!isObstacleActive}
                            />
                        </div>
                        <div>
                            <label style={{ fontSize: '0.75rem' }}>Z_obs (m)</label>
                            <input 
                                type="number" 
                                step="0.02" 
                                value={posZ} 
                                onChange={(e) => setPosZ(e.target.value)} 
                                className="input-field" 
                                style={{ width: '100%' }} 
                                disabled={!isObstacleActive}
                            />
                        </div>
                    </div>

                    <button 
                        onClick={handleMoveObstacle} 
                        disabled={!isObstacleActive}
                        className="btn btn-secondary" 
                        style={{ width: '100%', opacity: isObstacleActive ? 1 : 0.5, cursor: isObstacleActive ? 'pointer' : 'not-allowed' }}
                    >
                        Translate Obstacle in Space
                    </button>
                </div>
            )}
        </div>
    );
}
