import React, { useState } from 'react';
import { webCmdPub } from '../services/ros';

export default function ObstaclePositioner({ activeObstacle, resolution }) {
    const [posX, setPosX] = useState('0.30');
    const [posY, setPosY] = useState('0.00');
    const [posZ, setPosZ] = useState('0.15');
    const [isExpanded, setIsExpanded] = useState(false);

    const isObstacleActive = activeObstacle && activeObstacle !== 'no_obstacles';

    const handleMoveObstacle = () => {
        webCmdPub.publish({
            data: JSON.stringify({
                action: 'move_obstacle',
                obstacle_type: activeObstacle,
                position_xyz: [parseFloat(posX), parseFloat(posY), parseFloat(posZ)],
                step_size_deg: parseFloat(resolution)
            })
        });
    };

    return (
        <div className="section">
            <div
                className="section-header"
                style={{ cursor: 'pointer', userSelect: 'none', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}
                onClick={() => setIsExpanded(v => !v)}
            >
                <span>Obstacle positioning</span>
                <span style={{ fontSize: '0.8em', opacity: 0.5 }}>{isExpanded ? '▲' : '▼'}</span>
            </div>

            {isExpanded && (
                <div>
                    <p style={{ fontSize: '0.78em', color: 'var(--text-muted)', marginBottom: '10px', lineHeight: '1.5' }}>
                        Translate active obstacle in Cartesian world space (X, Y, Z in meters).
                    </p>

                    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '6px', marginBottom: '10px' }}>
                        <div>
                            <label style={{ fontSize: '0.72em', color: 'var(--text-label)', display: 'block', marginBottom: '3px' }}>X (m)</label>
                            <input type="number" step="0.02" value={posX} onChange={(e) => setPosX(e.target.value)} className="input-field" disabled={!isObstacleActive} />
                        </div>
                        <div>
                            <label style={{ fontSize: '0.72em', color: 'var(--text-label)', display: 'block', marginBottom: '3px' }}>Y (m)</label>
                            <input type="number" step="0.02" value={posY} onChange={(e) => setPosY(e.target.value)} className="input-field" disabled={!isObstacleActive} />
                        </div>
                        <div>
                            <label style={{ fontSize: '0.72em', color: 'var(--text-label)', display: 'block', marginBottom: '3px' }}>Z (m)</label>
                            <input type="number" step="0.02" value={posZ} onChange={(e) => setPosZ(e.target.value)} className="input-field" disabled={!isObstacleActive} />
                        </div>
                    </div>

                    <button onClick={handleMoveObstacle} disabled={!isObstacleActive} className="btn btn-secondary" style={{ width: '100%' }}>
                        Apply obstacle position
                    </button>

                    {!isObstacleActive && (
                        <p style={{ fontSize: '0.75em', color: 'var(--text-muted)', marginTop: '6px' }}>
                            No active obstacle selected.
                        </p>
                    )}
                </div>
            )}
        </div>
    );
}
