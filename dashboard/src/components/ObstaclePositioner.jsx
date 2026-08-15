import React, { useState, useEffect, useRef } from 'react';
import { webCmdPub } from '../services/ros';

// Initial default positions (centroid / joint origin) per obstacle type in meters
const DEFAULT_POSITIONS = {
    no_obstacles:   { x: '0.00', y: '0.00', z: '0.00' },
    box_obstacle:   { x: '0.30', y: '0.00', z: '0.15' },
    narrow_passage: { x: '0.30', y: '0.00', z: '0.20' },
    u_obstacle:     { x: '0.28', y: '0.00', z: '0.15' },
    toroidal_wall:  { x: '0.25', y: '0.00', z: '0.18' },
};

export default function ObstaclePositioner({ activeObstacle, resolution, onApplyPosition }) {
    const [posX, setPosX] = useState('0.30');
    const [posY, setPosY] = useState('0.00');
    const [posZ, setPosZ] = useState('0.15');
    const [isExpanded, setIsExpanded] = useState(true);

    const isObstacleActive = activeObstacle && activeObstacle !== 'no_obstacles';

    // Sync position inputs whenever activeObstacle changes
    useEffect(() => {
        const def = DEFAULT_POSITIONS[activeObstacle] || { x: '0.30', y: '0.00', z: '0.15' };
        setPosX(def.x);
        setPosY(def.y);
        setPosZ(def.z);
    }, [activeObstacle]);

    // Live preview in RViz2 (updates static TF / URDF origin without recalculating C-space)
    const sendPreview = (x, y, z) => {
        if (!isObstacleActive) return;
        webCmdPub.publish({
            data: JSON.stringify({
                action: 'preview_obstacle',
                obstacle_type: activeObstacle,
                position_xyz: [parseFloat(x), parseFloat(y), parseFloat(z)]
            })
        });
    };

    const handleXChange = (val) => {
        setPosX(val);
        sendPreview(val, posY, posZ);
    };

    const handleYChange = (val) => {
        setPosY(val);
        sendPreview(posX, val, posZ);
    };

    const handleZChange = (val) => {
        setPosZ(val);
        sendPreview(posX, posY, val);
    };

    // Full update: recompute C-space voxels and update collision models
    const handleMoveObstacle = () => {
        if (!isObstacleActive) return;
        if (onApplyPosition) onApplyPosition();
        webCmdPub.publish({
            data: JSON.stringify({
                action: 'move_obstacle',
                obstacle_type: activeObstacle,
                position_xyz: [parseFloat(posX), parseFloat(posY), parseFloat(posZ)],
                step_size_deg: parseFloat(resolution)
            })
        });
    };

    const handleResetPosition = () => {
        const def = DEFAULT_POSITIONS[activeObstacle] || { x: '0.30', y: '0.00', z: '0.15' };
        setPosX(def.x);
        setPosY(def.y);
        setPosZ(def.z);
        sendPreview(def.x, def.y, def.z);
    };

    return (
        <div className="section">
            <div
                className="section-header"
                style={{ cursor: 'pointer', userSelect: 'none', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}
                onClick={() => setIsExpanded(v => !v)}
            >
                <span>Obstacle positioning {isObstacleActive ? `(${activeObstacle.replace('_', ' ')})` : '— disabled'}</span>
                <span style={{ fontSize: '0.8em', opacity: 0.5 }}>{isExpanded ? '▲' : '▼'}</span>
            </div>

            {isExpanded && (
                <div style={{ opacity: isObstacleActive ? 1 : 0.45 }}>
                    <p style={{ fontSize: '0.78em', color: 'var(--text-muted)', marginBottom: '10px', lineHeight: '1.5' }}>
                        {isObstacleActive
                            ? 'Adjust X, Y, Z (m) to move obstacle live in RViz2. Click Apply to calculate C-space.'
                            : 'Select an obstacle in the top-right bar to enable live positioning.'}
                    </p>

                    {/* X slider & input */}
                    <div className="slider-group" style={{ marginBottom: '8px' }}>
                        <div className="slider-header">
                            <span>X position (m)</span>
                            <input
                                type="number" step="0.01" min="0.0" max="0.6"
                                value={posX}
                                onChange={(e) => handleXChange(e.target.value)}
                                className="input-field"
                                style={{ width: '70px', padding: '2px 5px', fontSize: '0.85em', textAlign: 'right' }}
                                disabled={!isObstacleActive}
                            />
                        </div>
                        <input
                            type="range" step="0.01" min="0.0" max="0.6"
                            value={posX}
                            onChange={(e) => handleXChange(e.target.value)}
                            className="slider"
                            disabled={!isObstacleActive}
                        />
                    </div>

                    {/* Y slider & input */}
                    <div className="slider-group" style={{ marginBottom: '8px' }}>
                        <div className="slider-header">
                            <span>Y position (m)</span>
                            <input
                                type="number" step="0.01" min="-0.4" max="0.4"
                                value={posY}
                                onChange={(e) => handleYChange(e.target.value)}
                                className="input-field"
                                style={{ width: '70px', padding: '2px 5px', fontSize: '0.85em', textAlign: 'right' }}
                                disabled={!isObstacleActive}
                            />
                        </div>
                        <input
                            type="range" step="0.01" min="-0.4" max="0.4"
                            value={posY}
                            onChange={(e) => handleYChange(e.target.value)}
                            className="slider"
                            disabled={!isObstacleActive}
                        />
                    </div>

                    {/* Z slider & input */}
                    <div className="slider-group" style={{ marginBottom: '12px' }}>
                        <div className="slider-header">
                            <span>Z position (m)</span>
                            <input
                                type="number" step="0.01" min="0.0" max="0.6"
                                value={posZ}
                                onChange={(e) => handleZChange(e.target.value)}
                                className="input-field"
                                style={{ width: '70px', padding: '2px 5px', fontSize: '0.85em', textAlign: 'right' }}
                                disabled={!isObstacleActive}
                            />
                        </div>
                        <input
                            type="range" step="0.01" min="0.0" max="0.6"
                            value={posZ}
                            onChange={(e) => handleZChange(e.target.value)}
                            className="slider"
                            disabled={!isObstacleActive}
                        />
                    </div>

                    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '8px' }}>
                        <button
                            onClick={handleResetPosition}
                            disabled={!isObstacleActive}
                            className="btn btn-secondary"
                            style={{ fontSize: '0.82em' }}
                        >
                            Reset position
                        </button>
                        <button
                            onClick={handleMoveObstacle}
                            disabled={!isObstacleActive}
                            className="btn btn-primary"
                            style={{ fontSize: '0.82em' }}
                        >
                            Apply position
                        </button>
                    </div>
                </div>
            )}
        </div>
    );
}
