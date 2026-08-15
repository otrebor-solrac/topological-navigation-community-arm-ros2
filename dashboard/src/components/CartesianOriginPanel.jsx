import React, { useState } from 'react';
import { webCmdPub } from '../services/ros';
import { computeFK, computeIK } from '../utils/kinematics';

export default function CartesianOriginPanel({ currentQ, homeQ, homeCartesian, setHomeCartesian }) {
    const [origX, setOrigX] = useState('-24.6');
    const [origY, setOrigY] = useState('0.0');
    const [origZ, setOrigZ] = useState('16.0');

    React.useEffect(() => {
        if (homeCartesian) {
            setOrigX((homeCartesian[0] * 100).toFixed(1));
            setOrigY((homeCartesian[1] * 100).toFixed(1));
            setOrigZ((homeCartesian[2] * 100).toFixed(1));
        } else if (homeQ) {
            const fk = computeFK(homeQ.q1, homeQ.q2, homeQ.q3);
            setOrigX(fk.x_cm);
            setOrigY(fk.y_cm);
            setOrigZ(fk.z_cm);
        }
    }, [homeCartesian, homeQ]);

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
        if (isNaN(x_cm) || isNaN(y_cm) || isNaN(z_cm)) { alert('Please enter valid coordinates.'); return; }
        const x = x_cm / 100.0, y = y_cm / 100.0, z = z_cm / 100.0;
        if (setHomeCartesian) setHomeCartesian([x, y, z]);
        webCmdPub.publish({ data: JSON.stringify({ action: 'set_origin', xyz: [x, y, z] }) });
    };

    const handleMoveToOrigin = () => {
        const x_cm = parseFloat(origX);
        const y_cm = parseFloat(origY);
        const z_cm = parseFloat(origZ);
        if (isNaN(x_cm) || isNaN(y_cm) || isNaN(z_cm)) { alert('Please enter valid coordinates.'); return; }
        const x = x_cm / 100.0, y = y_cm / 100.0, z = z_cm / 100.0;
        webCmdPub.publish({ data: JSON.stringify({ action: 'set_origin', xyz: [x, y, z] }) });
    };

    return (
        <div className="section">
            <div className="section-header">Cartesian origin</div>
            <p style={{ fontSize: '0.78em', color: 'var(--text-muted)', marginBottom: '10px', lineHeight: '1.5' }}>
                Set or move to origin using X, Y, Z coordinates in centimeters.
            </p>

            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '6px', marginBottom: '8px' }}>
                <div>
                    <label style={{ fontSize: '0.82em', color: 'var(--text-label)', display: 'block', marginBottom: '3px' }}>X (cm)</label>
                    <input type="number" step="0.1" value={origX} onChange={(e) => setOrigX(e.target.value)} className="input-field" />
                </div>
                <div>
                    <label style={{ fontSize: '0.82em', color: 'var(--text-label)', display: 'block', marginBottom: '3px' }}>Y (cm)</label>
                    <input type="number" step="0.1" value={origY} onChange={(e) => setOrigY(e.target.value)} className="input-field" />
                </div>
                <div>
                    <label style={{ fontSize: '0.82em', color: 'var(--text-label)', display: 'block', marginBottom: '3px' }}>Z (cm)</label>
                    <input type="number" step="0.1" value={origZ} onChange={(e) => setOrigZ(e.target.value)} className="input-field" />
                </div>
            </div>

            <button
                onClick={handleSyncCurrentPosition}
                className="btn btn-secondary"
                style={{ width: '100%', marginBottom: '8px', fontSize: '0.78em' }}
            >
                Get current position (FK)
            </button>

            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '6px' }}>
                <button onClick={handleSetOrigin} className="btn btn-secondary">Set origin</button>
                <button onClick={handleMoveToOrigin} className="btn btn-primary">Move to origin</button>
            </div>
        </div>
    );
}
