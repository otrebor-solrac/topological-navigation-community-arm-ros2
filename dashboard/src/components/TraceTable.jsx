import React from 'react';

export default function TraceTable({ plannedPath }) {
    const rad2deg = 180.0 / Math.PI;

    return (
        <div className="card traceability-panel" style={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
            <h2>Planned path waypoints</h2>
            <div style={{ flex: 1, overflowY: 'auto', maxHeight: '350px' }}>
                <table>
                    <thead>
                        <tr>
                            <th>#</th>
                            <th>θ₁ (yaw)</th>
                            <th>θ₂ (shoulder)</th>
                            <th>θ₃ (elbow)</th>
                        </tr>
                    </thead>
                    <tbody>
                        {!plannedPath || plannedPath.length === 0 ? (
                            <tr>
                                <td colSpan="4" style={{ textAlign: 'center', color: 'var(--text-muted)' }}>
                                    No active path planned
                                </td>
                            </tr>
                        ) : (
                            plannedPath.map((p, idx) => (
                                <tr key={idx}>
                                    <td>{idx + 1}</td>
                                    <td>{(p[0] * rad2deg).toFixed(1)}°</td>
                                    <td>{(p[1] * rad2deg).toFixed(1)}°</td>
                                    <td>{(p[2] * rad2deg).toFixed(1)}°</td>
                                </tr>
                            ))
                        )}
                    </tbody>
                </table>
            </div>
        </div>
    );
}
