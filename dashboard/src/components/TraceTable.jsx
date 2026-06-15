import React, { useState, useEffect } from 'react';

export default function TraceTable({ currentQ }) {
    const [tracePoints, setTracePoints] = useState([]);
    const MAX_TRACE_LOG = 30;

    useEffect(() => {
        if (currentQ) {
            setTracePoints(prev => {
                const next = [[...currentQ], ...prev];
                if (next.length > MAX_TRACE_LOG) {
                    next.pop();
                }
                return next;
            });
        }
    }, [currentQ]);

    return (
        <div className="card traceability-panel" style={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
            <h2>Live Joint State Traceability</h2>
            <div style={{ flex: 1, overflowY: 'auto', maxHeight: '350px' }}>
                <table>
                    <thead>
                        <tr>
                            <th>#</th>
                            <th>θ₁ (Yaw)</th>
                            <th>θ₂ (Shoulder)</th>
                            <th>θ₃ (Elbow)</th>
                        </tr>
                    </thead>
                    <tbody>
                        {tracePoints.length === 0 ? (
                            <tr>
                                <td colSpan="4" style={{ textAlign: 'center', color: 'var(--text-muted)' }}>
                                    Waiting for joint states...
                                </td>
                            </tr>
                        ) : (
                            tracePoints.map((p, idx) => (
                                <tr key={idx} className={idx === 0 ? 'current-row' : ''}>
                                    <td>{tracePoints.length - idx}</td>
                                    <td>{p[0].toFixed(3)}</td>
                                    <td>{p[1].toFixed(3)}</td>
                                    <td>{p[2].toFixed(3)}</td>
                                </tr>
                            ))
                        )}
                    </tbody>
                </table>
            </div>
        </div>
    );
}
