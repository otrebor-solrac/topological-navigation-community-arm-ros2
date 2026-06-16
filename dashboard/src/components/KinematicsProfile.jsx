import React from 'react';

function LineChart({ title, data, colors }) {
    if (!data || data.length === 0) return null;

    const width = 280;
    const height = 100;
    const padding = { top: 12, right: 10, bottom: 18, left: 30 };

    const numPoints = data.length;
    const numSeries = data[0].length;

    // Find min and max for scaling Y axis
    let minY = Infinity;
    let maxY = -Infinity;
    for (let i = 0; i < numPoints; i++) {
        for (let s = 0; s < numSeries; s++) {
            if (data[i][s] < minY) minY = data[i][s];
            if (data[i][s] > maxY) maxY = data[i][s];
        }
    }
    // Add small buffer to avoid zero height issues
    if (minY === maxY) {
        minY -= 1;
        maxY += 1;
    } else {
        const buffer = (maxY - minY) * 0.1;
        minY -= buffer;
        maxY += buffer;
    }

    // Map index to SVG X position
    const getX = (index) => {
        if (numPoints <= 1) return padding.left;
        return padding.left + (index / (numPoints - 1)) * (width - padding.left - padding.right);
    };

    // Map value to SVG Y position
    const getY = (val) => {
        return height - padding.bottom - ((val - minY) / (maxY - minY)) * (height - padding.top - padding.bottom);
    };

    // Generate path lines
    const paths = Array.from({ length: numSeries }, (_, s) => {
        let d = '';
        for (let i = 0; i < numPoints; i++) {
            const x = getX(i);
            const y = getY(data[i][s]);
            d += `${i === 0 ? 'M' : 'L'} ${x.toFixed(1)} ${y.toFixed(1)}`;
        }
        return d;
    });

    return (
        <div style={{ marginBottom: '14px' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '4px' }}>
                <span style={{ fontSize: '0.8em', color: '#e2e8f0', fontWeight: 500 }}>{title}</span>
                <span style={{ fontSize: '0.7em', color: 'var(--text-muted)' }}>
                    [{minY.toFixed(1)} to {maxY.toFixed(1)}]
                </span>
            </div>
            <svg width="100%" height={height} style={{ background: 'rgba(0, 0, 0, 0.25)', borderRadius: '6px', border: '1px solid var(--glass-border)' }}>
                {/* Horizontal reference lines */}
                {[0, 0.5, 1].map((ratio, idx) => {
                    const yVal = minY + ratio * (maxY - minY);
                    const y = getY(yVal);
                    return (
                        <g key={idx}>
                            <line 
                                x1={padding.left} 
                                y1={y} 
                                x2={width - padding.right} 
                                y2={y} 
                                stroke="rgba(255,255,255,0.05)" 
                                strokeDasharray="2,2" 
                            />
                            <text 
                                x={padding.left - 6} 
                                y={y + 3} 
                                fill="var(--text-muted)" 
                                fontSize="8px" 
                                textAnchor="end"
                            >
                                {yVal.toFixed(0)}
                            </text>
                        </g>
                    );
                })}

                {/* X labels */}
                {[0, Math.floor((numPoints-1)/2), numPoints-1].map((index, idx) => {
                    if (index < 0 || index >= numPoints) return null;
                    const x = getX(index);
                    return (
                        <text 
                            key={idx}
                            x={x} 
                            y={height - 4} 
                            fill="var(--text-muted)" 
                            fontSize="8px" 
                            textAnchor="middle"
                        >
                            {index}
                        </text>
                    );
                })}

                {/* Series lines */}
                {paths.map((d, s) => (
                    <path
                        key={s}
                        d={d}
                        fill="none"
                        stroke={colors[s]}
                        strokeWidth="1.5"
                    />
                ))}
            </svg>
        </div>
    );
}

export default function KinematicsProfile({ plannedPath }) {
    if (!plannedPath || plannedPath.length === 0) {
        return (
            <div className="card">
                <h2>Kinematics profile</h2>
                <div style={{ color: 'var(--text-muted)', fontSize: '0.85em', textAlign: 'center', padding: '20px 0' }}>
                    No active planned path to plot.
                </div>
            </div>
        );
    }

    const rad2deg = 180.0 / Math.PI;

    // 1. Joint Positions (degrees)
    const positions = plannedPath.map(p => [
        p[0] * rad2deg,
        p[1] * rad2deg,
        p[2] * rad2deg
    ]);

    // 2. Joint Velocities (deg/step)
    const velocities = [];
    for (let i = 0; i < positions.length - 1; i++) {
        velocities.push([
            positions[i+1][0] - positions[i][0],
            positions[i+1][1] - positions[i][1],
            positions[i+1][2] - positions[i][2]
        ]);
    }

    // 3. Joint Accelerations (deg/step^2)
    const accelerations = [];
    for (let i = 0; i < velocities.length - 1; i++) {
        accelerations.push([
            velocities[i+1][0] - velocities[i][0],
            velocities[i+1][1] - velocities[i][1],
            velocities[i+1][2] - velocities[i][2]
        ]);
    }

    // Color theme matching the dashboard
    const colors = ['#00ff9d', '#00d4ff', '#ffff00'];

    return (
        <div className="card">
            <h2>Kinematics profile</h2>
            <div style={{ display: 'flex', gap: '10px', justifyContent: 'center', marginBottom: '12px', fontSize: '0.75em' }}>
                <span style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
                    <span style={{ display: 'inline-block', width: '8px', height: '8px', borderRadius: '50%', background: colors[0] }}></span>
                    θ₁
                </span>
                <span style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
                    <span style={{ display: 'inline-block', width: '8px', height: '8px', borderRadius: '50%', background: colors[1] }}></span>
                    θ₂
                </span>
                <span style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
                    <span style={{ display: 'inline-block', width: '8px', height: '8px', borderRadius: '50%', background: colors[2] }}></span>
                    θ₃
                </span>
            </div>

            <LineChart title="Joint positions (deg)" data={positions} colors={colors} />
            
            {velocities.length > 0 ? (
                <LineChart title="Joint velocities (deg/step)" data={velocities} colors={colors} />
            ) : (
                <div style={{ fontSize: '0.75em', color: 'var(--text-muted)', textAlign: 'center', margin: '8px 0' }}>
                    Velocity requires at least 2 points.
                </div>
            )}

            {accelerations.length > 0 ? (
                <LineChart title="Joint accelerations (deg/step²)" data={accelerations} colors={colors} />
            ) : (
                <div style={{ fontSize: '0.75em', color: 'var(--text-muted)', textAlign: 'center', margin: '8px 0' }}>
                    Acceleration requires at least 3 points.
                </div>
            )}
        </div>
    );
}
