import React, { useState, useEffect, useRef } from 'react';

function useContainerWidth(ref) {
    const [width, setWidth] = useState(280);

    useEffect(() => {
        if (!ref.current) return;
        const observer = new ResizeObserver((entries) => {
            for (let entry of entries) {
                setWidth(entry.contentRect.width);
            }
        });
        observer.observe(ref.current);
        return () => observer.disconnect();
    }, [ref]);

    return width;
}

function LineChart({ title, data, colors, xValues, height = 100, yUnit = "" }) {
    const containerRef = useRef(null);
    const width = useContainerWidth(containerRef);

    if (!data || data.length === 0) return null;

    const padding = { top: 15, right: 15, bottom: 22, left: 35 };
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

    const minX = xValues ? xValues[0] : 0;
    const maxX = xValues ? xValues[numPoints - 1] : numPoints - 1;

    // Map time to SVG X position
    const getX = (index) => {
        if (numPoints <= 1) return padding.left;
        const xVal = xValues ? xValues[index] : index;
        if (maxX === minX) return padding.left;
        return padding.left + ((xVal - minX) / (maxX - minX)) * (width - padding.left - padding.right);
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
        <div ref={containerRef} style={{ marginBottom: '16px', width: '100%' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '6px', padding: '0 4px' }}>
                <span style={{ fontSize: '0.85em', color: '#e2e8f0', fontWeight: 500 }}>{title}</span>
                <span style={{ fontSize: '0.75em', color: 'var(--text-muted)' }}>
                    [{minY.toFixed(2)} to {maxY.toFixed(2)}{yUnit}]
                </span>
            </div>
            <svg width="100%" height={height} style={{ background: 'rgba(6, 6, 12, 0.4)', borderRadius: '8px', border: '1px solid var(--glass-border)' }}>
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
                                stroke="rgba(255,255,255,0.06)" 
                                strokeDasharray="3,3" 
                            />
                            <text 
                                x={padding.left - 6} 
                                y={y + 3} 
                                fill="var(--text-muted)" 
                                fontSize="9px" 
                                fontFamily="Share Tech Mono, monospace"
                                textAnchor="end"
                            >
                                {yVal.toFixed(1)}
                            </text>
                        </g>
                    );
                })}

                {/* X labels */}
                {[0, Math.floor((numPoints-1)/2), numPoints-1].map((index, idx) => {
                    if (index < 0 || index >= numPoints) return null;
                    const x = getX(index);
                    const xVal = xValues ? xValues[index] : index;
                    return (
                        <text 
                            key={idx}
                            x={x} 
                            y={height - 6} 
                            fill="var(--text-muted)" 
                            fontSize="9px" 
                            fontFamily="Share Tech Mono, monospace"
                            textAnchor="middle"
                        >
                            {xValues ? `${xVal.toFixed(2)}s` : `Step ${xVal}`}
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
                        strokeWidth="2"
                        strokeLinecap="round"
                        strokeLinejoin="round"
                    />
                ))}
            </svg>
        </div>
    );
}

function computeManipulability(q1, q2, q3, useHorizontal = false) {
    const l1 = 0.140;
    const l2 = 0.140;
    
    const c1 = Math.cos(q1);
    const s1 = Math.sin(q1);
    const c2 = Math.cos(q2);
    const s2 = Math.sin(q2);
    const c23 = Math.cos(q2 + q3);
    const s23 = Math.sin(q2 + q3);
    
    const R = l1 * c2 + l2 * c23;
    
    const col1 = [-R * s1, R * c1, 0.0];
    const col2 = [
        -(l1 * s2 + l2 * s23) * c1,
        -(l1 * s2 + l2 * s23) * s1,
        l1 * c2 + l2 * c23
    ];
    
    if (useHorizontal) {
        const dot11 = col1[0]*col1[0] + col1[1]*col1[1] + col1[2]*col1[2];
        const dot12 = col1[0]*col2[0] + col1[1]*col2[1] + col1[2]*col2[2];
        const dot22 = col2[0]*col2[0] + col2[1]*col2[1] + col2[2]*col2[2];
        const det = dot11 * dot22 - dot12 * dot12;
        return Math.sqrt(Math.max(0.0, det));
    } else {
        const col3 = [
            -l2 * s23 * c1,
            -l2 * s23 * s1,
            l2 * c23
        ];
        const det = col1[0]*(col2[1]*col3[2] - col2[2]*col3[1]) 
                  - col1[1]*(col2[0]*col3[2] - col2[2]*col3[0]) 
                  + col1[2]*(col2[0]*col3[1] - col2[1]*col3[0]);
        return Math.abs(det);
    }
}

export default function KinematicsProfile({ plannedPath, plannedManipulability = [], isFullScreen = false }) {
    if (!plannedPath || plannedPath.length === 0) {
        return (
            <div className="card" style={{ height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
                <div style={{ color: 'var(--text-muted)', fontSize: '0.95em', textAlign: 'center', padding: '40px 0' }}>
                    <h2>Kinematics & trajectory profile</h2>
                    No active planned path to analyze. Start a search first.
                </div>
            </div>
        );
    }

    const rad2deg = 180.0 / Math.PI;
    const numDof = plannedPath[0].length;
    const useHorizontal = (numDof === 2);

    // Calculate time durations between steps (matching TrajectoryGenerator logic)
    const dts = [];
    const times = [0.0];
    for (let i = 0; i < plannedPath.length - 1; i++) {
        let maxDq = 0;
        for (let j = 0; j < numDof; j++) {
            const dq = Math.abs(plannedPath[i+1][j] - plannedPath[i][j]);
            if (dq > maxDq) maxDq = dq;
        }
        const dt = Math.max(maxDq / 1.0, 0.4);
        dts.push(dt);
        times.push(times[times.length - 1] + dt);
    }
    const totalDuration = times[times.length - 1];

    // 1. Joint Positions (degrees)
    const positions = plannedPath.map(p => {
        if (numDof === 2) {
            return [p[0] * rad2deg, p[1] * rad2deg];
        }
        return [p[0] * rad2deg, p[1] * rad2deg, p[2] * rad2deg];
    });

    // 2. Joint Velocities (deg/s)
    const velocities = [];
    for (let i = 0; i < positions.length - 1; i++) {
        const dt = dts[i];
        if (numDof === 2) {
            velocities.push([
                (positions[i+1][0] - positions[i][0]) / dt,
                (positions[i+1][1] - positions[i][1]) / dt
            ]);
        } else {
            velocities.push([
                (positions[i+1][0] - positions[i][0]) / dt,
                (positions[i+1][1] - positions[i][1]) / dt,
                (positions[i+1][2] - positions[i][2]) / dt
            ]);
        }
    }

    // 3. Joint Accelerations (deg/s²)
    const accelerations = [];
    for (let i = 0; i < velocities.length - 1; i++) {
        const dt = 0.5 * (dts[i] + dts[i+1]);
        if (numDof === 2) {
            accelerations.push([
                (velocities[i+1][0] - velocities[i][0]) / dt,
                (velocities[i+1][1] - velocities[i][1]) / dt
            ]);
        } else {
            accelerations.push([
                (velocities[i+1][0] - velocities[i][0]) / dt,
                (velocities[i+1][1] - velocities[i][1]) / dt,
                (velocities[i+1][2] - velocities[i][2]) / dt
            ]);
        }
    }

    // 4. Manipulability Index (w)
    const manipulability = plannedManipulability && plannedManipulability.length === plannedPath.length
        ? plannedManipulability.map(w => [w])
        : plannedPath.map(p => {
            const q1 = p[0];
            const q2 = p[1];
            const q3 = useHorizontal ? 0.0 : p[2];
            return [computeManipulability(q1, q2, q3, useHorizontal)];
        });

    // Color theme matching the dashboard
    const colors = ['#00ff9d', '#00d4ff', '#ffff00'];
    const manipColors = ['#ff007f'];

    const chartHeight = isFullScreen ? 220 : 100;

    const renderHeader = () => (
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '20px', borderBottom: '1px solid var(--glass-border)', paddingBottom: '12px' }}>
            <div>
                <h1 style={{ margin: 0, fontSize: '1.4em', color: '#fff', fontWeight: 600 }}>Kinematic & Differential Trajectory Analysis</h1>
                <span style={{ fontSize: '0.85em', color: 'var(--text-muted)' }}>
                    Continuous Time Interpolation (Quintic Spline) • Total Duration: {totalDuration.toFixed(2)}s • {plannedPath.length} Waypoints
                </span>
            </div>
            <div style={{ display: 'flex', gap: '15px', fontSize: '0.85em', background: 'rgba(255, 255, 255, 0.02)', padding: '6px 14px', borderRadius: '6px', border: '1px solid var(--glass-border)' }}>
                <span style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                    <span style={{ display: 'inline-block', width: '10px', height: '10px', borderRadius: '50%', background: colors[0] }}></span>
                    θ₁ (Base Yaw)
                </span>
                <span style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                    <span style={{ display: 'inline-block', width: '10px', height: '10px', borderRadius: '50%', background: colors[1] }}></span>
                    θ₂ (Shoulder Pitch)
                </span>
                {!useHorizontal && (
                    <span style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                        <span style={{ display: 'inline-block', width: '10px', height: '10px', borderRadius: '50%', background: colors[2] }}></span>
                        θ₃ (Elbow Pitch)
                    </span>
                )}
                <span style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                    <span style={{ display: 'inline-block', width: '10px', height: '10px', borderRadius: '50%', background: manipColors[0] }}></span>
                    w (Yoshikawa Index)
                </span>
            </div>
        </div>
    );

    if (isFullScreen) {
        return (
            <div style={{ display: 'flex', flexDirection: 'column', height: '100%', width: '100%', boxSizing: 'border-box' }}>
                {renderHeader()}
                
                <div style={{ display: 'grid', gridTemplateColumns: 'repeat(2, 1fr)', gap: '20px', flex: 1, overflowY: 'auto' }}>
                    <div className="card" style={{ display: 'flex', flexDirection: 'column', justifyContent: 'center' }}>
                        <LineChart title="Joint Positions" data={positions} colors={colors} xValues={times} height={chartHeight} yUnit="°" />
                    </div>
                    <div className="card" style={{ display: 'flex', flexDirection: 'column', justifyContent: 'center' }}>
                        <LineChart title="Joint Velocities" data={velocities} colors={colors} xValues={times.slice(0, -1)} height={chartHeight} yUnit="°/s" />
                    </div>
                    <div className="card" style={{ display: 'flex', flexDirection: 'column', justifyContent: 'center' }}>
                        <LineChart title="Joint Accelerations" data={accelerations} colors={colors} xValues={times.slice(1, -1)} height={chartHeight} yUnit="°/s²" />
                    </div>
                    <div className="card" style={{ display: 'flex', flexDirection: 'column', justifyContent: 'center' }}>
                        <LineChart title="Yoshikawa Manipulability (Singularity Proximity)" data={manipulability} colors={manipColors} xValues={times} height={chartHeight} yUnit="" />
                    </div>
                </div>
            </div>
        );
    }

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
                {!useHorizontal && (
                    <span style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
                        <span style={{ display: 'inline-block', width: '8px', height: '8px', borderRadius: '50%', background: colors[2] }}></span>
                        θ₃
                    </span>
                )}
            </div>

            <LineChart title="Joint positions (deg)" data={positions} colors={colors} xValues={times} height={chartHeight} yUnit="°" />
            
            {velocities.length > 0 ? (
                <LineChart title="Joint velocities (deg/s)" data={velocities} colors={colors} xValues={times.slice(0, -1)} height={chartHeight} yUnit="°/s" />
            ) : (
                <div style={{ fontSize: '0.75em', color: 'var(--text-muted)', textAlign: 'center', margin: '8px 0' }}>
                    Velocity requires at least 2 points.
                </div>
            )}

            {accelerations.length > 0 ? (
                <LineChart title="Joint accelerations (deg/s²)" data={accelerations} colors={colors} xValues={times.slice(1, -1)} height={chartHeight} yUnit="°/s²" />
            ) : (
                <div style={{ fontSize: '0.75em', color: 'var(--text-muted)', textAlign: 'center', margin: '8px 0' }}>
                    Acceleration requires at least 3 points.
                </div>
            )}
        </div>
    );
}
