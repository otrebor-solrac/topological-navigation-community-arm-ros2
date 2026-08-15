import React, { useState, useEffect, useRef } from 'react';
import { ros, startConfigSub, webCmdPub, statusSub } from './services/ros';

import ThreeVisualizer from './components/ThreeVisualizer';
import ControlPanel from './components/ControlPanel';
import CartesianOriginPanel from './components/CartesianOriginPanel';
import CartesianWaypointManager from './components/CartesianWaypointManager';
import ObstaclePositioner from './components/ObstaclePositioner';
import WaypointManager from './components/WaypointManager';
import TraceTable from './components/TraceTable';
import KinematicsProfile from './components/KinematicsProfile';


import { computeFK } from './utils/kinematics';

export default function App() {
    const [currentQ, setCurrentQ] = useState(null);
    const [firstPositionReceived, setFirstPositionReceived] = useState(false);
    const [activeTab, setActiveTab] = useState('joint'); // 'joint', 'cartesian', 'kinematics'
    const [connectionStatus, setConnectionStatus] = useState('disconnected');

    // homeQ: loaded from planner_params.yaml via /whitebox_planner/get_parameters service
    const [homeQ, setHomeQ] = useState(null);
    const [homeCartesian, setHomeCartesian] = useState(null);

    // Global obstacle & resolution states
    const [obstacle, setObstacle] = useState('no_obstacles');
    const [resolution, setResolution] = useState('15.0');

    // Trajectory animation execution state
    const [isExecuting, setIsExecuting] = useState(false);

    // Planned path state
    const [plannedPath, setPlannedPath] = useState([]);
    const [plannedManipulability, setPlannedManipulability] = useState([]);

    // Visibility states
    const [showTrail, setShowTrail] = useState(false);
    const [showSelfCollision, setShowSelfCollision] = useState(false);
    const [showObstacleCollision, setShowObstacleCollision] = useState(false);
    const [cspaceMode, setCspaceMode] = useState('obs'); // 'obs' or 'free'
    const [isLoadingVoxels, setIsLoadingVoxels] = useState(true);
    const [showVisibilitySettings, setShowVisibilitySettings] = useState(false);

    // Waypoints state lifted up from WaypointManager
    const [waypoints, setWaypoints] = useState([]);
    const [cartesianWaypoints, setCartesianWaypoints] = useState([]);

    const trailRef = useRef(null);


    // Monitor ROS connection
    useEffect(() => {
        const handleConnect = () => {
            setConnectionStatus('connected');
        };

        const handleError = () => {
            setConnectionStatus('error');
        };

        const handleClose = () => {
            setConnectionStatus('disconnected');
        };

        ros.on('connection', handleConnect);
        ros.on('error', handleError);
        ros.on('close', handleClose);

        if (ros.isConnected) {
            handleConnect();
        }

        return () => {
            ros.off('connection', handleConnect);
            ros.off('error', handleError);
            ros.off('close', handleClose);
        };
    }, []);

    // Subscribe to the latched /planner_start_config topic.
    // TRANSIENT_LOCAL QoS ensures we receive the last message even if we connect after the planner.
    useEffect(() => {
        const handleStartConfig = (msg) => {
            try {
                const data = JSON.parse(msg.data);
                if (data.start && data.start.length >= 3) {
                    setHomeQ({ q1: data.start[0], q2: data.start[1], q3: data.start[2] });
                    const fk = computeFK(data.start[0], data.start[1], data.start[2]);
                    const x_m = parseFloat(fk.x_cm) / 100.0;
                    const y_m = parseFloat(fk.y_cm) / 100.0;
                    const z_m = parseFloat(fk.z_cm) / 100.0;
                    setHomeCartesian([x_m, y_m, z_m]);
                }
            } catch (e) {
                console.error('Failed to parse planner_start_config:', e);
            }
        };
        startConfigSub.subscribe(handleStartConfig);
        return () => startConfigSub.unsubscribe(handleStartConfig);
    }, []);

    useEffect(() => {
        const handleStatus = (msg) => {
            try {
                const data = JSON.parse(msg.data);
                if (data.success && data.path) {
                    setPlannedPath(data.path);
                    if (data.manipulability) {
                        setPlannedManipulability(data.manipulability);
                    } else {
                        setPlannedManipulability([]);
                    }
                    handleClearTrail(); // Clear old trail before starting the new animation
                    setIsExecuting(true);
                } else if (data.success && data.message && data.message.includes("complete ✅")) {
                    setIsExecuting(false);
                } else if (!data.success) {
                    setPlannedPath([]);
                    setPlannedManipulability([]);
                    setIsExecuting(false);
                }
            } catch (e) {
                console.error("Failed to parse status message in App:", e);
            }
        };

        statusSub.subscribe(handleStatus);
        return () => {
            statusSub.unsubscribe(handleStatus);
        };
    }, []);

    const handleQUpdate = (q) => {
        setCurrentQ(q);
        if (!firstPositionReceived) {
            setFirstPositionReceived(true);
        }
    };

    const handleClearTrail = () => {
        if (trailRef.current && trailRef.current.clear) {
            trailRef.current.clear();
        }
        webCmdPub.publish({
            data: JSON.stringify({ action: 'clear_trail' })
        });
    };

    const handleEnvChange = (newObstacle, newResolution) => {
        setIsLoadingVoxels(true);
        handleClearTrail();
        setPlannedPath([]); // Reset current path since environment changed

        const payload = {
            action: "change_cspace",
            obstacle_type: newObstacle,
            step_size_deg: parseFloat(newResolution)
        };

        webCmdPub.publish({
            data: JSON.stringify(payload)
        });
        console.log("Published change_cspace command:", payload);
    };

    const handleObstacleChange = (val) => {
        setObstacle(val);
        handleEnvChange(val, resolution);
    };

    const handleResolutionChange = (val) => {
        setResolution(val);
        handleEnvChange(obstacle, val);
    };

    const handleToggleTrail = (checked) => {
        setShowTrail(checked);
        webCmdPub.publish({
            data: JSON.stringify({
                action: 'toggle_trail',
                show: checked
            })
        });
    };

    const toggleCspaceMode = () => {
        setCspaceMode(prev => prev === 'obs' ? 'free' : 'obs');
    };    return (
        <div className="app-container">
            {/* Glassmorphic Sidebar */}
            <div className="sidebar" style={{ display: 'flex', flexDirection: 'column', height: '100%', overflow: 'hidden' }}>
                <div style={{ marginBottom: '5px', flexShrink: 0 }}>
                    <h3 style={{ fontSize: '1.2em', color: '#fff', fontWeight: 600 }}>
                        Control center
                    </h3>
                    <span style={{ fontSize: '0.75em', color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '0.5px' }}>
                        White-box topology monitor
                    </span>
                </div>

                {/* Tab Navigation */}
                <div className="tab-bar" style={{ flexShrink: 0, display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '4px' }}>
                    <button
                        onClick={() => setActiveTab('joint')}
                        className={`tab-btn ${activeTab === 'joint' ? 'active' : ''}`}
                        style={{ fontSize: '0.78rem', padding: '6px 4px' }}
                    >
                        Joint space
                    </button>
                    <button
                        onClick={() => setActiveTab('cartesian')}
                        className={`tab-btn ${activeTab === 'cartesian' ? 'active' : ''}`}
                        style={{ fontSize: '0.78rem', padding: '6px 4px' }}
                    >
                        Cartesian space
                    </button>
                    <button
                        onClick={() => setActiveTab('kinematics')}
                        className={`tab-btn ${activeTab === 'kinematics' ? 'active' : ''}`}
                        style={{ fontSize: '0.78rem', padding: '6px 4px' }}
                    >
                        Kinematics & Analytics
                    </button>
                </div>

                {/* Tab Contents Area */}
                <div style={{ flex: 1, overflowY: 'auto', paddingRight: '4px', marginBottom: '16px' }}>
                    {/* Tab 1: Joint space */}
                    <div className={`tab-content ${activeTab === 'joint' ? 'active' : ''}`}>
                        <ControlPanel
                            currentQ={currentQ}
                            firstPositionReceived={firstPositionReceived}
                            homeQ={homeQ}
                            setHomeQ={setHomeQ}
                        />
                        <div style={{ marginTop: '16px' }}>
                            <WaypointManager
                                currentQ={currentQ}
                                homeQ={homeQ}
                                waypoints={waypoints}
                                setWaypoints={setWaypoints}
                            />
                        </div>
                        
                        {/* Visibility & Settings Card */}
                        <div className="card" style={{ marginTop: '16px' }}>
                            <div 
                                onClick={() => setShowVisibilitySettings(!showVisibilitySettings)} 
                                style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', cursor: 'pointer', userSelect: 'none' }}
                            >
                                <h2 style={{ margin: '0', fontSize: '1.05em' }}>
                                    Visibility & settings
                                </h2>
                                <span style={{ fontSize: '0.8rem', color: 'var(--text-muted)', transition: 'transform 0.2s ease', transform: showVisibilitySettings ? 'rotate(180deg)' : 'rotate(0deg)' }}>
                                    ▼
                                </span>
                            </div>

                            {showVisibilitySettings && (
                                <div style={{ marginTop: '12px' }}>
                                    <button
                                        onClick={handleClearTrail}
                                        className="btn btn-secondary"
                                        style={{ width: '100%', marginBottom: '12px', padding: '6px 12px', fontSize: '0.85em' }}
                                    >
                                        Clear path trail
                                    </button>
                                    <div>
                                        <label className="checkbox-container" style={{ fontSize: '0.85em' }}>
                                            <input
                                                type="checkbox"
                                                checked={showTrail}
                                                onChange={(e) => handleToggleTrail(e.target.checked)}
                                            />
                                            Show path trail
                                        </label>
                                    </div>
                                    <div style={{ marginTop: '6px' }}>
                                        <label className="checkbox-container" style={{ fontSize: '0.85em' }}>
                                            <input
                                                type="checkbox"
                                                checked={showSelfCollision}
                                                onChange={(e) => setShowSelfCollision(e.target.checked)}
                                                disabled={cspaceMode !== 'obs'}
                                            />
                                            Show self-collisions (C-self)
                                        </label>
                                    </div>
                                    <div style={{ marginTop: '6px' }}>
                                        <label className="checkbox-container" style={{ fontSize: '0.85em' }}>
                                            <input
                                                type="checkbox"
                                                checked={showObstacleCollision}
                                                onChange={(e) => setShowObstacleCollision(e.target.checked)}
                                                disabled={cspaceMode !== 'obs'}
                                            />
                                            Show obstacle-collisions (C-obs)
                                        </label>
                                    </div>
                                    <div style={{ marginTop: '10px' }}>
                                        <button
                                            onClick={toggleCspaceMode}
                                            className={`btn ${cspaceMode === 'obs' ? 'btn-danger' : 'btn-secondary'}`}
                                            style={{ width: '100%', fontSize: '0.8em', padding: '6px' }}
                                        >
                                            {cspaceMode === 'obs' ? 'Current: C-obs (obstacles)' : 'Current: C-free (workspace)'}
                                        </button>
                                    </div>
                                </div>
                            )}
                        </div>
                    </div>

                    {/* Tab 2: Cartesian space */}
                    <div className={`tab-content ${activeTab === 'cartesian' ? 'active' : ''}`}>
                        <CartesianOriginPanel
                            currentQ={currentQ}
                            homeQ={homeQ}
                            setHomeQ={setHomeQ}
                            homeCartesian={homeCartesian}
                            setHomeCartesian={setHomeCartesian}
                        />
                        <CartesianWaypointManager
                            currentQ={currentQ}
                            homeQ={homeQ}
                            cartesianWaypoints={cartesianWaypoints}
                            setCartesianWaypoints={setCartesianWaypoints}
                            homeCartesian={homeCartesian}
                        />
                        <ObstaclePositioner activeObstacle={obstacle} resolution={resolution} />

                        {/* Visibility & Settings Card */}
                        <div className="card" style={{ marginTop: '16px' }}>
                            <div 
                                onClick={() => setShowVisibilitySettings(!showVisibilitySettings)} 
                                style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', cursor: 'pointer', userSelect: 'none' }}
                            >
                                <h2 style={{ margin: '0', fontSize: '1.05em' }}>
                                    Visibility & settings
                                </h2>
                                <span style={{ fontSize: '0.8rem', color: 'var(--text-muted)', transition: 'transform 0.2s ease', transform: showVisibilitySettings ? 'rotate(180deg)' : 'rotate(0deg)' }}>
                                    ▼
                                </span>
                            </div>

                            {showVisibilitySettings && (
                                <div style={{ marginTop: '12px' }}>
                                    <button
                                        onClick={handleClearTrail}
                                        className="btn btn-secondary"
                                        style={{ width: '100%', marginBottom: '12px', padding: '6px 12px', fontSize: '0.85em' }}
                                    >
                                        Clear path trail
                                    </button>
                                    <div>
                                        <label className="checkbox-container" style={{ fontSize: '0.85em' }}>
                                            <input
                                                type="checkbox"
                                                checked={showTrail}
                                                onChange={(e) => handleToggleTrail(e.target.checked)}
                                            />
                                            Show path trail
                                        </label>
                                    </div>
                                    <div style={{ marginTop: '6px' }}>
                                        <label className="checkbox-container" style={{ fontSize: '0.85em' }}>
                                            <input
                                                type="checkbox"
                                                checked={showSelfCollision}
                                                onChange={(e) => setShowSelfCollision(e.target.checked)}
                                                disabled={cspaceMode !== 'obs'}
                                            />
                                            Show self-collisions (C-self)
                                        </label>
                                    </div>
                                    <div style={{ marginTop: '6px' }}>
                                        <label className="checkbox-container" style={{ fontSize: '0.85em' }}>
                                            <input
                                                type="checkbox"
                                                checked={showObstacleCollision}
                                                onChange={(e) => setShowObstacleCollision(e.target.checked)}
                                                disabled={cspaceMode !== 'obs'}
                                            />
                                            Show obstacle-collisions (C-obs)
                                        </label>
                                    </div>
                                    <div style={{ marginTop: '10px' }}>
                                        <button
                                            onClick={toggleCspaceMode}
                                            className={`btn ${cspaceMode === 'obs' ? 'btn-danger' : 'btn-secondary'}`}
                                            style={{ width: '100%', fontSize: '0.8em', padding: '6px' }}
                                        >
                                            {cspaceMode === 'obs' ? 'Current: C-obs (obstacles)' : 'Current: C-free (workspace)'}
                                        </button>
                                    </div>
                                </div>
                            )}
                        </div>
                    </div>


                    {/* Tab 2: Kinematics & Analytics */}
                    <div className={`tab-content ${activeTab === 'kinematics' ? 'active' : ''}`}>
                        <div className="card" style={{ marginBottom: '16px' }}>
                            <h2 style={{ fontSize: '0.95em', margin: 0 }}>Trajectory Summary</h2>
                            <p style={{ fontSize: '0.8em', color: 'var(--text-muted)', marginTop: '8px', lineHeight: '1.4' }}>
                                The charts in the main viewport show the continuous-time joint positions, velocities, accelerations, and manipulability profile.
                            </p>
                        </div>
                        <TraceTable plannedPath={plannedPath} />
                    </div>
                </div>
            </div>

            {/* 3D Viewport container */}
            <div className="canvas-container">
                {isLoadingVoxels && (
                    <div style={{
                        position: 'absolute',
                        top: 0,
                        left: 0,
                        width: '100%',
                        height: '100%',
                        backgroundColor: 'rgba(6, 6, 12, 0.45)',
                        backdropFilter: 'blur(10px)',
                        WebkitBackdropFilter: 'blur(10px)',
                        zIndex: 150,
                        display: 'flex',
                        flexDirection: 'column',
                        justifyContent: 'center',
                        alignItems: 'center',
                        gap: '20px',
                        color: '#ffffff',
                        fontFamily: 'Share Tech Mono, monospace',
                        transition: 'all 0.35s ease'
                    }}>
                        <div style={{
                            width: '60px',
                            height: '60px',
                            border: '3px solid rgba(0, 212, 255, 0.15)',
                            borderTop: '3px solid #00d4ff',
                            borderRadius: '50%',
                            animation: 'spin 1.2s cubic-bezier(0.5, 0, 0.5, 1) infinite',
                            boxShadow: '0 0 20px rgba(0, 212, 255, 0.3), inset 0 0 20px rgba(0, 212, 255, 0.1)'
                        }} />
                        <div style={{
                            fontSize: '1.2em',
                            letterSpacing: '2px',
                            textTransform: 'uppercase',
                            color: '#00d4ff',
                            textShadow: '0 0 10px rgba(0, 212, 255, 0.5)',
                            fontWeight: 'bold'
                        }}>
                            C-Space Loading...
                        </div>
                        <div style={{
                            fontSize: '0.85em',
                            color: 'var(--text-muted)',
                            letterSpacing: '1px'
                        }}>
                            Generating or loading configuration space manifold
                        </div>
                    </div>
                )}
                {activeTab !== 'kinematics' && (
                    <div className="overlay-header" style={{ pointerEvents: 'none', display: 'flex', justifyContent: 'space-between', width: 'calc(100% - 48px)' }}>
                        <div>
                            <h1 style={{ pointerEvents: 'auto' }}>T³ manifold visualizer</h1>
                            <div className={`connection-status ${connectionStatus}`}>
                                {connectionStatus === 'connected' && 'ROS: Connected'}
                                {connectionStatus === 'disconnected' && 'ROS: Disconnected'}
                                {connectionStatus === 'error' && 'ROS: Connection error'}
                            </div>
                        </div>

                        {/* Global Environment & Resolution Settings elevated to top header */}
                        <div style={{ pointerEvents: 'auto', display: 'flex', gap: '15px', alignItems: 'center', background: 'rgba(6, 6, 12, 0.6)', padding: '10px 16px', borderRadius: '8px', border: '1px solid var(--glass-border)', boxShadow: '0 4px 15px rgba(0, 0, 0, 0.4)' }}>
                            <div style={{ display: 'flex', flexDirection: 'column', gap: '2px' }}>
                                <span style={{ fontSize: '0.75em', color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '0.5px' }}>Obstacles</span>
                                <select
                                    value={obstacle}
                                    onChange={(e) => handleObstacleChange(e.target.value)}
                                    className="select-field"
                                    style={{ width: '170px', padding: '4px 8px', fontSize: '0.85em', background: '#0b0c10' }}
                                >
                                    <option value="no_obstacles">No obstacles</option>
                                    <option value="box_obstacle">Single box obstacle</option>
                                    <option value="narrow_passage">Narrow passage</option>
                                    <option value="u_obstacle">U-shaped obstacle (trap)</option>
                                    <option value="toroidal_wall">Toroidal wall constraint</option>
                                </select>
                            </div>
                            <div style={{ display: 'flex', flexDirection: 'column', gap: '2px' }}>
                                <span style={{ fontSize: '0.75em', color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '0.5px' }}>Resolution</span>
                                <select
                                    value={resolution}
                                    onChange={(e) => handleResolutionChange(e.target.value)}
                                    className="select-field"
                                    style={{ width: '135px', padding: '4px 8px', fontSize: '0.85em', background: '#0b0c10' }}
                                >
                                    <option value="6.0">6.0° (fine / heavy)</option>
                                    <option value="8.0">8.0° (medium-fine)</option>
                                    <option value="10.0">10.0° (medium / fast)</option>
                                    <option value="12.0">12.0° (coarse)</option>
                                    <option value="15.0">15.0° (coarse / light)</option>
                                </select>
                            </div>
                        </div>
                    </div>
                )}

                <div style={{ display: activeTab === 'kinematics' ? 'none' : 'block', width: '100%', height: '100%' }}>
                    <ThreeVisualizer
                        showSelfCollision={showSelfCollision}
                        showObstacleCollision={showObstacleCollision}
                        cspaceMode={cspaceMode}
                        showTrail={showTrail}
                        trailRef={trailRef}
                        onQUpdate={handleQUpdate}
                        isExecuting={isExecuting}
                        waypoints={waypoints}
                        homeQ={homeQ}
                        onVoxelsReceived={() => setIsLoadingVoxels(false)}
                    />
                </div>
                {activeTab === 'kinematics' && (
                    <div className="analytics-container" style={{ width: '100%', height: '100%', padding: '24px', boxSizing: 'border-box', overflowY: 'auto' }}>
                        <KinematicsProfile plannedPath={plannedPath} plannedManipulability={plannedManipulability} isFullScreen={true} />
                    </div>
                )}
            </div>
        </div>
    );
}
