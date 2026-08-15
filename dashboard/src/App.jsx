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
import VisibilitySection from './components/VisibilitySection';

import { computeFK } from './utils/kinematics';

export default function App() {
    const [currentQ, setCurrentQ] = useState(null);
    const [firstPositionReceived, setFirstPositionReceived] = useState(false);
    const [activeTab, setActiveTab] = useState('joint');
    const [connectionStatus, setConnectionStatus] = useState('disconnected');
    const [sidebarOpen, setSidebarOpen] = useState(true);

    const [homeQ, setHomeQ] = useState(null);
    const [homeCartesian, setHomeCartesian] = useState(null);

    const [obstacle, setObstacle] = useState('no_obstacles');
    const [resolution, setResolution] = useState('15.0');

    const [isExecuting, setIsExecuting] = useState(false);

    const [plannedPath, setPlannedPath] = useState([]);
    const [plannedManipulability, setPlannedManipulability] = useState([]);

    const [showTrail, setShowTrail] = useState(false);
    const [showSelfCollision, setShowSelfCollision] = useState(false);
    const [showObstacleCollision, setShowObstacleCollision] = useState(false);
    const [cspaceMode, setCspaceMode] = useState('obs');
    const [isLoadingVoxels, setIsLoadingVoxels] = useState(true);
    const [loadingTitle, setLoadingTitle] = useState('LOADING MANIFOLD CACHE');
    const [loadingSubtext, setLoadingSubtext] = useState('Reading pre-computed C-space cache from disk...');
    const [showVisibilitySettings, setShowVisibilitySettings] = useState(false);

    const [waypoints, setWaypoints] = useState([]);
    const [cartesianWaypoints, setCartesianWaypoints] = useState([]);

    const trailRef = useRef(null);

    // Monitor ROS connection
    useEffect(() => {
        const handleConnect = () => setConnectionStatus('connected');
        const handleError   = () => setConnectionStatus('error');
        const handleClose   = () => setConnectionStatus('disconnected');

        ros.on('connection', handleConnect);
        ros.on('error', handleError);
        ros.on('close', handleClose);

        if (ros.isConnected) handleConnect();

        return () => {
            ros.off('connection', handleConnect);
            ros.off('error', handleError);
            ros.off('close', handleClose);
        };
    }, []);

    // Subscribe to latched /planner_start_config
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
                    setPlannedManipulability(data.manipulability ?? []);
                    handleClearTrail();
                    setIsExecuting(true);
                } else if (data.success && data.message?.includes('complete ✅')) {
                    setIsExecuting(false);
                } else if (!data.success) {
                    setPlannedPath([]);
                    setPlannedManipulability([]);
                    setIsExecuting(false);
                }
            } catch (e) {
                console.error('Failed to parse status message in App:', e);
            }
        };
        statusSub.subscribe(handleStatus);
        return () => statusSub.unsubscribe(handleStatus);
    }, []);

    const handleQUpdate = (q) => {
        setCurrentQ(q);
        if (!firstPositionReceived) setFirstPositionReceived(true);
    };

    const handleClearTrail = () => {
        if (trailRef.current?.clear) trailRef.current.clear();
        webCmdPub.publish({ data: JSON.stringify({ action: 'clear_trail' }) });
    };

    const handleEnvChange = (newObstacle, newResolution) => {
        setLoadingTitle('LOADING MANIFOLD CACHE');
        setLoadingSubtext('Reading pre-computed C-space cache from disk...');
        setIsLoadingVoxels(true);
        handleClearTrail();
        setPlannedPath([]);
        webCmdPub.publish({
            data: JSON.stringify({ action: 'change_cspace', obstacle_type: newObstacle, step_size_deg: parseFloat(newResolution) })
        });
    };

    const handleObstacleChange   = (val) => { setObstacle(val);   handleEnvChange(val, resolution); };
    const handleResolutionChange = (val) => { setResolution(val); handleEnvChange(obstacle, val); };

    const handleToggleTrail = (checked) => {
        setShowTrail(checked);
        webCmdPub.publish({ data: JSON.stringify({ action: 'toggle_trail', show: checked }) });
    };

    const toggleCspaceMode = () => setCspaceMode(prev => prev === 'obs' ? 'free' : 'obs');

    /* Shared visibility props — passed to VisibilitySection (proper external component) */
    const visibilityProps = {
        showVisibilitySettings,
        setShowVisibilitySettings,
        showTrail,
        handleToggleTrail,
        showSelfCollision,
        setShowSelfCollision,
        showObstacleCollision,
        setShowObstacleCollision,
        cspaceMode,
        toggleCspaceMode,
        handleClearTrail,
    };

    return (
        <div className="app-container">

            {/* ── Sidebar ── */}
            <div className={`sidebar ${sidebarOpen ? '' : 'collapsed'}`}>
                <div className="sidebar-inner">

                    {/* Header */}
                    <div className="sidebar-header">
                        <div className="sidebar-header-text">
                            <h3>Control center</h3>
                            <span>White-box topology monitor</span>
                        </div>
                        <button
                            className="sidebar-collapse-btn"
                            onClick={() => setSidebarOpen(false)}
                            title="Hide panel"
                        >
                            ‹
                        </button>
                    </div>

                    {/* Tab navigation */}
                    <div className="tab-bar">
                        <button onClick={() => setActiveTab('joint')}      className={`tab-btn ${activeTab === 'joint'      ? 'active' : ''}`}>Joint space</button>
                        <button onClick={() => setActiveTab('cartesian')}  className={`tab-btn ${activeTab === 'cartesian'  ? 'active' : ''}`}>Cartesian space</button>
                        <button onClick={() => setActiveTab('kinematics')} className={`tab-btn ${activeTab === 'kinematics' ? 'active' : ''}`}>Kinematics</button>
                    </div>

                    {/* Scrollable content */}
                    <div className="sidebar-scroll">

                        {/* ── Tab: Joint space ── */}
                        <div className={`tab-content ${activeTab === 'joint' ? 'active' : ''}`}>
                            <ControlPanel
                                currentQ={currentQ}
                                firstPositionReceived={firstPositionReceived}
                                homeQ={homeQ}
                                setHomeQ={setHomeQ}
                            />
                            <WaypointManager
                                currentQ={currentQ}
                                homeQ={homeQ}
                                waypoints={waypoints}
                                setWaypoints={setWaypoints}
                            />
                            <VisibilitySection {...visibilityProps} />
                        </div>

                        {/* ── Tab: Cartesian space ── */}
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
                            <ObstaclePositioner
                                activeObstacle={obstacle}
                                resolution={resolution}
                                onApplyPosition={() => {
                                    setLoadingTitle('COMPUTING C-SPACE ON-THE-FLY');
                                    setLoadingSubtext('Solving 13,824 manifold states in Rust for updated obstacle position...');
                                    setIsLoadingVoxels(true);
                                }}
                            />
                            <VisibilitySection {...visibilityProps} />
                        </div>

                        {/* ── Tab: Kinematics ── */}
                        <div className={`tab-content ${activeTab === 'kinematics' ? 'active' : ''}`}>
                            <div className="section">
                                <div className="section-header">Trajectory summary</div>
                                <p style={{ fontSize: '0.85em', color: 'var(--text-muted)', lineHeight: '1.5' }}>
                                    Charts in the main viewport show joint positions, velocities, accelerations, and manipulability over time.
                                </p>
                            </div>
                            <TraceTable plannedPath={plannedPath} />
                        </div>

                    </div>
                </div>
            </div>

            {/* ── Sidebar toggle (shown when collapsed) ── */}
            {!sidebarOpen && (
                <button
                    className="sidebar-toggle"
                    onClick={() => setSidebarOpen(true)}
                    title="Show panel"
                >
                    ›
                </button>
            )}

            {/* ── 3D Viewport ── */}
            <div className="canvas-container">

                {/* C-space loading overlay */}
                {isLoadingVoxels && (
                    <div style={{
                        position: 'absolute', top: 0, left: 0, width: '100%', height: '100%',
                        backgroundColor: 'rgba(3, 3, 7, 0.6)',
                        backdropFilter: 'blur(6px)',
                        zIndex: 150,
                        display: 'flex', flexDirection: 'column',
                        justifyContent: 'center', alignItems: 'center', gap: '16px',
                        fontFamily: 'Share Tech Mono, monospace',
                    }}>
                        <div style={{
                            width: '44px', height: '44px',
                            border: '2px solid rgba(0, 255, 157, 0.15)',
                            borderTop: '2px solid var(--accent-green)',
                            borderRadius: '50%',
                            animation: 'spin 1s linear infinite',
                        }} />
                        <div style={{ fontSize: '0.9em', letterSpacing: '2px', color: 'var(--accent-green)' }}>
                            {loadingTitle}
                        </div>
                        <div style={{ fontSize: '0.78em', color: 'var(--text-muted)' }}>
                            {loadingSubtext}
                        </div>
                    </div>
                )}

                {activeTab !== 'kinematics' && (
                    <div className="overlay-header" style={{ pointerEvents: 'none', display: 'flex', justifyContent: 'space-between', width: 'calc(100% - 40px)' }}>
                        <div>
                            <h1>T³ manifold visualizer</h1>
                            <div className={`connection-status ${connectionStatus}`}>
                                {connectionStatus === 'connected'    && 'ROS: Connected'}
                                {connectionStatus === 'disconnected' && 'ROS: Disconnected'}
                                {connectionStatus === 'error'        && 'ROS: Connection error'}
                            </div>
                        </div>

                        {/* Environment & resolution selectors */}
                        <div className="env-bar" style={{ pointerEvents: 'auto' }}>
                            <div className="env-bar-group">
                                <span className="env-bar-label">Obstacles</span>
                                <select value={obstacle} onChange={(e) => handleObstacleChange(e.target.value)} className="select-field" style={{ width: '165px' }}>
                                    <option value="no_obstacles">No obstacles</option>
                                    <option value="box_obstacle">Single box obstacle</option>
                                    <option value="narrow_passage">Narrow passage</option>
                                    <option value="u_obstacle">U-shaped obstacle (trap)</option>
                                    <option value="toroidal_wall">Toroidal wall constraint</option>
                                </select>
                            </div>
                            <div className="env-bar-group">
                                <span className="env-bar-label">Resolution</span>
                                <select value={resolution} onChange={(e) => handleResolutionChange(e.target.value)} className="select-field" style={{ width: '130px' }}>
                                    <option value="6.0">6.0° — fine / heavy</option>
                                    <option value="8.0">8.0° — medium fine</option>
                                    <option value="10.0">10.0° — medium</option>
                                    <option value="12.0">12.0° — coarse</option>
                                    <option value="15.0">15.0° — coarse / light</option>
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
