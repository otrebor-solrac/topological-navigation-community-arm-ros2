import React, { useState, useEffect, useRef } from 'react';
import { ros, loadParameters, webCmdPub, statusSub } from './services/ros';
import ThreeVisualizer from './components/ThreeVisualizer';
import JointSliders from './components/JointSliders';
import PlannerControls from './components/PlannerControls';
import WaypointManager from './components/WaypointManager';
import TraceTable from './components/TraceTable';
import KinematicsProfile from './components/KinematicsProfile';

export default function App() {
    const [currentQ, setCurrentQ] = useState(null);
    const [firstPositionReceived, setFirstPositionReceived] = useState(false);
    const [activeTab, setActiveTab] = useState('exploration');
    const [connectionStatus, setConnectionStatus] = useState('disconnected');

    // Global obstacle & resolution states
    const [obstacle, setObstacle] = useState('box_obstacle');
    const [resolution, setResolution] = useState('15.0');

    // Planned path state
    const [plannedPath, setPlannedPath] = useState([]);

    // Visibility states
    const [showTrail, setShowTrail] = useState(true);
    const [showSelfCollision, setShowSelfCollision] = useState(true);
    const [showObstacleCollision, setShowObstacleCollision] = useState(true);
    const [cspaceMode, setCspaceMode] = useState('obs'); // 'obs' or 'free'

    const trailRef = useRef(null);

    // Monitor ROS connection
    useEffect(() => {
        const handleConnect = () => {
            setConnectionStatus('connected');
            loadParameters();
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

    // Monitor Planner Status to get computed paths
    useEffect(() => {
        const handleStatus = (msg) => {
            try {
                const data = JSON.parse(msg.data);
                if (data.success && data.path) {
                    setPlannedPath(data.path);
                    setActiveTab('kinematics'); // Automatically switch to Kinematics tab on success
                } else if (!data.success) {
                    setPlannedPath([]);
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
    };

    return (
        <div className="app-container">
            {/* 3D Viewport container */}
            <div className="canvas-container">
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

                <ThreeVisualizer
                    showSelfCollision={showSelfCollision}
                    showObstacleCollision={showObstacleCollision}
                    cspaceMode={cspaceMode}
                    showTrail={showTrail}
                    trailRef={trailRef}
                    onQUpdate={handleQUpdate}
                />
            </div>

            {/* Glassmorphic Sidebar */}
            <div className="sidebar">
                <div style={{ marginBottom: '5px' }}>
                    <h3 style={{ fontSize: '1.2em', color: '#fff', fontWeight: 600 }}>
                        Control center
                    </h3>
                    <span style={{ fontSize: '0.75em', color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '0.5px' }}>
                        White-box topology monitor
                    </span>
                </div>

                {/* Tab Navigation */}
                <div className="tab-bar">
                    <button
                        onClick={() => setActiveTab('exploration')}
                        className={`tab-btn ${activeTab === 'exploration' ? 'active' : ''}`}
                    >
                        Exploration
                    </button>
                    <button
                        onClick={() => setActiveTab('planning')}
                        className={`tab-btn ${activeTab === 'planning' ? 'active' : ''}`}
                    >
                        Planning
                    </button>
                    <button
                        onClick={() => setActiveTab('kinematics')}
                        className={`tab-btn ${activeTab === 'kinematics' ? 'active' : ''}`}
                    >
                        Kinematics
                    </button>
                </div>

                {/* Tab 1: Exploration */}
                <div className={`tab-content ${activeTab === 'exploration' ? 'active' : ''}`}>
                    <div style={{ marginBottom: '12px' }}>
                        <span style={{ fontSize: '0.8em', color: 'var(--text-muted)' }}>
                            Manual joint positioning & waypoint creation
                        </span>
                    </div>
                    <JointSliders
                        currentQ={currentQ}
                        firstPositionReceived={firstPositionReceived}
                    />
                    <div style={{ marginTop: '16px' }}>
                        <WaypointManager currentQ={currentQ} />
                    </div>

                    {/* Visibility & settings moved here */}
                    <div className="card" style={{ marginTop: '16px' }}>
                        <h2>Visibility & settings</h2>

                        <button
                            onClick={handleClearTrail}
                            className="btn btn-secondary"
                            style={{ width: '100%', marginBottom: '15px' }}
                        >
                            Clear path trail
                        </button>

                        <div>
                            <label className="checkbox-container">
                                <input
                                    type="checkbox"
                                    checked={showTrail}
                                    onChange={(e) => handleToggleTrail(e.target.checked)}
                                />
                                Show path trail
                            </label>
                        </div>

                        <div style={{ marginTop: '8px' }}>
                            <label className="checkbox-container">
                                <input
                                    type="checkbox"
                                    checked={showSelfCollision}
                                    onChange={(e) => setShowSelfCollision(e.target.checked)}
                                    disabled={cspaceMode !== 'obs'}
                                />
                                Show self-collisions (C-self)
                            </label>
                        </div>

                        <div style={{ marginTop: '8px' }}>
                            <label className="checkbox-container">
                                <input
                                    type="checkbox"
                                    checked={showObstacleCollision}
                                    onChange={(e) => setShowObstacleCollision(e.target.checked)}
                                    disabled={cspaceMode !== 'obs'}
                                />
                                Show obstacle-collisions (C-obs)
                            </label>
                        </div>

                        <div style={{ marginTop: '12px' }}>
                            <button
                                onClick={toggleCspaceMode}
                                className={`btn ${cspaceMode === 'obs' ? 'btn-danger' : 'btn-secondary'}`}
                                style={{ width: '100%', fontSize: '0.8em' }}
                            >
                                {cspaceMode === 'obs' ? 'Current: C-obs (obstacles)' : 'Current: C-free (workspace)'}
                            </button>
                        </div>
                    </div>
                </div>

                {/* Tab 2: Planning */}
                <div className={`tab-content ${activeTab === 'planning' ? 'active' : ''}`}>
                    <div style={{ marginBottom: '12px' }}>
                        <span style={{ fontSize: '0.8em', color: 'var(--text-muted)' }}>
                            Trajectory solving & execution
                        </span>
                    </div>
                    <PlannerControls />

                    {/* Visibility & settings also present here */}
                    <div className="card" style={{ marginTop: '16px' }}>
                        <h2>Visibility & settings</h2>

                        <button
                            onClick={handleClearTrail}
                            className="btn btn-secondary"
                            style={{ width: '100%', marginBottom: '15px' }}
                        >
                            Clear path trail
                        </button>

                        <div>
                            <label className="checkbox-container">
                                <input
                                    type="checkbox"
                                    checked={showTrail}
                                    onChange={(e) => handleToggleTrail(e.target.checked)}
                                />
                                Show path trail
                            </label>
                        </div>

                        <div style={{ marginTop: '8px' }}>
                            <label className="checkbox-container">
                                <input
                                    type="checkbox"
                                    checked={showSelfCollision}
                                    onChange={(e) => setShowSelfCollision(e.target.checked)}
                                    disabled={cspaceMode !== 'obs'}
                                />
                                Show self-collisions (C-self)
                            </label>
                        </div>

                        <div style={{ marginTop: '8px' }}>
                            <label className="checkbox-container">
                                <input
                                    type="checkbox"
                                    checked={showObstacleCollision}
                                    onChange={(e) => setShowObstacleCollision(e.target.checked)}
                                    disabled={cspaceMode !== 'obs'}
                                />
                                Show obstacle-collisions (C-obs)
                            </label>
                        </div>

                        <div style={{ marginTop: '12px' }}>
                            <button
                                onClick={toggleCspaceMode}
                                className={`btn ${cspaceMode === 'obs' ? 'btn-danger' : 'btn-secondary'}`}
                                style={{ width: '100%', fontSize: '0.8em' }}
                            >
                                {cspaceMode === 'obs' ? 'Current: C-obs (obstacles)' : 'Current: C-free (workspace)'}
                            </button>
                        </div>
                    </div>
                </div>

                {/* Tab 3: Kinematics */}
                <div className={`tab-content ${activeTab === 'kinematics' ? 'active' : ''}`}>
                    <div style={{ marginBottom: '12px' }}>
                        <span style={{ fontSize: '0.8em', color: 'var(--text-muted)' }}>
                            Planned path velocity & acceleration plots
                        </span>
                    </div>
                    <KinematicsProfile plannedPath={plannedPath} />
                    <div style={{ marginTop: '16px' }}>
                        <TraceTable plannedPath={plannedPath} />
                    </div>
                </div>
            </div>
        </div>
    );
}
