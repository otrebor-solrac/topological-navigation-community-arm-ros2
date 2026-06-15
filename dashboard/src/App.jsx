import React, { useState, useEffect, useRef } from 'react';
import { ros, loadParameters, webCmdPub } from './services/ros';
import ThreeVisualizer from './components/ThreeVisualizer';
import JointSliders from './components/JointSliders';
import PlannerControls from './components/PlannerControls';
import ExplorerControls from './components/ExplorerControls';
import WaypointManager from './components/WaypointManager';
import TraceTable from './components/TraceTable';

export default function App() {
    const [currentQ, setCurrentQ] = useState(null);
    const [firstPositionReceived, setFirstPositionReceived] = useState(false);
    const [activeTab, setActiveTab] = useState('explorer');
    const [connectionStatus, setConnectionStatus] = useState('disconnected');
    
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
            // Dynamically load offsets and multipliers from ROS parameter server
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

        // Check if already connected (in case component mounts after connection)
        if (ros.isConnected) {
            handleConnect();
        }

        return () => {
            ros.off('connection', handleConnect);
            ros.off('error', handleError);
            ros.off('close', handleClose);
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
                <div className="overlay-header">
                    <h1>T³ MANIFOLD VISUALIZER</h1>
                    <div className={`connection-status ${connectionStatus}`}>
                        {connectionStatus === 'connected' && 'ROS: CONNECTED'}
                        {connectionStatus === 'disconnected' && 'ROS: DISCONNECTED'}
                        {connectionStatus === 'error' && 'ROS: CONNECTION ERROR'}
                    </div>
                </div>

                <div className="top-buttons">
                    <button onClick={handleClearTrail} className="btn">
                        Clear Path Trail
                    </button>
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
                        Control Center
                    </h3>
                    <span style={{ fontSize: '0.75em', color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '0.5px' }}>
                        White-Box Topology Monitor
                    </span>
                </div>

                {/* Joint Sliders */}
                <JointSliders
                    currentQ={currentQ}
                    firstPositionReceived={firstPositionReceived}
                />

                {/* Tab Navigation */}
                <div className="tab-bar">
                    <button
                        onClick={() => setActiveTab('explorer')}
                        className={`tab-btn ${activeTab === 'explorer' ? 'active' : ''}`}
                    >
                        Explorer
                    </button>
                    <button
                        onClick={() => setActiveTab('planner')}
                        className={`tab-btn ${activeTab === 'planner' ? 'active' : ''}`}
                    >
                        Planner
                    </button>
                    <button
                        onClick={() => setActiveTab('sequencer')}
                        className={`tab-btn ${activeTab === 'sequencer' ? 'active' : ''}`}
                    >
                        Sequencer
                    </button>
                </div>

                {/* Tab Content */}
                <div className={`tab-content ${activeTab === 'explorer' ? 'active' : ''}`}>
                    <ExplorerControls />
                </div>

                <div className={`tab-content ${activeTab === 'planner' ? 'active' : ''}`}>
                    <PlannerControls />
                </div>

                <div className={`tab-content ${activeTab === 'sequencer' ? 'active' : ''}`}>
                    <WaypointManager currentQ={currentQ} />
                </div>

                {/* Visibility Controls */}
                <div className="card">
                    <h2>⚙️ Visibility & Settings</h2>
                    <label className="checkbox-container">
                        <input
                            type="checkbox"
                            checked={showTrail}
                            onChange={(e) => handleToggleTrail(e.target.checked)}
                        />
                        Show Trajectory Trail
                    </label>
                    
                    <div style={{ marginTop: '8px' }}>
                        <label className="checkbox-container">
                            <input
                                type="checkbox"
                                checked={showSelfCollision}
                                onChange={(e) => setShowSelfCollision(e.target.checked)}
                                disabled={cspaceMode !== 'obs'}
                            />
                            Show Self-Collisions (C-Self)
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
                            Show Obstacle-Collisions (C-Obs)
                        </label>
                    </div>

                    <div style={{ marginTop: '12px' }}>
                        <button
                            onClick={toggleCspaceMode}
                            className={`btn ${cspaceMode === 'obs' ? 'btn-danger' : 'btn-secondary'}`}
                            style={{ width: '100%', fontSize: '0.8em' }}
                        >
                            {cspaceMode === 'obs' ? 'CURRENT: C-OBS (OBSTACLES)' : 'CURRENT: C-FREE (WORKSPACE)'}
                        </button>
                    </div>
                </div>

                {/* Live Traceability Table */}
                <TraceTable currentQ={currentQ} />
            </div>
        </div>
    );
}
