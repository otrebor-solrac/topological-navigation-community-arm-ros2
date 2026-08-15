import React from 'react';

/**
 * VisibilitySection — standalone component (NOT inline in App).
 * Defining it inline inside App would cause React to unmount/remount
 * it on every render (new function reference = new component type).
 */
export default function VisibilitySection({
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
    handleClearTrail
}) {
    return (
        <div className="section" style={{ marginTop: '4px' }}>
            <div
                className="section-header"
                style={{ cursor: 'pointer', userSelect: 'none', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}
                onClick={() => setShowVisibilitySettings(v => !v)}
            >
                <span>Visibility</span>
                <span style={{ fontSize: '0.85em', opacity: 0.5, fontFamily: 'monospace' }}>
                    {showVisibilitySettings ? '▲' : '▼'}
                </span>
            </div>

            {showVisibilitySettings && (
                <div>
                    <button
                        onClick={handleClearTrail}
                        className="btn btn-secondary"
                        style={{ width: '100%', marginBottom: '10px' }}
                    >
                        Clear path trail
                    </button>

                    <label className="checkbox-container">
                        <input
                            type="checkbox"
                            checked={showTrail}
                            onChange={(e) => handleToggleTrail(e.target.checked)}
                        />
                        Show path trail
                    </label>

                    <label className="checkbox-container">
                        <input
                            type="checkbox"
                            checked={showSelfCollision}
                            onChange={(e) => setShowSelfCollision(e.target.checked)}
                            disabled={cspaceMode !== 'obs'}
                        />
                        Show self-collisions (C-self)
                    </label>

                    <label className="checkbox-container">
                        <input
                            type="checkbox"
                            checked={showObstacleCollision}
                            onChange={(e) => setShowObstacleCollision(e.target.checked)}
                            disabled={cspaceMode !== 'obs'}
                        />
                        Show obstacle collisions (C-obs)
                    </label>

                    <div style={{ marginTop: '8px' }}>
                        <button
                            onClick={toggleCspaceMode}
                            className={`btn ${cspaceMode === 'obs' ? 'btn-danger' : 'btn-secondary'}`}
                            style={{ width: '100%', fontSize: '0.82em' }}
                        >
                            {cspaceMode === 'obs'
                                ? 'Active space: C-obs (obstacles)'
                                : 'Active space: C-free (workspace)'}
                        </button>
                    </div>
                </div>
            )}
        </div>
    );
}
