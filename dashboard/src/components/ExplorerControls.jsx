import React, { useState } from 'react';
import * as ROSLIB from 'roslib';
import { webCmdPub } from '../services/ros';

export default function ExplorerControls() {
    const [obstacle, setObstacle] = useState('no_obstacles');
    const [resolution, setResolution] = useState('15.0');

    const handleEnvChange = () => {
        const payload = {
            action: "change_cspace",
            obstacle_type: obstacle,
            step_size_deg: parseFloat(resolution)
        };

        webCmdPub.publish({
            data: JSON.stringify(payload)
        });
        console.log("Published change_cspace command:", payload);
    };

    return (
        <div className="card">
            <h2>C-Space Explorer</h2>
            
            <div className="form-group">
                <label htmlFor="select-obstacle">Workspace Obstacles</label>
                <select
                    id="select-obstacle"
                    value={obstacle}
                    onChange={(e) => setObstacle(e.target.value)}
                    className="select-field"
                >
                    <option value="no_obstacles">No Obstacles</option>
                    <option value="box_obstacle">Single Box Obstacle</option>
                    <option value="narrow_passage">Narrow Passage</option>
                    <option value="u_obstacle">U-Shaped Obstacle (Trap)</option>
                    <option value="toroidal_wall">Toroidal Wall Constraint</option>
                </select>
            </div>

            <div className="form-group">
                <label htmlFor="select-resolution">C-Space Resolution</label>
                <select
                    id="select-resolution"
                    value={resolution}
                    onChange={(e) => setResolution(e.target.value)}
                    className="select-field"
                >
                    <option value="6.0">6.0° (Fine / Heavy)</option>
                    <option value="8.0">8.0° (Medium-Fine / Recommended)</option>
                    <option value="10.0">10.0° (Medium / Fast)</option>
                    <option value="12.0">12.0° (Coarse)</option>
                    <option value="15.0">15.0° (Very Coarse / Light)</option>
                </select>
            </div>

            <button onClick={handleEnvChange} className="btn btn-secondary" style={{ width: '100%' }}>
                Load C-Space Cache
            </button>
        </div>
    );
}
