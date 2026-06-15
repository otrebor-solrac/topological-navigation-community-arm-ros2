// --- THREE.JS SCENE CONFIGURATION ---
const container = document.getElementById('canvas-container');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x030307);

const camera = new THREE.PerspectiveCamera(75, container.clientWidth / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(container.clientWidth, window.innerHeight);
renderer.setPixelRatio(window.devicePixelRatio);
container.appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

// --- THE FUNDAMENTAL CUBE BOUNDARY (T³) ---
const PI = Math.PI;
const size = PI * 2; 

const cubeGeom = new THREE.BoxGeometry(size, size, size);
const cubeEdges = new THREE.EdgesGeometry(cubeGeom);
const cubeLine = new THREE.LineSegments(cubeEdges, new THREE.LineBasicMaterial({ color: 0x00ff9d, transparent: true, opacity: 0.15 }));
scene.add(cubeLine);

const axesHelper = new THREE.AxesHelper(PI + 0.3);
scene.add(axesHelper);

// --- ROBOT POSITION INDICATOR (POINT IN T³) ---
const robotGeom = new THREE.SphereGeometry(0.18, 32, 32);
const robotMat = new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0xffffff, emissiveIntensity: 0.8 });
const robotPoint = new THREE.Mesh(robotGeom, robotMat);
scene.add(robotPoint);

// --- TRAJECTORY TRAIL ---
let pathSegments = [];
let showTrail = true;
const trailMaterial = new THREE.LineBasicMaterial({ color: 0xffff00, linewidth: 2.5 });

function createNewPathSegment() {
    const geo = new THREE.BufferGeometry();
    const positions = new Float32Array(5000 * 3);
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const line = new THREE.Line(geo, trailMaterial);
    line.visible = showTrail;
    scene.add(line);
    pathSegments.push(line);
    return { line, array: positions, count: 0 };
}

let activePath = createNewPathSegment();

window.clearTrajectory = function() {
    console.log("Clearing trajectory path...");
    pathSegments.forEach(s => scene.remove(s));
    pathSegments = [];
    activePath = createNewPathSegment();
    // Clear live trace table
    tracePoints = [];
    document.getElementById('trace-table').innerHTML = '';
};

window.toggleTrailVisibility = function(visible) {
    showTrail = visible;
    pathSegments.forEach(s => s.visible = showTrail);
    if (activePath && activePath.line) {
        activePath.line.visible = showTrail;
    }
};

// --- LIVE TRACEABILITY TABLE ---
const tableBody = document.getElementById('trace-table');
let tracePoints = [];
const MAX_TRACE_LOG = 30;

function updateTraceTable(q) {
    if (!tableBody) return;
    tracePoints.unshift([...q]);
    if (tracePoints.length > MAX_TRACE_LOG) tracePoints.pop();

    tableBody.innerHTML = tracePoints.map((p, i) => `
        <tr class="${i === 0 ? 'current-row' : ''}">
            <td>${tracePoints.length - i}</td>
            <td>${p[0].toFixed(3)}</td>
            <td>${p[1].toFixed(3)}</td>
            <td>${p[2].toFixed(3)}</td>
        </tr>
    `).join('');
}

// --- C-SPACE VOXELS RENDERER ---
let selfCollisionMesh = null;
let obstacleCollisionMesh = null;
let freeCollisionMesh = null;

let rawVoxelData = [];
let rawObstacleData = [];
let rawSelfCollisionData = [];

let showSelfCollision = true;
let showObstacleCollision = true;

let cspaceMode = 'obs'; // 'obs' or 'free'

scene.add(new THREE.AmbientLight(0xffffff, 0.45));
const light = new THREE.PointLight(0xffffff, 1.0, 100);
light.position.set(5, 5, 5);
scene.add(light);

camera.position.set(6, 6, 6);

// --- 3D LABELS ---
const labels = {
    th1: document.getElementById('label-th1'),
    th2: document.getElementById('label-th2'),
    th3: document.getElementById('label-th3')
};

function updateLabels() {
    const vectors = {
        th1: new THREE.Vector3(PI + 0.5, 0, 0),
        th2: new THREE.Vector3(0, PI + 0.5, 0),
        th3: new THREE.Vector3(0, 0, PI + 0.5)
    };
    for (let key in vectors) {
        if (labels[key]) {
            const v = vectors[key].clone().project(camera);
            labels[key].style.left = (v.x + 1) / 2 * container.clientWidth + 'px';
            labels[key].style.top = -(v.y - 1) / 2 * window.innerHeight + 'px';
        }
    }
}

// --- TAB NAVIGATION SWITCHING ---
window.switchTab = function(tabName) {
    document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
    document.querySelectorAll('.tab-btn').forEach(btn => btn.classList.remove('active'));
    
    const targetTab = document.getElementById(`tab-${tabName}`);
    const targetBtn = document.getElementById(`tab-btn-${tabName}`);
    
    if (targetTab) targetTab.classList.add('active');
    if (targetBtn) targetBtn.classList.add('active');

    const tracePanel = document.getElementById('panel-traceability');
    if (tracePanel) {
        if (tabName === 'explorer') {
            tracePanel.style.display = 'none';
        } else {
            tracePanel.style.display = 'flex';
        }
    }
};

// --- ROS2 BRIDGE CONNECTION ---
const ros = new ROSLIB.Ros({ url : 'ws://localhost:9090' });
const connTag = document.getElementById('connection');

ros.on('connection', () => {
    connTag.innerText = "ROS: CONNECTED";
    connTag.style.color = "#00ff9d";
    connTag.style.borderColor = "#00ff9d";
    connTag.style.background = "rgba(0, 255, 157, 0.04)";
});

ros.on('error', (error) => {
    connTag.innerText = "ROS: ERROR";
    connTag.style.color = "#ff4d4d";
    connTag.style.borderColor = "#ff4d4d";
    connTag.style.background = "rgba(255, 77, 77, 0.04)";
});

ros.on('close', () => {
    connTag.innerText = "ROS: DISCONNECTED";
    connTag.style.color = "#ff4d4d";
    connTag.style.borderColor = "#ff4d4d";
    connTag.style.background = "rgba(255, 77, 77, 0.04)";
});

// ROS Topics
const webCmdPub = new ROSLIB.Topic({
    ros : ros,
    name : '/web_commands',
    messageType : 'std_msgs/String'
});

const jointSub = new ROSLIB.Topic({
    ros : ros,
    name : '/joint_states',
    messageType : 'sensor_msgs/JointState'
});

// Publisher for web-based joint control sliders (replaces joint_state_publisher_gui)
const guiJointPub = new ROSLIB.Topic({
    ros : ros,
    name : '/web_gui_master_states',
    messageType : 'sensor_msgs/JointState'
});

// Default joint offset and direction configurations (fallback, relative to world axes)
let jointOffsets = {
    base_yaw: 0.0,
    shoulder_pitch: 0.0,
    elbow_pitch: 0.0
};
let jointDirections = {
    base_yaw: 1,
    shoulder_pitch: 1,
    elbow_pitch: 1
};

// Function to query parameters dynamically from ROS 2
function loadParameters() {
    const pOffsets = ['base_yaw', 'shoulder_pitch', 'elbow_pitch'];
    pOffsets.forEach(key => {
        const param = new ROSLIB.Param({
            ros: ros,
            name: `/topological_planner_node/joint_offsets/${key}`
        });
        param.get((val) => {
            if (val !== null && val !== undefined) {
                jointOffsets[key] = val;
                console.log(`Loaded offset for ${key}: ${val}`);
            }
        });
    });

    const pDirs = ['base_yaw', 'shoulder_pitch', 'elbow_pitch'];
    pDirs.forEach(key => {
        const param = new ROSLIB.Param({
            ros: ros,
            name: `/topological_planner_node/joint_directions/${key}`
        });
        param.get((val) => {
            if (val !== null && val !== undefined) {
                jointDirections[key] = val;
                console.log(`Loaded direction for ${key}: ${val}`);
            }
        });
    });
}

let lastQ = null;
let lastTableUpdateTime = 0;
let firstPositionReceived = false;
let sliderPublishInterval = null;
let userDraggingSlider = false;

jointSub.subscribe((msg) => {
    const names = msg.name;
    const pos = msg.position;
    
    let q_urdf = [0, 0, 0];
    let found = 0;
    for(let i=0; i<names.length; i++) {
        if(names[i] === 'base_yaw_joint') { q_urdf[0] = pos[i]; found++; }
        if(names[i] === 'shoulder_pitch_joint') { q_urdf[1] = pos[i]; found++; }
        if(names[i] === 'elbow_pitch_joint') { q_urdf[2] = pos[i]; found++; }
    }

    if (found >= 2) {
        // Convert URDF to World coordinates (in radians) using configured offsets and directions
        const offsetBaseYawRad = jointOffsets.base_yaw * Math.PI / 180.0;
        const offsetShoulderPitchRad = jointOffsets.shoulder_pitch * Math.PI / 180.0;
        const offsetElbowPitchRad = jointOffsets.elbow_pitch * Math.PI / 180.0;

        const q = [
            (q_urdf[0] - offsetBaseYawRad) / jointDirections.base_yaw,
            (q_urdf[1] - offsetShoulderPitchRad) / jointDirections.shoulder_pitch,
            (q_urdf[2] - offsetElbowPitchRad) / jointDirections.elbow_pitch
        ];

        robotPoint.position.set(q[0], q[1], q[2]);

        if (!firstPositionReceived) {
            controls.target.set(q[0], q[1], q[2]);
            camera.position.set(q[0] + 5, q[1] + 5, q[2] + 5);
            controls.update();
            
            // Set the sliders to the actual initial position (World coordinates)
            const rad2deg = 180.0 / Math.PI;
            const degQ1 = q[0] * rad2deg;
            const degQ2 = q[1] * rad2deg;
            const degQ3 = q[2] * rad2deg;
            
            document.getElementById('slider-q1').value = Math.round(degQ1);
            document.getElementById('slider-q2').value = Math.round(degQ2);
            document.getElementById('slider-q3').value = Math.round(degQ3);
            
            document.getElementById('val-q1').textContent = degQ1.toFixed(1) + '°';
            document.getElementById('val-q2').textContent = degQ2.toFixed(1) + '°';
            document.getElementById('val-q3').textContent = degQ3.toFixed(1) + '°';

            firstPositionReceived = true;

            // Start 10Hz publisher only after initializing sliders
            if (sliderPublishInterval) clearInterval(sliderPublishInterval);
            sliderPublishInterval = setInterval(() => {
                publishSliderJointState();
            }, 100);
        }

        // Update sliders if user is not currently dragging them
        if (!userDraggingSlider) {
            const rad2deg = 180.0 / Math.PI;
            const degQ1 = q[0] * rad2deg;
            const degQ2 = q[1] * rad2deg;
            const degQ3 = q[2] * rad2deg;
            
            document.getElementById('slider-q1').value = Math.round(degQ1);
            document.getElementById('slider-q2').value = Math.round(degQ2);
            document.getElementById('slider-q3').value = Math.round(degQ3);
            
            document.getElementById('val-q1').textContent = degQ1.toFixed(1) + '°';
            document.getElementById('val-q2').textContent = degQ2.toFixed(1) + '°';
            document.getElementById('val-q3').textContent = degQ3.toFixed(1) + '°';
        }

        const now = Date.now();
        if (now - lastTableUpdateTime > 250) {
            updateTraceTable(q);
            lastTableUpdateTime = now;
        }

        if (lastQ) {
            const dist = Math.sqrt((q[0]-lastQ[0])**2 + (q[1]-lastQ[1])**2 + (q[2]-lastQ[2])**2);
            if (dist > 1.2) {
                activePath = createNewPathSegment();
            }
        }

        if (activePath.count < 5000) {
            const idx = activePath.count * 3;
            activePath.array[idx] = q[0];
            activePath.array[idx+1] = q[1];
            activePath.array[idx+2] = q[2];
            activePath.count++;
            activePath.line.geometry.attributes.position.needsUpdate = true;
            activePath.line.geometry.setDrawRange(0, activePath.count);
        }
        lastQ = [...q];
    }
});

function computeComplement(data) {
    if (!data || data.length === 0) return [];
    
    const u0 = new Set();
    const u1 = new Set();
    const u2 = new Set();
    
    data.forEach(p => {
        u0.add(p[0].toFixed(4));
        u1.add(p[1].toFixed(4));
        u2.add(p[2].toFixed(4));
    });
    
    const arr0 = Array.from(u0).map(Number).sort((a,b) => a-b);
    const arr1 = Array.from(u1).map(Number).sort((a,b) => a-b);
    const arr2 = Array.from(u2).map(Number).sort((a,b) => a-b);
    
    const forbiddenSet = new Set(data.map(p => `${p[0].toFixed(4)},${p[1].toFixed(4)},${p[2].toFixed(4)}`));
    
    const freePoints = [];
    for (let x of arr0) {
        for (let y of arr1) {
            for (let z of arr2) {
                const key = `${x.toFixed(4)},${y.toFixed(4)},${z.toFixed(4)}`;
                if (!forbiddenSet.has(key)) {
                    freePoints.push([x, y, z]);
                }
            }
        }
    }
    return freePoints;
}

window.toggleCSpaceLayers = function() {
    showSelfCollision = document.getElementById('chk-show-self-collision').checked;
    showObstacleCollision = document.getElementById('chk-show-obstacle-collision').checked;
    renderVoxels();
};

function renderVoxels() {
    if (selfCollisionMesh) { scene.remove(selfCollisionMesh); selfCollisionMesh = null; }
    if (obstacleCollisionMesh) { scene.remove(obstacleCollisionMesh); obstacleCollisionMesh = null; }
    if (freeCollisionMesh) { scene.remove(freeCollisionMesh); freeCollisionMesh = null; }
    
    if (cspaceMode === 'obs') {
        const geo = new THREE.BoxGeometry(0.12, 0.12, 0.12);
        
        // 1. Render self-collisions (C-Self)
        if (showSelfCollision && rawSelfCollisionData.length > 0) {
            const selfMat = new THREE.MeshPhongMaterial({ 
                color: 0x5d4778, 
                transparent: true, 
                opacity: 0.22 
            });
            selfCollisionMesh = new THREE.InstancedMesh(geo, selfMat, rawSelfCollisionData.length);
            const dummy = new THREE.Object3D();
            for (let i = 0; i < rawSelfCollisionData.length; i++) {
                dummy.position.set(rawSelfCollisionData[i][0], rawSelfCollisionData[i][1], rawSelfCollisionData[i][2]);
                dummy.updateMatrix();
                selfCollisionMesh.setMatrixAt(i, dummy.matrix);
            }
            scene.add(selfCollisionMesh);
        }
        
        // 2. Render obstacle-collisions (C-Obs)
        const activeObstacles = (rawObstacleData.length > 0) ? rawObstacleData : rawVoxelData;
        const renderObstacleLayer = showObstacleCollision && (rawObstacleData.length > 0 || (rawObstacleData.length === 0 && !rawSelfCollisionData.length));
        
        if (renderObstacleLayer && activeObstacles.length > 0) {
            const obsMat = new THREE.MeshPhongMaterial({ 
                color: 0xff3333, 
                transparent: true, 
                opacity: 0.5 
            });
            obstacleCollisionMesh = new THREE.InstancedMesh(geo, obsMat, activeObstacles.length);
            const dummy = new THREE.Object3D();
            for (let i = 0; i < activeObstacles.length; i++) {
                dummy.position.set(activeObstacles[i][0], activeObstacles[i][1], activeObstacles[i][2]);
                dummy.updateMatrix();
                obstacleCollisionMesh.setMatrixAt(i, dummy.matrix);
            }
            scene.add(obstacleCollisionMesh);
        }
    } else {
        // C-Free space rendering
        const freePoints = computeComplement(rawVoxelData);
        if (freePoints.length > 0) {
            const geo = new THREE.BoxGeometry(0.12, 0.12, 0.12);
            const freeMat = new THREE.MeshPhongMaterial({ 
                color: 0x00d4ff, 
                transparent: true, 
                opacity: 0.5 
            });
            freeCollisionMesh = new THREE.InstancedMesh(geo, freeMat, freePoints.length);
            const dummy = new THREE.Object3D();
            for (let i = 0; i < freePoints.length; i++) {
                dummy.position.set(freePoints[i][0], freePoints[i][1], freePoints[i][2]);
                dummy.updateMatrix();
                freeCollisionMesh.setMatrixAt(i, dummy.matrix);
            }
            scene.add(freeCollisionMesh);
        }
    }
}

window.toggleCSpaceMode = function() {
    cspaceMode = (cspaceMode === 'obs') ? 'free' : 'obs';
    const btn = document.getElementById('btn-toggle-cspace');
    if (cspaceMode === 'obs') {
        btn.innerText = "CURRENT: C-OBS (OBSTACLES)";
        btn.style.color = "var(--accent-red)";
        btn.style.borderColor = "var(--accent-red)";
        btn.style.backgroundColor = "rgba(255, 51, 51, 0.05)";
    } else {
        btn.innerText = "CURRENT: C-FREE (WORKSPACE)";
        btn.style.color = "var(--accent-blue)";
        btn.style.borderColor = "var(--accent-blue)";
        btn.style.backgroundColor = "rgba(0, 212, 255, 0.05)";
    }
    renderVoxels();
};

const voxelSub = new ROSLIB.Topic({ ros : ros, name : '/cspace_voxels', messageType : 'std_msgs/String' });
voxelSub.subscribe((msg) => {
    try {
        const parsed = JSON.parse(msg.data);
        if (parsed && !Array.isArray(parsed) && parsed.forbidden_voxels) {
            rawVoxelData = parsed.forbidden_voxels;
            rawObstacleData = parsed.obstacle_voxels || [];
            rawSelfCollisionData = parsed.self_collision_voxels || [];
        } else {
            rawVoxelData = parsed || [];
            rawObstacleData = [];
            rawSelfCollisionData = [];
        }
        console.log(`Loaded ${rawVoxelData.length} voxels (Self: ${rawSelfCollisionData.length}, Obstacles: ${rawObstacleData.length}) from ROS topic.`);
    } catch(e) {
        console.error("Error parsing voxel data:", e);
        rawVoxelData = [];
        rawObstacleData = [];
        rawSelfCollisionData = [];
    }
    renderVoxels();
});

// --- PLANNER CONTROL & WAYPOINT SEQUENCE ---
let waypointsSequence = [];

window.updatePlannerParams = function() {
    const planner = document.getElementById('select-planner').value;
    const heuristicGroup = document.getElementById('heuristic-group');
    if (planner === 'rrt') {
        heuristicGroup.style.display = 'none';
    } else {
        heuristicGroup.style.display = 'block';
    }
};

window.planToCurrentGoal = function() {
    // Inputs are now in degrees — convert to radians for the planner
    const deg2rad = Math.PI / 180.0;
    const th1 = (parseFloat(document.getElementById('goal-th1').value) || 0.0) * deg2rad;
    const th2 = (parseFloat(document.getElementById('goal-th2').value) || 0.0) * deg2rad;
    const th3 = (parseFloat(document.getElementById('goal-th3').value) || 0.0) * deg2rad;

    const planner = document.getElementById('select-planner').value;
    const heuristic = document.getElementById('select-heuristic').value;

    const payload = {
        action: "plan",
        planner_type: planner,
        heuristic_type: heuristic,
        goal: [th1, th2, th3]
    };

    const msg = new ROSLIB.Message({
        data: JSON.stringify(payload)
    });
    webCmdPub.publish(msg);
    console.log("Published plan command:", payload);
};

window.addCurrentAsWaypoint = function() {
    if (!lastQ) {
        alert("No joint state received yet. Is the robot visualization running?");
        return;
    }
    const wp = [lastQ[0], lastQ[1], lastQ[2]];
    waypointsSequence.push(wp);
    renderWaypointsList();
};

window.clearWaypointSequence = function() {
    waypointsSequence = [];
    renderWaypointsList();
};

window.removeWaypoint = function(idx) {
    waypointsSequence.splice(idx, 1);
    renderWaypointsList();
};

function renderWaypointsList() {
    const listDiv = document.getElementById('waypoint-list');
    const execBtn = document.getElementById('btn-exec-seq');

    if (waypointsSequence.length === 0) {
        listDiv.innerHTML = '<div style="color: var(--text-muted); text-align: center; padding: 10px;">No waypoints added</div>';
        execBtn.style.display = 'none';
        return;
    }

    listDiv.innerHTML = waypointsSequence.map((wp, idx) => `
        <div style="display: flex; justify-content: space-between; align-items: center; padding: 6px 4px; border-bottom: 1px solid var(--glass-border);">
            <span>WP #${idx + 1}: [${wp[0].toFixed(2)}, ${wp[1].toFixed(2)}, ${wp[2].toFixed(2)}]</span>
            <span onclick="removeWaypoint(${idx})" style="color: var(--accent-red); cursor: pointer; font-weight: bold; padding: 0 6px; font-size: 1.1em;">&times;</span>
        </div>
    `).join('');

    execBtn.style.display = 'block';
}

window.executeWaypointSequence = function() {
    if (waypointsSequence.length < 2) {
        alert("At least 2 waypoints are required to execute a sequential path.");
        return;
    }

    const planner = document.getElementById('select-planner').value;
    const heuristic = document.getElementById('select-heuristic').value;

    const payload = {
        action: "plan_sequential",
        planner_type: planner,
        heuristic_type: heuristic,
        waypoints: waypointsSequence
    };

    const msg = new ROSLIB.Message({
        data: JSON.stringify(payload)
    });
    webCmdPub.publish(msg);
    console.log("Published sequential plan command:", payload);
};

window.changeCSpaceEnv = function() {
    const obstacle = document.getElementById('select-obstacle').value;
    const resolution = document.getElementById('select-resolution').value;
    console.log(`Requesting C-space environment change to: ${obstacle} at ${resolution}deg`);
    
    const cmd = {
        action: "change_cspace",
        obstacle_type: obstacle,
        step_size_deg: parseFloat(resolution)
    };
    
    const msg = new ROSLIB.Message({
        data: JSON.stringify(cmd)
    });
    webCmdPub.publish(msg);
};

// --- JOINT SLIDER CONTROL ---
function publishSliderJointState() {
    const deg2rad = Math.PI / 180.0;
    const q1 = parseFloat(document.getElementById('slider-q1').value) * deg2rad;
    const q2 = parseFloat(document.getElementById('slider-q2').value) * deg2rad;
    const q3 = parseFloat(document.getElementById('slider-q3').value) * deg2rad;

    const jointMsg = new ROSLIB.Message({
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
        name: ['base_yaw_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint'],
        position: [q1, q2, q3],
        velocity: [],
        effort: []
    });
    guiJointPub.publish(jointMsg);
}

window.onSliderChange = function() {
    const v1 = document.getElementById('slider-q1').value;
    const v2 = document.getElementById('slider-q2').value;
    const v3 = document.getElementById('slider-q3').value;

    document.getElementById('val-q1').textContent = v1 + '°';
    document.getElementById('val-q2').textContent = v2 + '°';
    document.getElementById('val-q3').textContent = v3 + '°';

    userDraggingSlider = true;
    publishSliderJointState();

    // Auto-stop dragging flag after 500ms of inactivity
    clearTimeout(window._sliderDragTimeout);
    window._sliderDragTimeout = setTimeout(() => { userDraggingSlider = false; }, 500);
};

window.resetSliders = function() {
    document.getElementById('slider-q1').value = 0;
    document.getElementById('slider-q2').value = 0;
    document.getElementById('slider-q3').value = 0;
    document.getElementById('val-q1').textContent = '0°';
    document.getElementById('val-q2').textContent = '0°';
    document.getElementById('val-q3').textContent = '0°';
    publishSliderJointState();
};

// Continuous publishing at 10Hz to keep the robot alive in RViz
ros.on('connection', () => {
    // Load offsets and directions dynamically from ROS parameters
    loadParameters();
    // We defer starting the publisher interval until the first /joint_states is received
    // to avoid publishing default [0, 0, 0] values and resetting the robot.
});

ros.on('close', () => {
    if (sliderPublishInterval) {
        clearInterval(sliderPublishInterval);
        sliderPublishInterval = null;
    }
});

// --- ANIMATION LOOP ---
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    updateLabels();
    renderer.render(scene, camera);
}
animate();

window.addEventListener('resize', () => {
    camera.aspect = container.clientWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, window.innerHeight);
});
