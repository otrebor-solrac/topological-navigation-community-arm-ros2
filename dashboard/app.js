// --- CONFIGURACIÓN DE ESCENA ---
const container = document.getElementById('canvas-container');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x010103);

const camera = new THREE.PerspectiveCamera(75, container.clientWidth / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(container.clientWidth, window.innerHeight);
renderer.setPixelRatio(window.devicePixelRatio);
container.appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

// --- EL CUBO FUNDAMENTAL (T³) ---
const PI = Math.PI;
const size = PI * 2; 

const cubeGeom = new THREE.BoxGeometry(size, size, size);
const cubeEdges = new THREE.EdgesGeometry(cubeGeom);
const cubeLine = new THREE.LineSegments(cubeEdges, new THREE.LineBasicMaterial({ color: 0x00ff9d, transparent: true, opacity: 0.15 }));
scene.add(cubeLine);

const axesHelper = new THREE.AxesHelper(PI + 0.3);
scene.add(axesHelper);

// --- EL PUNTO DEL ROBOT ---
const robotGeom = new THREE.SphereGeometry(0.18, 32, 32);
const robotMat = new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0xffffff, emissiveIntensity: 1.0 });
const robotPoint = new THREE.Mesh(robotGeom, robotMat);
scene.add(robotPoint);

// --- LA ESTELA AMARILLA ---
let pathSegments = [];
const trailMaterial = new THREE.LineBasicMaterial({ color: 0xffff00, linewidth: 2.5 });

function createNewPathSegment() {
    const geo = new THREE.BufferGeometry();
    const positions = new Float32Array(5000 * 3);
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const line = new THREE.Line(geo, trailMaterial);
    scene.add(line);
    pathSegments.push(line);
    return { line, array: positions, count: 0 };
}

let activePath = createNewPathSegment();

window.clearTrajectory = function() {
    console.log("Limpiando trayectoria...");
    pathSegments.forEach(s => scene.remove(s));
    pathSegments = [];
    activePath = createNewPathSegment();
    // Limpiar tabla
    tracePoints = [];
    document.getElementById('trace-table').innerHTML = '';
};

// --- TRAZABILIDAD (TABLA) ---
const tableBody = document.getElementById('trace-table');
let tracePoints = [];
const MAX_TRACE_LOG = 30;

function updateTraceTable(q) {
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

// --- VOXELS ---
let voxelMesh = null;
let rawVoxelData = [];
let cspaceMode = 'obs'; // 'obs' or 'free'
const voxelMat = new THREE.MeshPhongMaterial({ color: 0xff3333, transparent: true, opacity: 0.4 });


scene.add(new THREE.AmbientLight(0xffffff, 0.4));
const light = new THREE.PointLight(0xffffff, 1, 100);
light.position.set(5, 5, 5);
scene.add(light);

camera.position.set(6, 6, 6);

// --- ETIQUETAS ---
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
        const v = vectors[key].clone().project(camera);
        labels[key].style.left = (v.x + 1) / 2 * container.clientWidth + 'px';
        labels[key].style.top = -(v.y - 1) / 2 * window.innerHeight + 'px';
    }
}

// --- CONEXIÓN ROS2 ---
const ros = new ROSLIB.Ros({ url : 'ws://localhost:9090' });
const connTag = document.getElementById('connection');

ros.on('connection', () => {
    connTag.innerText = "ROS: CONNECTED";
    connTag.style.color = "#00ff9d";
    connTag.style.borderColor = "#00ff9d";
});

ros.on('error', (error) => {
    connTag.innerText = "ROS: ERROR";
    connTag.style.color = "#ff4d4d";
    connTag.style.borderColor = "#ff4d4d";
});

ros.on('close', () => {
    connTag.innerText = "ROS: DISCONNECTED";
    connTag.style.color = "#ff4d4d";
    connTag.style.borderColor = "#ff4d4d";
});

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

let lastQ = null;
let lastTableUpdateTime = 0;
let firstPositionReceived = false;

jointSub.subscribe((msg) => {
    const names = msg.name;
    const pos = msg.position;
    
    let q = [0, 0, 0];
    let found = 0;
    for(let i=0; i<names.length; i++) {
        if(names[i] === 'base_yaw_joint') { q[0] = pos[i]; found++; }
        if(names[i] === 'shoulder_pitch_joint') { q[1] = pos[i]; found++; }
        if(names[i] === 'elbow_pitch_joint') { q[2] = pos[i]; found++; }
    }

    if (found >= 2) {
        robotPoint.position.set(q[0], q[1], q[2]);

        if (!firstPositionReceived) {
            controls.target.set(q[0], q[1], q[2]);
            camera.position.set(q[0] + 5, q[1] + 5, q[2] + 5);
            controls.update();
            firstPositionReceived = true;
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

function renderVoxels() {
    if (voxelMesh) scene.remove(voxelMesh);
    
    let pointsToRender = [];
    if (cspaceMode === 'obs') {
        pointsToRender = rawVoxelData;
        voxelMat.color.setHex(0xff3333); // Red
        voxelMat.opacity = 0.4;
    } else {
        pointsToRender = computeComplement(rawVoxelData);
        voxelMat.color.setHex(0x00d4ff); // Cyan
        voxelMat.opacity = 0.5;
    }
    
    if (pointsToRender.length === 0) return;
    
    const geo = new THREE.BoxGeometry(0.12, 0.12, 0.12);
    voxelMesh = new THREE.InstancedMesh(geo, voxelMat, pointsToRender.length);
    const dummy = new THREE.Object3D();
    for (let i = 0; i < pointsToRender.length; i++) {
        dummy.position.set(pointsToRender[i][0], pointsToRender[i][1], pointsToRender[i][2]);
        dummy.updateMatrix();
        voxelMesh.setMatrixAt(i, dummy.matrix);
    }
    scene.add(voxelMesh);
}

window.toggleCSpaceMode = function() {
    cspaceMode = (cspaceMode === 'obs') ? 'free' : 'obs';
    const btn = document.getElementById('btn-toggle-cspace');
    if (cspaceMode === 'free') {
        btn.innerText = "SHOW: C-OBS (OBSTACLES)";
        btn.style.color = "#ff3333";
        btn.style.borderColor = "#ff3333";
        btn.style.boxShadow = "0 0 15px rgba(255, 51, 51, 0.1)";
    } else {
        btn.innerText = "SHOW: C-FREE (WORKSPACE)";
        btn.style.color = "#00d4ff";
        btn.style.borderColor = "#00d4ff";
        btn.style.boxShadow = "0 0 15px rgba(0, 212, 255, 0.1)";
    }
    renderVoxels();
};

const voxelSub = new ROSLIB.Topic({ ros : ros, name : '/cspace_voxels', messageType : 'std_msgs/String' });
voxelSub.subscribe((msg) => {
    rawVoxelData = JSON.parse(msg.data);
    renderVoxels();
});

// --- PLANNER CONTROL & SEQUENCER INTEGRATION ---
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
    const th1 = parseFloat(document.getElementById('goal-th1').value) || 0.0;
    const th2 = parseFloat(document.getElementById('goal-th2').value) || 0.0;
    const th3 = parseFloat(document.getElementById('goal-th3').value) || 0.0;

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
        listDiv.innerHTML = '<div style="color: #666; text-align: center; padding: 10px;">No waypoints added</div>';
        execBtn.style.display = 'none';
        return;
    }

    listDiv.innerHTML = waypointsSequence.map((wp, idx) => `
        <div style="display: flex; justify-content: space-between; align-items: center; padding: 6px 4px; border-bottom: 1px solid #222;">
            <span>WP #${idx + 1}: [${wp[0].toFixed(2)}, ${wp[1].toFixed(2)}, ${wp[2].toFixed(2)}]</span>
            <span onclick="removeWaypoint(${idx})" style="color: #ff5555; cursor: pointer; font-weight: bold; padding: 0 6px; font-size: 1.1em;">&times;</span>
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
