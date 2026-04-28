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

const jointSub = new ROSLIB.Topic({
    ros : ros,
    name : '/joint_states',
    messageType : 'sensor_msgs/JointState'
});

let lastQ = null;
let lastTableUpdateTime = 0;

jointSub.subscribe((msg) => {
    const names = msg.name;
    const pos = msg.position;
    
    let q = [0, 0, 0];
    let found = 0;
    for(let i=0; i<names.length; i++) {
        if(names[i] === 'revolute_1_0') { q[0] = pos[i]; found++; }
        if(names[i] === 'revolute_9_0') { q[1] = pos[i]; found++; }
        if(names[i] === 'revolute_10_0') { q[2] = pos[i]; found++; }
    }

    if (found >= 2) {
        robotPoint.position.set(q[0], q[1], q[2]);

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

const voxelSub = new ROSLIB.Topic({ ros : ros, name : '/cspace_voxels', messageType : 'std_msgs/String' });
voxelSub.subscribe((msg) => {
    const data = JSON.parse(msg.data);
    if (voxelMesh) scene.remove(voxelMesh);
    const geo = new THREE.BoxGeometry(0.12, 0.12, 0.12);
    voxelMesh = new THREE.InstancedMesh(geo, voxelMat, data.length);
    const dummy = new THREE.Object3D();
    for (let i = 0; i < data.length; i++) {
        dummy.position.set(data[i][0], data[i][1], data[i][2]);
        dummy.updateMatrix();
        voxelMesh.setMatrixAt(i, dummy.matrix);
    }
    scene.add(voxelMesh);
});

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
