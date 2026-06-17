import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { jointSub, voxelSub, jointOffsets, jointDirections } from '../services/ros';

const wrapToPi = (val) => {
    let a = (val + Math.PI) % (2 * Math.PI);
    if (a < 0) a += 2 * Math.PI;
    return a - Math.PI;
};

export default function ThreeVisualizer({
    showSelfCollision,
    showObstacleCollision,
    cspaceMode,
    showTrail,
    trailRef,
    onQUpdate,
    isExecuting
}) {
    const containerRef = useRef(null);
    const sceneRef = useRef(null);
    const cameraRef = useRef(null);
    const rendererRef = useRef(null);
    const controlsRef = useRef(null);

    // Three meshes
    const robotPointRef = useRef(null);
    const pathSegmentsRef = useRef([]);
    const activePathRef = useRef(null);
    const selfCollisionMeshRef = useRef(null);
    const obstacleCollisionMeshRef = useRef(null);
    const freeCollisionMeshRef = useRef(null);

    // Voxel data refs
    const rawVoxelDataRef = useRef([]);
    const rawObstacleDataRef = useRef([]);
    const rawSelfCollisionDataRef = useRef([]);
    const stepRadRef = useRef(0.12);

    const lastQRef = useRef(null);

    // 3D labels
    const labelTh1Ref = useRef(null);
    const labelTh2Ref = useRef(null);
    const labelTh3Ref = useRef(null);

    // Sync visibility props to refs to prevent stale closure bugs in ROS callbacks
    const showSelfCollisionRef = useRef(showSelfCollision);
    const showObstacleCollisionRef = useRef(showObstacleCollision);
    const cspaceModeRef = useRef(cspaceMode);
    const isExecutingRef = useRef(isExecuting);

    useEffect(() => {
        showSelfCollisionRef.current = showSelfCollision;
        showObstacleCollisionRef.current = showObstacleCollision;
        cspaceModeRef.current = cspaceMode;
        isExecutingRef.current = isExecuting;
    }, [showSelfCollision, showObstacleCollision, cspaceMode, isExecuting]);

    const PI = Math.PI;

    // Helper to create a new path segment for the trail
    const createNewPathSegment = (scene) => {
        const geo = new THREE.BufferGeometry();
        const positions = new Float32Array(5000 * 3);
        geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        const trailMaterial = new THREE.LineBasicMaterial({ color: 0xffff00, linewidth: 2.5 });
        const line = new THREE.Line(geo, trailMaterial);
        line.visible = showTrail;
        scene.add(line);
        pathSegmentsRef.current.push(line);
        activePathRef.current = { line, array: positions, count: 0 };
    };

    // Helper to clear trail (exposed via trailRef)
    useEffect(() => {
        if (trailRef) {
            trailRef.current = {
                clear: () => {
                    if (sceneRef.current) {
                        pathSegmentsRef.current.forEach(s => sceneRef.current.remove(s));
                        pathSegmentsRef.current = [];
                        createNewPathSegment(sceneRef.current);
                    }
                }
            };
        }
    }, [trailRef]);

    // Update trail visibility when showTrail prop changes
    useEffect(() => {
        pathSegmentsRef.current.forEach(s => {
            s.visible = showTrail;
        });
        if (activePathRef.current && activePathRef.current.line) {
            activePathRef.current.line.visible = showTrail;
        }
    }, [showTrail]);

    // Compute complement (free space)
    const computeComplement = (data) => {
        if (!data) return [];
        
        const step = stepRadRef.current || 0.12;
        const stepsPerCircle = Math.round((2 * Math.PI) / step);
        
        // Generate the exact set of possible coordinates on each axis
        const axisValues = [];
        for (let i = 0; i < stepsPerCircle; i++) {
            axisValues.push(wrapToPi(i * step));
        }
        
        const forbiddenSet = new Set(data.map(p => `${p[0].toFixed(3)},${p[1].toFixed(3)},${p[2].toFixed(3)}`));
        const freePoints = [];
        for (let x of axisValues) {
            for (let y of axisValues) {
                for (let z of axisValues) {
                    const key = `${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)}`;
                    if (!forbiddenSet.has(key)) {
                        freePoints.push([x, y, z]);
                    }
                }
            }
        }
        return freePoints;
    };

    // Voxel renderer function
    const renderVoxels = () => {
        const scene = sceneRef.current;
        if (!scene) return;

        // Clear existing meshes
        if (selfCollisionMeshRef.current) { scene.remove(selfCollisionMeshRef.current); selfCollisionMeshRef.current = null; }
        if (obstacleCollisionMeshRef.current) { scene.remove(obstacleCollisionMeshRef.current); obstacleCollisionMeshRef.current = null; }
        if (freeCollisionMeshRef.current) { scene.remove(freeCollisionMeshRef.current); freeCollisionMeshRef.current = null; }

        const stepSize = stepRadRef.current || 0.12;
        const size = stepSize * 0.95;
        const geo = new THREE.BoxGeometry(size, size, size);

        if (cspaceModeRef.current === 'obs') {
            // Render self-collisions
            if (showSelfCollisionRef.current && rawSelfCollisionDataRef.current.length > 0) {
                const selfMat = new THREE.MeshPhongMaterial({ color: 0x5d4778, transparent: true, opacity: 0.22 });
                const mesh = new THREE.InstancedMesh(geo, selfMat, rawSelfCollisionDataRef.current.length);
                const dummy = new THREE.Object3D();
                for (let i = 0; i < rawSelfCollisionDataRef.current.length; i++) {
                    dummy.position.set(rawSelfCollisionDataRef.current[i][0], rawSelfCollisionDataRef.current[i][1], rawSelfCollisionDataRef.current[i][2]);
                    dummy.updateMatrix();
                    mesh.setMatrixAt(i, dummy.matrix);
                }
                scene.add(mesh);
                selfCollisionMeshRef.current = mesh;
            }

            // Render obstacle-collisions
            const activeObstacles = (rawObstacleDataRef.current.length > 0) ? rawObstacleDataRef.current : rawVoxelDataRef.current;
            const renderObstacleLayer = showObstacleCollisionRef.current && (rawObstacleDataRef.current.length > 0 || (rawObstacleDataRef.current.length === 0 && !rawSelfCollisionDataRef.current.length));

            if (renderObstacleLayer && activeObstacles.length > 0) {
                const obsMat = new THREE.MeshPhongMaterial({ color: 0xff3333, transparent: true, opacity: 0.5 });
                const mesh = new THREE.InstancedMesh(geo, obsMat, activeObstacles.length);
                const dummy = new THREE.Object3D();
                for (let i = 0; i < activeObstacles.length; i++) {
                    dummy.position.set(activeObstacles[i][0], activeObstacles[i][1], activeObstacles[i][2]);
                    dummy.updateMatrix();
                    mesh.setMatrixAt(i, dummy.matrix);
                }
                scene.add(mesh);
                obstacleCollisionMeshRef.current = mesh;
            }
        } else {
            // Render free space
            const freePoints = computeComplement(rawVoxelDataRef.current);
            if (freePoints.length > 0) {
                const freeMat = new THREE.MeshPhongMaterial({ color: 0x00d4ff, transparent: true, opacity: 0.5 });
                const mesh = new THREE.InstancedMesh(geo, freeMat, freePoints.length);
                const dummy = new THREE.Object3D();
                for (let i = 0; i < freePoints.length; i++) {
                    dummy.position.set(freePoints[i][0], freePoints[i][1], freePoints[i][2]);
                    dummy.updateMatrix();
                    mesh.setMatrixAt(i, dummy.matrix);
                }
                scene.add(mesh);
                freeCollisionMeshRef.current = mesh;
            }
        }
    };

    // Re-render voxels when toggles or mode change
    useEffect(() => {
        renderVoxels();
    }, [showSelfCollision, showObstacleCollision, cspaceMode]);

    // Initial Three.js setup
    useEffect(() => {
        const container = containerRef.current;
        if (!container) return;

        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x030307);
        sceneRef.current = scene;

        const camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
        camera.position.set(6, 6, 6);
        cameraRef.current = camera;

        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
        container.appendChild(renderer.domElement);
        rendererRef.current = renderer;

        const controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controlsRef.current = controls;

        // T3 Boundary Cube
        const size = PI * 2;
        const cubeGeom = new THREE.BoxGeometry(size, size, size);
        const cubeEdges = new THREE.EdgesGeometry(cubeGeom);
        const cubeLine = new THREE.LineSegments(cubeEdges, new THREE.LineBasicMaterial({ color: 0x00ff9d, transparent: true, opacity: 0.15 }));
        scene.add(cubeLine);

        // Axes Helper
        const axesHelper = new THREE.AxesHelper(PI + 0.3);
        scene.add(axesHelper);

        // Robot Position indicator point
        const robotGeom = new THREE.SphereGeometry(0.18, 32, 32);
        const robotMat = new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0xffffff, emissiveIntensity: 0.8 });
        const robotPoint = new THREE.Mesh(robotGeom, robotMat);
        scene.add(robotPoint);
        robotPointRef.current = robotPoint;

        // Trail Init
        createNewPathSegment(scene);

        // Lighting
        scene.add(new THREE.AmbientLight(0xffffff, 0.45));
        const light = new THREE.PointLight(0xffffff, 1.0, 100);
        light.position.set(5, 5, 5);
        scene.add(light);

        // Label update logic
        const updateLabels = () => {
            const vectors = {
                th1: new THREE.Vector3(PI + 0.5, 0, 0),
                th2: new THREE.Vector3(0, PI + 0.5, 0),
                th3: new THREE.Vector3(0, 0, PI + 0.5)
            };
            const labels = {
                th1: labelTh1Ref.current,
                th2: labelTh2Ref.current,
                th3: labelTh3Ref.current
            };
            for (let key in vectors) {
                if (labels[key]) {
                    const v = vectors[key].clone().project(camera);
                    labels[key].style.left = (v.x + 1) / 2 * container.clientWidth + 'px';
                    labels[key].style.top = -(v.y - 1) / 2 * container.clientHeight + 'px';
                }
            }
        };

        // Animation Loop
        let animationFrameId;
        const animate = () => {
            animationFrameId = requestAnimationFrame(animate);
            controls.update();
            updateLabels();
            renderer.render(scene, camera);
        };
        animate();

        // ROS Subscriptions
        const jointStateCallback = (msg) => {
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
                const offsetBaseYawRad = jointOffsets.base_yaw * Math.PI / 180.0;
                const offsetShoulderPitchRad = jointOffsets.shoulder_pitch * Math.PI / 180.0;
                const offsetElbowPitchRad = jointOffsets.elbow_pitch * Math.PI / 180.0;



                const q = [
                    wrapToPi((q_urdf[0] - offsetBaseYawRad) / jointDirections.base_yaw),
                    wrapToPi((q_urdf[1] - offsetShoulderPitchRad) / jointDirections.shoulder_pitch),
                    wrapToPi((q_urdf[2] - offsetElbowPitchRad) / jointDirections.elbow_pitch)
                ];

                if (robotPointRef.current) {
                    robotPointRef.current.position.set(q[0], q[1], q[2]);
                }

                // Callback to parent for slider sync & traceability table
                onQUpdate(q);

                // Check for toroidal wrap-around first, so wrap-around point is placed in a new path segment
                const lastQ = lastQRef.current;
                if (lastQ) {
                    const dist = Math.sqrt((q[0]-lastQ[0])**2 + (q[1]-lastQ[1])**2 + (q[2]-lastQ[2])**2);
                    if (dist > 1.2) {
                        createNewPathSegment(sceneRef.current);
                    }
                }
                lastQRef.current = [...q];

                // Add to active path segment only when executing a planned trajectory
                const activePath = activePathRef.current;
                if (activePath && isExecutingRef.current && activePath.count < 5000) {
                    const idx = activePath.count * 3;
                    activePath.array[idx] = q[0];
                    activePath.array[idx+1] = q[1];
                    activePath.array[idx+2] = q[2];
                    activePath.count++;
                    activePath.line.geometry.attributes.position.needsUpdate = true;
                    activePath.line.geometry.setDrawRange(0, activePath.count);
                }
            }
        };

        const voxelCallback = (msg) => {
            try {
                const parsed = JSON.parse(msg.data);
                if (parsed && !Array.isArray(parsed) && parsed.forbidden_voxels) {
                    rawVoxelDataRef.current = parsed.forbidden_voxels;
                    rawObstacleDataRef.current = parsed.obstacle_voxels || [];
                    rawSelfCollisionDataRef.current = parsed.self_collision_voxels || [];
                    stepRadRef.current = parsed.step_rad || 0.12;
                } else {
                    rawVoxelDataRef.current = parsed || [];
                    rawObstacleDataRef.current = [];
                    rawSelfCollisionDataRef.current = [];
                    stepRadRef.current = 0.12;
                }
            } catch(e) {
                console.error("Error parsing voxel data:", e);
                rawVoxelDataRef.current = [];
                rawObstacleDataRef.current = [];
                rawSelfCollisionDataRef.current = [];
            }
            renderVoxels();

            if (robotPointRef.current) {
                const stepSize = stepRadRef.current || 0.12;
                // Original sphere geometry radius is 0.18, so diameter is 0.36.
                // We want the new diameter to equal stepSize.
                const scaleFactor = stepSize / 0.36;
                robotPointRef.current.scale.set(scaleFactor, scaleFactor, scaleFactor);
            }
        };

        jointSub.subscribe(jointStateCallback);
        voxelSub.subscribe(voxelCallback);

        // Resize Listener
        const resizeHandler = () => {
            camera.aspect = container.clientWidth / container.clientHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(container.clientWidth, container.clientHeight);
        };
        window.addEventListener('resize', resizeHandler);

        // Cleanup on unmount
        return () => {
            cancelAnimationFrame(animationFrameId);
            window.removeEventListener('resize', resizeHandler);
            jointSub.unsubscribe(jointStateCallback);
            voxelSub.unsubscribe(voxelCallback);
            if (renderer && renderer.dispose) {
                renderer.dispose();
            }
            if (container && renderer.domElement && container.contains(renderer.domElement)) {
                container.removeChild(renderer.domElement);
            }
        };
    }, []);

    return (
        <div style={{ position: 'relative', width: '100%', height: '100%' }}>
            <div ref={containerRef} style={{ width: '100%', height: '100%', minHeight: '550px' }} />
            
            {/* 3D Label overlays */}
            <div ref={labelTh1Ref} id="label-th1" className="axis-label">θ₁</div>
            <div ref={labelTh2Ref} id="label-th2" className="axis-label">θ₂</div>
            <div ref={labelTh3Ref} id="label-th3" className="axis-label">θ₃</div>
        </div>
    );
}
