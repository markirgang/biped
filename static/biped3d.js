/**
 * Biped 3D Renderer (Three.js WebGL)
 * Procedural 17-DOF humanoid model:
 *   2 head joints, 1 waist, 3 per arm, 4 per leg.
 * Includes eyes, jaw (L298 channel 1), head/neck bob (channel 2),
 * ultrasonic sensor pucks and a live ground grid.
 */

class Biped3DRenderer {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        if (!this.container) throw new Error('Biped3D container not found: ' + containerId);

        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(
            42,
            Math.max(1, this.container.clientWidth) / Math.max(1, this.container.clientHeight),
            1,
            6000
        );
        this.camera.position.set(420, 360, 620);

        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.container.appendChild(this.renderer.domElement);

        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.06;
        this.controls.maxPolarAngle = Math.PI / 2 + 0.08;
        this.controls.target.set(0, 170, 0);

        this.mouthEnergized = false;
        this.neckPulsating = false;
        this.baseHeight = 330;
        this.currentJoints = null;
        this.currentBody = { height: 0, roll: 0, pitch: 0, yaw: 0 };
        this.mouthValue = 0;

        this.setupEnvironment();
        this.buildRobot();

        window.addEventListener('resize', () => this.onWindowResize());
        this.clock = new THREE.Clock();
        this.animate = this.animate.bind(this);
        requestAnimationFrame(this.animate);
    }

    setupEnvironment() {
        this.scene.background = new THREE.Color(0x0b0e14);
        this.scene.fog = new THREE.FogExp2(0x0b0e14, 0.00065);

        this.scene.add(new THREE.AmbientLight(0xffffff, 0.65));

        const dirLight = new THREE.DirectionalLight(0xffffff, 1.25);
        dirLight.position.set(360, 620, 320);
        dirLight.castShadow = true;
        dirLight.shadow.mapSize.width = 2048;
        dirLight.shadow.mapSize.height = 2048;
        dirLight.shadow.camera.near = 100;
        dirLight.shadow.camera.far = 2200;
        this.scene.add(dirLight);

        const rimLight = new THREE.PointLight(0x00f0ff, 1.6, 900);
        rimLight.position.set(-260, 320, -300);
        this.scene.add(rimLight);

        const warmLight = new THREE.PointLight(0xec4899, 1.1, 800);
        warmLight.position.set(300, 180, 260);
        this.scene.add(warmLight);

        const grid = new THREE.GridHelper(2200, 44, 0x00f0ff, 0x1e293b);
        grid.position.y = 0;
        this.scene.add(grid);

        const ground = new THREE.Mesh(
            new THREE.PlaneGeometry(2600, 2600),
            new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.85, metalness: 0.1 })
        );
        ground.rotation.x = -Math.PI / 2;
        ground.receiveShadow = true;
        this.scene.add(ground);
    }

    box(w, h, d, color, metalness, roughness) {
        const mesh = new THREE.Mesh(
            new THREE.BoxGeometry(w, h, d),
            new THREE.MeshStandardMaterial({
                color: color,
                metalness: metalness === undefined ? 0.65 : metalness,
                roughness: roughness === undefined ? 0.35 : roughness
            })
        );
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        return mesh;
    }

    cylinder(radiusTop, radiusBottom, height, color, segments) {
        const mesh = new THREE.Mesh(
            new THREE.CylinderGeometry(radiusTop, radiusBottom, height, segments || 16),
            new THREE.MeshStandardMaterial({ color: color, metalness: 0.7, roughness: 0.3 })
        );
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        return mesh;
    }

    buildRobot() {
        this.robotGroup = new THREE.Group();
        this.scene.add(this.robotGroup);

        this.bodyGroup = new THREE.Group();
        this.bodyGroup.position.y = this.baseHeight;
        this.robotGroup.add(this.bodyGroup);

        // Pelvis / chassis
        const pelvis = this.box(170, 70, 100, 0x1e293b, 0.8, 0.25);
        this.bodyGroup.add(pelvis);

        const belt = this.box(180, 16, 106, 0x00f0ff, 0.5, 0.3);
        belt.position.y = 38;
        this.bodyGroup.add(belt);

        // Torso (waist yaw)
        this.torsoGroup = new THREE.Group();
        this.torsoGroup.position.y = 42;
        this.bodyGroup.add(this.torsoGroup);

        const torso = this.box(200, 170, 112, 0x334155, 0.75, 0.3);
        torso.position.y = 88;
        this.torsoGroup.add(torso);

        const chestPlate = this.box(150, 70, 18, 0x0284c7, 0.85, 0.2);
        chestPlate.position.set(0, 110, 64);
        this.torsoGroup.add(chestPlate);

        const reactor = new THREE.Mesh(
            new THREE.CircleGeometry(17, 24),
            new THREE.MeshBasicMaterial({ color: 0x00f0ff, side: THREE.DoubleSide })
        );
        reactor.position.set(0, 112, 74);
        this.torsoGroup.add(reactor);

        this.buildHead();
        this.buildArm('l');
        this.buildArm('r');
        this.buildLeg('l');
        this.buildLeg('r');
        this.buildSensors();
    }

    buildHead() {
        this.headPanGroup = new THREE.Group();
        this.headPanGroup.position.set(0, 178, 0);
        this.torsoGroup.add(this.headPanGroup);

        const neck = this.cylinder(18, 22, 32, 0x475569, 16);
        neck.position.y = 10;
        this.headPanGroup.add(neck);

        this.headTiltGroup = new THREE.Group();
        this.headTiltGroup.position.y = 34;
        this.headPanGroup.add(this.headTiltGroup);

        const head = this.box(96, 82, 90, 0x1e293b, 0.8, 0.25);
        head.position.y = 34;
        this.headTiltGroup.add(head);

        const visor = this.box(82, 30, 14, 0x0f172a, 0.9, 0.15);
        visor.position.set(0, 40, 48);
        this.headTiltGroup.add(visor);

        const eyeMat = new THREE.MeshBasicMaterial({ color: 0x00f0ff });
        [-24, 24].forEach(x => {
            const eye = new THREE.Mesh(new THREE.SphereGeometry(7, 16, 16), eyeMat);
            eye.position.set(x, 42, 56);
            this.headTiltGroup.add(eye);
        });

        const antenna = this.cylinder(2.5, 2.5, 34, 0x00f0ff, 8);
        antenna.position.set(0, 94, 0);
        this.headTiltGroup.add(antenna);

        const antennaTip = new THREE.Mesh(
            new THREE.SphereGeometry(6, 12, 12),
            new THREE.MeshBasicMaterial({ color: 0xec4899 })
        );
        antennaTip.position.set(0, 112, 0);
        this.headTiltGroup.add(antennaTip);

        // L298 channel 1 driven jaw
        this.jawGroup = new THREE.Group();
        this.jawGroup.position.set(0, 6, 18);
        this.headTiltGroup.add(this.jawGroup);
        const jaw = this.box(58, 12, 54, 0x0f172a, 0.9, 0.2);
        jaw.position.set(0, -6, 22);
        this.jawGroup.add(jaw);

        const mouthGlow = new THREE.Mesh(
            new THREE.BoxGeometry(44, 5, 8),
            new THREE.MeshBasicMaterial({ color: 0xec4899 })
        );
        mouthGlow.position.set(0, 2, 48);
        this.headTiltGroup.add(mouthGlow);
    }

    buildArm(side) {
        const sign = side === 'l' ? -1 : 1;
        const shoulderPitch = new THREE.Group();
        shoulderPitch.position.set(sign * 116, 142, 0);
        this.torsoGroup.add(shoulderPitch);

        const shoulderRoll = new THREE.Group();
        shoulderPitch.add(shoulderRoll);

        const shoulderBall = new THREE.Mesh(
            new THREE.SphereGeometry(24, 16, 16),
            new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.8, roughness: 0.25 })
        );
        shoulderBall.castShadow = true;
        shoulderRoll.add(shoulderBall);

        const upperArm = this.box(38, 132, 42, 0x0284c7, 0.7, 0.3);
        upperArm.position.y = -72;
        shoulderRoll.add(upperArm);

        const elbow = new THREE.Group();
        elbow.position.y = -140;
        shoulderRoll.add(elbow);

        const elbowBall = new THREE.Mesh(
            new THREE.SphereGeometry(18, 14, 14),
            new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.8, roughness: 0.25 })
        );
        elbowBall.castShadow = true;
        elbow.add(elbowBall);

        const forearm = this.box(30, 120, 34, 0x1e293b, 0.8, 0.25);
        forearm.position.y = -64;
        elbow.add(forearm);

        const hand = this.box(34, 40, 26, 0x00f0ff, 0.5, 0.3);
        hand.position.y = -136;
        elbow.add(hand);

        if (side === 'l') {
            this.lShoulderPitch = shoulderPitch;
            this.lShoulderRoll = shoulderRoll;
            this.lElbow = elbow;
        } else {
            this.rShoulderPitch = shoulderPitch;
            this.rShoulderRoll = shoulderRoll;
            this.rElbow = elbow;
        }
    }

    buildLeg(side) {
        const sign = side === 'l' ? -1 : 1;
        const hipYaw = new THREE.Group();
        hipYaw.position.set(sign * 62, -28, 0);
        this.bodyGroup.add(hipYaw);

        const hipPitch = new THREE.Group();
        hipYaw.add(hipPitch);

        const hipBall = new THREE.Mesh(
            new THREE.SphereGeometry(28, 16, 16),
            new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.8, roughness: 0.25 })
        );
        hipBall.castShadow = true;
        hipPitch.add(hipBall);

        const thigh = this.box(48, 150, 54, 0x0284c7, 0.7, 0.3);
        thigh.position.y = -82;
        hipPitch.add(thigh);

        const knee = new THREE.Group();
        knee.position.y = -160;
        hipPitch.add(knee);

        const kneeBall = new THREE.Mesh(
            new THREE.SphereGeometry(22, 14, 14),
            new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.8, roughness: 0.25 })
        );
        kneeBall.castShadow = true;
        knee.add(kneeBall);

        const shin = this.box(38, 150, 44, 0x1e293b, 0.8, 0.25);
        shin.position.y = -82;
        knee.add(shin);

        const ankle = new THREE.Group();
        ankle.position.y = -160;
        knee.add(ankle);

        const foot = this.box(58, 28, 104, 0x00f0ff, 0.5, 0.3);
        foot.position.set(0, -10, 22);
        ankle.add(foot);

        const heel = this.box(48, 18, 24, 0x334155, 0.7, 0.3);
        heel.position.set(0, -14, -26);
        ankle.add(heel);

        if (side === 'l') {
            this.lHipYaw = hipYaw;
            this.lHipPitch = hipPitch;
            this.lKnee = knee;
            this.lAnkle = ankle;
        } else {
            this.rHipYaw = hipYaw;
            this.rHipPitch = hipPitch;
            this.rKnee = knee;
            this.rAnkle = ankle;
        }
    }

    buildSensors() {
        this.sensorPucks = {};
        const positions = {
            front: [0, 80, 70],
            rear: [0, 80, -70],
            left: [-116, 80, 0],
            right: [116, 80, 0]
        };
        Object.keys(positions).forEach(name => {
            const puck = new THREE.Mesh(
                new THREE.CylinderGeometry(9, 9, 8, 16),
                new THREE.MeshStandardMaterial({ color: 0x10b981, metalness: 0.5, roughness: 0.35 })
            );
            puck.position.set(positions[name][0], positions[name][1], positions[name][2]);
            puck.rotation.z = Math.PI / 2;
            if (name === 'left' || name === 'right') puck.rotation.x = Math.PI / 2;
            this.torsoGroup.add(puck);
            this.sensorPucks[name] = puck;
        });
    }

    setPose(joints, body) {
        if (!joints) return;
        this.currentJoints = joints;
        this.currentBody = body || { height: 0, roll: 0, pitch: 0, yaw: 0 };
        const d = Math.PI / 180;

        this.bodyGroup.position.y = this.baseHeight + (this.currentBody.height || 0);
        this.bodyGroup.rotation.x = (this.currentBody.pitch || 0) * d;
        this.bodyGroup.rotation.z = (this.currentBody.roll || 0) * d;
        this.bodyGroup.rotation.y = (this.currentBody.yaw || 0) * d;

        this.torsoGroup.rotation.y = (joints.waist_yaw || 0) * d;
        this.headPanGroup.rotation.y = (joints.head_pan || 0) * d;
        this.headTiltGroup.rotation.x = (joints.head_tilt || 0) * d;

        this.lShoulderPitch.rotation.x = (joints.l_shoulder_pitch || 0) * d;
        this.lShoulderRoll.rotation.z = (joints.l_shoulder_roll || 0) * d;
        this.lElbow.rotation.x = (joints.l_elbow || 0) * d;

        this.rShoulderPitch.rotation.x = (joints.r_shoulder_pitch || 0) * d;
        this.rShoulderRoll.rotation.z = -(joints.r_shoulder_roll || 0) * d;
        this.rElbow.rotation.x = (joints.r_elbow || 0) * d;

        this.lHipYaw.rotation.y = (joints.l_hip_yaw || 0) * d;
        this.lHipPitch.rotation.x = (joints.l_hip_pitch || 0) * d;
        this.lKnee.rotation.x = (joints.l_knee || 0) * d;
        this.lAnkle.rotation.x = (joints.l_ankle || 0) * d;

        this.rHipYaw.rotation.y = (joints.r_hip_yaw || 0) * d;
        this.rHipPitch.rotation.x = (joints.r_hip_pitch || 0) * d;
        this.rKnee.rotation.x = (joints.r_knee || 0) * d;
        this.rAnkle.rotation.x = (joints.r_ankle || 0) * d;
    }

    setHardwareState(mouthEnergized, neckPulsating) {
        this.mouthEnergized = !!mouthEnergized;
        this.neckPulsating = !!neckPulsating;
    }

    setSensorDistances(distances) {
        if (!distances) return;
        Object.keys(this.sensorPucks || {}).forEach(name => {
            const puck = this.sensorPucks[name];
            const distance = Number(distances[name] || 0);
            const color = distance < 35 ? 0xef4444 : (distance < 70 ? 0xf59e0b : 0x10b981);
            puck.material.color.setHex(color);
        });
    }

    onWindowResize() {
        if (!this.container) return;
        const w = Math.max(1, this.container.clientWidth);
        const h = Math.max(1, this.container.clientHeight);
        this.camera.aspect = w / h;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(w, h);
    }

    resetCamera() {
        this.camera.position.set(420, 360, 620);
        this.controls.target.set(0, 170, 0);
        this.controls.update();
    }

    animate() {
        requestAnimationFrame(this.animate);
        const dt = Math.min(0.05, this.clock.getDelta());
        const t = performance.now();

        // Jaw opening driven by L298 channel 1
        const targetMouth = this.mouthEnergized ? 0.55 : 0.0;
        this.mouthValue += (targetMouth - this.mouthValue) * Math.min(1, dt * 8);
        if (this.jawGroup) this.jawGroup.rotation.x = this.mouthValue;

        // Neck pulsation driven by L298 channel 2
        if (this.neckPulsating) {
            this.headPanGroup.position.y = 178 + Math.sin(t * 0.012) * 6;
        } else {
            this.headPanGroup.position.y += (178 - this.headPanGroup.position.y) * Math.min(1, dt * 6);
        }

        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}

if (typeof window !== 'undefined') {
    window.Biped3DRenderer = Biped3DRenderer;
}
