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
        this.camera.position.set(620, 500, 900);

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
        this.controls.target.set(0, 280, 0);

        this.mouthEnergized = false;
        this.neckPulsating = false;
        this.baseHeight = 330;
        this.currentJoints = null;
        this.currentBody = { height: 0, roll: 0, pitch: 0, yaw: 0 };
        this.mouthValue = 0;

        // Animatronic talking-mouth robot (no body, front edge of the stage)
        this.mouthRobotTalking = false;
        this.mouthRobotDancing = false;
        this.mouthRobotMouth = 0;
        this.mouthRobotNeckPhase = 0;
        this.mouthRobotNeckOffset = 0;
        this.mouthRobotIdleTimer = 2.5;
        this.mouthRobotIdleImpulse = 0;
        this.speechLevel = 0;

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
        this.buildMouthRobot();
    }

    buildMouthRobot() {
        // A body-less animatronic mouth that sits on the front edge of the
        // stage. L298 channel 1 opens the jaw, channel 2 lifts the neck.
        this.mouthRobotGroup = new THREE.Group();
        this.mouthRobotGroup.position.set(-310, 0, 585);
        this.mouthRobotGroup.rotation.y = 0.24;
        this.scene.add(this.mouthRobotGroup);

        const riser = this.cylinder(80, 98, 22, 0x111827, 32);
        riser.position.y = 11;
        this.mouthRobotGroup.add(riser);

        const ring = new THREE.Mesh(
            new THREE.TorusGeometry(86, 4, 10, 40),
            new THREE.MeshBasicMaterial({ color: 0x00f0ff })
        );
        ring.rotation.x = Math.PI / 2;
        ring.position.y = 23;
        this.mouthRobotGroup.add(ring);

        this.mouthNeckGroup = new THREE.Group();
        this.mouthNeckGroup.position.y = 22;
        this.mouthRobotGroup.add(this.mouthNeckGroup);

        const neckBase = this.cylinder(30, 36, 20, 0x334155, 24);
        neckBase.position.y = 10;
        this.mouthNeckGroup.add(neckBase);

        this.mouthNeckPiston = this.cylinder(17, 22, 54, 0x475569, 20);
        this.mouthNeckPiston.position.y = 46;
        this.mouthNeckGroup.add(this.mouthNeckPiston);

        this.mouthHeadGroup = new THREE.Group();
        this.mouthHeadGroup.position.y = 76;
        this.mouthNeckGroup.add(this.mouthHeadGroup);

        const skull = new THREE.Mesh(
            new THREE.SphereGeometry(62, 24, 18),
            new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.8, roughness: 0.25 })
        );
        skull.scale.set(1.05, 0.92, 1.0);
        skull.position.set(0, 22, -4);
        skull.castShadow = true;
        skull.receiveShadow = true;
        this.mouthHeadGroup.add(skull);

        const band = this.cylinder(60, 60, 10, 0x00f0ff, 28);
        band.rotation.x = Math.PI / 2;
        band.position.set(0, 22, 26);
        this.mouthHeadGroup.add(band);

        const cavity = this.box(102, 48, 62, 0x05070c, 0.2, 0.9);
        cavity.position.set(0, -10, 22);
        this.mouthHeadGroup.add(cavity);

        const upperLip = this.box(118, 16, 28, 0x00f0ff, 0.5, 0.3);
        upperLip.position.set(0, 18, 52);
        this.mouthHeadGroup.add(upperLip);

        const toothMat = new THREE.MeshStandardMaterial({ color: 0xf8fafc, metalness: 0.15, roughness: 0.5 });
        for (let i = 0; i < 7; i++) {
            const tooth = new THREE.Mesh(new THREE.BoxGeometry(11, 12, 10), toothMat);
            tooth.position.set(-42 + i * 14, 5, 47);
            tooth.castShadow = true;
            this.mouthHeadGroup.add(tooth);
        }

        const eyeMat = new THREE.MeshBasicMaterial({ color: 0x00f0ff });
        [-27, 27].forEach(x => {
            const eye = new THREE.Mesh(new THREE.SphereGeometry(8, 16, 16), eyeMat);
            eye.position.set(x, 48, 44);
            this.mouthHeadGroup.add(eye);
        });

        this.mouthRobotGlow = new THREE.Mesh(
            new THREE.PlaneGeometry(88, 40),
            new THREE.MeshBasicMaterial({ color: 0xec4899, transparent: true, opacity: 0.18, side: THREE.DoubleSide })
        );
        this.mouthRobotGlow.position.set(0, -8, 48);
        this.mouthHeadGroup.add(this.mouthRobotGlow);

        // Hinged lower jaw
        this.mouthJawGroup = new THREE.Group();
        this.mouthJawGroup.position.set(0, -24, -8);
        this.mouthHeadGroup.add(this.mouthJawGroup);

        const jaw = this.box(114, 26, 78, 0x334155, 0.8, 0.3);
        jaw.position.set(0, -10, 34);
        this.mouthJawGroup.add(jaw);

        const lowerLip = this.box(120, 14, 24, 0x00f0ff, 0.5, 0.3);
        lowerLip.position.set(0, 3, 68);
        this.mouthJawGroup.add(lowerLip);

        const tongue = this.box(64, 12, 50, 0xe11d48, 0.3, 0.55);
        tongue.position.set(0, 8, 32);
        this.mouthJawGroup.add(tongue);

        for (let i = 0; i < 7; i++) {
            const tooth = new THREE.Mesh(new THREE.BoxGeometry(11, 12, 10), toothMat);
            tooth.position.set(-42 + i * 14, 8, 58);
            tooth.castShadow = true;
            this.mouthJawGroup.add(tooth);
        }

        const chinGlow = new THREE.Mesh(
            new THREE.BoxGeometry(56, 6, 8),
            new THREE.MeshBasicMaterial({ color: 0x00f0ff })
        );
        chinGlow.position.set(0, -16, 74);
        this.mouthJawGroup.add(chinGlow);
    }

    setAnimatronicState(state) {
        if (!state) return;
        if (state.talking !== undefined) this.mouthRobotTalking = !!state.talking;
        if (state.dancing !== undefined) this.mouthRobotDancing = !!state.dancing;
        if (state.mouthEnergized !== undefined) this.mouthEnergized = !!state.mouthEnergized;
        if (state.neckEnergized !== undefined) this.neckPulsating = !!state.neckEnergized;
    }

    setSpeechLevel(level) {
        const value = Number(level);
        this.speechLevel = isNaN(value) ? 0 : Math.max(0, Math.min(1, value));
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
        this.camera.position.set(620, 500, 900);
        this.controls.target.set(0, 280, 0);
        this.controls.update();
    }

    animateMouthRobot(dt, t) {
        if (!this.mouthHeadGroup) return;
        const tSec = t / 1000;

        // Mouth: opens when the L298 channel 1 is energized, and flaps with
        // the speech envelope while the AI is talking. Dancing adds a beat.
        let target = 0.02;
        if (this.mouthEnergized && !this.mouthRobotTalking) target = Math.max(target, 0.62);
        if (this.mouthRobotTalking) {
            target = Math.max(target, 0.10 + this.speechLevel * 0.62);
        }
        if (this.mouthRobotDancing) {
            const beat = Math.abs(Math.sin(tSec * Math.PI * 2.1));
            target = Math.max(target, 0.08 + beat * 0.50);
        }
        this.mouthRobotMouth += (target - this.mouthRobotMouth) * Math.min(1, dt * 16);
        this.mouthJawGroup.rotation.x = this.mouthRobotMouth * 0.80;

        if (this.mouthRobotGlow) {
            this.mouthRobotGlow.material.opacity = 0.12 + this.mouthRobotMouth * 0.70;
        }

        const sway = this.mouthRobotTalking ? 0.045 : 0.02;
        this.mouthHeadGroup.rotation.z = Math.sin(tSec * 1.7) * sway;
        this.mouthHeadGroup.rotation.x = Math.sin(tSec * 0.9) * 0.02;

        // Neck: powered bob when the L298 channel 2 is energized, plus
        // occasional life while idle, talking or dancing.
        this.mouthRobotNeckPhase += dt * (this.mouthRobotDancing ? 8.0 : (this.mouthRobotTalking ? 4.2 : 1.7));
        let neckLift = 0;
        if (this.neckPulsating) {
            neckLift = (Math.sin(this.mouthRobotNeckPhase) + 1) * 0.5 * 24;
        } else if (this.mouthRobotDancing) {
            neckLift = (Math.sin(this.mouthRobotNeckPhase) + 1) * 0.5 * 18;
        } else if (this.mouthRobotTalking) {
            neckLift = (Math.sin(this.mouthRobotNeckPhase) + 1) * 0.5 * 10;
        } else {
            this.mouthRobotIdleTimer -= dt;
            if (this.mouthRobotIdleTimer <= 0) {
                this.mouthRobotIdleTimer = 3.5 + Math.random() * 5.0;
                this.mouthRobotIdleImpulse = 1;
            }
            this.mouthRobotIdleImpulse = Math.max(0, this.mouthRobotIdleImpulse - dt * 0.7);
            neckLift = (Math.sin(this.mouthRobotNeckPhase) + 1) * 0.5 * this.mouthRobotIdleImpulse * 12;
        }
        this.mouthRobotNeckOffset += (neckLift - this.mouthRobotNeckOffset) * Math.min(1, dt * 8);
        this.mouthNeckGroup.position.y = 22 + this.mouthRobotNeckOffset;
        this.mouthNeckPiston.scale.y = 1 + this.mouthRobotNeckOffset / 180;
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

        this.animateMouthRobot(dt, t);

        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}

if (typeof window !== 'undefined') {
    window.Biped3DRenderer = Biped3DRenderer;
}
