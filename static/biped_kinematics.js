/**
 * Biped 17-DOF Kinematics & Gait Engine
 *
 * Servo / joint layout (17 degrees of freedom):
 *   0  head_pan          1  head_tilt
 *   2  waist_yaw
 *   3  l_shoulder_pitch  4  l_shoulder_roll  5  l_elbow
 *   6  r_shoulder_pitch  7  r_shoulder_roll  8  r_elbow
 *   9  l_hip_yaw        10  l_hip_pitch      11  l_knee        12  l_ankle
 *  13  r_hip_yaw        14  r_hip_pitch      15  r_knee        16  r_ankle
 *
 * Angles are logical degrees. 0 is the mechanical centre, positive values
 * follow the servo orientation documented in the wiring tab.
 */

class BipedKinematics {
    constructor() {
        this.JOINTS = [
            'head_pan', 'head_tilt', 'waist_yaw',
            'l_shoulder_pitch', 'l_shoulder_roll', 'l_elbow',
            'r_shoulder_pitch', 'r_shoulder_roll', 'r_elbow',
            'l_hip_yaw', 'l_hip_pitch', 'l_knee', 'l_ankle',
            'r_hip_yaw', 'r_hip_pitch', 'r_knee', 'r_ankle'
        ];

        this.HOME = {
            head_pan: 0, head_tilt: 0, waist_yaw: 0,
            l_shoulder_pitch: 0, l_shoulder_roll: 0, l_elbow: 10,
            r_shoulder_pitch: 0, r_shoulder_roll: 0, r_elbow: 10,
            l_hip_yaw: 0, l_hip_pitch: 0, l_knee: 5, l_ankle: -5,
            r_hip_yaw: 0, r_hip_pitch: 0, r_knee: 5, r_ankle: -5
        };

        this.LIMITS = {
            head_pan: [-80, 80], head_tilt: [-40, 40], waist_yaw: [-60, 60],
            l_shoulder_pitch: [-170, 60], l_shoulder_roll: [-20, 110], l_elbow: [0, 140],
            r_shoulder_pitch: [-170, 60], r_shoulder_roll: [-110, 20], r_elbow: [0, 140],
            l_hip_yaw: [-45, 45], l_hip_pitch: [-90, 90], l_knee: [-120, 5], l_ankle: [-40, 40],
            r_hip_yaw: [-45, 45], r_hip_pitch: [-90, 90], r_knee: [-120, 5], r_ankle: [-40, 40]
        };

        // Geometry (mm) used for inverse kinematics / export helpers
        this.HIP_WIDTH = 110;
        this.THIGH_LEN = 150;
        this.SHIN_LEN = 150;
        this.ANKLE_LEN = 38;
        this.DEFAULT_HEIGHT = 300;

        this.calibrationTrims = {};
        this.JOINTS.forEach(name => { this.calibrationTrims[name] = 0; });

        this.activePose = 'Stand';
        this.walkPhase = 0;
        this.stepHeight = 35;
        this.stepLength = 60;
        this.lastTime = performance.now();
    }

    setCalibration(trims) {
        this.JOINTS.forEach(name => {
            const value = trims && typeof trims[name] === 'number' ? trims[name] : 0;
            this.calibrationTrims[name] = value;
        });
    }

    clamp(name, value) {
        const lim = this.LIMITS[name] || [-180, 180];
        return Math.max(lim[0], Math.min(lim[1], value));
    }

    basePose() {
        return Object.assign({}, this.HOME);
    }

    applyPoseOverrides(joints, overrides) {
        if (!overrides) return;
        Object.keys(overrides).forEach(key => {
            if (joints[key] !== undefined) joints[key] = overrides[key];
        });
    }

    /**
     * Two-link inverse kinematics for one leg. Returns logical joint angles.
     * target is relative to the hip: {x: lateral, y: vertical (negative down), z: forward}
     */
    solveLegIK(target, side) {
        const x = target.x || 0;
        const y = target.y || -this.DEFAULT_HEIGHT;
        const z = target.z || 0;

        const hipYaw = Math.atan2(x, z);
        const planar = Math.sqrt(x * x + z * z);
        let d = Math.sqrt(planar * planar + y * y);
        const maxReach = this.THIGH_LEN + this.SHIN_LEN - 1;
        const minReach = Math.abs(this.THIGH_LEN - this.SHIN_LEN) + 1;
        d = Math.max(minReach, Math.min(maxReach, d));

        const cosKnee = (this.THIGH_LEN * this.THIGH_LEN + this.SHIN_LEN * this.SHIN_LEN - d * d) /
            (2 * this.THIGH_LEN * this.SHIN_LEN);
        const kneeInner = Math.acos(Math.max(-1, Math.min(1, cosKnee)));
        const knee = -(Math.PI - kneeInner);

        const cosHip = (this.THIGH_LEN * this.THIGH_LEN + d * d - this.SHIN_LEN * this.SHIN_LEN) /
            (2 * this.THIGH_LEN * d);
        const hipOffset = Math.acos(Math.max(-1, Math.min(1, cosHip)));
        const hipPitch = Math.atan2(planar, -y) - hipOffset;

        const ankle = -(hipPitch + knee);
        const prefix = side === 'l' ? 'l_' : 'r_';
        return {
            [prefix + 'hip_yaw']: this.clamp(prefix + 'hip_yaw', hipYaw * 180 / Math.PI),
            [prefix + 'hip_pitch']: this.clamp(prefix + 'hip_pitch', hipPitch * 180 / Math.PI),
            [prefix + 'knee']: this.clamp(prefix + 'knee', knee * 180 / Math.PI),
            [prefix + 'ankle']: this.clamp(prefix + 'ankle', ankle * 180 / Math.PI)
        };
    }

    poseFromName(name) {
        const P = {
            'Stand': {},
            'T-Pose': {
                l_shoulder_roll: 80, r_shoulder_roll: -80, l_elbow: 0, r_elbow: 0
            },
            'Wave Right Hand': {
                r_shoulder_pitch: -120, r_shoulder_roll: -35, r_elbow: 55,
                head_pan: 20, head_tilt: 8
            },
            'Wave Left Hand': {
                l_shoulder_pitch: -120, l_shoulder_roll: 35, l_elbow: 55,
                head_pan: -20, head_tilt: 8
            },
            'Both Arms Up': {
                l_shoulder_pitch: -160, r_shoulder_pitch: -160,
                l_shoulder_roll: 15, r_shoulder_roll: -15,
                l_elbow: 10, r_elbow: 10, head_tilt: -10
            },
            'Dance': {
                l_shoulder_pitch: -70, r_shoulder_pitch: -20,
                l_shoulder_roll: 55, r_shoulder_roll: -55,
                l_elbow: 70, r_elbow: 40, waist_yaw: 20
            },
            'Squat': {
                l_hip_pitch: -55, r_hip_pitch: -55, l_knee: 95, r_knee: 95,
                l_ankle: -40, r_ankle: -40, l_shoulder_pitch: -40, r_shoulder_pitch: -40
            },
            'Bow / Respect': {
                l_shoulder_pitch: 30, r_shoulder_pitch: 30, head_tilt: 25
            },
            'Sit / Park': {
                l_hip_pitch: -80, r_hip_pitch: -80, l_knee: 90, r_knee: 90,
                l_ankle: 10, r_ankle: 10, l_shoulder_pitch: 20, r_shoulder_pitch: 20
            },
            'Sleep / Park': {
                l_hip_pitch: -40, r_hip_pitch: -40, l_knee: 100, r_knee: 100,
                l_ankle: -20, r_ankle: -20, head_tilt: 30
            },
            'Curious Tilt': { head_pan: 18, head_tilt: 22, waist_yaw: 8 },
            'Greeting Wave': {
                r_shoulder_pitch: -130, r_shoulder_roll: -30, r_elbow: 45,
                head_tilt: -8, waist_yaw: -10
            },
            'Low Stance': {
                l_hip_pitch: -30, r_hip_pitch: -30, l_knee: 55, r_knee: 55,
                l_ankle: -25, r_ankle: -25
            },
            'High Stance': {
                l_hip_pitch: 10, r_hip_pitch: 10, l_knee: 0, r_knee: 0,
                l_ankle: 0, r_ankle: 0
            },
            'Attack / Guard': {
                l_shoulder_pitch: -80, r_shoulder_pitch: -80, l_elbow: 90, r_elbow: 90,
                l_shoulder_roll: 25, r_shoulder_roll: -25, head_tilt: -12
            },
            'Walk in Place': {}
        };
        return P[name] || {};
    }

    bodyFromName(name) {
        const B = {
            'Stand': { height: 0, roll: 0, pitch: 0, yaw: 0 },
            'T-Pose': { height: 0, roll: 0, pitch: 0, yaw: 0 },
            'Wave Right Hand': { height: 0, roll: -4, pitch: 0, yaw: 0 },
            'Wave Left Hand': { height: 0, roll: 4, pitch: 0, yaw: 0 },
            'Both Arms Up': { height: 15, roll: 0, pitch: -8, yaw: 0 },
            'Dance': { height: 15, roll: 10, pitch: 0, yaw: 15 },
            'Squat': { height: -70, roll: 0, pitch: 0, yaw: 0 },
            'Bow / Respect': { height: -25, roll: 0, pitch: 30, yaw: 0 },
            'Sit / Park': { height: -150, roll: 0, pitch: 0, yaw: 0 },
            'Sleep / Park': { height: -240, roll: 0, pitch: 0, yaw: 0 },
            'Curious Tilt': { height: 0, roll: 8, pitch: 0, yaw: 0 },
            'Greeting Wave': { height: 0, roll: -6, pitch: 0, yaw: -10 },
            'Low Stance': { height: -55, roll: 0, pitch: 0, yaw: 0 },
            'High Stance': { height: 35, roll: 0, pitch: 0, yaw: 0 },
            'Attack / Guard': { height: -20, roll: 0, pitch: 12, yaw: 0 },
            'Walk in Place': { height: 0, roll: 0, pitch: 0, yaw: 0 }
        };
        return Object.assign({ height: 0, roll: 0, pitch: 0, yaw: 0 }, B[name] || {});
    }

    computePose(name, timeMs) {
        this.activePose = name || 'Stand';
        const t = (timeMs === undefined ? performance.now() : timeMs);
        const joints = this.basePose();
        this.applyPoseOverrides(joints, this.poseFromName(this.activePose));
        const body = this.bodyFromName(this.activePose);

        if (this.activePose === 'Walk in Place') {
            return this.computeWalk(1 / 30, { fwd: 1, strafe: 0, turn: 0 }, body);
        }

        if (this.activePose === 'Wave Right Hand' || this.activePose === 'Greeting Wave') {
            joints.r_elbow += Math.sin(t * 0.012) * 22;
            joints.r_shoulder_roll += Math.sin(t * 0.012) * 10;
            joints.head_tilt += Math.sin(t * 0.004) * 4;
        } else if (this.activePose === 'Wave Left Hand') {
            joints.l_elbow += Math.sin(t * 0.012) * 22;
            joints.l_shoulder_roll += Math.sin(t * 0.012) * 10;
            joints.head_tilt += Math.sin(t * 0.004) * 4;
        } else if (this.activePose === 'Both Arms Up') {
            joints.l_elbow += Math.sin(t * 0.009) * 15;
            joints.r_elbow += Math.sin(t * 0.009 + Math.PI) * 15;
        } else if (this.activePose === 'Dance') {
            const s = Math.sin(t * 0.004);
            const c = Math.cos(t * 0.0032);
            joints.waist_yaw = 25 * s;
            joints.l_shoulder_pitch = -70 + 30 * c;
            joints.r_shoulder_pitch = -20 - 30 * c;
            joints.l_shoulder_roll = 55 + 15 * s;
            joints.r_shoulder_roll = -55 + 15 * s;
            joints.l_elbow = 60 + 30 * Math.abs(s);
            joints.r_elbow = 40 + 30 * Math.abs(c);
            joints.l_hip_pitch = -8 * s;
            joints.r_hip_pitch = 8 * s;
            joints.l_knee = 5 + 12 * Math.max(0, s);
            joints.r_knee = 5 + 12 * Math.max(0, -s);
            body.roll = 10 * s;
            body.yaw = 15 * s;
            body.height = 15 + 8 * Math.abs(s);
        } else if (this.activePose === 'Squat') {
            const bounce = Math.sin(t * 0.005) * 8;
            joints.l_knee += bounce;
            joints.r_knee += bounce;
            body.height += bounce;
        } else if (this.activePose === 'Curious Tilt') {
            joints.head_tilt += Math.sin(t * 0.002) * 5;
        } else if (this.activePose === 'Attack / Guard') {
            joints.l_elbow += Math.sin(t * 0.01) * 6;
            joints.r_elbow += Math.sin(t * 0.01 + Math.PI) * 6;
        } else if (this.activePose === 'Stand') {
            const breathe = Math.sin(t * 0.0016);
            joints.l_shoulder_pitch += breathe * 2;
            joints.r_shoulder_pitch += breathe * 2;
            body.height += breathe * 2;
        }

        this.JOINTS.forEach(key => { joints[key] = this.clamp(key, joints[key]); });
        return { joints, body };
    }

    computeWalk(dt, moveVector, bodyOverride) {
        const { fwd = 0, strafe = 0, turn = 0 } = moveVector || {};
        const magnitude = Math.min(1, Math.abs(fwd) + Math.abs(strafe) + Math.abs(turn));
        const speed = 1.35;
        this.walkPhase = (this.walkPhase + dt * speed * (0.4 + magnitude)) % 1;

        const joints = this.basePose();
        const body = Object.assign({ height: 0, roll: 0, pitch: 0, yaw: 0 }, bodyOverride || {});

        const phase = this.walkPhase;
        const stepAmp = (this.stepLength / 60) * 26 * (0.35 + 0.65 * magnitude);
        const liftAmp = (this.stepHeight / 35) * 30 * (0.35 + 0.65 * magnitude);

        ['l', 'r'].forEach((side, index) => {
            const offset = index === 0 ? 0 : 0.5;
            const p = (phase + offset) % 1;
            const swing = Math.sin(2 * Math.PI * p);
            const lift = Math.max(0, Math.sin(2 * Math.PI * p));

            joints[side + '_hip_pitch'] = -stepAmp * swing * (fwd >= 0 ? 1 : -1);
            joints[side + '_knee'] = 5 + liftAmp * lift;
            joints[side + '_ankle'] = -5 - liftAmp * lift * 0.35 + stepAmp * swing * 0.25;
            joints[side + '_hip_yaw'] = strafe * 18 * (index === 0 ? 1 : -1);

            if (Math.abs(fwd) < 0.05 && Math.abs(strafe) < 0.05 && Math.abs(turn) > 0.05) {
                joints[side + '_hip_pitch'] = stepAmp * 0.55 * swing * (index === 0 ? 1 : -1) * Math.sign(turn);
                joints[side + '_knee'] = 5 + liftAmp * 0.55 * lift;
            }
        });

        const armSwing = 22 * (0.4 + 0.6 * magnitude) * Math.sin(2 * Math.PI * phase);
        joints.l_shoulder_pitch = armSwing;
        joints.r_shoulder_pitch = -armSwing;
        joints.l_elbow = 10 + Math.abs(armSwing) * 0.35;
        joints.r_elbow = 10 + Math.abs(armSwing) * 0.35;
        joints.waist_yaw = turn * 16;
        body.height = 10 * Math.sin(4 * Math.PI * phase) + (bodyOverride ? bodyOverride.height || 0 : 0);
        body.roll = strafe * 6;
        body.yaw = turn * 10;

        this.JOINTS.forEach(key => { joints[key] = this.clamp(key, joints[key]); });
        return { joints, body };
    }

    /** Apply calibration trim then return servo-ready 0..180 values. */
    servoAngles(joints) {
        const result = {};
        this.JOINTS.forEach(name => {
            const logical = (joints[name] || 0) + (this.calibrationTrims[name] || 0);
            result[name] = Math.max(0, Math.min(180, 90 + logical));
        });
        return result;
    }
}

if (typeof window !== 'undefined') {
    window.BipedKinematics = BipedKinematics;
}
