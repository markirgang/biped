/**
 * Biped 17-DOF Control Center
 * Connects the 3D workbench, 17-servo calibration, Gemini AI, camera,
 * microphone, MP3 studio, ultrasonic telemetry, L298 accessories and
 * Python export to the local biped server on port 2090.
 */
(function () {
    'use strict';

    const $ = id => document.getElementById(id);
    const kinematics = new BipedKinematics();
    let renderer = null;
    let calibRenderer = null;

    let servos = [];
    let calibration = {};
    let activePose = 'Stand';
    let gaitType = 'walk';
    let moveVector = { fwd: 0, strafe: 0, turn: 0 };
    let moving = false;
    let bodyOffsets = { height: 0, pitch: 0, roll: 0, yaw: 0 };
    let sensorData = { front: 85, rear: 140, left: 70, right: 75 };
    let obstacleAvoidance = false;
    let motorState = { mouth: false, neck: false };
    let apiKey = localStorage.getItem('biped_gemini_key') || '';
    let selectedVoice = 'ai_female_us';
    let selectedSpeaker = 'browser';
    let lastServoSend = 0;
    let lastTelemetry = 0;
    let lastSensorPoll = 0;
    let lastGesturePoll = 0;
    let audioPlayer = null;
    let audioProgressTimer = null;
    let currentSong = null;

    let micStream = null;
    let micContext = null;
    let micAnalyser = null;
    let micData = null;
    let micRecording = false;
    let mediaRecorder = null;
    let recordedChunks = [];
    let micSensitivity = 1.0;

    function addChat(role, text) {
        const box = $('chatHistory');
        if (!box) return;
        const div = document.createElement('div');
        div.className = 'msg ' + (role === 'user' ? 'user' : 'ai');
        div.innerHTML = '<strong>' + (role === 'user' ? 'You' : 'Gemini AI') + ':</strong> ' +
            String(text || '').replace(/</g, '&lt;').replace(/>/g, '&gt;');
        box.appendChild(div);
        box.scrollTop = box.scrollHeight;
    }

    function setStatus(text, online) {
        const el = $('statusText');
        if (el) el.textContent = text;
        const dot = document.querySelector('.status-dot');
        if (dot) dot.style.background = online === false ? 'var(--accent-red)' : 'var(--accent-green)';
    }

    function withTrims(joints) {
        const out = {};
        kinematics.JOINTS.forEach(name => {
            out[name] = kinematics.clamp(name, (joints[name] || 0) + (calibration[name] || 0));
        });
        return out;
    }

    function computeCurrent(now) {
        let result;
        if (moving) {
            const dt = gaitType === 'run' ? 1 / 24 : 1 / 55;
            const mv = Object.assign({}, moveVector);
            if (gaitType === 'march') {
                mv.fwd = Math.abs(mv.fwd) > 0.05 ? mv.fwd : 1;
                mv.strafe = 0;
                mv.turn = 0;
            }
            result = kinematics.computeWalk(dt, mv);
        } else {
            result = kinematics.computePose(activePose, now);
        }
        result.body.height += bodyOffsets.height;
        result.body.pitch += bodyOffsets.pitch;
        result.body.roll += bodyOffsets.roll;
        result.body.yaw += bodyOffsets.yaw;
        return result;
    }

    function animate() {
        requestAnimationFrame(animate);
        const now = performance.now();
        const result = computeCurrent(now);
        const display = withTrims(result.joints);

        if (renderer) renderer.setPose(display, result.body);
        if (calibRenderer) calibRenderer.setPose(display, result.body);

        if (now - lastTelemetry > 100) {
            lastTelemetry = now;
            updateTelemetry(display);
        }

        if (now - lastServoSend > 120) {
            lastServoSend = now;
            sendServoState(result.joints, result.body);
        }

        if (now - lastSensorPoll > 900) {
            lastSensorPoll = now;
            pollSensors();
        }

        if (now - lastGesturePoll > 6000) {
            lastGesturePoll = now;
            pollGesture();
        }
    }

    function sendServoState(joints, body) {
        fetch('/api/servo/set', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ angles: joints, body: body })
        }).catch(function () {});
    }

    function updateTelemetry(joints) {
        if (!servos.length) return;
        let reachable = true;
        servos.forEach(function (cfg) {
            const el = $('telemetry_' + cfg.name);
            if (el) {
                const value = joints[cfg.name];
                el.textContent = (value > 0 ? '+' : '') + Math.round(value) + '°';
                if (value < cfg.min || value > cfg.max) reachable = false;
            }
        });
        const ik = $('ikStatus');
        if (ik) {
            ik.textContent = reachable ? 'IK VALID' : 'LIMIT';
            ik.style.color = reachable ? 'var(--accent-cyan)' : 'var(--accent-orange)';
        }
    }

    function pollSensors() {
        fetch('/api/sensors/distances').then(function (r) { return r.json(); }).then(function (data) {
            sensorData = data;
            const map = { front: 'distFront', rear: 'distRear', left: 'distLeft', right: 'distRight' };
            Object.keys(map).forEach(function (key) {
                const el = $(map[key]);
                if (el) el.textContent = Math.round(data[key] || 0) + ' cm';
            });
            if (renderer) renderer.setSensorDistances(data);
            if (calibRenderer) calibRenderer.setSensorDistances(data);

            if (obstacleAvoidance && data.front < 35 && (moveVector.fwd > 0 || moving)) {
                stopMotion();
                moveVector = { fwd: 0, strafe: 0, turn: 0.8 };
                moving = true;
                setTimeout(stopMotion, 900);
            }
        }).catch(function () {});
    }

    function pollGesture() {
        fetch('/api/gemini/gesture', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' })
            .then(function (r) { return r.json(); })
            .then(function (data) {
                const badge = $('gestureBadge');
                if (badge) badge.textContent = 'Gesture: ' + (data.gesture || 'None');
                if (data.action) handleAction(data.action);
            }).catch(function () {});
    }

    function handleAction(action) {
        if (!action || !action.name) return;
        const args = action.args || {};
        if (action.name === 'biped_walk') {
            const dir = String(args.direction || 'forward');
            const duration = Math.max(1, Math.min(15, Number(args.duration_seconds) || 3));
            const vectors = {
                forward: { fwd: 1, strafe: 0, turn: 0 },
                backward: { fwd: -1, strafe: 0, turn: 0 },
                run: { fwd: 1, strafe: 0, turn: 0 },
                run_forward: { fwd: 1, strafe: 0, turn: 0 },
                left: { fwd: 0, strafe: -1, turn: 0 },
                right: { fwd: 0, strafe: 1, turn: 0 },
                turn_left: { fwd: 0, strafe: 0, turn: -1 },
                turn_right: { fwd: 0, strafe: 0, turn: 1 }
            };
            moveVector = vectors[dir] || vectors.forward;
            if (dir.indexOf('run') === 0) setGait('run');
            moving = true;
            setTimeout(stopMotion, duration * 1000);
        } else if (action.name === 'biped_pose') {
            setPose(String(args.pose_name || 'Stand'));
        } else if (action.name === 'biped_stop') {
            stopMotion();
        }
    }

    function stopMotion() {
        moving = false;
        moveVector = { fwd: 0, strafe: 0, turn: 0 };
        document.querySelectorAll('.btn-dpad').forEach(function (b) { b.classList.remove('active'); });
    }

    function setPose(name) {
        activePose = name;
        moving = false;
        const select = $('calibPoseSelect');
        if (select && select.value !== name) select.value = name;
        fetch('/api/pose', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name: name })
        }).catch(function () {});
    }

    function setGait(name) {
        gaitType = name;
        document.querySelectorAll('#gaitTabs .tab-btn').forEach(function (b) {
            b.classList.toggle('active', b.dataset.gait === name);
        });
        const sel = $('calibGaitSelect');
        if (sel && sel.value !== name) sel.value = name;
    }

    // ---------------------------------------------------------------------
    // UI construction
    // ---------------------------------------------------------------------
    function buildTelemetry() {
        const grid = $('telemetryGrid');
        if (!grid) return;
        grid.innerHTML = '';
        servos.forEach(function (cfg) {
            const card = document.createElement('div');
            card.className = 'leg-card';
            card.innerHTML =
                '<div class="leg-title" style="font-size:0.68rem;">' + cfg.label + '</div>' +
                '<div class="joint-row"><span style="color:var(--text-muted); font-size:0.65rem;">' + cfg.board + ' / CH' + cfg.channel + '</span>' +
                '<span id="telemetry_' + cfg.name + '" class="joint-deg">0°</span></div>';
            grid.appendChild(card);
        });
    }

    function buildCalibration() {
        const grid = $('calibrationGrid');
        if (!grid) return;
        grid.innerHTML = '';
        servos.forEach(function (cfg) {
            const card = document.createElement('div');
            card.className = 'calib-leg-card';
            const value = calibration[cfg.name] || 0;
            card.innerHTML =
                '<div class="leg-title" style="font-size:0.8rem; margin-bottom:8px;">' + cfg.label +
                '<span style="float:right; color:var(--text-muted); font-size:0.65rem;">' + cfg.board + ' CH' + cfg.channel + '</span></div>' +
                '<div class="slider-control">' +
                '<div class="slider-label"><span>Trim Offset</span><span class="slider-val" id="valTrim_' + cfg.name + '">' + (value > 0 ? '+' : '') + value + '°</span></div>' +
                '<input type="range" class="trim-slider" data-name="' + cfg.name + '" id="slideTrim_' + cfg.name + '" min="-30" max="30" value="' + value + '">' +
                '</div>';
            grid.appendChild(card);
        });

        grid.querySelectorAll('.trim-slider').forEach(function (slider) {
            slider.addEventListener('input', function (e) {
                const name = e.target.dataset.name;
                const value = parseInt(e.target.value, 10);
                calibration[name] = value;
                kinematics.setCalibration(calibration);
                const display = $('valTrim_' + name);
                if (display) display.textContent = (value > 0 ? '+' : '') + value + '°';
                saveCalibration();
            });
        });
    }

    let calibrationTimer = null;
    function saveCalibration() {
        if (calibrationTimer) clearTimeout(calibrationTimer);
        calibrationTimer = setTimeout(function () {
            fetch('/api/calibration', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ calibration: calibration })
            }).catch(function () {});
        }, 300);
    }

    function buildServoMap() {
        const tbody = $('servoMapTable');
        if (!tbody) return;
        tbody.innerHTML = '';
        servos.forEach(function (cfg, index) {
            const tr = document.createElement('tr');
            tr.innerHTML =
                '<td>' + (index + 1) + '</td>' +
                '<td><strong>' + cfg.label + '</strong></td>' +
                '<td>' + cfg.board + '</td>' +
                '<td>CH ' + cfg.channel + '</td>' +
                '<td>' + cfg.group + '</td>' +
                '<td>' + cfg.min + '° to ' + cfg.max + '°</td>';
            tbody.appendChild(tr);
        });
    }

    function loadServos() {
        return fetch('/api/servos').then(function (r) { return r.json(); }).then(function (data) {
            servos = data.servos || [];
            calibration = data.calibration || {};
            kinematics.setCalibration(calibration);
            buildTelemetry();
            buildCalibration();
            buildServoMap();
        }).catch(function () {});
    }

    function loadStatus() {
        fetch('/api/status').then(function (r) { return r.json(); }).then(function (data) {
            setStatus('ONLINE', true);
            const hw = $('hardwareStatus');
            if (hw) {
                hw.textContent = data.hardware_connected ? 'CONNECTED' : 'SIMULATION';
                hw.style.color = data.hardware_connected ? 'var(--accent-green)' : 'var(--accent-orange)';
            }
        }).catch(function () { setStatus('OFFLINE', false); });
    }

    // ---------------------------------------------------------------------
    // Sliders, buttons, tabs
    // ---------------------------------------------------------------------
    function bindUI() {
        document.querySelectorAll('.nav-tab-btn').forEach(function (btn) {
            btn.addEventListener('click', function () {
                document.querySelectorAll('.nav-tab-btn').forEach(function (b) { b.classList.remove('active'); });
                document.querySelectorAll('.view-screen').forEach(function (v) { v.classList.remove('active'); });
                btn.classList.add('active');
                const view = $(btn.dataset.view);
                if (view) view.classList.add('active');
                setTimeout(function () {
                    if (renderer) renderer.onWindowResize();
                    if (calibRenderer) calibRenderer.onWindowResize();
                }, 60);
            });
        });

        document.querySelectorAll('#gaitTabs .tab-btn').forEach(function (btn) {
            btn.addEventListener('click', function () { setGait(btn.dataset.gait); });
        });

        const height = $('slideHeight');
        if (height) height.addEventListener('input', function () {
            bodyOffsets.height = parseInt(height.value, 10);
            $('valHeight').textContent = bodyOffsets.height + ' mm';
        });
        const pitch = $('slidePitch');
        if (pitch) pitch.addEventListener('input', function () {
            bodyOffsets.pitch = parseInt(pitch.value, 10);
            $('valPitch').textContent = bodyOffsets.pitch + '°';
        });
        const roll = $('slideRoll');
        if (roll) roll.addEventListener('input', function () {
            bodyOffsets.roll = parseInt(roll.value, 10);
            $('valRoll').textContent = bodyOffsets.roll + '°';
        });
        const yaw = $('slideYaw');
        if (yaw) yaw.addEventListener('input', function () {
            bodyOffsets.yaw = parseInt(yaw.value, 10);
            $('valYaw').textContent = bodyOffsets.yaw + '°';
        });
        const stepH = $('slideStepHeight');
        if (stepH) stepH.addEventListener('input', function () {
            kinematics.stepHeight = parseInt(stepH.value, 10);
            $('valStepHeight').textContent = kinematics.stepHeight + ' mm';
        });
        const stepL = $('slideStepLen');
        if (stepL) stepL.addEventListener('input', function () {
            kinematics.stepLength = parseInt(stepL.value, 10);
            $('valStepLen').textContent = kinematics.stepLength + ' mm';
        });

        const resetAttitude = $('btnResetAttitude');
        if (resetAttitude) resetAttitude.addEventListener('click', function () {
            bodyOffsets = { height: 0, pitch: 0, roll: 0, yaw: 0 };
            ['Height', 'Pitch', 'Roll', 'Yaw'].forEach(function (name) {
                const slider = $('slide' + name);
                if (slider) slider.value = 0;
            });
            $('valHeight').textContent = '0 mm';
            $('valPitch').textContent = '0°';
            $('valRoll').textContent = '0°';
            $('valYaw').textContent = '0°';
        });

        const resetCam = $('btnResetCam');
        if (resetCam) resetCam.addEventListener('click', function () { if (renderer) renderer.resetCamera(); });
        const resetCamCalib = $('btnResetCamCalib');
        if (resetCamCalib) resetCamCalib.addEventListener('click', function () { if (calibRenderer) calibRenderer.resetCamera(); });

        bindDpad('btnFwd', { fwd: 1, strafe: 0, turn: 0 });
        bindDpad('btnBwd', { fwd: -1, strafe: 0, turn: 0 });
        bindDpad('btnLeft', { fwd: 0, strafe: -1, turn: 0 });
        bindDpad('btnRight', { fwd: 0, strafe: 1, turn: 0 });
        bindDpad('btnTurnL', { fwd: 0, strafe: 0, turn: -1 });
        bindDpad('btnTurnR', { fwd: 0, strafe: 0, turn: 1 });
        const stop = $('btnStop');
        if (stop) stop.addEventListener('click', stopMotion);

        document.querySelectorAll('#presetButtons [data-pose], #calibQuickPoses [data-pose]').forEach(function (btn) {
            btn.addEventListener('click', function () { setPose(btn.dataset.pose); });
        });

        const calibPose = $('calibPoseSelect');
        if (calibPose) calibPose.addEventListener('change', function () { setPose(calibPose.value); });
        const calibGait = $('calibGaitSelect');
        if (calibGait) calibGait.addEventListener('change', function () { setGait(calibGait.value); });

        const testFwd = $('btnTestGaitForward');
        if (testFwd) testFwd.addEventListener('click', function () {
            moveVector = { fwd: 1, strafe: 0, turn: 0 };
            moving = true;
        });
        const testStop = $('btnTestGaitStop');
        if (testStop) testStop.addEventListener('click', stopMotion);
        const testL = $('btnTestGaitTurnL');
        if (testL) testL.addEventListener('click', function () {
            moveVector = { fwd: 0, strafe: 0, turn: -1 };
            moving = true;
        });
        const testR = $('btnTestGaitTurnR');
        if (testR) testR.addEventListener('click', function () {
            moveVector = { fwd: 0, strafe: 0, turn: 1 };
            moving = true;
        });

        const resetCal = $('btnResetCalibration');
        if (resetCal) resetCal.addEventListener('click', function () {
            Object.keys(calibration).forEach(function (key) { calibration[key] = 0; });
            kinematics.setCalibration(calibration);
            buildCalibration();
            saveCalibration();
        });

        document.addEventListener('keydown', function (e) {
            if (e.target && /INPUT|SELECT|TEXTAREA/.test(e.target.tagName)) return;
            const key = e.key.toLowerCase();
            if (key === 'w' || key === 'arrowup') { moveVector = { fwd: 1, strafe: 0, turn: 0 }; moving = true; }
            else if (key === 's' || key === 'arrowdown') { moveVector = { fwd: -1, strafe: 0, turn: 0 }; moving = true; }
            else if (key === 'a' || key === 'arrowleft') { moveVector = { fwd: 0, strafe: -1, turn: 0 }; moving = true; }
            else if (key === 'd' || key === 'arrowright') { moveVector = { fwd: 0, strafe: 1, turn: 0 }; moving = true; }
            else if (key === 'q') { moveVector = { fwd: 0, strafe: 0, turn: -1 }; moving = true; }
            else if (key === 'e') { moveVector = { fwd: 0, strafe: 0, turn: 1 }; moving = true; }
            else if (key === ' ') { e.preventDefault(); stopMotion(); }
            else return;
            e.preventDefault();
        });
        document.addEventListener('keyup', function (e) {
            const key = e.key.toLowerCase();
            if (['w', 'a', 's', 'd', 'q', 'e', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].indexOf(key) >= 0) {
                stopMotion();
            }
        });

        bindSettings();
        bindGemini();
        bindMusic();
        bindExport();
    }

    function bindDpad(id, vector) {
        const btn = $(id);
        if (!btn) return;
        const start = function (e) {
            e.preventDefault();
            moveVector = Object.assign({}, vector);
            moving = true;
            btn.classList.add('active');
        };
        const end = function () {
            btn.classList.remove('active');
            stopMotion();
        };
        btn.addEventListener('pointerdown', start);
        btn.addEventListener('pointerup', end);
        btn.addEventListener('pointerleave', end);
        btn.addEventListener('pointercancel', end);
    }

    // ---------------------------------------------------------------------
    // Settings
    // ---------------------------------------------------------------------
    function bindSettings() {
        const savedKey = apiKey;
        if ($('settingsApiKey')) $('settingsApiKey').value = savedKey;
        if ($('inputApiKey')) $('inputApiKey').value = savedKey;

        const saveKey = function (value) {
            apiKey = value || '';
            localStorage.setItem('biped_gemini_key', apiKey);
            if ($('settingsApiKey')) $('settingsApiKey').value = apiKey;
            if ($('inputApiKey')) $('inputApiKey').value = apiKey;
        };

        const btnSaveSettings = $('btnSaveSettings');
        if (btnSaveSettings) btnSaveSettings.addEventListener('click', function () {
            saveKey($('settingsApiKey') ? $('settingsApiKey').value : '');
            addChat('ai', 'AI settings saved.');
        });
        const btnSaveApiKey = $('btnSaveApiKey');
        if (btnSaveApiKey) btnSaveApiKey.addEventListener('click', function () {
            saveKey($('inputApiKey') ? $('inputApiKey').value : '');
            addChat('ai', 'Gemini API key saved.');
        });
        const btnApiKey = $('btnApiKeySettings');
        if (btnApiKey) btnApiKey.addEventListener('click', function () {
            const box = $('apiKeyBox');
            if (box) box.style.display = box.style.display === 'none' ? 'block' : 'none';
        });

        const switchAvoid = $('switchObstacleAvoidance');
        if (switchAvoid) switchAvoid.addEventListener('change', function () {
            obstacleAvoidance = switchAvoid.checked;
        });

        const btnSwitchCamera = $('btnSwitchCamera');
        if (btnSwitchCamera) btnSwitchCamera.addEventListener('click', function () {
            const source = $('selectCamera') ? $('selectCamera').value : 'usb';
            fetch('/api/camera/select', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ source: source })
            }).then(function () {
                if ($('cameraStatusMsg')) $('cameraStatusMsg').textContent = 'Active Camera: ' + source;
                const feed = $('camFeed');
                if (feed) feed.src = '/api/camera/frame?t=' + Date.now();
            }).catch(function () {});
        });
        const btnDefaultCam = $('btnDefaultEmeetPixy');
        if (btnDefaultCam) btnDefaultCam.addEventListener('click', function () {
            if ($('selectCamera')) $('selectCamera').value = 'usb';
            if (btnSwitchCamera) btnSwitchCamera.click();
        });

        const selectSpeaker = $('selectSpeaker');
        if (selectSpeaker) selectSpeaker.addEventListener('change', function () { selectedSpeaker = selectSpeaker.value; });

        const selectVoice = $('selectVoice');
        if (selectVoice) {
            selectVoice.innerHTML = '';
            [
                ['ai_female_us', 'AI Female (US)'], ['ai_male_us', 'AI Male (US)'],
                ['ai_male_uk', 'AI Male (UK)'], ['ai_female_uk', 'AI Female (UK)'],
                ['ai_male_au', 'AI Male (Australia)'], ['ai_female_au', 'AI Female (Australia)'],
                ['ai_male_ca', 'AI Male (Canada)'], ['ai_female_ca', 'AI Female (Canada)'],
                ['ai_male_in', 'AI Male (India)'], ['ai_female_in', 'AI Female (India)'],
                ['ai_male_ie', 'AI Male (Ireland)'], ['ai_male_za', 'AI Male (South Africa)']
            ].forEach(function (pair) {
                const opt = document.createElement('option');
                opt.value = pair[0];
                opt.textContent = pair[1];
                selectVoice.appendChild(opt);
            });
            selectVoice.addEventListener('change', function () { selectedVoice = selectVoice.value; });
        }
    }

    // ---------------------------------------------------------------------
    // Gemini chat / voice / TTS
    // ---------------------------------------------------------------------
    function speakText(text) {
        if (!text) return;
        const clean = String(text).replace(/[*_#`]/g, '').slice(0, 600);
        const url = '/api/tts/speak?text=' + encodeURIComponent(clean) + '&voice=' + encodeURIComponent(selectedVoice);
        if (selectedSpeaker && selectedSpeaker !== 'browser') {
            fetch('/api/audio/play', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ url: window.location.origin + url, speaker: selectedSpeaker, volume: 100 })
            }).catch(function () {});
            return;
        }
        const audio = new Audio(url);
        audio.play().catch(function () {});
    }

    function sendChat(text) {
        const prompt = (text || '').trim();
        if (!prompt) return;
        addChat('user', prompt);
        addChat('ai', 'Thinking...');
        const box = $('chatHistory');
        const thinking = box ? box.lastElementChild : null;

        fetch('/api/gemini/chat', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ prompt: prompt, api_key: apiKey, include_vision: true })
        }).then(function (r) { return r.json(); }).then(function (data) {
            if (thinking) thinking.remove();
            const reply = data.text || 'No response.';
            addChat('ai', reply);
            if (data.action) handleAction(data.action);
            speakText(reply);
        }).catch(function (err) {
            if (thinking) thinking.remove();
            addChat('ai', 'Request failed: ' + err);
        });
    }

    function bindGemini() {
        const btnSend = $('btnSendChat');
        if (btnSend) btnSend.addEventListener('click', function () {
            const input = $('chatInput');
            const text = input ? input.value : '';
            if (input) input.value = '';
            sendChat(text);
        });
        const chatInput = $('chatInput');
        if (chatInput) chatInput.addEventListener('keydown', function (e) {
            if (e.key === 'Enter') btnSend.click();
        });
        const btnVision = $('btnAskVision');
        if (btnVision) btnVision.addEventListener('click', function () {
            sendChat('Describe what you see through the camera and suggest an action.');
        });

        const btnMic = $('btnMic');
        if (btnMic) btnMic.addEventListener('click', toggleMic);

        const sens = $('slideMicSensitivity');
        if (sens) sens.addEventListener('input', function () {
            micSensitivity = parseInt(sens.value, 10) / 100;
            if ($('valMicSensitivity')) $('valMicSensitivity').textContent = sens.value + '%';
        });

        // Camera preview refresh
        setInterval(function () {
            const feed = $('camFeed');
            if (feed && feed.offsetParent !== null) {
                feed.src = '/api/camera/frame?t=' + Date.now();
            }
        }, 700);
    }

    function toggleMic() {
        if (micRecording) {
            stopMic();
            return;
        }
        navigator.mediaDevices.getUserMedia({ audio: true }).then(function (stream) {
            micStream = stream;
            micContext = new (window.AudioContext || window.webkitAudioContext)();
            const source = micContext.createMediaStreamSource(stream);
            micAnalyser = micContext.createAnalyser();
            micAnalyser.fftSize = 256;
            micData = new Uint8Array(micAnalyser.frequencyBinCount);
            source.connect(micAnalyser);

            recordedChunks = [];
            const options = { mimeType: 'audio/webm' };
            try {
                mediaRecorder = new MediaRecorder(stream, options);
            } catch (e) {
                mediaRecorder = new MediaRecorder(stream);
            }
            mediaRecorder.ondataavailable = function (e) {
                if (e.data && e.data.size > 0) recordedChunks.push(e.data);
            };
            mediaRecorder.onstop = function () {
                const blob = new Blob(recordedChunks, { type: mediaRecorder.mimeType || 'audio/webm' });
                if (blob.size > 0) sendVoice(blob);
            };
            mediaRecorder.start();
            micRecording = true;
            if ($('micStatusText')) $('micStatusText').textContent = 'Status: Recording... click Activate Mic again to send.';
            if ($('btnMic')) $('btnMic').textContent = '⏹️ Stop & Send';
            meterMic();
        }).catch(function (err) {
            if ($('micStatusText')) $('micStatusText').textContent = 'Mic error: ' + err.message;
        });
    }

    function meterMic() {
        if (!micRecording || !micAnalyser) return;
        micAnalyser.getByteFrequencyData(micData);
        let sum = 0;
        for (let i = 0; i < micData.length; i++) sum += micData[i];
        let level = Math.min(100, Math.round((sum / micData.length) * micSensitivity * 1.4));
        const bar = $('micVuBar');
        const val = $('vuVal');
        if (bar) bar.style.width = level + '%';
        if (val) val.textContent = level + '%';
        requestAnimationFrame(meterMic);
    }

    function stopMic() {
        micRecording = false;
        if (mediaRecorder && mediaRecorder.state !== 'inactive') mediaRecorder.stop();
        if (micStream) micStream.getTracks().forEach(function (t) { t.stop(); });
        if (micContext) micContext.close().catch(function () {});
        micStream = null;
        micContext = null;
        micAnalyser = null;
        if ($('btnMic')) $('btnMic').textContent = '🎙️ Activate Mic';
        if ($('micStatusText')) $('micStatusText').textContent = 'Status: Sending voice to Gemini...';
        const bar = $('micVuBar');
        if (bar) bar.style.width = '0%';
        if ($('vuVal')) $('vuVal').textContent = '0%';
    }

    function sendVoice(blob) {
        const reader = new FileReader();
        reader.onloadend = function () {
            const base64 = String(reader.result).split(',')[1] || '';
            addChat('user', '🎙️ (voice message)');
            addChat('ai', 'Listening...');
            const box = $('chatHistory');
            const thinking = box ? box.lastElementChild : null;
            fetch('/api/gemini/voice', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ audio_b64: base64, mime_type: blob.type || 'audio/webm', api_key: apiKey, include_vision: true })
            }).then(function (r) { return r.json(); }).then(function (data) {
                if (thinking) thinking.remove();
                const reply = data.text || 'No response.';
                addChat('ai', reply);
                if (data.action) handleAction(data.action);
                speakText(reply);
                if ($('micStatusText')) $('micStatusText').textContent = 'Status: Ready. Click Activate Mic to talk again.';
            }).catch(function (err) {
                if (thinking) thinking.remove();
                addChat('ai', 'Voice request failed: ' + err);
            });
        };
        reader.readAsDataURL(blob);
    }

    // ---------------------------------------------------------------------
    // MP3 studio
    // ---------------------------------------------------------------------
    function bindMusic() {
        audioPlayer = new Audio();
        audioPlayer.volume = 0.8;

        const volume = $('slideMusicVolume');
        if (volume) volume.addEventListener('input', function () {
            audioPlayer.volume = parseInt(volume.value, 10) / 100;
            if ($('valMusicVolume')) $('valMusicVolume').textContent = volume.value + '%';
        });

        const progress = $('slideMusicProgress');
        if (progress) progress.addEventListener('input', function () {
            if (audioPlayer.duration) {
                audioPlayer.currentTime = (parseInt(progress.value, 10) / 100) * audioPlayer.duration;
            }
        });

        const fileInput = $('mp3FileInput');
        const uploadBtn = $('btnUploadMp3');
        if (uploadBtn && fileInput) {
            uploadBtn.addEventListener('click', function () { fileInput.click(); });
            fileInput.addEventListener('change', function () {
                const file = fileInput.files && fileInput.files[0];
                if (!file) return;
                const reader = new FileReader();
                reader.onloadend = function () {
                    const b64 = String(reader.result).split(',')[1] || '';
                    fetch('/api/audio/upload', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify({ filename: file.name, data_b64: b64 })
                    }).then(function (r) { return r.json(); }).then(function () {
                        loadAudioList();
                    }).catch(function () {});
                };
                reader.readAsDataURL(file);
            });
        }

        const playBtn = $('btnPlayMusic');
        if (playBtn) playBtn.addEventListener('click', playSong);
        const pauseBtn = $('btnPauseMusic');
        if (pauseBtn) pauseBtn.addEventListener('click', function () {
            if (audioPlayer) audioPlayer.pause();
        });
        const stopBtn = $('btnStopMusic');
        if (stopBtn) stopBtn.addEventListener('click', stopSong);

        audioPlayer.addEventListener('timeupdate', updateMusicProgress);
        audioPlayer.addEventListener('ended', function () {
            stopSong();
        });

        loadAudioList();
    }

    function loadAudioList() {
        fetch('/api/audio/list').then(function (r) { return r.json(); }).then(function (data) {
            const select = $('selectMp3Song');
            if (!select) return;
            select.innerHTML = '';
            const files = data.files || [];
            if (!files.length) {
                const opt = document.createElement('option');
                opt.value = '';
                opt.textContent = 'No MP3 files in static/audio/ yet';
                select.appendChild(opt);
                return;
            }
            files.forEach(function (name) {
                const opt = document.createElement('option');
                opt.value = name;
                opt.textContent = name;
                select.appendChild(opt);
            });
        }).catch(function () {});
    }

    function playSong() {
        const select = $('selectMp3Song');
        const name = select ? select.value : '';
        if (!name) return;
        stopSong(true);
        currentSong = name;
        audioPlayer.src = '/audio/' + encodeURIComponent(name);
        audioPlayer.volume = parseInt($('slideMusicVolume') ? $('slideMusicVolume').value : 80, 10) / 100;
        audioPlayer.play().catch(function () {});
        if ($('musicTrackName')) $('musicTrackName').textContent = 'Track: ' + name;

        const dance = $('selectMusicDance') ? $('selectMusicDance').value : 'Dance';
        setPose(dance);

        if (selectedSpeaker && selectedSpeaker !== 'browser') {
            fetch('/api/audio/play', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ url: window.location.origin + '/audio/' + encodeURIComponent(name), speaker: selectedSpeaker, volume: 80 })
            }).catch(function () {});
        }
    }

    function stopSong(keepPose) {
        if (audioPlayer) {
            audioPlayer.pause();
            audioPlayer.currentTime = 0;
        }
        if (audioProgressTimer) {
            clearInterval(audioProgressTimer);
            audioProgressTimer = null;
        }
        if ($('slideMusicProgress')) $('slideMusicProgress').value = 0;
        if ($('musicTimeVal')) $('musicTimeVal').textContent = '0:00 / 0:00';
        if (!keepPose) {
            currentSong = null;
            if ($('musicTrackName')) $('musicTrackName').textContent = 'Track: None';
            setPose('Stand');
        }
    }

    function updateMusicProgress() {
        if (!audioPlayer) return;
        const duration = audioPlayer.duration || 0;
        const current = audioPlayer.currentTime || 0;
        if ($('slideMusicProgress') && duration > 0) {
            $('slideMusicProgress').value = Math.round((current / duration) * 100);
        }
        if ($('musicTimeVal')) {
            $('musicTimeVal').textContent = formatTime(current) + ' / ' + formatTime(duration);
        }
    }

    function formatTime(seconds) {
        const s = Math.max(0, Math.floor(seconds || 0));
        return Math.floor(s / 60) + ':' + String(s % 60).padStart(2, '0');
    }

    // ---------------------------------------------------------------------
    // L298 accessory toggles
    // ---------------------------------------------------------------------
    function bindMotors() {
        const mouth = $('btnToggleMouth');
        if (mouth) mouth.addEventListener('click', function () {
            motorState.mouth = !motorState.mouth;
            sendMotorState();
        });
        const neck = $('btnToggleNeck');
        if (neck) neck.addEventListener('click', function () {
            motorState.neck = !motorState.neck;
            sendMotorState();
        });
    }

    function sendMotorState() {
        fetch('/api/motor/l298', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ mouth: motorState.mouth, neck: motorState.neck })
        }).catch(function () {});
        const mouthEl = $('motorMouthStatus');
        const neckEl = $('motorNeckStatus');
        if (mouthEl) {
            mouthEl.textContent = motorState.mouth ? 'ON' : 'OFF';
            mouthEl.style.color = motorState.mouth ? 'var(--accent-green)' : 'var(--text-muted)';
        }
        if (neckEl) {
            neckEl.textContent = motorState.neck ? 'ON' : 'OFF';
            neckEl.style.color = motorState.neck ? 'var(--accent-green)' : 'var(--text-muted)';
        }
        if (renderer) renderer.setHardwareState(motorState.mouth, motorState.neck);
        if (calibRenderer) calibRenderer.setHardwareState(motorState.mouth, motorState.neck);
    }

    // ---------------------------------------------------------------------
    // Export & print
    // ---------------------------------------------------------------------
    function bindExport() {
        const exportBtn = $('btnExportScript');
        if (exportBtn) exportBtn.addEventListener('click', function () {
            const result = computeCurrent(performance.now());
            const angles = kinematics.servoAngles(withTrims(result.joints));
            fetch('/api/export-python', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ angles: angles })
            }).then(function (r) { return r.blob(); }).then(function (blob) {
                const url = URL.createObjectURL(blob);
                const a = document.createElement('a');
                a.href = url;
                a.download = 'biped_17dof_control.py';
                a.click();
                URL.revokeObjectURL(url);
            }).catch(function () {});
        });

        const wiringPDF = $('btnExportWiringPDF');
        if (wiringPDF) wiringPDF.addEventListener('click', function () {
            const el = $('viewWiring');
            if (!el || typeof html2pdf === 'undefined') {
                window.print();
                return;
            }
            document.body.classList.add('exporting-pdf');
            html2pdf().set({
                margin: 5,
                filename: 'biped_17dof_wiring.pdf',
                image: { type: 'jpeg', quality: 0.95 },
                html2canvas: { scale: 2, useCORS: true },
                jsPDF: { unit: 'mm', format: 'a4', orientation: 'landscape' }
            }).from(el).save().then(function () {
                document.body.classList.remove('exporting-pdf');
            }).catch(function () {
                document.body.classList.remove('exporting-pdf');
            });
        });

        const printBtn = $('btnPrintWiringPDF');
        if (printBtn) printBtn.addEventListener('click', function () { window.print(); });
    }

    // ---------------------------------------------------------------------
    // Boot
    // ---------------------------------------------------------------------
    function boot() {
        renderer = new Biped3DRenderer('viewport3d');
        calibRenderer = new Biped3DRenderer('calibViewport3d');

        bindUI();
        bindMotors();
        loadServos();
        loadStatus();

        // Refresh status occasionally
        setInterval(loadStatus, 10000);
        setInterval(loadAudioList, 30000);

        requestAnimationFrame(animate);
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', boot);
    } else {
        boot();
    }
})();
