# Biped 17-DOF Humanoid Control Center

A standalone Raspberry Pi web application for a 17-servo biped humanoid. It
brings the full Shobots Hexapod feature set to a two-legged robot: a Three.js
3D workbench, live servo calibration, Gemini multimodal AI, camera/gesture
control, speech synthesis, MP3 playback with dance sync, ultrasonic telemetry,
L298 accessory control and one-click Python export.

- **Local URL:** http://localhost:2090
- **Public URL:** https://biped.shobots.party
- **Shobots menu:** https://shobots.party
- **Project path:** `/home/markirgang/biped`

## 17 Degrees of Freedom

| # | Joint | Board | Channel | Group |
|---|-------|-------|---------|-------|
| 0 | Head Pan | 0x40 | 0 | Head |
| 1 | Head Tilt | 0x40 | 1 | Head |
| 2 | Waist Yaw | 0x40 | 2 | Torso |
| 3 | L Shoulder Pitch | 0x40 | 3 | Left Arm |
| 4 | L Shoulder Roll | 0x40 | 4 | Left Arm |
| 5 | L Elbow | 0x40 | 5 | Left Arm |
| 6 | R Shoulder Pitch | 0x40 | 6 | Right Arm |
| 7 | R Shoulder Roll | 0x40 | 7 | Right Arm |
| 8 | R Elbow | 0x40 | 8 | Right Arm |
| 9 | L Hip Yaw | 0x41 | 0 | Left Leg |
| 10 | L Hip Pitch | 0x41 | 1 | Left Leg |
| 11 | L Knee | 0x41 | 2 | Left Leg |
| 12 | L Ankle | 0x41 | 3 | Left Leg |
| 13 | R Hip Yaw | 0x41 | 4 | Right Leg |
| 14 | R Hip Pitch | 0x41 | 5 | Right Leg |
| 15 | R Knee | 0x41 | 6 | Right Leg |
| 16 | R Ankle | 0x41 | 7 | Right Leg |

**2 head + 1 waist + 6 arms + 8 legs = 17 servos.**

## Features

- **3D control workbench** — procedural 17-joint humanoid, orbit camera, body
  height/pitch/roll/yaw, walking, marching, turning and running gaits.
- **Servo calibration** — per-joint trim sliders, live 3D preview and test gait
  buttons. Trims are applied to live motion and exported code.
- **Gemini AI** — text chat, camera scene description, voice input and hand
  gesture actions (`biped_walk`, `biped_pose`, `biped_stop`).
- **Camera** — USB webcam or Raspberry Pi CSI camera via Picamera2, with a
  synthetic fallback frame when no camera is attached.
- **Audio** — MP3 import/playback, browser or local speaker output, gTTS speech
  synthesis and music-synced dance poses.
- **Sensors & accessories** — 4x HC-SR04 telemetry, obstacle avoidance and an
  L298N dual H-bridge for a mouth and pulsating-neck motor.
- **Python export** — downloads a ready-to-run `biped_17dof_control.py` script
  for both PCA9685 boards, the L298N and the ultrasonic sensors.

## Quick Start

```bash
cd /home/markirgang/biped
python3 server.py --port 2090 --host 0.0.0.0
```

Then open http://localhost:2090. The server also accepts HTTP and HTTPS on the
same port when a certificate is available.

## Hardware

- Raspberry Pi 4/5
- 2x PCA9685 16-channel PWM drivers (I2C addresses `0x40` and `0x41`)
- 17x servos
- 5V 10A UBEC for servo power (never power the servos from the Pi 5V rail)
- L298N dual H-bridge (optional mouth / neck motors)
- 4x HC-SR04 ultrasonic sensors (front, rear, left, right)
- USB webcam or Raspberry Pi CSI camera

Wiring details, GPIO pinouts and power/safety notes are available in the
**Wiring Schematic** tab of the web application.

## API Overview

| Method | Endpoint | Purpose |
|--------|----------|---------|
| GET | `/api/status` | Server, DOF and hardware status |
| GET | `/api/servos` | 17-servo map, state, calibration and body state |
| GET | `/api/poses` | Saved pose library |
| GET | `/api/sensors/distances` | Ultrasonic distances |
| GET | `/api/camera/frame` | JPEG camera frame |
| GET | `/api/audio/list` | MP3 files in `static/audio/` |
| GET | `/api/tts/speak?text=...` | Speech synthesis MP3 |
| POST | `/api/servo/set` | Set joint angles and body attitude |
| POST | `/api/pose` | Apply a named pose |
| POST | `/api/calibration` | Save servo trim offsets |
| POST | `/api/motor/l298` | Toggle mouth / neck motors |
| POST | `/api/gemini/chat` | Gemini text + vision chat |
| POST | `/api/gemini/voice` | Gemini voice chat |
| POST | `/api/gemini/gesture` | Hand gesture detection |
| POST | `/api/export-python` | Download hardware control script |
| POST | `/api/audio/upload` | Upload an MP3 file |

## Shobots Integration

The biped is registered in the Shobots master menu (`Shobots/config.json` and
`Shobots/server.py`) and in the Cloudflare tunnel ingress as
`biped.shobots.party -> http://localhost:2090`.

Start/stop it from the Shobots dashboard, or directly:

```bash
curl -X POST http://127.0.0.1:7090/api/start  -d '{"robot":"biped"}' -H 'Content-Type: application/json'
curl -X POST http://127.0.0.1:7090/api/stop   -d '{"robot":"biped"}' -H 'Content-Type: application/json'
curl -X POST http://127.0.0.1:7090/api/restart -d '{"robot":"biped"}' -H 'Content-Type: application/json'
```

## Notes

- The server runs in simulation mode when PCA9685 / RPi.GPIO libraries are not
  available. Set up the Adafruit CircuitPython PCA9685 and `RPi.GPIO` packages
  on the robot for real servo output.
- Servo calibration trims are stored in memory for the current run. Export the
  Python script after calibrating to persist the final angles in code.
- Keep all grounds (Pi, PCA9685 boards, UBEC, L298N) tied to a common star
  point and add a large capacitor across the servo power rail.
