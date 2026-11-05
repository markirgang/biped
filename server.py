#!/usr/bin/env python3
"""Biped 17-DOF Humanoid Control Server.

A standalone control hub for a 17-servo biped robot. It provides the same
feature set as the Shobots Hexapod application (3D workbench, servo
calibration, camera, Gemini multimodal AI, TTS, MP3 playback, ultrasonic
telemetry, L298 accessories and Python export) with a 17-DOF humanoid model.

Local port: 2090
Public host: https://biped.shobots.party
"""
import http.server
import socketserver
import json
import os
import sys
import socket
import ssl
import base64
import threading
import time
import random
import math
import urllib.parse
import re
import local_audio
from gemini_bridge import GeminiBipedBridge

try:
    import cv2
    import numpy as np
except Exception:  # pragma: no cover - camera is optional on non-Pi hosts
    cv2 = None
    np = None

PORT = int(os.environ.get("PORT", 2090))
HOST = os.environ.get("HOST", os.environ.get("IP", "0.0.0.0"))
DIRECTORY = os.environ.get("DIRECTORY", os.path.join(os.path.dirname(os.path.abspath(__file__)), "static"))
AUDIO_DIR = os.path.join(DIRECTORY, "audio")
os.makedirs(AUDIO_DIR, exist_ok=True)

DOF = 17

# ---------------------------------------------------------------------------
# 17 Servo map
#   Board 0x40: head (2), waist (1), both arms (6) = 9 channels
#   Board 0x41: both legs (8)                     = 8 channels
# ---------------------------------------------------------------------------
SERVOS = [
    {"id": 0,  "name": "head_pan",         "label": "Head Pan",          "group": "Head",       "board": "0x40", "channel": 0,  "min": -80, "max": 80,  "home": 0},
    {"id": 1,  "name": "head_tilt",        "label": "Head Tilt",         "group": "Head",       "board": "0x40", "channel": 1,  "min": -40, "max": 40,  "home": 0},
    {"id": 2,  "name": "waist_yaw",        "label": "Waist Yaw",         "group": "Torso",      "board": "0x40", "channel": 2,  "min": -60, "max": 60,  "home": 0},
    {"id": 3,  "name": "l_shoulder_pitch", "label": "L Shoulder Pitch",  "group": "Left Arm",   "board": "0x40", "channel": 3,  "min": -170, "max": 60, "home": 0},
    {"id": 4,  "name": "l_shoulder_roll",  "label": "L Shoulder Roll",   "group": "Left Arm",   "board": "0x40", "channel": 4,  "min": -20, "max": 110, "home": 0},
    {"id": 5,  "name": "l_elbow",          "label": "L Elbow",           "group": "Left Arm",   "board": "0x40", "channel": 5,  "min": 0,   "max": 140, "home": 10},
    {"id": 6,  "name": "r_shoulder_pitch", "label": "R Shoulder Pitch",  "group": "Right Arm",  "board": "0x40", "channel": 6,  "min": -170, "max": 60, "home": 0},
    {"id": 7,  "name": "r_shoulder_roll",  "label": "R Shoulder Roll",   "group": "Right Arm",  "board": "0x40", "channel": 7,  "min": -110, "max": 20, "home": 0},
    {"id": 8,  "name": "r_elbow",          "label": "R Elbow",           "group": "Right Arm",  "board": "0x40", "channel": 8,  "min": 0,   "max": 140, "home": 10},
    {"id": 9,  "name": "l_hip_yaw",        "label": "L Hip Yaw",         "group": "Left Leg",   "board": "0x41", "channel": 0,  "min": -45, "max": 45,  "home": 0},
    {"id": 10, "name": "l_hip_pitch",      "label": "L Hip Pitch",       "group": "Left Leg",   "board": "0x41", "channel": 1,  "min": -90, "max": 90,  "home": 0},
    {"id": 11, "name": "l_knee",           "label": "L Knee",            "group": "Left Leg",   "board": "0x41", "channel": 2,  "min": -120, "max": 5,  "home": 5},
    {"id": 12, "name": "l_ankle",          "label": "L Ankle",           "group": "Left Leg",   "board": "0x41", "channel": 3,  "min": -40, "max": 40,  "home": -5},
    {"id": 13, "name": "r_hip_yaw",        "label": "R Hip Yaw",         "group": "Right Leg",  "board": "0x41", "channel": 4,  "min": -45, "max": 45,  "home": 0},
    {"id": 14, "name": "r_hip_pitch",      "label": "R Hip Pitch",       "group": "Right Leg",  "board": "0x41", "channel": 5,  "min": -90, "max": 90,  "home": 0},
    {"id": 15, "name": "r_knee",           "label": "R Knee",            "group": "Right Leg",  "board": "0x41", "channel": 6,  "min": -120, "max": 5,  "home": 5},
    {"id": 16, "name": "r_ankle",          "label": "R Ankle",           "group": "Right Leg",  "board": "0x41", "channel": 7,  "min": -40, "max": 40,  "home": -5},
]
SERVO_BY_NAME = {s["name"]: s for s in SERVOS}

NEUTRAL = {s["name"]: s["home"] for s in SERVOS}
servo_state = dict(NEUTRAL)
calibration_trims = {s["name"]: 0 for s in SERVOS}
body_state = {"height": 0, "roll": 0, "pitch": 0, "yaw": 0, "stance": 0}
state_lock = threading.Lock()


def _pose(joints=None, height=0, roll=0, pitch=0, yaw=0, description=""):
    pose = dict(NEUTRAL)
    if joints:
        pose.update(joints)
    return {
        "body": {"height": height, "roll": roll, "pitch": pitch, "yaw": yaw},
        "joints": pose,
        "description": description,
    }


SAVED_POSES = {
    "Stand": _pose(description="Neutral standing pose with both feet planted"),
    "T-Pose": _pose({
        "l_shoulder_roll": 80, "r_shoulder_roll": -80,
        "l_elbow": 0, "r_elbow": 0,
    }, description="Both arms extended horizontally"),
    "Wave Right Hand": _pose({
        "r_shoulder_pitch": -120, "r_shoulder_roll": -35, "r_elbow": 55,
        "head_pan": 20, "head_tilt": 8,
    }, description="Raises the right hand and waves hello"),
    "Wave Left Hand": _pose({
        "l_shoulder_pitch": -120, "l_shoulder_roll": 35, "l_elbow": 55,
        "head_pan": -20, "head_tilt": 8,
    }, description="Raises the left hand and waves hello"),
    "Both Arms Up": _pose({
        "l_shoulder_pitch": -160, "r_shoulder_pitch": -160,
        "l_shoulder_roll": 15, "r_shoulder_roll": -15,
        "l_elbow": 10, "r_elbow": 10, "head_tilt": -10,
    }, description="Victory celebration with both arms raised"),
    "Dance": _pose({
        "l_shoulder_pitch": -70, "r_shoulder_pitch": -20,
        "l_shoulder_roll": 55, "r_shoulder_roll": -55,
        "l_elbow": 70, "r_elbow": 40, "waist_yaw": 20,
    }, height=15, roll=10, yaw=15, description="Rhythmic dance groove"),
    "Squat": _pose({
        "l_hip_pitch": -55, "r_hip_pitch": -55,
        "l_knee": 95, "r_knee": 95,
        "l_ankle": -40, "r_ankle": -40,
        "l_shoulder_pitch": -40, "r_shoulder_pitch": -40,
    }, height=-70, description="Deep knee bend squat"),
    "Bow / Respect": _pose({
        "waist_yaw": 0, "l_shoulder_pitch": 30, "r_shoulder_pitch": 30,
        "head_tilt": 25,
    }, pitch=30, height=-25, description="Polite forward bow"),
    "Sit / Park": _pose({
        "l_hip_pitch": -80, "r_hip_pitch": -80,
        "l_knee": 90, "r_knee": 90,
        "l_ankle": 10, "r_ankle": 10,
        "l_shoulder_pitch": 20, "r_shoulder_pitch": 20,
    }, height=-150, description="Seated resting pose"),
    "Sleep / Park": _pose({
        "l_hip_pitch": -40, "r_hip_pitch": -40,
        "l_knee": 100, "r_knee": 100,
        "l_ankle": -20, "r_ankle": -20,
        "head_tilt": 30,
    }, height=-240, description="Folded low-power sleep pose"),
    "Curious Tilt": _pose({
        "head_pan": 18, "head_tilt": 22, "waist_yaw": 8,
    }, roll=8, description="Curious head tilt"),
    "Greeting Wave": _pose({
        "r_shoulder_pitch": -130, "r_shoulder_roll": -30, "r_elbow": 45,
        "head_tilt": -8, "waist_yaw": -10,
    }, description="Friendly greeting wave"),
    "Low Stance": _pose({
        "l_hip_pitch": -30, "r_hip_pitch": -30,
        "l_knee": 55, "r_knee": 55,
        "l_ankle": -25, "r_ankle": -25,
    }, height=-55, description="Crouched defensive stance"),
    "High Stance": _pose({
        "l_hip_pitch": 10, "r_hip_pitch": 10,
        "l_knee": 0, "r_knee": 0,
        "l_ankle": 0, "r_ankle": 0,
    }, height=35, description="Elevated observation stance"),
    "Attack / Guard": _pose({
        "l_shoulder_pitch": -80, "r_shoulder_pitch": -80,
        "l_elbow": 90, "r_elbow": 90,
        "l_shoulder_roll": 25, "r_shoulder_roll": -25,
        "head_tilt": -12,
    }, pitch=12, height=-20, description="Guard / combat ready posture"),
    "Walk in Place": _pose(description="Alternating marching gait in place"),
}


def generate_synthetic_camera_frame():
    if cv2 is None or np is None:
        return None
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:, :] = (30, 25, 20)
    cv2.putText(frame, "BIPED 17-DOF CAMERA FEED", (95, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 240, 255), 2)
    cv2.putText(frame, "STATUS: Virtual Camera Active", (160, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 255, 160), 1)
    cv2.putText(frame, "LIVE TIME: " + time.strftime("%H:%M:%S"), (220, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    return frame


class CameraStream:
    """Thread-safe USB / CSI camera stream with synthetic fallback."""

    def __init__(self, source="usb"):
        self.source = str(source).lower()
        self.cap = None
        self.picam2 = None
        self.frame = None
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def set_source(self, source):
        with self.lock:
            self.source = str(source).lower()
            self._close_cameras()

    def _close_cameras(self):
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
        if self.picam2:
            try:
                self.picam2.stop()
                self.picam2.close()
            except Exception:
                pass
            self.picam2 = None

    def _update(self):
        while self.running:
            if cv2 is None:
                time.sleep(1.0)
                continue
            with self.lock:
                cur_source = self.source

            if cur_source in ["csi", "picamera", "raspberry_pi", "ov5647"]:
                if self.picam2 is None:
                    self._close_cameras()
                    try:
                        from picamera2 import Picamera2
                        p = Picamera2()
                        p.configure(p.create_preview_configuration(main={"size": (640, 480)}))
                        p.start()
                        time.sleep(0.3)
                        self.picam2 = p
                        print("Picamera2 CSI Camera initialized successfully")
                    except Exception as e:
                        print("Picamera2 CSI Camera init error:", e)
                        self.picam2 = None
                        time.sleep(1.0)
                        continue
                try:
                    frame_array = self.picam2.capture_array("main")
                    if frame_array is not None and frame_array.size > 0:
                        if frame_array.shape[2] == 4:
                            frame_array = cv2.cvtColor(frame_array, cv2.COLOR_RGBA2BGR)
                        with self.lock:
                            self.frame = frame_array
                    time.sleep(0.04)
                except Exception as e:
                    print("CSI frame capture error:", e)
                    self._close_cameras()
                    time.sleep(0.5)
            else:
                if self.cap is None or not self.cap.isOpened():
                    self._close_cameras()
                    candidates = [0, 1, 6, 2, 8]
                    if cur_source.isdigit():
                        candidates.insert(0, int(cur_source))
                    candidates = list(dict.fromkeys(candidates))
                    opened = False
                    for idx in candidates:
                        try:
                            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                            if cap.isOpened():
                                ret, frame = cap.read()
                                if ret and frame is not None and frame.size > 0:
                                    self.cap = cap
                                    with self.lock:
                                        self.frame = frame
                                    opened = True
                                    break
                                cap.release()
                        except Exception:
                            pass
                    if not opened:
                        time.sleep(4.0)
                if self.cap and self.cap.isOpened():
                    try:
                        ret, frame = self.cap.read()
                        if ret and frame is not None and frame.size > 0:
                            with self.lock:
                                self.frame = frame
                            time.sleep(0.04)
                        else:
                            self._close_cameras()
                            time.sleep(1.0)
                    except Exception:
                        self._close_cameras()
                        time.sleep(1.0)
                else:
                    time.sleep(0.5)

    def get_frame(self):
        with self.lock:
            if self.frame is not None:
                return self.frame.copy()
        return generate_synthetic_camera_frame()


camera_stream = CameraStream("usb")
gemini_bridge = GeminiBipedBridge()

motor_state = {"mouth_channel1": False, "neck_channel2": False}
manual_motor_state = {"mouth": False, "neck": False}
sensor_state = {"front": 85, "rear": 140, "left": 70, "right": 75}

# ---------------------------------------------------------------------------
# Animatronic talking-mouth robot (no body, front edge of the stage)
#   L298 channel 1 -> mouth motor (energized = mouth opens)
#   L298 channel 2 -> neck motor  (energized = neck rises/falls)
# The animator drives the channels so the mouth flaps while the AI talks and
# the neck bobs occasionally while idle, talking or dancing.
# ---------------------------------------------------------------------------
animatronic_lock = threading.Lock()
animatronic_state = {
    "mode": "idle",              # idle | talking | dancing
    "mouth_channel1": False,
    "neck_channel2": False,
    "neck_direction": 1,
    "mouth_open": False,
    "neck_energized": False,
    "updated": time.time(),
}


def update_simulated_sensors():
    while True:
        for key in sensor_state:
            sensor_state[key] = max(10, min(220, sensor_state[key] + random.randint(-2, 2)))
        time.sleep(0.5)


threading.Thread(target=update_simulated_sensors, daemon=True).start()


# ---------------------------------------------------------------------------
# Optional real hardware bridge (PCA9685 + L298N + HC-SR04)
# ---------------------------------------------------------------------------
class HardwareBridge:
    def __init__(self):
        self.enabled = False
        self.pca = {}
        self.servo = None
        self.gpio = None
        self.pins = {}
        self.pwm = {}
        self._init()

    def _init(self):
        try:
            import board
            import busio
            from adafruit_pca9685 import PCA9685
            from adafruit_motor import servo as servo_lib
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca["0x40"] = PCA9685(i2c, address=0x40)
            self.pca["0x41"] = PCA9685(i2c, address=0x41)
            for p in self.pca.values():
                p.frequency = 50
            self.servo = servo_lib
            self.enabled = True
            print("PCA9685 servo drivers initialized (0x40 / 0x41)")
        except Exception as e:
            print("PCA9685 unavailable, running in simulation:", e)
            return
        try:
            import RPi.GPIO as GPIO
            self.gpio = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            self.pins = {
                "ENA": 18, "IN1": 23, "IN2": 24,
                "ENB": 19, "IN3": 25, "IN4": 8,
            }
            GPIO.setup(list(self.pins.values()), GPIO.OUT)
            self.pwm["mouth"] = GPIO.PWM(self.pins["ENA"], 1000)
            self.pwm["neck"] = GPIO.PWM(self.pins["ENB"], 1000)
            self.pwm["mouth"].start(0)
            self.pwm["neck"].start(0)
            print("L298N motor driver initialized")
        except Exception as e:
            print("L298N unavailable, running in simulation:", e)

    def write_servo(self, cfg, logical_angle):
        if not self.enabled:
            return
        try:
            board = self.pca[cfg["board"]]
            min_pulse, max_pulse = 500, 2500
            servo_angle = max(0.0, min(180.0, 90.0 + float(logical_angle)))
            s = self.servo.Servo(board.channels[cfg["channel"]], min_pulse=min_pulse, max_pulse=max_pulse)
            s.angle = servo_angle
        except Exception:
            pass

    def set_motors(self, mouth, neck, neck_dir=1):
        if self.gpio is None:
            return
        try:
            if mouth:
                self.gpio.output(self.pins["IN1"], self.gpio.HIGH)
                self.gpio.output(self.pins["IN2"], self.gpio.LOW)
                self.pwm["mouth"].ChangeDutyCycle(100)
            else:
                self.pwm["mouth"].ChangeDutyCycle(0)
            if neck:
                if neck_dir >= 0:
                    self.gpio.output(self.pins["IN3"], self.gpio.HIGH)
                    self.gpio.output(self.pins["IN4"], self.gpio.LOW)
                else:
                    self.gpio.output(self.pins["IN3"], self.gpio.LOW)
                    self.gpio.output(self.pins["IN4"], self.gpio.HIGH)
                self.pwm["neck"].ChangeDutyCycle(80)
            else:
                self.pwm["neck"].ChangeDutyCycle(0)
        except Exception:
            pass


hardware = HardwareBridge()
_last_motor_output = (False, False, 1)


def apply_motor_outputs(force=False):
    """Combine manual L298 toggles with the animatronic pattern."""
    global _last_motor_output
    with animatronic_lock:
        mouth = bool(manual_motor_state["mouth"] or animatronic_state["mouth_channel1"])
        neck = bool(manual_motor_state["neck"] or animatronic_state["neck_channel2"])
        neck_dir = animatronic_state["neck_direction"]
        motor_state["mouth_channel1"] = mouth
        motor_state["neck_channel2"] = neck
        animatronic_state["mouth_open"] = mouth
        animatronic_state["neck_energized"] = neck
        animatronic_state["updated"] = time.time()
    output = (mouth, neck, neck_dir)
    if force or output != _last_motor_output:
        _last_motor_output = output
        hardware.set_motors(mouth, neck, neck_dir)


class MouthRobotAnimator(threading.Thread):
    """Drives the talking mouth robot's L298 channels.

    talking  - rapid mouth flapping with occasional neck nods
    dancing  - mouth/neck pulses locked to a dance beat
    idle     - closed mouth with an occasional neck bob while waiting
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.started = time.time()
        self.last_mouth_toggle = 0.0
        self.mouth_phase_on = False
        self.next_neck = self.started + random.uniform(2.5, 5.0)
        self.neck_until = 0.0
        self.neck_on_since = 0.0

    def run(self):
        while True:
            now = time.time()
            with animatronic_lock:
                mode = animatronic_state["mode"]
                mouth_auto = False
                neck_auto = False

                if mode == "talking":
                    if now - self.last_mouth_toggle >= 0.13:
                        self.last_mouth_toggle = now
                        self.mouth_phase_on = not self.mouth_phase_on
                    mouth_auto = self.mouth_phase_on
                    if now >= self.next_neck:
                        self.neck_until = now + random.uniform(0.20, 0.34)
                        self.next_neck = now + random.uniform(1.2, 2.6)
                        animatronic_state["neck_direction"] *= -1
                    neck_auto = now < self.neck_until

                elif mode == "dancing":
                    beat = now - self.started
                    mouth_auto = math.sin(beat * math.pi * 4.0) > 0.1
                    if now >= self.next_neck:
                        self.neck_until = now + 0.22
                        self.next_neck = now + 0.5
                        animatronic_state["neck_direction"] *= -1
                    neck_auto = now < self.neck_until

                else:  # idle - wait for commands with the occasional neck bob
                    mouth_auto = False
                    if now >= self.next_neck:
                        self.neck_until = now + random.uniform(0.18, 0.40)
                        self.next_neck = now + random.uniform(3.5, 8.0)
                        animatronic_state["neck_direction"] *= -1
                    neck_auto = now < self.neck_until

                animatronic_state["mouth_channel1"] = mouth_auto
                animatronic_state["neck_channel2"] = neck_auto

                # A held neck motor must reverse periodically so the neck
                # travels up and down instead of stalling at one limit.
                if neck_auto or manual_motor_state["neck"]:
                    if self.neck_on_since == 0.0:
                        self.neck_on_since = now
                    elif now - self.neck_on_since >= 0.8:
                        animatronic_state["neck_direction"] *= -1
                        self.neck_on_since = now
                else:
                    self.neck_on_since = 0.0

            apply_motor_outputs()
            time.sleep(0.03)


def set_animatronic_mode(mode):
    mode = mode if mode in ("idle", "talking", "dancing") else "idle"
    with animatronic_lock:
        animatronic_state["mode"] = mode
        animatronic_state["updated"] = time.time()
    apply_motor_outputs(force=True)
    return mode


animatronic_animator = MouthRobotAnimator()
animatronic_animator.start()


def apply_servo_state():
    with state_lock:
        snapshot = dict(servo_state)
    for cfg in SERVOS:
        hardware.write_servo(cfg, snapshot.get(cfg["name"], cfg["home"]))


class BipedRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def end_headers(self):
        if not self.path.startswith("/api/"):
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
        super().end_headers()

    def do_HEAD(self):
        if self.path.startswith("/api/"):
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.end_headers()
        else:
            super().do_HEAD()

    def do_GET(self):
        if self.path.startswith("/api/status"):
            self.send_json({
                "status": "online",
                "device": "Raspberry Pi",
                "robot": "Biped",
                "dof": DOF,
                "servo_count": DOF,
                "hardware_connected": hardware.enabled,
                "python_version": sys.version,
            })
        elif self.path.startswith("/api/poses"):
            self.send_json(SAVED_POSES)
        elif self.path.startswith("/api/servos"):
            with state_lock:
                self.send_json({
                    "servos": SERVOS,
                    "state": dict(servo_state),
                    "calibration": dict(calibration_trims),
                    "body": dict(body_state),
                    "dof": DOF,
                })
        elif self.path.startswith("/api/servo/state"):
            with state_lock:
                self.send_json({"state": dict(servo_state), "body": dict(body_state)})
        elif self.path.startswith("/api/audio/list"):
            try:
                files = [f for f in os.listdir(AUDIO_DIR) if f.lower().endswith((".mp3", ".wav", ".ogg", ".m4a", ".flac"))]
                files.sort()
                self.send_json({"files": files})
            except Exception as e:
                self.send_json({"files": [], "error": str(e)})
        elif self.path.startswith("/api/camera/frame"):
            self.serve_camera_frame()
        elif self.path.startswith("/api/sensors/distances"):
            self.send_json(sensor_state)
        elif self.path.startswith("/api/motor/state"):
            self.send_json(motor_state)
        elif self.path.startswith("/api/animatronic/state"):
            with animatronic_lock:
                payload = dict(animatronic_state)
                payload.update({
                    "manual_mouth": manual_motor_state["mouth"],
                    "manual_neck": manual_motor_state["neck"],
                    "mouth_channel1": motor_state["mouth_channel1"],
                    "neck_channel2": motor_state["neck_channel2"],
                })
            self.send_json(payload)
        elif self.path.startswith("/api/tts/speak"):
            self.handle_tts()
        elif self.path.startswith("/api/audio/outputs") or self.path.startswith("/api/audio/sinks"):
            self.send_json({"outputs": local_audio.list_local_audio_outputs()})
        else:
            super().do_GET()

    def do_POST(self):
        content_length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_length).decode("utf-8") if content_length > 0 else "{}"
        try:
            data = json.loads(body)
        except json.JSONDecodeError:
            data = {}

        if self.path in ["/api/audio/play", "/api/audio/play_local"]:
            audio_bytes, content_type = local_audio.load_audio_bytes_from_url(data.get("url", ""))
            if not audio_bytes:
                self.send_json({"error": "Unable to load audio for local playback"}, status=500)
            else:
                local_audio.play_audio_on_local_output(
                    audio_bytes, content_type,
                    data.get("speaker", "default"), data.get("volume", 100)
                )
                self.send_json({"success": True, "speaker": data.get("speaker", "default")})
            return

        if self.path == "/api/servo/set":
            angles = data.get("angles", {})
            with state_lock:
                for name, value in angles.items():
                    cfg = SERVO_BY_NAME.get(name)
                    if not cfg:
                        continue
                    try:
                        value = float(value) + float(calibration_trims.get(name, 0))
                    except Exception:
                        continue
                    servo_state[name] = max(cfg["min"], min(cfg["max"], value))
                if "body" in data and isinstance(data["body"], dict):
                    for key in ("height", "roll", "pitch", "yaw", "stance"):
                        if key in data["body"]:
                            try:
                                body_state[key] = float(data["body"][key])
                            except Exception:
                                pass
            apply_servo_state()
            with state_lock:
                self.send_json({"success": True, "state": dict(servo_state), "body": dict(body_state)})

        elif self.path == "/api/pose":
            name = data.get("name", "Stand")
            pose = SAVED_POSES.get(name)
            if not pose:
                self.send_json({"success": False, "error": "Unknown pose"}, status=404)
                return
            with state_lock:
                for key, value in pose.get("joints", {}).items():
                    if key in servo_state:
                        servo_state[key] = value
                for key, value in pose.get("body", {}).items():
                    if key in body_state:
                        body_state[key] = value
            apply_servo_state()
            self.send_json({"success": True, "pose": name, "state": dict(servo_state)})

        elif self.path == "/api/calibration":
            trims = data.get("calibration", data.get("trims", {}))
            with state_lock:
                for name, value in trims.items():
                    if name in calibration_trims:
                        try:
                            calibration_trims[name] = float(value)
                        except Exception:
                            pass
            self.send_json({"success": True, "calibration": dict(calibration_trims)})

        elif self.path == "/api/motor/l298":
            with animatronic_lock:
                manual_motor_state["mouth"] = bool(data.get("mouth", False))
                manual_motor_state["neck"] = bool(data.get("neck", False))
            apply_motor_outputs(force=True)
            self.send_json({"success": True, "state": dict(motor_state)})

        elif self.path == "/api/animatronic/mode":
            mode = set_animatronic_mode(data.get("mode", "idle"))
            with animatronic_lock:
                payload = dict(animatronic_state)
            payload["success"] = True
            self.send_json(payload)

        elif self.path == "/api/animatronic/mouth":
            # Direct speech-envelope override: open/close the mouth motor now.
            with animatronic_lock:
                animatronic_state["mouth_channel1"] = bool(data.get("open", False))
                animatronic_state["updated"] = time.time()
            apply_motor_outputs()
            self.send_json({"success": True, "mouth_open": animatronic_state["mouth_channel1"]})

        elif self.path == "/api/camera/select":
            source = data.get("source", "usb")
            camera_stream.set_source(source)
            self.send_json({"success": True, "active_source": source})

        elif self.path == "/api/gemini/chat":
            user_text = data.get("prompt", "")
            DEFAULT_GEMINI_KEY = base64.b64decode("QVEuQWI4Uk42SU9GSDQ4UXNyWWNLLWEzUnp0OFc5RVZiZG0yTHJWRmlnQlE1Y0RySTBjaHc=").decode("utf-8")
            api_key = data.get("api_key") or os.environ.get("GEMINI_API_KEY") or DEFAULT_GEMINI_KEY
            include_vision = data.get("include_vision", True)
            image_bytes = self.capture_camera_bytes() if include_vision else None
            self.send_json(gemini_bridge.generate_chat_response(api_key, user_text, image_bytes))

        elif self.path == "/api/gemini/voice":
            DEFAULT_GEMINI_KEY = base64.b64decode("QVEuQWI4Uk42SU9GSDQ4UXNyWWNLLWEzUnp0OFc5RVZiZG0yTHJWRmlnQlE1Y0RySTBjaHc=").decode("utf-8")
            api_key = data.get("api_key") or os.environ.get("GEMINI_API_KEY") or DEFAULT_GEMINI_KEY
            audio_b64 = data.get("audio_b64", "")
            mime_type = data.get("mime_type", "audio/webm")
            include_vision = data.get("include_vision", True)
            audio_bytes = base64.b64decode(audio_b64) if audio_b64 else None
            image_bytes = self.capture_camera_bytes() if include_vision else None
            self.send_json(gemini_bridge.generate_voice_chat_response(api_key, audio_bytes, mime_type, image_bytes))

        elif self.path == "/api/gemini/gesture":
            frame = self.capture_camera_frame()
            if frame is None:
                self.send_json({"gesture": "None", "finger_count": 0, "action": None})
                return
            gesture, finger_count, _ = gemini_bridge.detect_hand_gestures(frame)
            action = None
            if gesture == "5_FINGERS_FORWARD":
                action = {"name": "biped_walk", "args": {"direction": "forward"}}
            elif gesture == "2_FINGERS_WAVE":
                action = {"name": "biped_pose", "args": {"pose_name": "Greeting Wave"}}
            elif gesture == "1_FINGER_ATTACK":
                action = {"name": "biped_pose", "args": {"pose_name": "Attack / Guard"}}
            self.send_json({"gesture": gesture, "finger_count": finger_count, "action": action})

        elif self.path == "/api/export-python":
            python_script = self.generate_hardware_script(data)
            self.send_response(200)
            self.send_header("Content-type", "application/x-python")
            self.send_header("Content-Disposition", 'attachment; filename="biped_17dof_control.py"')
            self.end_headers()
            self.wfile.write(python_script.encode("utf-8"))

        elif self.path == "/api/save-pose":
            pose_name = data.get("name", "Custom Pose")
            pose = data.get("pose", {})
            SAVED_POSES[pose_name] = pose
            self.send_json({"success": True, "message": "Saved pose '%s'" % pose_name})

        elif self.path == "/api/audio/upload":
            filename = data.get("filename", "track.mp3")
            data_b64 = data.get("data_b64", "")
            safe_filename = re.sub(r"[^a-zA-Z0-9_\-\.]", "_", filename)
            if not safe_filename.lower().endswith((".mp3", ".wav", ".ogg", ".m4a", ".flac")):
                safe_filename += ".mp3"
            try:
                raw_bytes = base64.b64decode(data_b64)
                with open(os.path.join(AUDIO_DIR, safe_filename), "wb") as f:
                    f.write(raw_bytes)
                self.send_json({"success": True, "filename": safe_filename, "url": "/audio/" + safe_filename})
            except Exception as e:
                self.send_json({"success": False, "error": str(e)}, status=500)

        elif self.path == "/api/audio/delete":
            filename = os.path.basename(data.get("filename", ""))
            file_path = os.path.join(AUDIO_DIR, filename)
            if os.path.exists(file_path):
                try:
                    os.remove(file_path)
                    self.send_json({"success": True, "message": "Deleted " + filename})
                except Exception as e:
                    self.send_json({"success": False, "error": str(e)}, status=500)
            else:
                self.send_json({"success": False, "error": "File not found"}, status=404)
        else:
            self.send_json({"error": "Endpoint not found"}, status=404)

    def handle_tts(self):
        query_params = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
        text_to_speak = query_params.get("text", ["Hello!"])[0]
        voice_id = query_params.get("voice", ["ai_female_us"])[0]
        try:
            from gtts import gTTS
            import io
            clean_text = re.sub(r"\*[^*]*\*", "", text_to_speak).strip() or "Hello!"
            tld_map = {
                "ai_female_us": ("en", "com"), "ai_male_us": ("en", "us"),
                "ai_male_uk": ("en", "co.uk"), "ai_male_au": ("en", "com.au"),
                "ai_male_ie": ("en", "ie"), "ai_male_in": ("en", "co.in"),
                "ai_male_za": ("en", "co.za"), "ai_male_ca": ("en", "ca"),
                "ai_female_uk": ("en", "co.uk"), "ai_female_au": ("en", "com.au"),
                "ai_female_ca": ("en", "ca"), "ai_female_in": ("en", "co.in"),
            }
            lang, tld = tld_map.get(voice_id, ("en", "com"))
            tts = gTTS(text=clean_text, lang=lang, tld=tld)
            mp3_fp = io.BytesIO()
            tts.write_to_fp(mp3_fp)
            mp3_fp.seek(0)
            audio_bytes = mp3_fp.read()
            self.send_response(200)
            self.send_header("Content-type", "audio/mpeg")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(audio_bytes)
        except Exception as e:
            self.send_json({"error": str(e)}, status=500)

    def capture_camera_frame(self):
        return camera_stream.get_frame()

    def capture_camera_bytes(self):
        if cv2 is None:
            return None
        frame = self.capture_camera_frame()
        if frame is None:
            frame = generate_synthetic_camera_frame()
        if frame is None:
            return None
        ret, jpeg = cv2.imencode(".jpg", frame)
        return jpeg.tobytes() if ret else None

    def serve_camera_frame(self):
        jpeg_bytes = self.capture_camera_bytes()
        if jpeg_bytes is None:
            try:
                self.send_response(503)
                self.end_headers()
            except Exception:
                pass
            return
        try:
            self.send_response(200)
            self.send_header("Content-type", "image/jpeg")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
            self.wfile.write(jpeg_bytes)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def send_json(self, data, status=200):
        try:
            self.send_response(status)
            self.send_header("Content-type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(data).encode("utf-8"))
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def generate_hardware_script(self, data):
        angles = data.get("angles", {})
        servos_literal = json.dumps(SERVOS, indent=4)
        script = '''#!/usr/bin/env python3
"""
Biped 17-DOF Hardware Control Script
Generated by the Biped Control Center.

Servo bus:
  PCA9685 #1 (0x40): head pan/tilt, waist, both arms  (9 servos)
  PCA9685 #2 (0x41): both legs                        (8 servos)
Optional accessories:
  L298N dual H-bridge (mouth + neck)
  4x HC-SR04 ultrasonic sensors (front, rear, left, right)
"""

import time
import random
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

SERVOS = __SERVOS__

L298_ENA, L298_IN1, L298_IN2 = 18, 23, 24
L298_ENB, L298_IN3, L298_IN4 = 19, 25, 8

SENSORS = {
    "front": {"trig": 5, "echo": 6},
    "rear":  {"trig": 12, "echo": 13},
    "left":  {"trig": 16, "echo": 20},
    "right": {"trig": 21, "echo": 26},
}

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([L298_ENA, L298_IN1, L298_IN2, L298_ENB, L298_IN3, L298_IN4], GPIO.OUT)
pwm_mouth = GPIO.PWM(L298_ENA, 1000)
pwm_neck = GPIO.PWM(L298_ENB, 1000)
pwm_mouth.start(0)
pwm_neck.start(0)

for name, pins in SENSORS.items():
    GPIO.setup(pins["trig"], GPIO.OUT)
    GPIO.setup(pins["echo"], GPIO.IN)
    GPIO.output(pins["trig"], False)

i2c = busio.I2C(board.SCL, board.SDA)
pca1 = PCA9685(i2c, address=0x40)
pca1.frequency = 50
pca2 = PCA9685(i2c, address=0x41)
pca2.frequency = 50
BOARDS = {"0x40": pca1, "0x41": pca2}

TARGET_ANGLES = __ANGLES__

def set_servo(name, logical_angle):
    cfg = next((s for s in SERVOS if s["name"] == name), None)
    if not cfg:
        return
    angle = max(0.0, min(180.0, 90.0 + float(logical_angle)))
    s = servo.Servo(BOARDS[cfg["board"]].channels[cfg["channel"]], min_pulse=500, max_pulse=2500)
    s.angle = angle

def apply_pose():
    for name, angle in TARGET_ANGLES.items():
        set_servo(name, angle)

def set_mouth_energized(energized):
    if energized:
        GPIO.output(L298_IN1, GPIO.HIGH)
        GPIO.output(L298_IN2, GPIO.LOW)
        pwm_mouth.ChangeDutyCycle(100)
    else:
        pwm_mouth.ChangeDutyCycle(0)

def set_neck_energized(energized, direction=1):
    """L298 channel 2: energize the talking-mouth robot's neck motor."""
    if energized:
        if direction >= 0:
            GPIO.output(L298_IN3, GPIO.HIGH)
            GPIO.output(L298_IN4, GPIO.LOW)
        else:
            GPIO.output(L298_IN3, GPIO.LOW)
            GPIO.output(L298_IN4, GPIO.HIGH)
        pwm_neck.ChangeDutyCycle(80)
    else:
        pwm_neck.ChangeDutyCycle(0)


def set_neck_pulsating(pulsating):
    set_neck_energized(pulsating)


def animate_talking_mouth(duration=5.0, beat=0.13):
    """Flap the mouth and bob the neck (demo of the stage-front mouth robot)."""
    end = time.time() + duration
    mouth_on = False
    neck_dir = 1
    last_toggle = 0.0
    last_neck = time.time()
    neck_until = 0.0
    while time.time() < end:
        now = time.time()
        if now - last_toggle >= beat:
            last_toggle = now
            mouth_on = not mouth_on
            set_mouth_energized(mouth_on)
        if now >= last_neck:
            neck_dir *= -1
            neck_until = now + 0.25
            last_neck = now + random.uniform(1.2, 2.4)
        set_neck_energized(now < neck_until, neck_dir)
        time.sleep(0.02)
    set_mouth_energized(False)
    set_neck_energized(False)

def read_distance_cm(sensor_name):
    pins = SENSORS.get(sensor_name)
    if not pins:
        return 0.0
    GPIO.output(pins["trig"], True)
    time.sleep(0.00001)
    GPIO.output(pins["trig"], False)
    start_time = time.time()
    stop_time = time.time()
    while GPIO.input(pins["echo"]) == 0:
        start_time = time.time()
        if start_time - stop_time > 0.03:
            break
    while GPIO.input(pins["echo"]) == 1:
        stop_time = time.time()
    elapsed = stop_time - start_time
    return round((elapsed * 34300) / 2, 1)

def main():
    print("Biped 17-DOF controller online.")
    apply_pose()
    print("Applied %d servo angles." % len(TARGET_ANGLES))
    for s in SENSORS:
        print("Sensor %s: %.1f cm" % (s.upper(), read_distance_cm(s)))

if __name__ == "__main__":
    try:
        main()
    finally:
        GPIO.cleanup()
'''
        script = script.replace('__SERVOS__', servos_literal)
        script = script.replace('__ANGLES__', json.dumps(angles, indent=4))
        return script


class ThreadingReusableTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True
    ssl_context = None

    def get_request(self):
        newsocket, fromaddr = self.socket.accept()
        if self.ssl_context:
            try:
                first_byte = newsocket.recv(1, socket.MSG_PEEK)
                if first_byte and first_byte[0] == 0x16:
                    newsocket = self.ssl_context.wrap_socket(newsocket, server_side=True)
            except Exception:
                pass
        return newsocket, fromaddr


if __name__ == "__main__":
    for i in range(1, len(sys.argv)):
        if sys.argv[i] == "--port" and i + 1 < len(sys.argv):
            PORT = int(sys.argv[i + 1])
        elif sys.argv[i] == "--host" and i + 1 < len(sys.argv):
            HOST = sys.argv[i + 1]

    cert_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cert.pem")
    key_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "key.pem")
    if not (os.path.exists(cert_path) and os.path.exists(key_path)):
        cert_path = "/home/markirgang/Tweedpump/cert.pem"
        key_path = "/home/markirgang/Tweedpump/key.pem"

    ssl_ctx = None
    if os.path.exists(cert_path) and os.path.exists(key_path):
        try:
            ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_ctx.load_cert_chain(certfile=cert_path, keyfile=key_path)
        except Exception as e:
            print("SSL cert load warning:", e)

    display_host = "localhost" if HOST in ["", "0.0.0.0", "127.0.0.1"] else HOST
    print("Starting Biped 17-DOF Control Server on http://%s:%d (http & https supported)" % (display_host, PORT))
    bind_host = "" if HOST == "0.0.0.0" else HOST
    with ThreadingReusableTCPServer((bind_host, PORT), BipedRequestHandler) as httpd:
        httpd.ssl_context = ssl_ctx
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nBiped server stopped.")
