#!/usr/bin/env python3
"""
 ===========================================================================
   FLASK + ROS 2 BRIDGE - Micro-ROS Web Interface (v2.0)
 ===========================================================================

   Connects ROS 2 topics to Flask web interface for robot control.

   Subscribes to:
   - /imu_data (geometry_msgs/Accel)
   - /ultrasonic_data (geometry_msgs/Point)
   - /cmd_result (std_msgs/String)
   - /sensor_health (std_msgs/String) - JSON sensor status
   - /localization_confidence (std_msgs/Float32)
   - /robot_mode (std_msgs/String) - NORMAL/DEGRADED/FAULT/ABORT
   - /box_info (std_msgs/String) - JSON box info
   - /gripper_status (std_msgs/String) - JSON gripper state
   - /scan_status (std_msgs/String) - auto-scan progress
   - /task_status (std_msgs/String) - mission state
   - /robot_pose (geometry_msgs/Pose)

   Publishes to:
   - /cmd_vel (geometry_msgs/Twist) - velocity commands

   Services:
   - /calibrate_gripper (std_srvs/Empty)
   - /start_scan (std_srvs/Empty)
   - /emergency_stop (std_srvs/Empty)
   - /reset_recovery (std_srvs/Empty)

   Web Interface: http://localhost:5000

 ===========================================================================
"""

import json
import math
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Accel, Point, Pose, PoseStamped, Twist
from std_msgs.msg import String, Float32
from std_srvs.srv import Empty
from flask import Flask, render_template, Response, jsonify, request
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import threading
import io
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import deque

# ─────────────────────────────────────────────────────────────────────────────
# CAMERA CALIBRATION (from calibrate_camera.py)
# ─────────────────────────────────────────────────────────────────────────────
CAM_MATRIX = np.array([
    [828.3951714345817, 0.0, 337.4603949402347],
    [0.0, 812.655944490612, 213.6221133390383],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

DIST_COEFFS = np.array([[-1.4359167977905247, 14.759970080276391, -0.005699505649195278, 0.05434415294245828, -37.11140303416461]], dtype=np.float32)

# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 NODE
# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 NODE
# ─────────────────────────────────────────────────────────────────────────────

class RobotRosBridge(Node):
    def __init__(self):
        super().__init__('flask_ros_bridge')

        # Subscribers
        self.create_subscription(Accel, '/imu_data', self.imu_callback, 10)
        self.create_subscription(String, '/ultrasonic_data', self.us_callback, 10)
        self.create_subscription(String, '/cmd_result', self.result_callback, 10)

        # NEW - Fault-tolerant topic subscriptions
        self.create_subscription(String, '/sensor_health', self.sensor_health_callback, 10)
        self.create_subscription(Float32, '/localization_confidence', self.confidence_callback, 10)
        self.create_subscription(String, '/robot_mode', self.robot_mode_callback, 10)
        self.create_subscription(String, '/box_info', self.box_info_callback, 10)
        self.create_subscription(String, '/gripper_status', self.gripper_status_callback, 10)
        self.create_subscription(String, '/scan_status', self.scan_status_callback, 10)
        self.create_subscription(String, '/task_status', self.task_status_callback, 10)
        self.create_subscription(String, '/mission_state', self.mission_state_callback, 10)
        self.create_subscription(String, '/mission_count', self.mission_count_callback, 10)
        self.create_subscription(PoseStamped, '/robot_pose',  self.pose_callback,  10)
        self.create_subscription(Point,       '/odom_data',   self.odom_callback,  10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist,  '/cmd_vel',    10)
        self.cfg_pub = self.create_publisher(String, '/robot_cfg',  10)
        self.grip_pub= self.create_publisher(String, '/gripper_cmd',10)
        self.mission_ctrl_pub = self.create_publisher(String, '/mission_ctrl', 10)

        # Camera image subscription
        from sensor_msgs.msg import Image
        self.create_subscription(Image, '/image_raw', self.image_callback, 1)
        self.latest_image = None

        # State
        self.yaw_deg = 0.0
        self.omega_z = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.us = [-1.0, -1.0, -1.0, -1.0]
        self.us_mask = [1, 1, 1, 1]
        self.last_cmd_result = ""
        self.odom = {"x": 0.0, "y": 0.0, "yaw": 0.0}

        # NEW - State for all telemetry
        self.sensor_health = {}
        self.localization_confidence = 0.0
        self.robot_mode = "UNKNOWN"
        self.box_info = {"color": "none", "orientation": "unknown", "distance": 0.0, "confidence": 0.0}
        self.gripper_status = {"state": "unknown", "has_box": False}
        self.scan_status = "IDLE"
        self.task_status = "IDLE"
        self.mission_state = {
            "state": "IDLE",
            "running": False,
            "target_tag": None,
            "target_color": None,
            "lstm": {"enabled": False, "recording_enabled": True, "last_prediction": None, "last_fallback_reason": "boot"},
        }
        self.mission_count = {"total": 0, "completed": 0, "remaining": 0, "status": "idle"}
        self.robot_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}

        # NEW - Service clients
        self.calibrate_gripper_client = self.create_client(Empty, '/calibrate_gripper')
        self.start_scan_client = self.create_client(Empty, '/start_scan')
        self.emergency_stop_client = self.create_client(Empty, '/emergency_stop')
        self.reset_recovery_client = self.create_client(Empty, '/reset_recovery')
        self.start_task_client = self.create_client(Empty, '/start_task')
        self.cancel_task_client = self.create_client(Empty, '/cancel_task')
        self.reset_odom_client = self.create_client(Empty, '/reset_odom')

        self.get_logger().info('Flask-ROS Bridge started!')

    def imu_callback(self, msg):
        self.yaw_deg = msg.linear.x
        self.omega_z = msg.linear.y
        self.ax = msg.angular.x
        self.ay = msg.angular.y
        self.az = msg.angular.z

    def us_callback(self, msg):
        # Format JSON: {"us":[d0,d1,d2,d3]}
        # US1=avant droit, US2=avant gauche, US3=arrière gauche, US4=arrière droite
        try:
            data = json.loads(msg.data)
            self.us = [float(v) for v in data['us']]
            if 'usm' in data and isinstance(data['usm'], list) and len(data['usm']) >= 4:
                self.us_mask = [1 if bool(v) else 0 for v in data['usm'][:4]]
        except Exception:
            pass

    def result_callback(self, msg):
        self.last_cmd_result = msg.data

    def odom_callback(self, msg: Point):
        self.odom = {"x": msg.x, "y": msg.y, "yaw": msg.z}

    def send_cmd(self, cmd_byte):
        pass  # No longer used, kept for compatibility

    def send_config(self, cfg: dict):
        """Envoie JSON de config PID/trims/rampe au topic /robot_cfg"""
        msg = String()
        msg.data = json.dumps(cfg)
        self.cfg_pub.publish(msg)

    def send_gripper(self, value):
        """Envoie commande gripper (angle int ou 'o'/'c') au topic /gripper_cmd"""
        msg = String()
        msg.data = str(value)
        self.grip_pub.publish(msg)

    def send_mission_ctrl(self, payload: dict):
        """Envoie un contrôle LSTM/mission au topic /mission_ctrl."""
        msg = String()
        msg.data = json.dumps(payload)
        self.mission_ctrl_pub.publish(msg)

    def send_velocity(self, linear_x, angular_z):
        """Send velocity command to robot via /cmd_vel"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.send_velocity(0.0, 0.0)

    # NEW - Fault-tolerant callbacks
    def sensor_health_callback(self, msg: String):
        try:
            self.sensor_health = json.loads(msg.data)
            if 'usm' in self.sensor_health and isinstance(self.sensor_health['usm'], list) and len(self.sensor_health['usm']) >= 4:
                self.us_mask = [1 if bool(v) else 0 for v in self.sensor_health['usm'][:4]]
        except json.JSONDecodeError:
            pass

    def confidence_callback(self, msg: Float32):
        self.localization_confidence = msg.data

    def robot_mode_callback(self, msg: String):
        self.robot_mode = msg.data

    def box_info_callback(self, msg: String):
        try:
            self.box_info = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def gripper_status_callback(self, msg: String):
        try:
            self.gripper_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def scan_status_callback(self, msg: String):
        self.scan_status = msg.data

    def task_status_callback(self, msg: String):
        self.task_status = msg.data

    def mission_state_callback(self, msg: String):
        try:
            self.mission_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.mission_state = {"state": msg.data, "running": False}

    def mission_count_callback(self, msg: String):
        try:
            self.mission_count = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def pose_callback(self, msg: PoseStamped):
        self.robot_pose = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "theta": self._quaternion_to_yaw(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        }

    def _quaternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def image_callback(self, msg):
        """Store latest camera image for Flask streaming."""
        try:
            # Convert ROS Image to numpy array (BGR8)
            if msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            else:
                return  # Unsupported encoding
            self.latest_image = img
        except Exception:
            pass

# ─────────────────────────────────────────────────────────────────────────────
# FLASK APP
# ─────────────────────────────────────────────────────────────────────────────

app = Flask(__name__)

state_lock = threading.Lock()
app_state = {
    "imu": {"yaw_deg": 0, "omega_z": 0, "ax": 0, "ay": 0, "az": 0, "us": [-1, -1, -1, -1], "us_mask": [1, 1, 1, 1]},
    "cmd_result": "",
    "map_frame": None,
    "imu_3d_frame": None,
    # NEW - Fault-tolerant state
    "sensor_health": {},
    "localization_confidence": 0.0,
    "robot_mode": "UNKNOWN",
    "box_info": {},
    "gripper_status": {},
    "scan_status": "IDLE",
    "task_status": "IDLE",
    "mission_state": {"state": "IDLE", "running": False, "target_tag": None, "target_color": None, "lstm": {"enabled": False, "recording_enabled": True, "last_prediction": None, "last_fallback_reason": "boot"}},
    "mission_count": {"total": 0, "completed": 0, "remaining": 0, "status": "idle"},
}

imu_history = deque(maxlen=200)
fig_imu = None
ax_imu = None

# ═══════════════════════════════════════════════════════════════════════════
# IMU 3D VISUALIZATION
# ═══════════════════════════════════════════════════════════════════════════

def create_imu_3d_plot(yaw_deg, ax, ay, az):
    global fig_imu, ax_imu, imu_history
    imu_history.append((yaw_deg, ax, ay, az))

    if fig_imu is None:
        fig_imu, ax_imu = plt.subplots(subplot_kw={'projection': '3d'}, figsize=(4, 4))
        fig_imu.tight_layout()

    ax_imu.clear()

    roll = np.radians(np.arctan2(ay, az) * 180 / np.pi)
    pitch = np.radians(np.arctan2(-ax, np.sqrt(ay**2 + az**2)) * 180 / np.pi)
    yaw = np.radians(-yaw_deg)

    axis_length = 0.8
    vectors = [
        (axis_length * np.cos(pitch) * np.cos(yaw), axis_length * np.cos(pitch) * np.sin(yaw), -axis_length * np.sin(pitch), 'red'),
        (axis_length * np.sin(roll) * np.sin(yaw) + axis_length * np.cos(roll) * np.sin(pitch) * np.cos(yaw),
         -axis_length * np.cos(roll) * np.sin(pitch) * np.sin(yaw) + axis_length * np.sin(roll) * np.cos(pitch) * np.cos(yaw),
         axis_length * np.sin(roll) * np.cos(pitch), 'green'),
    ]

    for vx, vy, vz, color in vectors:
        ax_imu.quiver(0, 0, 0, vx, vy, vz, color=color, arrow_length_ratio=0.3)

    ax_imu.set_xlim([-1, 1])
    ax_imu.set_ylim([-1, 1])
    ax_imu.set_zlim([-1, 1])
    ax_imu.set_xlabel('X')
    ax_imu.set_ylabel('Y')
    ax_imu.set_zlabel('Z')
    ax_imu.set_title(f'IMU Yaw:{yaw_deg:.0f}deg')

    buf = io.BytesIO()
    fig_imu.savefig(buf, format='jpg', dpi=50, bbox_inches='tight')
    buf.seek(0)
    plt.close(fig_imu)
    fig_imu = None
    return buf.read()

def draw_map(yaw_deg, ax, ay, az, us_distances, robot_x=0.0, robot_y=0.0, w=640, h=480, scale=60.0):
    """
    Draw 2D map with robot at real position.
    scale: pixels per meter (60px = 1 meter)
    """
    img = np.zeros((h, w, 3), dtype=np.uint8)

    # Center of map (can be offset to show more area)
    map_center_x = w // 2
    map_center_y = h // 2

    # Draw grid lines (every 0.5m = 30px with scale=60)
    for i in range(-10, 11):
        # Vertical lines
        x_pos = map_center_x + int(i * scale * 0.5)  # every 0.5m
        cv2.line(img, (x_pos, 0), (x_pos, h), (40, 40, 40), 1)
        # Horizontal lines
        y_pos = map_center_y - int(i * scale * 0.5)
        cv2.line(img, (0, y_pos), (w, y_pos), (40, 40, 40), 1)

    # Draw station markers (fixed positions — cm converted to meters)
    stations = {
        'HOME': (0, 0),
        'MFG': (0, 0.70),
        'Station A': (-0.60, 0.30),
        'Station B': (0.60, 0.55),
    }

    for name, (sx, sy) in stations.items():
        px = map_center_x + int(sx * scale)
        py = map_center_y - int(sy * scale)
        cv2.circle(img, (px, py), 8, (100, 100, 100), -1)
        cv2.putText(img, name, (px + 12, py + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

    # Calculate robot screen position
    rx = map_center_x + int(robot_x * scale)
    ry = map_center_y - int(robot_y * scale)

    # Clamp to screen bounds
    rx = max(10, min(w - 10, rx))
    ry = max(10, min(h - 10, ry))

    # Draw robot position circle
    cv2.circle(img, (rx, ry), 10, (0, 255, 255), -1)

    # Draw robot direction arrow
    yaw = np.radians(-yaw_deg)  # Negate for screen coords
    arrow_len = 25
    tip_x = int(rx + arrow_len * np.sin(yaw))
    tip_y = int(ry - arrow_len * np.cos(yaw))
    cv2.arrowedLine(img, (rx, ry), (tip_x, tip_y), (0, 255, 0), 3, tip_length=8)

    # Draw ultrasonic sensors (from robot position)
    if us_distances and len(us_distances) >= 4:
        us_angles = [yaw, yaw + np.pi, yaw + np.pi/2, yaw - np.pi/2]
        us_labels = ['F', 'B', 'L', 'R']
        us_colors = [(0, 255, 0), (0, 200, 255), (255, 165, 0), (255, 100, 0)]
        max_dist_px = int(100 * scale / 100)  # 100cm max

        for i, (dist, angle, label, color) in enumerate(zip(us_distances, us_angles, us_labels, us_colors)):
            if dist < 0 or dist > 200:
                continue
            dist_px = min(int(dist * scale / 100), max_dist_px)  # cm to px
            ex = int(rx + dist_px * np.sin(angle))
            ey = int(ry - dist_px * np.cos(angle))
            cv2.line(img, (rx, ry), (ex, ey), color, 2)
            cv2.circle(img, (ex, ey), 4, color, -1)

    # Info overlay
    color = (0, 255, 255)
    lines = [
        f"POS: ({robot_x:.2f}, {robot_y:.2f})m",
        f"YAW: {abs(yaw_deg):.0f}deg",
    ]
    for i, line in enumerate(lines):
        cv2.putText(img, line, (10, 20 + i * 18),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    # Draw scale legend
    cv2.putText(img, "1m = 60px", (w - 90, h - 15),
               cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 100), 1)

    return img

# ═══════════════════════════════════════════════════════════════════════════
# FLASK ROUTES  (HTML → templates/index.html  |  JS → static/js/ros_bridge.js)
# ═══════════════════════════════════════════════════════════════════════════

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/map_feed')
def map_feed():
    def gen():
        while True:
            time.sleep(0.05)
            with state_lock:
                frame = app_state.get('map_frame')
            if frame:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/imu3d_feed')
def imu3d_feed():
    def gen():
        while True:
            time.sleep(0.1)
            with state_lock:
                frame = app_state.get('imu_3d_frame')
            if frame:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/camera_feed')
def camera_feed():
    """Stream raw camera feed from /image_raw topic."""
    def gen():
        while True:
            time.sleep(0.033)  # ~30 FPS
            img = None
            if ros_bridge and ros_bridge.latest_image is not None:
                img = ros_bridge.latest_image
            if img is not None:
                # Encode to JPEG
                _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if _:
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/state')
def state():
    with state_lock:
        return jsonify({
            "imu": app_state["imu"].copy(),
            "odom": app_state.get("odom", {"x": 0.0, "y": 0.0, "yaw": 0.0}),
            "cmd_result": app_state["cmd_result"],
            "sensor_health": app_state.get("sensor_health", {}),
            "us_mask": app_state["imu"].get("us_mask", [1, 1, 1, 1]),
            "localization_confidence": app_state.get("localization_confidence", 0.0),
            "robot_mode": app_state.get("robot_mode", "UNKNOWN"),
            "box_info": app_state.get("box_info", {}),
            "gripper_status": app_state.get("gripper_status", {}),
            "scan_status": app_state.get("scan_status", "IDLE"),
            "task_status": app_state.get("task_status", "IDLE"),
            "mission_state": app_state.get("mission_state", {"state": "IDLE", "running": False}),
            "mission_count": app_state.get("mission_count", {"total": 0, "completed": 0, "remaining": 0, "status": "idle"}),
        })

@app.route('/cmd/<int:cmd>', methods=['POST'])
def cmd_route(cmd):
    # Legacy - kept for compatibility but motor control now uses /cmd_vel
    return jsonify({"status": "ok", "cmd": cmd})

@app.route('/velocity', methods=['POST'])
def velocity_route():
    # H5/C1: refuser commande manuelle si une mission est active
    # pour ne pas écraser les commandes mission sur /cmd_vel
    with state_lock:
        mission_st = app_state.get('mission_state', {})
    if mission_st.get('running', False):
        return jsonify({"status": "blocked",
                        "reason": "Mission active — arrêtez la mission avant de piloter manuellement"}), 409
    if ros_bridge:
        data = json.loads(request.data)
        ros_bridge.send_velocity(data.get('linear', 0.0), data.get('angular', 0.0))
    return jsonify({"status": "ok"})

# fonctions _load_robot_config / _save_robot_config / _robot_config_to_esp32_payload
# définies plus bas dans le fichier (section ROBOT CONFIG GLOBAL)

@app.route('/api/config', methods=['GET'])
def api_config_get():
    """Retourne la configuration globale actuelle (robot_config.json)."""
    cfg = _load_robot_config()
    return jsonify(cfg if cfg else {"error": "robot_config.json introuvable"})

@app.route('/api/config', methods=['POST'])
def api_config():
    """Envoie config PID/trims/rampe au robot via /robot_cfg ET sauvegarde dans robot_config.json.
    Body JSON: {ykp, yki, ykd, ta, tb, ma, mb, rs, rb, rn, reset_odom, save, us_en, us_mask}"""
    if not ros_bridge:
        return jsonify({"status": "error", "message": "No ROS bridge"})
    data = json.loads(request.data or '{}')
    # Persister dans la source de vérité globale
    cfg = _load_robot_config()
    if data.get("ykp") is not None: cfg.setdefault("pid", {})["kp"]    = data["ykp"]
    if data.get("yki") is not None: cfg.setdefault("pid", {})["ki"]    = data["yki"]
    if data.get("ykd") is not None: cfg.setdefault("pid", {})["kd"]    = data["ykd"]
    if data.get("ta")  is not None: cfg.setdefault("trims", {})["a"]   = data["ta"]
    if data.get("tb")  is not None: cfg.setdefault("trims", {})["b"]   = data["tb"]
    if data.get("ma")  is not None: cfg.setdefault("minpwm", {})["a"]  = data["ma"]
    if data.get("mb")  is not None: cfg.setdefault("minpwm", {})["b"]  = data["mb"]
    if data.get("rs")  is not None: cfg.setdefault("ramp", {})["speed"]   = data["rs"]
    if data.get("rb")  is not None: cfg.setdefault("ramp", {})["brake"]   = data["rb"]
    if data.get("rn")  is not None: cfg.setdefault("ramp", {})["neutral"] = data["rn"]
    if data.get("us_mask") is not None:
        mask = data.get("us_mask")
        if isinstance(mask, list) and len(mask) >= 4:
            cfg["us_mask"] = [1 if bool(mask[i]) else 0 for i in range(4)]
            data["us0"] = cfg["us_mask"][0]
            data["us1"] = cfg["us_mask"][1]
            data["us2"] = cfg["us_mask"][2]
            data["us3"] = cfg["us_mask"][3]
    ros_bridge.send_config(data)
    _save_robot_config(cfg)
    return jsonify({"status": "ok", "sent": data})

@app.route('/api/gripper', methods=['POST'])
def api_gripper():
    """Commande gripper. Body JSON: {value: 90} ou {value: 'o'} ou {value: 'c'}"""
    if not ros_bridge:
        return jsonify({"status": "error", "message": "No ROS bridge"})
    data = json.loads(request.data or '{}')
    ros_bridge.send_gripper(data.get('value', 'o'))
    return jsonify({"status": "ok"})

@app.route('/api/odom', methods=['GET'])
def api_odom():
    """Retourne l'odométrie actuelle (posX, posY, yaw)."""
    if not ros_bridge:
        return jsonify({"x": 0.0, "y": 0.0, "yaw": 0.0})
    return jsonify(ros_bridge.odom)

@app.route('/api/mission_ctrl', methods=['POST'])
def api_mission_ctrl():
    """Publie un contrôle LSTM/mission vers /mission_ctrl."""
    if not ros_bridge:
        return jsonify({"status": "error", "message": "No ROS bridge"})
    data = json.loads(request.data or '{}')
    ros_bridge.send_mission_ctrl(data)
    return jsonify({"status": "ok", "sent": data})

# ─── ROBOT CONFIG GLOBAL (source de vérité partagée) ───────────────────────
_DATA_ROOT   = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROBOT_CONFIG_FILE = os.path.join(_DATA_ROOT, "data", "robot_config.json")

def _load_robot_config() -> dict:
    """Charge robot_config.json. Retourne {} si absent."""
    try:
        with open(ROBOT_CONFIG_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}

def _save_robot_config(cfg: dict):
    """Sauvegarde robot_config.json de façon atomique."""
    try:
        tmp = ROBOT_CONFIG_FILE + ".tmp"
        os.makedirs(os.path.dirname(ROBOT_CONFIG_FILE), exist_ok=True)
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
        os.replace(tmp, ROBOT_CONFIG_FILE)
    except Exception as e:
        print(f"[CFG] Sauvegarde globale échouée: {e}")

def _robot_config_to_esp32_payload(cfg: dict) -> dict:
    """Convertit robot_config.json en payload JSON pour /robot_cfg (micro-ROS)."""
    pid    = cfg.get("pid",    {})
    trims  = cfg.get("trims",  {})
    minpwm = cfg.get("minpwm", {})
    ramp   = cfg.get("ramp",   {})
    us_mask = cfg.get("us_mask", [1, 1, 1, 1])
    if not isinstance(us_mask, list) or len(us_mask) < 4:
        us_mask = [1, 1, 1, 1]
    return {
        "ykp": pid.get("kp",  4.0),
        "yki": pid.get("ki",  0.02),
        "ykd": pid.get("kd",  0.7),
        "ta":  trims.get("a",  0.0),
        "tb":  trims.get("b",  0.0),
        "ma":  minpwm.get("a", 55.0),
        "mb":  minpwm.get("b", 55.0),
        "rs":  ramp.get("speed",   80.0),
        "rb":  ramp.get("brake",  120.0),
        "rn":  ramp.get("neutral", 200.0),
        "us0": 1 if bool(us_mask[0]) else 0,
        "us1": 1 if bool(us_mask[1]) else 0,
        "us2": 1 if bool(us_mask[2]) else 0,
        "us3": 1 if bool(us_mask[3]) else 0,
    }

# ─── MISSIONS CONFIG API ──────────────────────────────────────────────────
MISSIONS_FILE = os.path.join(_DATA_ROOT, "data", "missions", "missions.json")

@app.route('/api/missions', methods=['GET'])
def api_get_missions():
    """Retourne la config des missions depuis le JSON."""
    try:
        with open(MISSIONS_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        return jsonify(data)
    except Exception as e:
        return jsonify({"error": str(e), "missions": [], "home_tag": 12, "repeat": True})

@app.route('/api/missions', methods=['POST'])
def api_set_missions():
    """Met à jour la config des missions."""
    try:
        data = json.loads(request.data or '{}')
        # Valider la structure
        if "missions" not in data:
            return jsonify({"error": "missing 'missions' key"}), 400
        for m in data["missions"]:
            if "pickup_tag" not in m or "drop_tag" not in m:
                return jsonify({"error": "each mission needs 'pickup_tag' and 'drop_tag'"}), 400
            m.setdefault("color", "blue")
            m.setdefault("label", f"Pickup {m['pickup_tag']} → Drop {m['drop_tag']}")
        data.setdefault("home_tag", 12)
        data.setdefault("repeat", True)
        os.makedirs(os.path.dirname(MISSIONS_FILE), exist_ok=True)
        with open(MISSIONS_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        return jsonify({"status": "ok", "count": len(data["missions"])})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/api/reset_odom', methods=['POST'])
def api_reset_odom():
    """Reset odométrie sur l'ESP32."""
    if not ros_bridge:
        return jsonify({"status": "error", "message": "No ROS bridge"})
    ros_bridge.send_config({"reset_odom": 1})
    return jsonify({"status": "ok"})

@app.route('/service/<service_name>', methods=['POST'])
def service_route(service_name):
    if not ros_bridge:
        return jsonify({"status": "error", "message": "No ROS bridge"})

    if service_name == 'calibrate_gripper':
        future = ros_bridge.calibrate_gripper_client.call_async(Empty.Request())
    elif service_name == 'start_task':
        future = ros_bridge.start_task_client.call_async(Empty.Request())
    elif service_name == 'cancel_task':
        future = ros_bridge.cancel_task_client.call_async(Empty.Request())
    elif service_name == 'start_scan':
        future = ros_bridge.start_scan_client.call_async(Empty.Request())
    elif service_name == 'emergency_stop':
        ros_bridge.send_velocity(0.0, 0.0)  # Immediate stop
        return jsonify({"status": "ok", "message": "Emergency stop"})
    elif service_name == 'reset_recovery':
        future = ros_bridge.reset_recovery_client.call_async(Empty.Request())
    elif service_name == 'reset_odom':
        future = ros_bridge.reset_odom_client.call_async(Empty.Request())
    else:
        return jsonify({"status": "error", "message": "Unknown service"})

    return jsonify({"status": "ok", "service": service_name})

# ═══════════════════════════════════════════════════════════════════════════
# MAIN - ROS SPINNER
# ═══════════════════════════════════════════════════════════════════════════

ros_bridge = None
ros_thread = None

def ros_spinner():
    rclpy.spin(ros_bridge)

def main():
    global ros_bridge

    print("=" * 50)
    print("  MICRO-ROS FLASK BRIDGE")
    print("  Open: http://localhost:5000")
    print("=" * 50)

    rclpy.init(args=None)
    ros_bridge = RobotRosBridge()

    ros_thread = threading.Thread(target=ros_spinner, daemon=True)
    ros_thread.start()

    # Charger et pousser la config globale vers l'ESP32 au démarrage
    _gcfg = _load_robot_config()
    if _gcfg:
        payload = _robot_config_to_esp32_payload(_gcfg)
        payload["save"] = 0  # Pas de save EEPROM automatique au boot
        time.sleep(1.5)      # Laisser le temps au bridge ROS de s'initialiser
        ros_bridge.send_config(payload)
        print(f"[CFG] Config globale envoyée à l'ESP32: kp={payload['ykp']} ki={payload['yki']} kd={payload['ykd']} ta={payload['ta']} tb={payload['tb']}")
    else:
        print("[CFG] robot_config.json absent — valeurs par défaut du firmware")

    # Create Flask server in background
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False), daemon=True)
    flask_thread.start()

    print("[SETUP] Flask + ROS bridge running!")
    print("[SETUP] Waiting for ESP32 connection...")

    imu_frame_skip = 0

    try:
        while True:
            time.sleep(0.03)

            with state_lock:
                app_state["imu"] = {
                    "yaw_deg": ros_bridge.yaw_deg,
                    "omega_z": ros_bridge.omega_z,
                    "ax": ros_bridge.ax,
                    "ay": ros_bridge.ay,
                    "az": ros_bridge.az,
                    "us": ros_bridge.us.copy(),
                    "us_mask": ros_bridge.us_mask.copy(),
                }
                app_state["cmd_result"]   = ros_bridge.last_cmd_result
                app_state["odom"]         = ros_bridge.odom.copy()
                app_state["sensor_health"]          = ros_bridge.sensor_health
                app_state["localization_confidence"] = ros_bridge.localization_confidence
                app_state["robot_mode"]   = ros_bridge.robot_mode
                app_state["box_info"]     = ros_bridge.box_info
                app_state["gripper_status"] = ros_bridge.gripper_status
                app_state["scan_status"]  = ros_bridge.scan_status
                app_state["task_status"]  = ros_bridge.task_status
                app_state["mission_state"] = ros_bridge.mission_state
                app_state["mission_count"]= ros_bridge.mission_count

            imu_frame_skip += 1
            if imu_frame_skip % 3 == 0:
                imu_3d = create_imu_3d_plot(ros_bridge.yaw_deg, ros_bridge.ax, ros_bridge.ay, ros_bridge.az)
            else:
                imu_3d = app_state.get("imu_3d_frame")

            map_img = draw_map(ros_bridge.yaw_deg, ros_bridge.ax, ros_bridge.ay, ros_bridge.az, ros_bridge.us, ros_bridge.odom.get('x', 0.0), ros_bridge.odom.get('y', 0.0))
            map_bytes = cv2.imencode('.jpg', map_img, [cv2.IMWRITE_JPEG_QUALITY, 60])[1].tobytes()

            with state_lock:
                app_state["map_frame"] = map_bytes
                if imu_frame_skip % 3 == 0:
                    app_state["imu_3d_frame"] = imu_3d

    except KeyboardInterrupt:
        print("\n[SHUTDOWN] Stopping...")

if __name__ == '__main__':
    main()