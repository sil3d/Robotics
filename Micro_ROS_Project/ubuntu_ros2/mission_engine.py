#!/usr/bin/env python3
"""
===========================================================================
  MISSION ENGINE — Machine d'états autonome
  Robot de tri de cubes colorés (bleu / cyan) par AprilTag

  CYCLE MISSION :
    IDLE → SCAN_360 → NAVIGATE_TAG → DETECT_CUBE → NAVIGATE_CUBE
         → OPEN_GRIPPER → APPROACH_CUBE → CLOSE_GRIPPER
         → NAVIGATE_DROP → RELEASE → RECORD → IDLE

  OBSTACLES :
    - usSpeedLimit géré par le firmware (ralentissement 3→15 cm)
    - Si obstacle < STOP_DIST pendant navigation → AVOID (rotation)
    - Si coincé (3+ capteurs bloqués) → STUCK → rotation 180°

  INTERFACE :
    - ROS2 : publie /cmd_vel, /gripper_cmd, /robot_cfg
    - Lit  : /imu_data, /ultrasonic_data, /odom_data, /sensor_health
    - Publie: /mission_state (JSON), /mission_log (String)
===========================================================================
"""

import math
import time
import json
import threading
import numpy as np
import cv2
import cv2.aruco as aruco
import os

from lstm_assistant import LSTMAssistant

# ─── CONFIG ────────────────────────────────────────────────────────────────────
# Tags dans l'environnement
# Structure: {tag_id: {"role": str, "x": float, "y": float}}
# role: "pickup_blue" | "pickup_cyan" | "drop_blue" | "drop_cyan" | "home"
TAG_MAP = {
    1: {"role": "home",        "x": 0.0,  "y": 0.0},
    2: {"role": "pickup_blue", "x": 1.5,  "y": 0.0},
    3: {"role": "pickup_cyan", "x": 1.5,  "y": 1.5},
    4: {"role": "drop_blue",   "x": 0.0,  "y": 1.5},
    5: {"role": "drop_cyan",   "x": 0.5,  "y": 1.5},
}

# Couleurs cubes (HSV)
COLOR_RANGES = {
    "blue": {
        "lower": np.array([100, 120, 80],  dtype=np.uint8),
        "upper": np.array([130, 255, 255], dtype=np.uint8),
    },
    "cyan": {
        "lower": np.array([80, 100, 80],   dtype=np.uint8),
        "upper": np.array([100, 255, 255], dtype=np.uint8),
    },
}

DROP_TAG = {"blue": 4, "cyan": 5}   # tag de dépôt par couleur

# Navigation
NAV_DIST_THRESHOLD  = 0.12   # m — considéré "arrivé" si < cette distance
NAV_ANGLE_THRESHOLD = 8.0    # deg — aligné si < cet angle
APPROACH_DIST_CM    = 12.0   # cm — distance finale pour saisir le cube
SCAN_OMEGA          = 0.4    # rad/s — vitesse rotation scan
NAV_LINEAR_SPEED    = 0.18   # m/s — vitesse navigation
STOP_DIST_CM        = 8.0    # cm — distance arrêt obstacle (firmware gère < 3cm)
STUCK_TIMEOUT_S     = 4.0    # s sans avancer = coincé

# Gripper
GRIPPER_OPEN  = 0
GRIPPER_CLOSE = 160

# Calibration caméra
_CAL_FILE = os.path.join(os.path.dirname(__file__), "..", "..", "data", "camera_calibration", "camera_calibration.json")
if os.path.exists(_CAL_FILE):
    with open(_CAL_FILE) as _f:
        _cal = json.load(_f)
    CAM_MATRIX  = np.array(_cal["camera_matrix"], dtype=np.float32)
    DIST_COEFFS = np.array(_cal["dist_coeffs"],   dtype=np.float32)
    TAG_SIZE_M  = float(_cal.get("apriltag_size_cm", 10.0)) / 100.0
else:
    CAM_MATRIX  = np.array([[828, 0, 337], [0, 812, 213], [0, 0, 1]], dtype=np.float32)
    DIST_COEFFS = np.zeros((1, 5), dtype=np.float32)
    TAG_SIZE_M  = 0.10


# ─── ÉTATS ─────────────────────────────────────────────────────────────────────
class State:
    IDLE            = "IDLE"
    SCAN_360        = "SCAN_360"        # Rotation sur place, cherche tags + cubes
    NAVIGATE_TAG    = "NAVIGATE_TAG"    # Va vers un tag pickup
    DETECT_CUBE     = "DETECT_CUBE"     # Cherche le cube coloré devant lui
    NAVIGATE_CUBE   = "NAVIGATE_CUBE"   # S'approche du cube détecté
    OPEN_GRIPPER    = "OPEN_GRIPPER"    # Ouvre la pince
    APPROACH_CUBE   = "APPROACH_CUBE"   # Avance lentement vers cube
    CLOSE_GRIPPER   = "CLOSE_GRIPPER"   # Ferme la pince
    NAVIGATE_DROP   = "NAVIGATE_DROP"   # Va vers zone de dépôt
    RELEASE         = "RELEASE"         # Ouvre la pince pour lâcher
    BACK_HOME       = "BACK_HOME"       # Retour au tag home
    RECORD          = "RECORD"          # Enregistre la trajectoire pour LSTM
    AVOID           = "AVOID"           # Contourne obstacle
    STUCK           = "STUCK"           # Coincé — rotation 180°
    ERROR           = "ERROR"


class MissionEngine:
    """
    Machine d'états pour mission de tri de cubes.
    Utilise les données ROS2 injectées depuis task_manager_node.py.
    """

    def __init__(self, ros_node=None):
        self.ros_node   = ros_node      # Nœud ROS2 pour publier/logguer
        self.state      = State.IDLE
        self.running    = False
        self._lock      = threading.Lock()

        # Données capteurs (mises à jour par ROS callbacks)
        self.yaw_deg    = 0.0
        self.omega_z    = 0.0
        self.odom       = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.us         = [-1.0, -1.0, -1.0, -1.0]   # [front, back, left, right]
        self.us_limit   = 1.0

        # Caméra
        self.camera     = None
        self.frame      = None
        self._cam_lock  = threading.Lock()
        self._camera_running = False
        self._camera_thread  = None

        # Mission variables
        self.target_tag_id    = None
        self.target_color     = None    # "blue" | "cyan"
        self.cube_pixel_x     = None    # pixel x du cube dans l'image
        self.cube_dist_cm     = None
        self.scan_start_yaw   = None
        self.scan_turned_deg  = 0.0
        self.nav_start_time   = None
        self.last_odom_x      = None
        self.last_odom_y      = None
        self.stuck_timer      = None
        self.avoid_timer      = None
        self.mission_count    = {"total": 0, "blue": 0, "cyan": 0}

        # Historique pour LSTM (enregistré à chaque mission)
        self.trajectory_log   = []   # list de {state, odom, us, yaw, action, t}
        self.all_trajectories = []   # toutes les missions

        # LSTM advisory assistant
        self.lstm = LSTMAssistant(enabled=True, recording_enabled=True)
        self._last_lstm_hint = None
        self._last_linear_cmd = 0.0
        self._last_angular_cmd = 0.0
        self._last_gripper_cmd = None

        # Callbacks vers ROS (assignés par task_manager_node)
        self._velocity_cb = lambda lin, ang: None
        self._gripper_cb  = lambda val: None
        self._config_cb   = lambda cfg: None
        self._log_cb      = lambda msg: print(f"[MISSION] {msg}")

        # Détecteur AprilTag
        self._tag_dict   = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36H11)
        self._tag_params = aruco.DetectorParameters()
        self._detector   = aruco.ArucoDetector(self._tag_dict, self._tag_params)

        # Caméra locale (optionnelle)
        self.camera = cv2.VideoCapture(0)
        if self.camera.isOpened():
            self._camera_running = True
            self._camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
            self._camera_thread.start()
            self.log("Caméra locale ouverte")
        else:
            self.camera = None
            self.log("Caméra locale non disponible")

    # ─── API publique ──────────────────────────────────────────────────────────

    def set_velocity_callback(self, callback):
        self._velocity_cb = callback if callback is not None else (lambda lin, ang: None)

    def set_gripper_callback(self, callback):
        self._gripper_cb = callback if callback is not None else (lambda val: None)

    def set_config_callback(self, callback):
        self._config_cb = callback if callback is not None else (lambda cfg: None)

    def set_log_callback(self, callback):
        self._log_cb = callback if callback is not None else (lambda msg: None)

    def set_lstm_enabled(self, enabled: bool):
        self.lstm.set_enabled(enabled)

    def set_lstm_recording(self, enabled: bool):
        self.lstm.set_recording(enabled)

    def set_lstm_threshold(self, threshold: float):
        self.lstm.set_confidence_threshold(threshold)

    def get_lstm_status(self) -> dict:
        return self.lstm.get_status()

    def _hint_to_target(self, hint_target: str):
        if hint_target == "PICKUP_BLUE":
            return "blue", 2
        if hint_target == "PICKUP_CYAN":
            return "cyan", 3
        if hint_target == "DROP_BLUE":
            return "blue", 4
        if hint_target == "DROP_CYAN":
            return "cyan", 5
        return None, None

    def start(self):
        """Lance la mission en boucle."""
        if self.running:
            return
        if self.camera is None:
            self.camera = cv2.VideoCapture(0)
            if self.camera.isOpened():
                self._camera_running = True
                self._camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
                self._camera_thread.start()
                self.log("Caméra locale rouverte")
            else:
                self.camera = None
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()
        self.log("Mission démarrée")

    def stop(self):
        """Arrête proprement la mission."""
        self.running = False
        self.send_velocity(0.0, 0.0)
        self._set_state(State.IDLE)
        self.log("Mission arrêtée")
        self._release_camera()

    def get_status(self) -> dict:
        """Retourne l'état courant pour l'UI."""
        return {
            "state":         self.state,
            "running":       self.running,
            "target_tag":    self.target_tag_id,
            "target_color":  self.target_color,
            "cube_dist_cm":  self.cube_dist_cm,
            "odom":          self.odom,
            "yaw":           self.yaw_deg,
            "us":            self.us,
            "us_limit":      self.us_limit,
            "mission_count": self.mission_count,
            "lstm":          self.lstm.get_status(),
            "lstm_hint":     self._last_lstm_hint,
        }

    def update_sensors(self, yaw, omega_z, odom, us, us_limit=1.0):
        """Mis à jour par les callbacks ROS."""
        with self._lock:
            self.yaw_deg  = yaw
            self.omega_z  = omega_z
            self.odom     = odom
            self.us       = us
            self.us_limit = us_limit

    def update_frame(self, frame):
        """Mis à jour par le thread caméra."""
        with self._cam_lock:
            self.frame = frame.copy() if frame is not None else None

    def send_velocity(self, lin: float, ang: float):
        self._last_linear_cmd = float(lin)
        self._last_angular_cmd = float(ang)
        self._velocity_cb(float(lin), float(ang))

    def send_gripper(self, value):
        self._last_gripper_cmd = value
        self._gripper_cb(value)

    def send_config(self, cfg: dict):
        self._config_cb(cfg)

    def log(self, msg: str):
        self._log_cb(msg)

    # ─── BOUCLE PRINCIPALE ────────────────────────────────────────────────────

    def _loop(self):
        rate = 0.05   # 20 Hz
        while self.running:
            try:
                self._step()
            except Exception as e:
                self.log(f"ERREUR état {self.state}: {e}")
                self._set_state(State.ERROR)
            time.sleep(rate)

    def _camera_loop(self):
        while self._camera_running and self.camera is not None:
            ok, frame = self.camera.read()
            if ok and frame is not None:
                self.update_frame(frame)
            time.sleep(0.03)

    def _release_camera(self):
        self._camera_running = False
        if self.camera is not None:
            try:
                self.camera.release()
            except Exception:
                pass
            self.camera = None

    def _step(self):
        s = self.state

        if s == State.IDLE:
            self._step_idle()
        elif s == State.SCAN_360:
            self._step_scan()
        elif s == State.NAVIGATE_TAG:
            self._step_navigate_tag()
        elif s == State.DETECT_CUBE:
            self._step_detect_cube()
        elif s == State.NAVIGATE_CUBE:
            self._step_navigate_cube()
        elif s == State.OPEN_GRIPPER:
            self._step_open_gripper()
        elif s == State.APPROACH_CUBE:
            self._step_approach()
        elif s == State.CLOSE_GRIPPER:
            self._step_close_gripper()
        elif s == State.NAVIGATE_DROP:
            self._step_navigate_drop()
        elif s == State.RELEASE:
            self._step_release()
        elif s == State.BACK_HOME:
            self._step_back_home()
        elif s == State.RECORD:
            self._step_record()
        elif s == State.AVOID:
            self._step_avoid()
        elif s == State.STUCK:
            self._step_stuck()
        elif s == State.ERROR:
            self.send_velocity(0.0, 0.0)

        self._record_step(s)

    # ─── ÉTAPES ───────────────────────────────────────────────────────────────

    def _step_idle(self):
        self.send_velocity(0.0, 0.0)
        if self.running:
            self._set_state(State.SCAN_360)

    def _step_scan(self):
        """Rotation 360° pour cartographier les tags et détecter les cubes."""
        if self.scan_start_yaw is None:
            self.scan_start_yaw  = self.yaw_deg
            self.scan_turned_deg = 0.0
            self.log("Scan 360° démarré")

        # Mesure incrément yaw
        delta = abs(self._angle_diff(self.yaw_deg, self.scan_start_yaw))
        self.scan_turned_deg = max(self.scan_turned_deg, delta)

        self.send_velocity(0.0, SCAN_OMEGA)

        # Tentative détection tag pendant le scan
        tag_id, tag_dist = self._detect_nearest_tag()
        if tag_id is not None:
            self.log(f"Tag {tag_id} vu à {tag_dist:.2f}m")

        if self.scan_turned_deg >= 355.0:
            self.send_velocity(0.0, 0.0)
            self.scan_start_yaw = None
            self.log("Scan 360° terminé")
            # Choisir prochaine mission : priorité blue → cyan
            self._choose_mission()

    def _choose_mission(self):
        """Choisit la couleur cible et le tag pickup."""
        hint = self.lstm.get_latest_hint()
        hinted_color, hinted_tag = None, None
        if hint and float(hint.get("confidence", 0.0)) >= self.lstm.confidence_threshold:
            hinted_color, hinted_tag = self._hint_to_target(str(hint.get("target", "NONE")))

        # Utilise le hint seulement s'il pointe vers un pickup valide.
        if hinted_color in ("blue", "cyan") and hinted_tag in (2, 3):
            self.target_color = hinted_color
            self.target_tag_id = hinted_tag
            self._last_lstm_hint = hint
            self.log(f"LSTM hint accepté: {self.target_color} -> tag {self.target_tag_id}")
        else:
            self._last_lstm_hint = hint
            # Alternance blue/cyan pour équilibre
            if self.mission_count["blue"] <= self.mission_count["cyan"]:
                self.target_color  = "blue"
                self.target_tag_id = 2
            else:
                self.target_color  = "cyan"
                self.target_tag_id = 3
        self.log(f"Mission : ramasser cube {self.target_color} (tag {self.target_tag_id})")
        self._set_state(State.NAVIGATE_TAG)

    def _step_navigate_tag(self):
        """Navigation vers le tag pickup."""
        if self._check_obstacle():
            return
        tag_info = TAG_MAP.get(self.target_tag_id)
        if tag_info is None:
            self._set_state(State.ERROR)
            return
        arrived, lin, ang = self._navigate_to(tag_info["x"], tag_info["y"])
        if arrived:
            self.send_velocity(0.0, 0.0)
            self.log(f"Arrivé au tag {self.target_tag_id}")
            self._set_state(State.DETECT_CUBE)
        else:
            self.send_velocity(lin, ang)

    def _step_detect_cube(self):
        """Détection couleur cube dans l'image courante."""
        self.send_velocity(0.0, 0.0)
        frame = self._get_frame()
        if frame is None:
            return

        color_range = COLOR_RANGES.get(self.target_color)
        if color_range is None:
            self._set_state(State.ERROR)
            return

        cx, dist_cm = self._detect_color_cube(frame, color_range)
        if cx is not None:
            self.cube_pixel_x = cx
            self.cube_dist_cm = dist_cm
            self.log(f"Cube {self.target_color} détecté, dist={dist_cm:.1f}cm px={cx}")
            self._set_state(State.OPEN_GRIPPER)
        else:
            # Tourne légèrement pour chercher
            self.send_velocity(0.0, 0.2)

    def _step_open_gripper(self):
        self.send_velocity(0.0, 0.0)
        self.send_gripper(GRIPPER_OPEN)
        self.log("Pince ouverte")
        time.sleep(0.5)
        self._set_state(State.NAVIGATE_CUBE)

    def _step_navigate_cube(self):
        """Aligne et avance vers le cube avec feedback caméra."""
        if self._check_obstacle():
            return
        frame = self._get_frame()
        if frame is None:
            self.send_velocity(NAV_LINEAR_SPEED * 0.5, 0.0)
            return

        color_range = COLOR_RANGES.get(self.target_color)
        cx, dist_cm = self._detect_color_cube(frame, color_range)

        if cx is None:
            # Cube perdu — tourne pour retrouver
            self.send_velocity(0.0, 0.2)
            return

        self.cube_pixel_x = cx
        self.cube_dist_cm = dist_cm
        w = frame.shape[1]
        error_px = cx - w / 2
        ang = -float(error_px) / (w / 2) * 0.8   # rad/s proportionnel

        if dist_cm <= APPROACH_DIST_CM:
            self.send_velocity(0.0, 0.0)
            self._set_state(State.APPROACH_CUBE)
        else:
            speed = min(NAV_LINEAR_SPEED, (dist_cm - APPROACH_DIST_CM) / 100.0)
            self.send_velocity(speed, ang)

    def _step_approach(self):
        """Avance doucement pour bien positionner le cube dans la pince."""
        if self.us[0] > 0 and self.us[0] < APPROACH_DIST_CM:
            self.send_velocity(0.0, 0.0)
            self._set_state(State.CLOSE_GRIPPER)
        else:
            self.send_velocity(0.05, 0.0)   # avance très lentement

    def _step_close_gripper(self):
        self.send_velocity(0.0, 0.0)
        self.send_gripper(GRIPPER_CLOSE)
        self.log("Pince fermée — cube saisi")
        time.sleep(0.6)
        # Recule légèrement pour dégager
        self.send_velocity(-0.08, 0.0)
        time.sleep(0.8)
        self.send_velocity(0.0, 0.0)
        self._set_state(State.NAVIGATE_DROP)

    def _step_navigate_drop(self):
        """Navigation vers la zone de dépôt correspondant à la couleur."""
        if self._check_obstacle():
            return
        drop_tag_id = DROP_TAG.get(self.target_color)
        drop_info   = TAG_MAP.get(drop_tag_id)
        if drop_info is None:
            self._set_state(State.ERROR)
            return
        arrived, lin, ang = self._navigate_to(drop_info["x"], drop_info["y"])
        if arrived:
            self.send_velocity(0.0, 0.0)
            self.log(f"Arrivé zone dépôt {self.target_color} (tag {drop_tag_id})")
            self._set_state(State.RELEASE)
        else:
            self.send_velocity(lin, ang)

    def _step_release(self):
        self.send_velocity(0.0, 0.0)
        self.send_gripper(GRIPPER_OPEN)
        self.log(f"Cube {self.target_color} déposé")
        time.sleep(0.5)
        # Recule pour dégager la zone de dépôt
        self.send_velocity(-0.10, 0.0)
        time.sleep(1.0)
        self.send_velocity(0.0, 0.0)
        self.mission_count["total"] += 1
        self.mission_count[self.target_color] += 1
        self._set_state(State.RECORD)

    def _step_back_home(self):
        if self._check_obstacle():
            return
        home = TAG_MAP[1]
        arrived, lin, ang = self._navigate_to(home["x"], home["y"])
        if arrived:
            self.send_velocity(0.0, 0.0)
            self._set_state(State.RECORD)
        else:
            self.send_velocity(lin, ang)

    def _step_record(self):
        """Sauvegarde la trajectoire pour le LSTM futur."""
        if self.trajectory_log:
            self.all_trajectories.append(list(self.trajectory_log))
            self.trajectory_log.clear()
        self.log(f"Mission #{self.mission_count['total']} enregistrée "
                 f"(blue={self.mission_count['blue']}, cyan={self.mission_count['cyan']})")
        # Relance
        self.target_tag_id = None
        self.target_color  = None
        self.cube_pixel_x  = None
        self.cube_dist_cm  = None
        self._set_state(State.SCAN_360)

    def _step_avoid(self):
        """Contournement obstacle : tourne jusqu'à dégager."""
        if self.avoid_timer is None:
            self.avoid_timer = time.time()
            self.log("Évitement obstacle")
            self.send_velocity(0.0, 0.6)
        if time.time() - self.avoid_timer > 2.0:
            self.avoid_timer = None
            self._set_state(self._prev_state)

    def _step_stuck(self):
        """Coincé — rotation 180°."""
        self.log("Robot coincé — rotation 180°")
        self.send_velocity(0.0, 0.8)
        time.sleep(math.pi / 0.8)
        self.send_velocity(0.0, 0.0)
        self._set_state(State.SCAN_360)

    # ─── NAVIGATION ───────────────────────────────────────────────────────────

    def _navigate_to(self, tx: float, ty: float):
        """
        Retourne (arrived, linear_speed, angular_speed).
        Navigation proportionnelle simple cap→avance.
        """
        ox  = self.odom["x"]
        oy  = self.odom["y"]
        dx  = tx - ox
        dy  = ty - oy
        dist = math.hypot(dx, dy)

        if dist < NAV_DIST_THRESHOLD:
            return True, 0.0, 0.0

        target_angle = math.degrees(math.atan2(dy, dx))
        angle_err    = self._angle_diff(target_angle, self.yaw_deg)

        # D'abord aligner, puis avancer
        if abs(angle_err) > NAV_ANGLE_THRESHOLD:
            ang = 1.0 * angle_err / 45.0
            ang = max(-1.2, min(1.2, ang))
            return False, 0.0, ang
        else:
            lin = min(NAV_LINEAR_SPEED, dist * 0.6)
            ang = 0.4 * angle_err / NAV_ANGLE_THRESHOLD
            return False, lin, ang

    def _check_obstacle(self) -> bool:
        """Vérifie obstacle imminent. Retourne True si on doit s'arrêter."""
        front = self.us[0] if self.us[0] > 0 else 999
        if front < STOP_DIST_CM:
            self.send_velocity(0.0, 0.0)
            self._prev_state = self.state
            self._set_state(State.AVOID)
            return True

        # Détection coincé: uniquement si au moins un capteur valide est présent
        valid_us = [v for v in self.us if v > 0]
        all_blocked = bool(valid_us) and all(v < STOP_DIST_CM * 2 for v in valid_us)
        if all_blocked:
            self.send_velocity(0.0, 0.0)
            self._set_state(State.STUCK)
            return True
        return False

    # ─── VISION ───────────────────────────────────────────────────────────────

    def _get_frame(self):
        with self._cam_lock:
            return self.frame.copy() if self.frame is not None else None

    def _detect_color_cube(self, frame, color_range):
        """
        Retourne (center_x_pixel, distance_cm) ou (None, None).
        Distance estimée par la taille du blob dans l'image.
        """
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, color_range["lower"], color_range["upper"])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None, None
        biggest = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(biggest)
        if area < 400:   # trop petit = bruit
            return None, None
        M  = cv2.moments(biggest)
        cx = int(M["m10"] / M["m00"])
        # Estimation distance : cube 5cm, focal ~800px → dist = (5*800)/sqrt(area)
        dist_cm = (5.0 * 800.0) / max(math.sqrt(area), 1)
        return cx, dist_cm

    def _detect_nearest_tag(self):
        """Détecte le tag le plus proche dans le frame courant."""
        frame = self._get_frame()
        if frame is None:
            return None, None
        gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None:
            return None, None
        best_id, best_dist = None, float("inf")
        obj_pts = np.array([
            [-TAG_SIZE_M/2,  TAG_SIZE_M/2, 0],
            [ TAG_SIZE_M/2,  TAG_SIZE_M/2, 0],
            [ TAG_SIZE_M/2, -TAG_SIZE_M/2, 0],
            [-TAG_SIZE_M/2, -TAG_SIZE_M/2, 0],
        ], dtype=np.float32)
        for i, corner in enumerate(corners):
            _, tvec, _ = cv2.solvePnP(obj_pts, corner[0], CAM_MATRIX, DIST_COEFFS)
            dist = float(np.linalg.norm(tvec))
            if dist < best_dist:
                best_dist = dist
                best_id   = int(ids[i][0])
        return best_id, best_dist

    # ─── UTILITAIRES ──────────────────────────────────────────────────────────

    def _set_state(self, new_state: str):
        if self.state != new_state:
            self.log(f"État : {self.state} → {new_state}")
            self.state = new_state

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Différence d'angles normalisée dans [-180, 180]."""
        d = (a - b + 180) % 360 - 180
        return d

    def _mission_progress_percent(self) -> float:
        progress_map = {
            State.IDLE: 0.0,
            State.SCAN_360: 10.0,
            State.NAVIGATE_TAG: 20.0,
            State.DETECT_CUBE: 30.0,
            State.OPEN_GRIPPER: 40.0,
            State.NAVIGATE_CUBE: 50.0,
            State.APPROACH_CUBE: 60.0,
            State.CLOSE_GRIPPER: 70.0,
            State.NAVIGATE_DROP: 80.0,
            State.RELEASE: 90.0,
            State.BACK_HOME: 95.0,
            State.RECORD: 100.0,
            State.AVOID: 45.0,
            State.STUCK: 35.0,
            State.ERROR: 0.0,
        }
        return float(progress_map.get(self.state, 0.0))

    def _record_step(self, action: str):
        """Enregistre un pas pour le LSTM."""
        if self.state in (State.NAVIGATE_DROP, State.RELEASE):
            target_label = "DROP_BLUE" if self.target_color == "blue" else "DROP_CYAN" if self.target_color == "cyan" else "NONE"
        elif self.state == State.BACK_HOME:
            target_label = "HOME"
        elif self.target_color == "blue":
            target_label = "PICKUP_BLUE"
        elif self.target_color == "cyan":
            target_label = "PICKUP_CYAN"
        else:
            target_label = "NONE"

        sample = {
            "t":      time.time(),
            "state":  action,
            "next_state": self.state,
            "odom":   dict(self.odom),
            "yaw":    self.yaw_deg,
            "us":     list(self.us),
            "action": action,
            "target": target_label,
            "target_color": "NONE" if self.target_color is None else str(self.target_color),
            "target_tag_id": self.target_tag_id,
            "running": self.running,
            "linear_cmd": self._last_linear_cmd,
            "angular_cmd": self._last_angular_cmd,
            "cube_dist_cm": self.cube_dist_cm if self.cube_dist_cm is not None else -1.0,
            "cube_pixel_x": self.cube_pixel_x if self.cube_pixel_x is not None else -1.0,
            "color_confidence": 1.0 if self.cube_pixel_x is not None else 0.0,
            "tag_confidence": 1.0 if self.target_tag_id is not None else 0.0,
            "gripper_state": 1.0 if self._last_gripper_cmd in (GRIPPER_CLOSE, "close", 160) else 0.0,
            "mission_progress": self._mission_progress_percent(),
            "lstm_enabled": self.lstm.enabled,
            "recording_enabled": self.lstm.recording_enabled,
            "lstm_model_ready": self.lstm.model_ready,
        }
        self.trajectory_log.append(sample)
        self.lstm.observe(sample)
        self._last_lstm_hint = self.lstm.get_latest_hint()
