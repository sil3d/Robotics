#!/usr/bin/env python3
"""
===========================================================================
  MISSION ENGINE — Machine d'états autonome avec SLAM + A*
  Robot de tri de cubes colorés (bleu / vert) par AprilTag

  LOCALISATION :
    Camera + IMU (Arduino BMI160) → RobotTracker → position absolue (x, y, yaw)
    Tags fixés au sol → SLAM scan 360° → carte des 12 tags
    Optical Flow entre les tags → dead reckoning

  CYCLE MISSION (2 cubes par cycle) :
    IDLE → SCAN_360 (SLAM carte) → NAVIGATE_WAYPOINT (A* vers Manufacture)
         → DETECT_CUBE (bleu ou vert) → NAVIGATE_CUBE → APPROACH → CLOSE_GRIPPER
         → NAVIGATE_WAYPOINT (A* vers Station A/B) → RELEASE
         → NAVIGATE_WAYPOINT (A* vers Manufacture) → ... (2e cube)
         → NAVIGATE_WAYPOINT (A* vers HOME) → RECORD → nouveau cycle

  A* PATHFINDING :
    Graphe complet des 12 tags → chemin le plus court
    Waypoint par waypoint avec localisation absolue

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

import json
import math
import time
import threading
import cv2
import os
import sys
import numpy as np
import heapq

from lstm_assistant import LSTMAssistant

# Add project root and grandparent to path for imports.
_PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(_PROJECT_DIR)         # Micro_ROS_Project
ROBOTICS_ROOT = os.path.dirname(PROJECT_ROOT)        # Robotics (racine du repo)
for _p in (PROJECT_ROOT, ROBOTICS_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from color_detection_test import ColorDetector, ArucoDetector, PROCESS_WIDTH, PROCESS_HEIGHT
from auto_detetc_tag_arduino import (
    ArduinoReader,
    AprilTagDetector,
    TagMapSLAM,
    RobotTracker,
    build_T_robot_cam,
)

# ─── CONFIG ────────────────────────────────────────────────────────────────────
# Tags dans l'environnement
# Structure: {tag_id: {"role": str, "x": float, "y": float}}
# role: "home" | "manufacture" | "drop_blue" | "drop_green"
TAG_MAP = {
    12: {"role": "home",         "x": 0.934,  "y": 0.241},
    3:  {"role": "manufacture",  "x": 1.151,  "y": 0.135},
    6:  {"role": "drop_blue",    "x": 0.944,  "y": 0.483},   # Station B
    9:  {"role": "drop_green",   "x": 0.374,  "y": 0.029},   # Station A
}

HOME_TAG     = 12
PICKUP_TAG   = 3
DROP_TAG     = {"blue": 6, "green": 9}

# Fichier de missions configurable
MISSIONS_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                             "data", "missions", "missions.json")

# Navigation
NAV_DIST_THRESHOLD  = 0.12   # m — considéré "arrivé" si < cette distance
NAV_ANGLE_THRESHOLD = 8.0    # deg — aligné si < cet angle
APPROACH_DIST_CM    = 12.0   # cm — distance finale pour saisir le cube
SCAN_OMEGA          = 0.4    # rad/s — vitesse rotation scan
NAV_LINEAR_SPEED    = 0.18   # m/s — vitesse navigation
STOP_DIST_CM        = 8.0    # cm — distance arrêt obstacle (firmware gère < 3cm)
STUCK_TIMEOUT_S     = 4.0    # s sans avancer = coincé

# Gripper: 180° = ouvert, 20° = fermé (saisie)
GRIPPER_OPEN  = 180
GRIPPER_CLOSE = 20
# Distance min pour ouvrir sans écarter la box (cm)
GRIPPER_SAFE_OPEN_DIST_CM = 15.0


# ─── A* PATHFINDING ──────────────────────────────────────────────────────────
def _tag_positions_from_map(tag_map):
    """Construit un dict {tag_id: (x_cm, y_cm)} depuis TagMapSLAM."""
    return {tid: (t["x"], t["y"]) for tid, t in tag_map.tags.items()}


def astar_path(tag_map, start_tid, goal_tid):
    """
    A* sur graphe complet de tags.
    Retourne liste de tag_ids [start, ..., goal] ou [] si pas de chemin.
    Tous les tags sont connectés entre eux (graphe complet).
    Poids = distance euclidienne.
    """
    if start_tid == goal_tid:
        return [start_tid]

    positions = _tag_positions_from_map(tag_map)
    if start_tid not in positions or goal_tid not in positions:
        return []

    def heuristic(tid_a, tid_b):
        ax, ay = positions[tid_a]
        bx, by = positions[tid_b]
        return math.hypot(ax - bx, ay - by)

    # Graphe complet : chaque tag est connecté à tous les autres
    open_set = [(heuristic(start_tid, goal_tid), 0.0, start_tid, [start_tid])]
    visited = set()

    while open_set:
        f, g, current, path = heapq.heappop(open_set)

        if current == goal_tid:
            return path

        if current in visited:
            continue
        visited.add(current)

        for neighbor in positions:
            if neighbor in visited:
                continue
            cost = heuristic(current, neighbor)
            new_g = g + cost
            new_f = new_g + heuristic(neighbor, goal_tid)
            heapq.heappush(open_set, (new_f, new_g, neighbor, path + [neighbor]))

    return []


def path_to_waypoints(tag_map, tag_path):
    """Convertit une liste de tag_ids en liste de (x_m, y_m)."""
    positions = _tag_positions_from_map(tag_map)
    return [(positions[tid][0] / 100.0, positions[tid][1] / 100.0) for tid in tag_path]


# ─── ÉTATS ─────────────────────────────────────────────────────────────────────
class State:
    IDLE            = "IDLE"
    SCAN_360        = "SCAN_360"
    NAVIGATE_WAYPOINT = "NAVIGATE_WAYPOINT"  # Suivi de chemin A* (waypoints)
    DETECT_CUBE     = "DETECT_CUBE"
    NAVIGATE_CUBE   = "NAVIGATE_CUBE"
    OPEN_GRIPPER    = "OPEN_GRIPPER"
    APPROACH_CUBE   = "APPROACH_CUBE"
    CLOSE_GRIPPER   = "CLOSE_GRIPPER"
    RELEASE         = "RELEASE"
    RECORD          = "RECORD"
    AVOID           = "AVOID"
    STUCK           = "STUCK"
    ERROR           = "ERROR"


class _DummyArduino:
    """Fallback quand l'Arduino n'est pas connecté."""
    def get(self):
        return (0.0, 0.0, 0.0, 0.0, 0.0, time.perf_counter())
    def reset_yaw(self):
        pass
    def stop(self):
        pass


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

        # Optimized detectors from color_detection_test.py
        self.color_detector = ColorDetector()
        self.aruco_detector = ArucoDetector()

        # ─── Localisation (depuis auto_detetc_tag_arduino.py) ─────────────
        self.tag_detector = AprilTagDetector()
        self.tag_map      = TagMapSLAM()
        self.tag_map.load_prior()  # charge les positions théoriques du plan
        self.tag_map.load()        # écrase/enrichit avec la carte SLAM sauvegardée

        # Arduino IMU (optionnel — le robot peut fonctionner sans)
        self.arduino = None
        try:
            self.arduino = ArduinoReader()
            self.log("Arduino IMU connecté")
        except Exception as e:
            self.log(f"Arduino non disponible: {e}")
            self.arduino = _DummyArduino()
            self.log("Mode sans IMU (yaw = 0)")

        # Tracker : localisation absolue par tags + optical flow + IMU
        self.tracker = RobotTracker(self.tag_map, self.arduino)

        # Mission variables
        self.target_tag_id    = None
        self.target_color     = None    # "blue" | "green"
        self.drop_tag_id      = None    # tag de dépôt choisi (6 ou 9)
        self.cube_pixel_x     = None    # pixel x du cube dans l'image
        self.cube_dist_cm     = None
        self.scan_start_yaw   = None
        self.scan_turned_deg  = 0.0
        self.nav_start_time   = None
        self.last_odom_x      = None
        self.last_odom_y      = None
        self.stuck_timer      = None
        self.avoid_timer      = None
        self.mission_count    = {"total": 0, "blue": 0, "green": 0}

        # ─── Mission queue & A* pathfinding ─────────────────────────────
        # Missions chargées depuis data/missions/missions.json
        self.missions_list     = []    # liste de missions
        self.missions_repeat   = True  # répéter le cycle
        self.missions_home_tag = HOME_TAG
        self.pickup_index      = 0     # index mission courante
        self.current_path     = []    # chemin A* courant (waypoints en mètres)
        self.current_path_idx = 0     # waypoint courant dans le path
        self.cycle_count      = 0     # nombre de cycles complets

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

        # Caméra locale (optionnelle)
        self.camera = self._open_camera(0)
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
        if hint_target == "DROP_BLUE":
            return "blue", DROP_TAG["blue"]
        if hint_target == "DROP_GREEN":
            return "green", DROP_TAG["green"]
        return None, None

    def start(self):
        """Lance la mission en boucle."""
        if self.running:
            return
        if self.camera is None:
            self.camera = self._open_camera(0)
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
            "target_color":  self.target_color or "none",
            "cube_dist_cm":  self.cube_dist_cm,
            "odom":          self.odom,
            "yaw":           self.yaw_deg,
            "us":            self.us,
            "us_limit":      self.us_limit,
            "mission_count": self.mission_count,
            "lstm":          self.lstm.get_status(),
            "lstm_hint":     self._last_lstm_hint,
            # Localisation SLAM
            "tracker_x":     self.tracker.x,
            "tracker_y":     self.tracker.y,
            "tracker_yaw":   math.degrees(self.tracker.yaw),
            "tracker_initialized": self.tracker.initialized,
            "tags_mapped":   len(self.tag_map.tags),
            "arduino_ok":    not isinstance(self.arduino, _DummyArduino),
            # A* pathfinding
            "current_path":  self.current_path,
            "path_idx":      self.current_path_idx,
            "missions":      self.missions_list,
            "pickup_index":  self.pickup_index,
            "cycle_count":   self.cycle_count,
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
                # ─── Localisation : met à jour le tracker à chaque tick ───
                self._update_tracker()
                self._step()
            except Exception as e:
                self.log(f"ERREUR état {self.state}: {e}")
                self._set_state(State.ERROR)
            time.sleep(rate)

    def _update_tracker(self):
        """Met à jour la position absolue du robot via tags + optical flow + IMU."""
        frame = self._get_frame()
        if frame is None:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.tag_detector.detect(gray)
        t_now = time.perf_counter()
        scan = (self.state == State.SCAN_360)
        self.tracker.update(detections, gray, t_now, scan_mode=scan)

        # Sync odom avec le tracker (en mètres pour le reste du code)
        self.odom["x"]   = self.tracker.x / 100.0
        self.odom["y"]   = self.tracker.y / 100.0
        self.odom["yaw"] = math.degrees(self.tracker.yaw)
        self.yaw_deg     = math.degrees(self.tracker.yaw)

    def _camera_loop(self):
        while self._camera_running and self.camera is not None:
            ok, frame = self.camera.read()
            if ok and frame is not None:
                self.update_frame(frame)
            time.sleep(0.03)

    def _open_camera(self, index: int):
        """Open the local camera with a Linux-friendly backend first.

        On Raspberry Pi / Linux, V4L2 is usually the best option.
        On Windows, DirectShow is usually faster and more stable.
        """
        backends = []
        if os.name == 'nt':
            backends = [(cv2.CAP_DSHOW, "DirectShow"), (cv2.CAP_ANY, "Auto")]
        else:
            backends = [(cv2.CAP_V4L2, "V4L2"), (cv2.CAP_ANY, "Auto")]

        for backend, name in backends:
            cap = cv2.VideoCapture(index, backend)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 30)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self.log(f"Caméra locale ouverte ({name})")
                return cap
        return cv2.VideoCapture(index)

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
        elif s == State.NAVIGATE_WAYPOINT:
            self._step_navigate_waypoint()
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
        elif s == State.RELEASE:
            self._step_release()
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
        """Rotation 360° SLAM : cartographie les tags avec positions absolues."""
        if self.scan_start_yaw is None:
            # Initialisation scan : reset position et yaw
            self.arduino.reset_yaw()
            time.sleep(0.1)
            # M5: clear seulement si aucun tag connu (premier scan ou map vide).
            # Si des tags sont déjà cartographiés (prior_map), on fusionne pour
            # ne pas perdre les confirmations précédentes.
            if not self.tag_map.tags:
                self.tag_map.tags.clear()
            self.tracker.reset_to(0.0, 0.0, 0.0)
            self.scan_start_yaw  = 0.0
            self.scan_turned_deg = 0.0
            self.log("Scan 360° SLAM démarré — tourne lentement")

        # Rotation lente
        self.send_velocity(0.0, SCAN_OMEGA)

        # Le tracker est déjà mis à jour en scan_mode par _update_tracker()
        # Il cartographie automatiquement les tags pendant la rotation

        # Mesure rotation
        yaw_deg = math.degrees(self.tracker.yaw)
        delta = abs(self._angle_diff(yaw_deg, self.scan_start_yaw))
        self.scan_turned_deg = max(self.scan_turned_deg, delta)

        if self.scan_turned_deg >= 355.0:
            self.send_velocity(0.0, 0.0)

            # Confirme tous les tags scannés
            for t in self.tag_map.tags.values():
                t["views"] = max(t["views"], 3)
                t["conf"] = 1.0

            self.tag_map.save()
            self.tracker.initialized = True
            self.tracker.reset_to(0.0, 0.0, self.tracker.yaw)

            self.scan_start_yaw = None
            self.log(f"Scan terminé : {len(self.tag_map.tags)} tags cartographiés")
            self._choose_mission()

    def load_missions(self):
        """Charge les missions depuis le fichier JSON."""
        try:
            with open(MISSIONS_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.missions_list = data.get("missions", [])
            self.missions_repeat = data.get("repeat", True)
            self.missions_home_tag = data.get("home_tag", HOME_TAG)
            self.log(f"Missions chargées : {len(self.missions_list)} missions depuis {MISSIONS_FILE}")
        except Exception as e:
            self.log(f"Erreur chargement missions: {e}")
            # Fallback : missions par défaut
            self.missions_list = [
                {"pickup_tag": PICKUP_TAG, "drop_tag": DROP_TAG["blue"], "color": "blue", "label": "Cube bleu → Station B"},
                {"pickup_tag": PICKUP_TAG, "drop_tag": DROP_TAG["green"], "color": "green", "label": "Cube vert → Station A"},
            ]
            self.missions_repeat = True
            self.missions_home_tag = HOME_TAG

    def _choose_mission(self):
        """Lance le cycle de mission depuis le fichier de config."""
        self.pickup_index = 0
        self.load_missions()

        if not self.missions_list:
            self.log("Aucune mission configurée !")
            self._set_state(State.IDLE)
            return

        self.log(f"Cycle : {len(self.missions_list)} missions, repeat={self.missions_repeat}")
        self._advance_to_next_pickup()

    def _advance_to_next_pickup(self):
        """Calcule le prochain objectif depuis la liste de missions."""
        if self.pickup_index >= len(self.missions_list):
            # Toutes les missions du cycle faites
            if self.missions_repeat:
                self.cycle_count += 1
                self.log(f"Cycle #{self.cycle_count} terminé, restart")
                self.pickup_index = 0
                self.load_missions()  # recharge pour avoir les modifs en live
            else:
                self._navigate_to_tag(self.missions_home_tag, "HOME")
                self._set_state(State.RECORD)
                return

        if self.pickup_index >= len(self.missions_list):
            self.log("Plus de missions → IDLE")
            self._set_state(State.IDLE)
            return

        mission = self.missions_list[self.pickup_index]
        pickup_tag = mission["pickup_tag"]
        drop_tag = mission["drop_tag"]
        color = mission.get("color", "blue")
        label = mission.get("label", f"Mission #{self.pickup_index + 1}")

        # Stocke les infos de la mission courante
        self.target_color = color
        self.drop_tag_id = drop_tag
        self.log(f"Mission : {label}")

        # Aller chercher le cube au pickup
        self._navigate_to_tag(pickup_tag, f"Pickup → {label}")

    def _navigate_to_tag(self, target_tid, label):
        """Calcule le chemin A* vers un tag et lance la navigation."""
        self.target_tag_id = target_tid
        self.log(f"Mission : {label} → tag {target_tid}")

        current_tid = self._nearest_tag()
        if current_tid is not None and current_tid != target_tid:
            tag_path = astar_path(self.tag_map, current_tid, target_tid)
            self.current_path = path_to_waypoints(self.tag_map, tag_path)
            self.current_path_idx = 0
            self.log(f"Chemin A* : {tag_path}")
        else:
            tag_info = TAG_MAP.get(target_tid)
            if tag_info:
                self.current_path = [(tag_info["x"], tag_info["y"])]
                self.current_path_idx = 0
            else:
                self.current_path = []

        self._set_state(State.NAVIGATE_WAYPOINT)

    def _nearest_tag(self):
        """Trouve le tag connu le plus proche du robot."""
        if not self.tag_map.tags:
            return None
        rx, ry = self.tracker.x, self.tracker.y  # en cm
        best_tid = None
        best_dist = float("inf")
        for tid, t in self.tag_map.tags.items():
            d = math.hypot(t["x"] - rx, t["y"] - ry)
            if d < best_dist:
                best_dist = d
                best_tid = tid
        return best_tid

    def _step_navigate_waypoint(self):
        """Suit le chemin A* waypoint par waypoint."""
        if self._check_obstacle():
            return

        if not self.current_path or self.current_path_idx >= len(self.current_path):
            # Chemin terminé — on est arrivé au tag cible
            self._on_arrived_at_target()
            return

        # Navigue vers le waypoint courant
        wx, wy = self.current_path[self.current_path_idx]
        arrived, lin, ang = self._navigate_to(wx, wy)

        if arrived:
            self.current_path_idx += 1
            if self.current_path_idx >= len(self.current_path):
                self._on_arrived_at_target()
            else:
                next_wx, next_wy = self.current_path[self.current_path_idx]
                self.log(f"Waypoint {self.current_path_idx}/{len(self.current_path)} → ({next_wx:.2f}, {next_wy:.2f})")
        else:
            self.send_velocity(lin, ang)

    def _on_arrived_at_target(self):
        """Appelé quand le robot arrive au tag cible via A*."""
        self.send_velocity(0.0, 0.0)
        target = self.target_tag_id
        self.log(f"Arrivé au tag {target}")

        if target == self.drop_tag_id:
            # Arrivé à la station → déposer le cube
            self._set_state(State.RELEASE)
        elif target == self.missions_home_tag:
            # Retour à HOME → enregistrer
            self._save_trajectory()
            self._advance_to_next_pickup()
        else:
            # Arrivé au pickup (ou autre tag) → chercher le cube
            self._set_state(State.DETECT_CUBE)

    def _step_detect_cube(self):
        """Détection du cube de la couleur définie par la mission."""
        self.send_velocity(0.0, 0.0)
        frame = self._get_frame()
        if frame is None:
            return

        # target_color est défini par _advance_to_next_pickup depuis la mission
        color = self.target_color
        if color is None:
            # Fallback : cherche les deux couleurs
            for c in ("blue", "green"):
                cx, dist_cm = self._detect_color_cube(frame, c)
                if cx is not None:
                    color = c
                    self.target_color = c
                    break

        if color is not None:
            cx, dist_cm = self._detect_color_cube(frame, color)
            if cx is not None:
                self.cube_pixel_x = cx
                self.cube_dist_cm = dist_cm
                self.log(f"Cube {color} détecté, dist={dist_cm:.1f}cm px={cx}")
                self._set_state(State.OPEN_GRIPPER)
                return

        # Aucun cube trouvé — tourne légèrement pour chercher
        # M4: timeout 15s → ERROR si toujours rien
        if not hasattr(self, '_detect_cube_start') or self._detect_cube_start is None:
            self._detect_cube_start = time.perf_counter()
        if time.perf_counter() - self._detect_cube_start > 15.0:
            self._detect_cube_start = None
            self.log("Timeout détection cube (15s) → ERROR")
            self._set_state(State.ERROR)
            return
        self.send_velocity(0.0, 0.2)

    def _step_open_gripper(self):
        self.send_velocity(0.0, 0.0)
        # Ici on ouvre pour aller saisir (180° déjà ouvert → on s'assure juste)
        # Pas de recul nécessaire : ouverture = bras déjà écartés (180°=ouvert)
        self.send_gripper(GRIPPER_OPEN)
        self.log("Pince ouverte")
        self._detect_cube_start = None  # reset timeout détection
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

        cx, dist_cm = self._detect_color_cube(frame, self.target_color)

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
        self.send_velocity(-0.08, 0.0)
        time.sleep(0.8)
        self.send_velocity(0.0, 0.0)
        # target_color et drop_tag_id déjà définis par _advance_to_next_pickup
        self._navigate_to_tag(self.drop_tag_id, f"Drop {self.target_color} (tag {self.drop_tag_id})")

    def _step_release(self):
        self.send_velocity(0.0, 0.0)
        # Reculer avant d'ouvrir si le robot est trop près de la box de dépôt
        # (évite que les bras du gripper écartent la box en s'ouvrant)
        front_dist = self.us[0] if self.us[0] > 0 else 999.0
        if front_dist < GRIPPER_SAFE_OPEN_DIST_CM:
            self.log(f"Recul avant ouverture gripper (dist={front_dist:.1f}cm < {GRIPPER_SAFE_OPEN_DIST_CM}cm)")
            self.send_velocity(-0.06, 0.0)
            time.sleep(0.8)
            self.send_velocity(0.0, 0.0)
        self.send_gripper(GRIPPER_OPEN)
        self.log(f"Cube {self.target_color} déposé")
        time.sleep(0.5)
        # Recule pour dégager la zone de dépôt
        self.send_velocity(-0.10, 0.0)
        time.sleep(1.0)
        self.send_velocity(0.0, 0.0)
        self.mission_count["total"] += 1
        color = self.target_color or "blue"
        self.mission_count[color] += 1
        self._save_trajectory()
        # Avancer au prochain pickup du cycle
        self.pickup_index += 1
        self._advance_to_next_pickup()

    def _save_trajectory(self):
        """Sauvegarde la trajectoire pour le LSTM."""
        if self.trajectory_log:
            self.all_trajectories.append(list(self.trajectory_log))
            self.trajectory_log.clear()
        self.log(f"Mission #{self.mission_count['total']} enregistrée "
                 f"(blue={self.mission_count['blue']}, green={self.mission_count['green']})")

    def _step_record(self):
        """Sauvegarde + avance à la prochaine mission."""
        self._save_trajectory()
        self.target_tag_id = None
        self.target_color  = None
        self.drop_tag_id   = None
        self.cube_pixel_x  = None
        self.cube_dist_cm  = None
        self._advance_to_next_pickup()

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
        Utilise le tracker SLAM pour la position absolue.
        """
        # Position absolue depuis le tracker (tags + optical flow + IMU)
        ox  = self.tracker.x / 100.0   # tracker est en cm, navigate en mètres
        oy  = self.tracker.y / 100.0
        dx  = tx - ox
        dy  = ty - oy
        dist = math.hypot(dx, dy)

        if dist < NAV_DIST_THRESHOLD:
            return True, 0.0, 0.0

        target_angle = math.degrees(math.atan2(dy, dx))
        yaw_deg = math.degrees(self.tracker.yaw)
        angle_err = self._angle_diff(target_angle, yaw_deg)

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

    def _detect_color_cube(self, frame, target_color):
        """
        Retourne (center_x_pixel, distance_cm) ou (None, None).
        target_color: "blue" ou "green"
        Utilise le détecteur HSV optimisé partagé avec le reste du projet.
        """
        frame_small = cv2.resize(frame, (PROCESS_WIDTH, PROCESS_HEIGHT), interpolation=cv2.INTER_LINEAR)
        result = self.color_detector.detect(frame_small)
        if not result:
            return None, None

        # Cherche le cube de la bonne couleur
        color_key = "blue" if target_color == "blue" else "green"
        box = result.get(f"{color_key}_box")
        if box is None:
            return None, None

        cx = int(box["center_x"])
        dist_cm = float(box["distance_m"]) * 100.0
        return cx, dist_cm

    def _detect_nearest_tag(self):
        """Détecte le tag le plus proche dans le frame courant."""
        frame = self._get_frame()
        if frame is None:
            return None, None
        frame_small = cv2.resize(frame, (PROCESS_WIDTH, PROCESS_HEIGHT), interpolation=cv2.INTER_LINEAR)
        gray = cv2.cvtColor(frame_small, cv2.COLOR_BGR2GRAY)
        tags = self.aruco_detector.detect(gray)
        if not tags:
            return None, None

        best = min(tags, key=lambda tag: float(tag.get("distance", float("inf"))))
        return int(best["id"]), float(best["distance"])

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
            State.NAVIGATE_WAYPOINT: 75.0,
            State.RELEASE: 90.0,
            State.RECORD: 100.0,
            State.AVOID: 45.0,
            State.STUCK: 35.0,
            State.ERROR: 0.0,
        }
        return float(progress_map.get(self.state, 0.0))

    def _record_step(self, action: str):
        """Enregistre un pas pour le LSTM."""
        if self.state in (State.NAVIGATE_WAYPOINT, State.RELEASE):
            target_label = "DROP_BLUE" if self.target_color == "blue" else "DROP_GREEN" if self.target_color == "green" else "HOME" if self.target_tag_id == HOME_TAG else "NAVIGATE"
        elif self.target_tag_id == PICKUP_TAG:
            target_label = "PICKUP"
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
            "target_color": self.target_color or "NONE",
            "target_tag_id": self.target_tag_id,
            "drop_tag_id": self.drop_tag_id,
            "running": self.running,
            "linear_cmd": self._last_linear_cmd,
            "angular_cmd": self._last_angular_cmd,
            "cube_dist_cm": self.cube_dist_cm if self.cube_dist_cm is not None else -1.0,
            "cube_pixel_x": self.cube_pixel_x if self.cube_pixel_x is not None else -1.0,
            "color_confidence": 1.0 if self.cube_pixel_x is not None else 0.0,
            "tag_confidence": 1.0 if self.target_tag_id is not None else 0.0,
            "gripper_state": 1.0 if self._last_gripper_cmd in (GRIPPER_CLOSE, "close", 20) else 0.0,
            "mission_progress": self._mission_progress_percent(),
            "lstm_enabled": self.lstm.enabled,
            "recording_enabled": self.lstm.recording_enabled,
            "lstm_model_ready": self.lstm.model_ready,
        }
        self.trajectory_log.append(sample)
        self.lstm.observe(sample)
        self._last_lstm_hint = self.lstm.get_latest_hint()
