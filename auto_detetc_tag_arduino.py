"""
============================================================================
  APRILTAG + IMU SLAM — RASPBERRY PI / PC (VERSION CORRIGÉE)
  
  Matériel :
  • Caméra USB (640×480)
  • Arduino + BMI160 (envoie : IMU,yaw,omega_z,ax,ay,az)
  • 3–6 AprilTags posés au sol/murs
  
  Fonctionnement :
  1. Scan 360° au centre ([S] → tourne le robot → [E])
  2. Le robot cartographie tous les tags dans la pièce
  3. En mode NAV :
     - Tag visible → localisation absolue instantanée (parfait)
     - Pas de tag → Optical Flow suit le mouvement du sol
     - WASD → mode manuel (pour tester sans capteurs)
  4. [Q] pour quitter, la carte est sauvegardée dans tags_slam.json
============================================================================
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import time
import threading
import json
import math
import os
import sys
import serial
import serial.tools.list_ports
from collections import deque


# ═══════════════════════════════════════════════════════════════════════════
# CONFIG — AJUSTE CES VALEURS SELON TON SETUP
# ═══════════════════════════════════════════════════════════════════════════

CAMERA_INDEX       = 1
FRAME_WIDTH        = 640
FRAME_HEIGHT       = 480

APRILTAG_DICT     = cv2.aruco.DICT_4X4_250
APRILTAG_SIZE_CM   = 10.0
MAX_DISTANCE_M     = 5.0

# ─── Intrinsèques caméra — chargées depuis data/camera_calibration/camera_calibration.json ──────
_CAL_FILE = os.path.join(os.path.dirname(__file__), "data", "camera_calibration", "camera_calibration.json")
if os.path.exists(_CAL_FILE):
    with open(_CAL_FILE) as _f:
        _cal = json.load(_f)
    CAM_MATRIX       = np.array(_cal["camera_matrix"], dtype=np.float32)
    DIST_COEFFS      = np.array(_cal["dist_coeffs"],   dtype=np.float32)
    APRILTAG_SIZE_CM = float(_cal.get("apriltag_size_cm", APRILTAG_SIZE_CM))
else:
    CAM_MATRIX = np.array([
        [828.395, 0.0,     337.460],
        [0.0,     812.656, 213.622],
        [0.0,     0.0,     1.0]
    ], dtype=np.float32)
    DIST_COEFFS = np.array([[-1.436, 14.760, -0.00570, 0.0543, -37.111]], dtype=np.float32)

# ─── Offset Caméra → Robot ─────────────────────────────────────────────────
# Repère robot : X=avant, Y=gauche, Z=haut
CAM_FORWARD_CM     = 15.0   # cm devant le centre robot
CAM_LEFT_CM        = 0.0    # cm à gauche (0 = centré)
CAM_HEIGHT_CM      = 25.0   # cm au-dessus du sol
CAM_YAW_OFFSET_DEG = 0.0    # rotation horizontale caméra (rarement utile)
CAM_PITCH_DEG      = -45.0  # ↓ NÉGATIF = caméra penchée VERS LE BAS

# ─── SLAM ──────────────────────────────────────────────────────────────────
MIN_TAG_PX_DIAG    = 8
TAG_CONFIRM_VIEWS  = 3
MAP_FILE           = os.path.join(os.path.dirname(__file__), "data", "tags_slam", "tags_slam.json")

# ─── PRIOR MAP (plan physique) ───────────────────────────────────────────────
# Origine = ID12 (Home, mur bas centre)
# Unités : cm.  X = droite, Y = haut (vue de dessus)
# Seuil de matching : si le scan donne une position à moins de PRIOR_MATCH_CM
# de la prior, on pondère avec la prior.  Au-delà, la prior écrase (fausse détection).
PRIOR_MATCH_CM = 35.0   # cm — tolérance matching scan ↔ prior
PRIOR_WEIGHT   = 0.65   # poids donné à la prior (0=scan brut, 1=prior pure)

PRIOR_MAP = {
    #  ID : (x_cm, y_cm, role)
    #  Source : scan confirmé (12_tags.txt scan 2), recentré sur ID12=(0,0)
    #  Formule : x = x_scan - 93.4,  y = y_scan - 24.1
    12: (   0.0,   0.0, "home"),
    3:  (  21.7, -10.6, "manufacture"),
    6:  (   1.0,  24.2, "drop_blue"),   # Station B
    9:  ( -56.0, -21.2, "drop_green"),  # Station A
    1:  (  27.2,  22.7, "wall"),
    2:  (  25.0, -54.1, "wall"),
    4:  (   8.8, -39.9, "wall"),
    7:  ( -28.6, -13.1, "wall"),
    8:  ( -33.8,   4.5, "wall"),
    10: (  -3.3,   6.6, "wall"),
    11: ( -12.9,  -5.1, "wall"),
    5:  ( -40.9,   9.2, "wall"),
    # ID17 volontairement absent → ignoré au scan
}

# ─── Optical Flow ──────────────────────────────────────────────────────────
# Conversion pixel → cm au sol (à ajuster si le déplacement semble faux)
# Formule approx : CAM_HEIGHT_CM / fy  →  25 / 321 ≈ 0.078
PIXEL_TO_CM_SCALE  = 0.08
OF_MAX_CORNERS     = 80
OF_QUALITY         = 0.3
OF_MIN_DIST        = 7
OF_WIN_SIZE        = (15, 15)

# ─── Mode Manuel (test sans capteurs) ──────────────────────────────────────
MANUAL_SPEED_CM_S  = 30.0     # vitesse avant/arrière
MANUAL_TURN_RAD_S  = 1.2      # vitesse rotation


# ═══════════════════════════════════════════════════════════════════════════
# MATHS UTILITAIRES
# ═══════════════════════════════════════════════════════════════════════════

def rodrigues_to_matrix(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3]  = tvec.flatten()
    return T

def yaw_to_matrix(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)

def mean_angle(angles_rad: list) -> float:
    if len(angles_rad) == 0:
        return 0.0
    return np.arctan2(np.mean(np.sin(angles_rad)), np.mean(np.cos(angles_rad)))

def build_T_robot_cam() -> np.ndarray:
    """
    Repère caméra OpenCV → repère robot.
    OpenCV : X=droite, Y=bas, Z=avant (vers la scène)
    Robot  : X=avant, Y=gauche, Z=haut
    """
    # Changement de base : Z_cam→X_robot, X_cam→-Y_robot, Y_cam→-Z_robot
    R_base = np.array([[ 0,  0,  1],
                       [-1,  0,  0],
                       [ 0, -1,  0]], dtype=np.float64)

    # Pitch : caméra penchée vers le bas (autour de Y_robot)
    p = np.radians(CAM_PITCH_DEG)
    R_pitch = np.array([[np.cos(p), 0, np.sin(p)],
                        [0, 1, 0],
                        [-np.sin(p), 0, np.cos(p)]], dtype=np.float64)

    # Yaw offset caméra (autour de Z_robot)
    y = np.radians(CAM_YAW_OFFSET_DEG)
    R_yaw = np.array([[np.cos(y), -np.sin(y), 0],
                      [np.sin(y),  np.cos(y), 0],
                      [0, 0, 1]], dtype=np.float64)

    R = R_yaw @ R_pitch @ R_base

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3]  = [CAM_FORWARD_CM, CAM_LEFT_CM, CAM_HEIGHT_CM]
    return T


# ═══════════════════════════════════════════════════════════════════════════
# ARDUINO READER (thread série)
# ═══════════════════════════════════════════════════════════════════════════

class ArduinoReader:
    def __init__(self, port=None, baud=115200):
        self.yaw_deg = 0.0
        self.omega_z = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.last_t = time.perf_counter()
        self.stopped = False
        self.lock = threading.Lock()

        # Auto-détection port
        if port is None:
            ports = list(serial.tools.list_ports.comports())
            candidates = [p.device for p in ports
                          if any(x in p.description for x in ['Arduino', 'USB', 'Serial'])
                          or 'ACM' in p.device or 'ttyUSB' in p.device]
            if not candidates:
                print("[ARDUINO] Ports disponibles :")
                for p in ports:
                    print(f"  {p.device} : {p.description}")
                raise RuntimeError("Aucun Arduino trouvé. Branche-le ou spécifie le port.")
            port = candidates[0]
            print(f"[ARDUINO] Auto-detect : {port}")

        self.ser = serial.Serial(port, baud, timeout=0.5)
        time.sleep(1.5)  # Attente reset Arduino
        # Vérifier que l'Arduino envoie des données avant de démarrer le thread
        print("[ARDUINO] Vérification connexion...")
        start_check = time.time()
        data_received = False
        while time.time() - start_check < 10.0:  # 10 sec pour ESP32 boot + WiFi + IMU calib
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("IMU,"):
                        data_received = True
                        print(f"[ARDUINO] Données reçues: {line}")
                        break
                except:
                    pass
            time.sleep(0.1)
        if not data_received:
            print("[ARDUINO] ⚠️ Aucune donnée IMU reçue après 10s - vérifie le firmware Arduino")
            print("[ARDUINO] L'IMU est-elle bien connectée ? (I2C SDA=21 SCL=22)")
            print("[ARDUINO] Réessaye en appuyant sur RESET de l'ESP32")
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not self.stopped:
            try:
                # Ne bloque pas indéfiniment - vérifie d'abord s'il y a des données
                if self.ser.in_waiting == 0:
                    time.sleep(0.005)
                    continue
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line.startswith("IMU,"):
                    continue
                parts = line.split(',')
                if len(parts) != 6:
                    continue
                _, yaw, omega, ax, ay, az = parts
                with self.lock:
                    self.yaw_deg = float(yaw)
                    self.omega_z = float(omega)
                    self.ax = float(ax)
                    self.ay = float(ay)
                    self.az = float(az)
                    self.last_t = time.perf_counter()
            except Exception:
                pass
            time.sleep(0.001)

    def get(self):
        with self.lock:
            return (self.yaw_deg, self.omega_z, self.ax, self.ay, self.az, self.last_t)

    def reset_yaw(self):
        try:
            self.ser.write(b'R\n')
            print("[ARDUINO] Yaw reset envoyé")
        except Exception as e:
            print(f"[ARDUINO] Reset fail: {e}")

    def stop(self):
        self.stopped = True
        self.ser.close()


# ═══════════════════════════════════════════════════════════════════════════
# AUTO-DÉTECTION CAMÉRA
# ═══════════════════════════════════════════════════════════════════════════

def find_camera_index(preferred=1, max_index=3):
    """Trouve une caméra fonctionnelle rapidement.
    Sur Windows, essaie d'abord DirectShow (cv2.CAP_DSHOW).
    """
    print("[CAMERA] Recherche rapide...")
    
    # Indices à tester : préféré d'abord, puis 0, 1, 2, 3
    indices = [preferred] + [i for i in range(max_index + 1) if i != preferred]
    
    for cap_idx in indices:
        # Essayer DirectShow (plus rapide sur Windows)
        cap = cv2.VideoCapture(cap_idx, cv2.CAP_DSHOW)
        if cap.isOpened():
            # Une seule frame suffit
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                h, w = frame.shape[:2]
                print(f"[CAMERA] OK — index {cap_idx} ({w}x{h})")
                cap.release()
                return cap_idx
            cap.release()
        
        # Fallback sans backend
        cap = cv2.VideoCapture(cap_idx)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                h, w = frame.shape[:2]
                print(f"[CAMERA] OK — index {cap_idx} ({w}x{h})")
                cap.release()
                return cap_idx
            cap.release()
    
    print("[CAMERA] ⚠️ Aucune caméra trouvée !")
    return preferred


# ═══════════════════════════════════════════════════════════════════════════
# THREAD CAMÉRA (anti-lag buffer USB)
# ═══════════════════════════════════════════════════════════════════════════

class VideoCaptureThread:
    def __init__(self, src=0, width=640, height=480):
        # Windows: utiliser DirectShow pour ouverture plus rapide
        import sys
        if sys.platform == 'win32':
            self.cap = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Paramètres par défaut de la caméra
        # Note: Les réglages spécifiques sont supprimés - on utilise les défauts
        # car la lumière de la pièce est fixe
        self.ret = False
        self.frame = None
        self.stopped = False
        self.lock = threading.Lock()
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            with self.lock:
                self.ret = ret
                if frame is not None:
                    self.frame = frame
            time.sleep(0.001)

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return self.ret, self.frame.copy()

    def stop(self):
        self.stopped = True
        self.cap.release()


# ═══════════════════════════════════════════════════════════════════════════
# DÉTECTEUR ARUCO (léger pour RPi)
# ═══════════════════════════════════════════════════════════════════════════

class AprilTagDetector:
    def __init__(self, tag_size_cm=APRILTAG_SIZE_CM):
        self.dict = aruco.getPredefinedDictionary(APRILTAG_DICT)
        self.params = aruco.DetectorParameters()
        self.params.adaptiveThreshWinSizeMin   = 3
        self.params.adaptiveThreshWinSizeMax   = 43
        self.params.adaptiveThreshWinSizeStep  = 5
        self.params.adaptiveThreshConstant     = 9
        self.params.minMarkerPerimeterRate     = 0.015
        self.params.polygonalApproxAccuracyRate = 0.04
        self.params.minCornerDistanceRate      = 0.03
        self.params.minDistanceToBorder        = 2
        self.params.cornerRefinementMethod      = aruco.CORNER_REFINE_APRILTAG
        self.params.cornerRefinementWinSize     = 4
        self.params.cornerRefinementMaxIterations = 15
        self.params.cornerRefinementMinAccuracy  = 0.01
        self.detector = aruco.ArucoDetector(self.dict, self.params)

        half = tag_size_cm / 2.0
        self.obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

    def detect(self, gray):
        corners, ids, rejected = self.detector.detectMarkers(gray)
        results = []
        if ids is None:
            return results
        for i, tid in enumerate(ids.flatten()):
            tid = int(tid)
            c = corners[i][0]
            diag = float(np.linalg.norm(c[0] - c[2]))
            if diag < MIN_TAG_PX_DIAG:
                continue
            ok, rvec, tvec = cv2.solvePnP(
                self.obj_points, c, CAM_MATRIX, DIST_COEFFS,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not ok:
                continue
            tvec = tvec.flatten()
            if np.linalg.norm(tvec) > MAX_DISTANCE_M * 100.0:
                continue
            results.append({
                "tag_id": tid,
                "rvec": rvec.flatten(),
                "tvec": tvec,
                "corners": c,
                "px": int(diag),
            })
        return results


# ═══════════════════════════════════════════════════════════════════════════
# CARTE SLAM
# ═══════════════════════════════════════════════════════════════════════════

class TagMapSLAM:
    def __init__(self, prior_map=None):
        self.tags = {}
        self.prior = prior_map if prior_map is not None else PRIOR_MAP

    def load_prior(self):
        """Pre-populates the map with prior positions (conf=0) so scan can match."""
        for tid, (px, py, role) in self.prior.items():
            if tid not in self.tags:
                self.tags[tid] = {
                    "x": px, "y": py, "z": 0.0, "yaw": 0.0,
                    "views": 0, "conf": 0.0, "role": role
                }
        print(f"[SLAM] Prior chargée : {len(self.prior)} tags connus")

    def load(self, path=MAP_FILE):
        if not os.path.exists(path):
            return False
        with open(path, "r") as f:
            data = json.load(f)
        for tid, vals in data.items():
            self.tags[int(tid)] = {
                "x": vals["x"], "y": vals["y"], "z": 0.0,
                "yaw": np.radians(vals["yaw_deg"]),
                "views": vals.get("views", 10),
                "conf": 1.0
            }
        print(f"[SLAM] Carte chargée : {path} ({len(self.tags)} tags)")
        return True

    def save(self, path=MAP_FILE):
        data = {}
        for tid, t in self.tags.items():
            data[tid] = {
                "x": float(t["x"]), "y": float(t["y"]), "z": 0.0,
                "yaw_deg": float(np.degrees(t["yaw"])),
                "views": int(t["views"]),
            }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"[SLAM] Carte sauvegardée : {path}")

    def get_pose_matrix(self, tid):
        if tid not in self.tags:
            return None
        t = self.tags[tid]
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = yaw_to_matrix(t["yaw"])
        T[:3, 3]  = [t["x"], t["y"], t["z"]]
        return T

    def add_or_update(self, tid, x, y, z, yaw, is_confirmed_view=True):
        # ── Ancrage prior : si l'ID est connu dans le plan, on pondère ──────
        if tid in self.prior:
            px, py, _ = self.prior[tid]
            dist = math.hypot(x - px, y - py)
            if dist < PRIOR_MATCH_CM:
                # Bon match : moyenne pondérée prior + scan
                x = PRIOR_WEIGHT * px + (1.0 - PRIOR_WEIGHT) * x
                y = PRIOR_WEIGHT * py + (1.0 - PRIOR_WEIGHT) * y
            else:
                # Trop loin de la prior = bruit/fausse déco → on garde la prior
                x, y = px, py
                print(f"[SLAM] Tag ID{tid} scan loin prior ({dist:.0f}cm) → prior utilisée")
        elif tid not in self.tags:
            # ID totalement inconnu (ni prior, ni carte) → on ignore
            print(f"[SLAM] Tag ID{tid} inconnu ignoré @ ({x:.1f}, {y:.1f})")
            return False

        if tid not in self.tags:
            self.tags[tid] = {
                "x": x, "y": y, "z": 0.0, "yaw": yaw,
                "views": 1, "conf": 0.2,
                "role": self.prior.get(tid, (0, 0, "unknown"))[2]
            }
            print(f"[SLAM] Tag ID{tid} découvert @ ({x:.1f}, {y:.1f})")
            return True

        t = self.tags[tid]
        alpha = 1.0 / (t["views"] + 1.0)
        t["x"] = (1 - alpha) * t["x"] + alpha * x
        t["y"] = (1 - alpha) * t["y"] + alpha * y
        t["z"] = 0.0
        t["yaw"] = np.arctan2(
            (1 - alpha) * np.sin(t["yaw"]) + alpha * np.sin(yaw),
            (1 - alpha) * np.cos(t["yaw"]) + alpha * np.cos(yaw)
        )
        if is_confirmed_view:
            t["views"] += 1
        t["conf"] = min(1.0, t["views"] / TAG_CONFIRM_VIEWS)
        if t["views"] == TAG_CONFIRM_VIEWS:
            print(f"[SLAM] Tag ID{tid} confirmé !")
        return False

    def is_known(self, tid):
        return tid in self.tags and self.tags[tid]["conf"] >= 0.5


# ═══════════════════════════════════════════════════════════════════════════
# OPTICAL FLOW ODOMÉTRY (déplacement visuel entre frames)
# ═══════════════════════════════════════════════════════════════════════════

class OpticalFlowOdometry:
    """
    Estime le déplacement relatif du robot entre deux frames
    en suivant les coins du sol avec Lucas-Kanade.
    """
    def __init__(self):
        self.prev_gray: np.ndarray | None = None
        self.prev_points: np.ndarray | None = None

    def update(self, gray: np.ndarray, robot_yaw: float) -> tuple[float, float]:
        """
        Retourne (dx_cm, dy_cm) dans le frame monde.
        robot_yaw : radians, orientation actuelle du robot
        """
        # Initialisation première frame
        if self.prev_gray is None:
            self.prev_gray = gray.copy()
            self.prev_points = self._find_points(gray)
            return 0.0, 0.0

        # Vérifier qu'on a assez de points
        if self.prev_points is None or len(self.prev_points) < 5:
            self.prev_gray = gray.copy()
            self.prev_points = self._find_points(gray)
            return 0.0, 0.0

        # ─── Lucas-Kanade ────────────────────────────────────────────────
        # On passe explicitement un ndarray pour nextPts (pas None) pour éviter
        # les problèmes de type avec certains stubs OpenCV
        nextPts: np.ndarray = np.empty_like(self.prev_points)
        
        next_points, status, err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            self.prev_points,
            nextPts,
            winSize=OF_WIN_SIZE,
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )

        if next_points is None or status is None:
            self.prev_gray = gray.copy()
            self.prev_points = self._find_points(gray)
            return 0.0, 0.0

        # Filtre les points valides
        status_bool = status.reshape(-1).astype(bool)
        good_prev = self.prev_points[status_bool]
        good_next = next_points[status_bool]

        if len(good_prev) < 5:
            self.prev_gray = gray.copy()
            self.prev_points = self._find_points(gray)
            return 0.0, 0.0

        # Déplacement moyen en pixels (frame caméra)
        # On inverse car si le sol bouge vers la gauche, le robot va à droite
        dx_px = -np.mean(good_next[:, 0, 0] - good_prev[:, 0, 0])
        dy_px = -np.mean(good_next[:, 0, 1] - good_prev[:, 0, 1])

        # Conversion pixel → cm (approximation plan sol)
        dx_cam_cm = dx_px * PIXEL_TO_CM_SCALE
        dy_cam_cm = dy_px * PIXEL_TO_CM_SCALE

        # Rotation : frame caméra → frame robot (le sol)
        # Dans OpenCV : X=droite, Y=bas
        # Dans le robot : X=avant, Y=gauche
        dx_robot = dy_cam_cm   # vers l'avant du robot (Y_cam = bas = avant)
        dy_robot = -dx_cam_cm  # vers la gauche du robot (X_cam = droite = -gauche)

        # Rotation robot → monde
        c, s = np.cos(robot_yaw), np.sin(robot_yaw)
        dx_world = dx_robot * c - dy_robot * s
        dy_world = dx_robot * s + dy_robot * c

        # Mise à jour pour prochaine frame
        self.prev_gray = gray.copy()
        self.prev_points = good_next.reshape(-1, 1, 2)

        return float(dx_world), float(dy_world)

    def _find_points(self, gray: np.ndarray) -> np.ndarray | None:
        """Trouve les coins à suivre (uniquement dans le bas de l'image = le sol)."""
        h, w = gray.shape
        # Masque : on ne regarde que le tiers inférieur (le sol)
        mask = np.zeros_like(gray)
        mask[int(h * 0.6):, :] = 255
        
        pts = cv2.goodFeaturesToTrack(
            gray, maxCorners=OF_MAX_CORNERS,
            qualityLevel=OF_QUALITY, minDistance=OF_MIN_DIST,
            mask=mask
        )
        return pts


# ═══════════════════════════════════════════════════════════════════════════
# ROBOT TRACKER (Tags + Optical Flow + IMU yaw + Manuel)
# ═══════════════════════════════════════════════════════════════════════════

class RobotTracker:
    def __init__(self, tag_map, arduino):
        self.map = tag_map
        self.arduino = arduino
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.initialized = False
        self.traj = deque(maxlen=500)

        self.T_robot_cam = build_T_robot_cam()
        self.T_cam_robot = np.linalg.inv(self.T_robot_cam)

        self.flow = OpticalFlowOdometry()
        self.last_t = time.perf_counter()

        # Mode manuel (pour test)
        self.manual_vx = 0.0      # cm/s dans le frame robot
        self.manual_vy = 0.0
        self.manual_omega = 0.0   # rad/s

    def reset_to(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.initialized = True
        self.traj.append((x, y))
        self.flow.prev_gray = None  # reset optical flow
        self.flow.prev_points = None

    def set_manual_speed(self, vx, vy, omega):
        """Injecte une vitesse manuelle (cm/s, cm/s, rad/s) dans le frame robot."""
        self.manual_vx = vx
        self.manual_vy = vy
        self.manual_omega = omega

    def _pose_from_tag(self, det, tid):
        """Localisation absolue : T_world_robot = T_world_tag @ T_tag_cam @ T_cam_robot"""
        T_world_tag = self.map.get_pose_matrix(tid)
        if T_world_tag is None:
            return None

        T_cam_tag = rodrigues_to_matrix(det["rvec"], det["tvec"])
        T_tag_cam = np.linalg.inv(T_cam_tag)
        T_world_robot = T_world_tag @ T_tag_cam @ self.T_cam_robot

        R = T_world_robot[:3, :3]
        t = T_world_robot[:3, 3]
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
        return float(t[0]), float(t[1]), float(t[2]), yaw

    def _tag_from_pose(self, det):
        """Cartographie : T_world_tag = T_world_robot @ T_robot_cam @ T_cam_tag"""
        T_world_robot = np.eye(4, dtype=np.float64)
        T_world_robot[:3, :3] = yaw_to_matrix(self.yaw)
        T_world_robot[:3, 3]  = [self.x, self.y, self.z]

        T_cam_tag = rodrigues_to_matrix(det["rvec"], det["tvec"])
        T_world_tag = T_world_robot @ self.T_robot_cam @ T_cam_tag

        R = T_world_tag[:3, :3]
        t = T_world_tag[:3, 3]
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
        return float(t[0]), float(t[1]), 0.0, yaw  # Z forcé au sol

    def update(self, detections, gray, t_now, scan_mode=False):
        """
        scan_mode=True : robot au centre (0,0), tourne sur lui-même.
                         On cartographie les tags sans bouger le robot.
        scan_mode=False : mode navigation, optical flow + correction tags.
        """
        dt = t_now - self.last_t
        self.last_t = t_now

        # ─── 1. Lire IMU yaw ─────────────────────────────────────────────
        yaw_deg, _, _, _, _, _ = self.arduino.get()
        self.yaw = np.radians(yaw_deg)

        # ─── 2. SCAN MODE ────────────────────────────────────────────────
        if scan_mode:
            self.x, self.y = 0.0, 0.0
            for det in detections:
                tid = det["tag_id"]
                # Filtre hard : ignore les IDs absents de la prior
                if tid not in self.map.prior:
                    continue
                tx, ty, tz, tyaw = self._tag_from_pose(det)
                self.map.add_or_update(tid, tx, ty, tz, tyaw, is_confirmed_view=True)
            return True

        # ─── 3. MODE MANUEL (test clavier) ──────────────────────────────
        # Déplacement direct, pas d'intégration d'accéléromètre
        if abs(self.manual_vx) > 0.1 or abs(self.manual_vy) > 0.1 or abs(self.manual_omega) > 0.01:
            c, s = np.cos(self.yaw), np.sin(self.yaw)
            # vx,vy sont dans le frame robot, on les projette dans le monde
            dx = (self.manual_vx * c - self.manual_vy * s) * dt
            dy = (self.manual_vx * s + self.manual_vy * c) * dt
            self.x += dx
            self.y += dy
            self.yaw += self.manual_omega * dt
            self.traj.append((self.x, self.y))
            # Reset optical flow car on a bougé "manuellement"
            self.flow.prev_gray = None
            self.flow.prev_points = None
            return False

        # ─── 4. LOCALISATION PAR TAGS ──────────────────────────────────
        known   = [d for d in detections if self.map.is_known(d["tag_id"])]
        unknown = [d for d in detections if not self.map.is_known(d["tag_id"])]

        localized = False
        if len(known) > 0:
            poses = []
            for det in known:
                p = self._pose_from_tag(det, det["tag_id"])
                if p is not None:
                    poses.append(p)

            if len(poses) > 0:
                xs = [p[0] for p in poses]
                ys = [p[1] for p in poses]
                yaws = [p[3] for p in poses]

                self.x   = float(np.median(xs))
                self.y   = float(np.median(ys))
                self.yaw = mean_angle(yaws)
                self.initialized = True
                self.traj.append((self.x, self.y))
                localized = True
                # Reset optical flow quand on a une vérité absolue
                self.flow.prev_gray = None
                self.flow.prev_points = None

        # ─── 5. OPTICAL FLOW (entre les tags) ───────────────────────────
        elif self.initialized and gray is not None:
            dx, dy = self.flow.update(gray, self.yaw)
            # Seulement si déplacement significatif (évite le bruit statique)
            if abs(dx) > 0.05 or abs(dy) > 0.05:
                self.x += dx
                self.y += dy
                self.traj.append((self.x, self.y))

        # ─── 6. CARTOGRAPHIE (uniquement si localisé) ────────────────
        # Si pas localisé, la position est fausse → ne pas polluer la carte
        if localized:
            for det in unknown:
                tid = det["tag_id"]
                tx, ty, tz, tyaw = self._tag_from_pose(det)
                self.map.add_or_update(tid, tx, ty, tz, tyaw,
                                       is_confirmed_view=True)

        return localized


# ═══════════════════════════════════════════════════════════════════════════
# AFFICHAGE CARTE 2D
# ═══════════════════════════════════════════════════════════════════════════

def draw_map(tracker, expected_tags=0, scan_mode=False,
             w=800, h=600, scale=2.0):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    ox, oy = w // 2, h // 2  # origine monde au centre de l'image

    # Grille (carrés de 50 cm)
    for i in range(-1000, 1001, 50):
        x = ox + int(i * scale)
        cv2.line(img, (x, 0), (x, h), (30, 30, 30), 1)
        y = oy - int(i * scale)
        cv2.line(img, (0, y), (w, y), (30, 30, 30), 1)

    # Origine
    cv2.circle(img, (ox, oy), 4, (255, 255, 255), -1)
    cv2.putText(img, "ORIGINE", (ox + 6, oy - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

    # Tags
    found_confirmed = 0
    for tid, t in tracker.map.tags.items():
        px = ox + int(t["x"] * scale)
        py = oy - int(t["y"] * scale)
        conf = t["conf"]

        if conf >= 1.0:
            color = (0, 255, 0)      # vert = confirmé
            found_confirmed += 1
        elif conf >= 0.5:
            color = (0, 255, 255)    # cyan = en cours
        else:
            color = (0, 100, 255)    # orange = nouveau

        sz = max(6, int(APRILTAG_SIZE_CM * scale * 0.6))
        cv2.rectangle(img, (px - sz, py - sz), (px + sz, py + sz), color, 2)

        # Flèche orientation
        fx = int(px + sz * 2.5 * np.cos(t["yaw"]))
        fy = int(py - sz * 2.5 * np.sin(t["yaw"]))
        cv2.arrowedLine(img, (px, py), (fx, fy), color, 2, tipLength=0.3)

        label = f"ID{tid}" if t["views"] >= TAG_CONFIRM_VIEWS else f"ID{tid}?{t['views']}"
        cv2.putText(img, label, (px + 8, py - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    # Trajectoire
    pts = [(ox + int(x * scale), oy - int(y * scale)) for x, y in tracker.traj]
    for i in range(1, len(pts)):
        cv2.line(img, pts[i - 1], pts[i], (180, 180, 180), 1)

    # Robot
    if tracker.initialized or scan_mode:
        rx = ox + int(tracker.x * scale)
        ry = oy - int(tracker.y * scale)
        yaw = tracker.yaw
        L = 16
        tip  = (int(rx + L * 2.5 * np.cos(yaw)), int(ry - L * 2.5 * np.sin(yaw)))
        left = (int(rx + L * np.cos(yaw + 2.3)), int(ry - L * np.sin(yaw + 2.3)))
        rght = (int(rx + L * np.cos(yaw - 2.3)), int(ry - L * np.sin(yaw - 2.3)))
        cv2.fillConvexPoly(img, np.array([tip, left, rght]), (255, 100, 0))

        # Cône de vision
        fov = np.radians(55)
        for a in (-fov, fov):
            fx = int(rx + 90 * np.cos(yaw + a))
            fy = int(ry - 90 * np.sin(yaw + a))
            cv2.line(img, (rx, ry), (fx, fy), (80, 80, 80), 1)

    # HUD
    status = "SCAN..." if scan_mode else ("LOCALISÉ" if tracker.initialized else "FLOW/IMU")
    color = (0, 255, 255) if scan_mode else ((0, 255, 0) if tracker.initialized else (0, 165, 255))

    lines = [
        f"MODE : {status}",
        f"Robot  X={tracker.x:.1f}  Y={tracker.y:.1f}  Yaw={np.degrees(tracker.yaw):.1f}°",
        f"Tags : {found_confirmed}/{expected_tags} confirmés  ({len(tracker.map.tags)} total)",
        "",
        "[C] Scan    [E] Fin scan    [W/A/S/D] Manuel    [X] Stop    [R] Reset    [Q] Quitter",
    ]
    for i, line in enumerate(lines):
        cv2.putText(img, line, (10, 25 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    return img


# ═══════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════

def main():
    print("\n" + "=" * 60)
    print("  APRILTAG + BMI160 SLAM — Optical Flow (sans encodeurs)")
    print("=" * 60)

    # 1. Connexion Arduino
    try:
        arduino = ArduinoReader()
    except Exception as e:
        print(f"[ERREUR] {e}")
        sys.exit(1)

    # 2. Caméra (auto-détection)
    cam_idx = find_camera_index(CAMERA_INDEX)
    cap = VideoCaptureThread(cam_idx, FRAME_WIDTH, FRAME_HEIGHT)
    
    # Vérification - le thread a besoin de temps pour démarrer
    print("[CAMERA] Démarrage thread...")
    time.sleep(0.8)  # Laisser le thread capturer des frames
    
    # Essayer plusieurs lectures
    test_frame = None
    for i in range(5):
        ret, frame = cap.read()
        if ret and frame is not None:
            test_frame = frame
            break
        time.sleep(0.2)
    
    if test_frame is None:
        print("[ERREUR] Caméra ne répond pas !")
        print(f"  → Essaye CAMERA_INDEX=0 ou 1")
        cap.stop()
        sys.exit(1)
    
    print(f"[CAMERA] OK — {test_frame.shape[1]}x{test_frame.shape[0]}")
    
    detector = AprilTagDetector()
    tag_map = TagMapSLAM()
    tag_map.load_prior()   # charge les positions théoriques avant le scan
    tracker = RobotTracker(tag_map, arduino)

    time.sleep(0.5)

    # 3. Nombre de tags (avec timeout 5s, défaut = 4)
    print("\nPlace le robot au CENTRE de la salle.")
    print("Combien de tags as-tu placés ? (défaut: 4 dans 5s)")
    
    input_result = [None]
    def read_input():
        try:
            input_result[0] = input()
        except:
            pass
    
    input_thread = threading.Thread(target=read_input)
    input_thread.daemon = True
    input_thread.start()
    input_thread.join(timeout=5.0)  # Attendre max 5 secondes
    
    n_tags = input_result[0]
    if n_tags is None or n_tags.strip() == "":
        print("[INFO] Utilisation valeur par défaut: 4 tags")
        expected_tags = 4
    else:
        try:
            expected_tags = int(n_tags.strip())
            print(f"[INFO] {expected_tags} tags attendus")
        except:
            expected_tags = 4
            print("[INFO] Valeur invalide, utilisation défaut: 4 tags")

    print(f"\nAppuie sur [S] dans la fenêtre Camera pour démarrer le SCAN.")
    print("Tourne lentement le robot sur 360° (à la main).")
    print(f"Appuie sur [E] quand tu as fini (ou auto-fin à {expected_tags} tags).")
    print("Ensuite, déplace le robot : la carte se met à jour en temps réel.")
    print("Tu peux aussi utiliser [W/A/S/D] pour tester le déplacement manuel.\n")

    fps_q = deque(maxlen=30)
    t_prev = time.perf_counter()
    scan_mode = False

    # Vitesses manuelles
    man_vx = 0.0
    man_vy = 0.0
    man_omega = 0.0

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.005)
                continue

            t_now = time.perf_counter()
            dt = t_now - t_prev
            t_prev = t_now
            fps_q.append(t_now)
            fps = len(fps_q) / (fps_q[-1] - fps_q[0]) if len(fps_q) > 1 else 0.0

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray)

            # ─── Mise à jour tracker ───────────────────────────────────────
            tracker.set_manual_speed(man_vx, man_vy, man_omega)
            tracker.update(detections, gray, t_now, scan_mode=scan_mode)

            # ─── Overlay caméra ────────────────────────────────────────────
            for d in detections:
                c = d["corners"].astype(np.int32)
                tid = d["tag_id"]
                is_k = tag_map.is_known(tid)
                color = (0, 255, 0) if is_k else (0, 255, 255)
                cv2.polylines(frame, [c.reshape(-1, 1, 2)], True, color, 1)
                cx, cy = int(np.mean(c[:, 0])), int(np.mean(c[:, 1]))
                label = f"ID{tid}" if is_k else f"ID{tid} NEW"
                cv2.putText(frame, label, (cx + 5, cy - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

            # Barre info caméra
            cv2.rectangle(frame, (0, 0), (520, 30), (0, 0, 0), -1)
            mode_txt = "SCAN" if scan_mode else "NAV"
            cv2.putText(frame,
                        f"{mode_txt} | FPS:{fps:.1f} | Tags:{len(detections)} | "
                        f"X:{tracker.x:.0f} Y:{tracker.y:.0f} Yaw:{np.degrees(tracker.yaw):.0f}°",
                        (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # ─── Carte 2D ──────────────────────────────────────────────────
            map_img = draw_map(tracker, expected_tags=expected_tags,
                               scan_mode=scan_mode, w=800, h=600, scale=2.0)

            cv2.imshow("Camera", frame)
            cv2.imshow("SLAM MAP", map_img)

            # ─── Touches ───────────────────────────────────────────────────
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

            elif key == ord('c') and not scan_mode:
                # Démarrage scan [C] = Cartographier
                scan_mode = True
                tag_map.tags.clear()
                tracker.traj.clear()
                arduino.reset_yaw()
                time.sleep(0.1)
                tracker.reset_to(0.0, 0.0, 0.0)
                print("[SCAN] Démarré ! Tourne lentement sur toi-même...")

            elif key == ord('e') and scan_mode:
                # Fin scan manuelle
                scan_mode = False
                for t in tag_map.tags.values():
                    t["views"] = max(t["views"], TAG_CONFIRM_VIEWS)
                    t["conf"] = 1.0
                tracker.initialized = True
                tracker.reset_to(0.0, 0.0, tracker.yaw)
                print(f"[SCAN] Fin manuelle. {len(tag_map.tags)} tags cartographiés.")
                print("Tu peux maintenant te déplacer...")

            elif key == ord('r') and not scan_mode:
                # Reset yaw IMU
                arduino.reset_yaw()
                tracker.reset_to(0.0, 0.0, 0.0)

            elif not scan_mode:
                # Mouvement manuel UNIQUEMENT en mode NAV
                if key == ord('w'):
                    man_vx, man_vy, man_omega = MANUAL_SPEED_CM_S, 0.0, 0.0
                elif key == ord('s'):
                    man_vx, man_vy, man_omega = -MANUAL_SPEED_CM_S, 0.0, 0.0
                elif key == ord('a'):
                    man_vx, man_vy, man_omega = 0.0, 0.0, MANUAL_TURN_RAD_S
                elif key == ord('d'):
                    man_vx, man_vy, man_omega = 0.0, 0.0, -MANUAL_TURN_RAD_S
                elif key == ord('x') or key == ord(' '):
                    man_vx = man_vy = man_omega = 0.0

            # Auto-fin scan si tous les tags trouvés
            if scan_mode and len(tag_map.tags) >= expected_tags:
                scan_mode = False
                for t in tag_map.tags.values():
                    t["views"] = max(t["views"], TAG_CONFIRM_VIEWS)
                    t["conf"] = 1.0
                tracker.initialized = True
                tracker.reset_to(0.0, 0.0, tracker.yaw)
                print(f"[SCAN] Auto-fin ! {len(tag_map.tags)} tags trouvés.")
                print("Tu peux maintenant te déplacer...")

    finally:
        cap.stop()
        arduino.stop()
        cv2.destroyAllWindows()
        tag_map.save()
        print("\n[INFO] Terminé. Carte sauvegardée.")


if __name__ == "__main__":
    main()