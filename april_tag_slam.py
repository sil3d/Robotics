"""
============================================================================
  APRILTAG SLAM — RASPBERRY PI (VERSION CORRIGÉE)
  
  Corrections :
  • Repère Caméra→Robot correct (Z_caméra = X_robot)
  • Pitch caméra modélisé (CAM_PITCH_DEG, ex: -45°)
  • Tags projetés sur le sol (Z=0) après calcul
  • Origine exacte : le premier tag est forcé à (0,0,0)
  • Yaw robot basé sur l'AVANT du robot (axe X_robot)
============================================================================
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import time
import threading
import json
import os
import serial
import serial.tools.list_ports
from collections import deque

# ─────────────────────────────────────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────────────────────────────────────
CAMERA_INDEX       = 0
FRAME_WIDTH        = 640
FRAME_HEIGHT       = 480

APRILTAG_DICT      = aruco.DICT_APRILTAG_36H11
APRILTAG_SIZE_CM   = 10.0
MAX_DISTANCE_M     = 5.0

# ─── INTRINSÈQUES — chargées depuis data/camera_calibration/camera_calibration.json ──────────────
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

# ─── OFFSET CAMÉRA → ROBOT ────────────────────────────────────────────────────
# Frame robot : X=avant, Y=gauche, Z=haut
CAM_FORWARD_CM     = 15.0   # caméra devant le centre robot
CAM_LEFT_CM        = 0.0    # décalage latéral
CAM_HEIGHT_CM      = 25.0   # hauteur au-dessus du sol
CAM_YAW_OFFSET_DEG = 0.0    # rotation horizontale caméra (rarement utile)
CAM_PITCH_DEG      = -45.0  # ↓ NÉGATIF = caméra penchée VERS LE BAS (standard robot)

SERIAL_BAUD = 115200

# ─── SLAM ───────────────────────────────────────────────────────────────────
MIN_TAG_PX_DIAG    = 8
TAG_CONFIRM_VIEWS  = 5
MAP_FILE           = os.path.join(os.path.dirname(__file__), "data", "tags", "tags.json")
FORCE_Z_ZERO       = True   # projette les tags sur le sol (z=0)


# ─────────────────────────────────────────────────────────────────────────────
# LECTURE IMU ARDUINO (format: IMU,yaw,omega_z,ax,ay,az)
# ─────────────────────────────────────────────────────────────────────────────

class ArduinoReader:
    def __init__(self, port=None, baud=SERIAL_BAUD):
        self.yaw_deg = 0.0
        self.omega_z = 0.0
        self.stopped = False
        self.lock    = threading.Lock()
        if port is None:
            ports = list(serial.tools.list_ports.comports())
            cands = [p.device for p in ports
                     if any(x in p.description for x in ['Arduino','USB','Serial'])
                     or 'ACM' in p.device or 'ttyUSB' in p.device]
            if not cands:
                print("[ARDUINO] Pas de port — dead reckoning désactivé")
                self._dummy = True
                return
            port = cands[0]
            print(f"[ARDUINO] Port auto: {port}")
        self._dummy = False
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2.0)
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not self.stopped:
            try:
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
            except Exception:
                pass
            time.sleep(0.001)

    def get(self):
        with self.lock:
            return self.yaw_deg, self.omega_z

    def reset_yaw(self):
        if not self._dummy:
            try:
                self.ser.write(b'R\n')
            except Exception:
                pass

    def stop(self):
        self.stopped = True
        if not self._dummy:
            self.ser.close()


# ─────────────────────────────────────────────────────────────────────────────
# MATHS UTILITAIRES
# ─────────────────────────────────────────────────────────────────────────────

def rodrigues_to_matrix(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3]  = tvec.flatten()
    return T

def yaw_to_matrix(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)

def mean_angle(angles_rad):
    if len(angles_rad) == 0:
        return 0.0
    return np.arctan2(np.mean(np.sin(angles_rad)), np.mean(np.cos(angles_rad)))


# ─────────────────────────────────────────────────────────────────────────────
# CONSTRUCTION DE T_ROBOT_CAM (corrigée)
# ─────────────────────────────────────────────────────────────────────────────

def build_T_robot_cam():
    """
    Repère caméra OpenCV : X=droite, Y=bas, Z=avant (vers la scène)
    Repère robot         : X=avant, Y=gauche, Z=haut
    
    Alignement de base :
        Z_cam (avant)  →  X_robot (avant)
        X_cam (droite)  → -Y_robot (droite)
        Y_cam (bas)     → -Z_robot (bas)
    """
    R_base = np.array([[0,  0, 1],
                       [-1, 0, 0],
                       [0, -1, 0]], dtype=np.float64)

    # Pitch : rotation autour de Y_robot (négatif = vers le bas)
    p = np.radians(CAM_PITCH_DEG)
    R_pitch = np.array([[np.cos(p), 0, np.sin(p)],
                        [0, 1, 0],
                        [-np.sin(p), 0, np.cos(p)]], dtype=np.float64)

    # Yaw : rotation autour de Z_robot
    y = np.radians(CAM_YAW_OFFSET_DEG)
    R_yaw = np.array([[np.cos(y), -np.sin(y), 0],
                      [np.sin(y),  np.cos(y), 0],
                      [0, 0, 1]], dtype=np.float64)

    R = R_yaw @ R_pitch @ R_base

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3]  = [CAM_FORWARD_CM, CAM_LEFT_CM, CAM_HEIGHT_CM]
    return T


# ─────────────────────────────────────────────────────────────────────────────
# THREAD CAMÉRA
# ─────────────────────────────────────────────────────────────────────────────

class VideoCaptureThread:
    def __init__(self, src=0, width=640, height=480):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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


# ─────────────────────────────────────────────────────────────────────────────
# DÉTECTEUR ARUCO
# ─────────────────────────────────────────────────────────────────────────────

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


# ─────────────────────────────────────────────────────────────────────────────
# CARTE SLAM
# ─────────────────────────────────────────────────────────────────────────────

class TagMapSLAM:
    def __init__(self):
        self.tags = {}

    def load(self, path=MAP_FILE):
        if not os.path.exists(path):
            return False
        with open(path, "r") as f:
            data = json.load(f)
        for tid, vals in data.items():
            self.tags[int(tid)] = {
                "x": vals["x"], "y": vals["y"], "z": vals["z"],
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
                "x": float(t["x"]), "y": float(t["y"]), "z": float(t["z"]),
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
        if tid not in self.tags:
            self.tags[tid] = {
                "x": x, "y": y, "z": z, "yaw": yaw,
                "views": 1, "conf": 0.2
            }
            print(f"[SLAM] Nouveau tag ID{tid} @ ({x:.1f}, {y:.1f}, {z:.1f})")
            return

        t = self.tags[tid]
        alpha = 1.0 / (t["views"] + 1.0)
        t["x"] = (1-alpha) * t["x"] + alpha * x
        t["y"] = (1-alpha) * t["y"] + alpha * y
        t["z"] = (1-alpha) * t["z"] + alpha * z
        t["yaw"] = np.arctan2(
            (1-alpha)*np.sin(t["yaw"]) + alpha*np.sin(yaw),
            (1-alpha)*np.cos(t["yaw"]) + alpha*np.cos(yaw)
        )
        if is_confirmed_view:
            t["views"] += 1
        t["conf"] = min(1.0, t["views"] / TAG_CONFIRM_VIEWS)
        if t["views"] == TAG_CONFIRM_VIEWS:
            print(f"[SLAM] Tag ID{tid} confirmé !")

    def is_known(self, tid):
        return tid in self.tags and self.tags[tid]["conf"] >= 0.5


# ─────────────────────────────────────────────────────────────────────────────
# ROBOT TRACKER SLAM (corrigé)
# ─────────────────────────────────────────────────────────────────────────────

class RobotTrackerSLAM:
    def __init__(self, tag_map):
        self.map = tag_map
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.initialized = False
        self.traj = deque(maxlen=300)

        self.T_robot_cam = build_T_robot_cam()
        self.T_cam_robot = np.linalg.inv(self.T_robot_cam)

    def _compute_robot_pose_from_tag(self, det, tid):
        """Localisation absolue : T_world_robot = T_world_tag @ T_tag_cam @ T_cam_robot"""
        T_world_tag = self.map.get_pose_matrix(tid)
        if T_world_tag is None:
            return None

        T_cam_tag = rodrigues_to_matrix(det["rvec"], det["tvec"])
        T_tag_cam = np.linalg.inv(T_cam_tag)

        T_world_robot = T_world_tag @ T_tag_cam @ self.T_cam_robot

        R = T_world_robot[:3, :3]
        t = T_world_robot[:3, 3]

        # Yaw du robot = direction de l'axe X du robot dans le monde
        # (car X_robot = avant, grâce à R_base dans build_T_robot_cam)
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
        return float(t[0]), float(t[1]), float(t[2]), yaw

    def _compute_tag_pose_from_robot(self, det):
        """Cartographie : T_world_tag = T_world_robot @ T_robot_cam @ T_cam_tag"""
        T_world_robot = np.eye(4, dtype=np.float64)
        T_world_robot[:3, :3] = yaw_to_matrix(self.yaw)
        T_world_robot[:3, 3]  = [self.x, self.y, self.z]

        T_cam_tag = rodrigues_to_matrix(det["rvec"], det["tvec"])

        T_world_tag = T_world_robot @ self.T_robot_cam @ T_cam_tag

        R = T_world_tag[:3, :3]
        t = T_world_tag[:3, 3]
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))

        x, y, z = float(t[0]), float(t[1]), float(t[2])

        # Projection au sol : les tags sont sur le plancher (z=0)
        if FORCE_Z_ZERO:
            z = 0.0
            # Optionnel : on peut aussi recaler x,y en traçant un rayon depuis la caméra
            # vers le sol, mais pour l'instant on force juste z=0

        return x, y, z, yaw

    def update(self, detections, t_now):
        known_dets   = [d for d in detections if self.map.is_known(d["tag_id"])]
        unknown_dets = [d for d in detections if not self.map.is_known(d["tag_id"])]

        # ─── Aucun tag ─────────────────────────────────────────────────────
        if len(detections) == 0:
            return False

        # ─── Premier démarrage : premier tag = ORIGINE exacte (0,0,0) ────
        if not self.initialized and len(known_dets) == 0 and len(unknown_dets) > 0:
            det = unknown_dets[0]
            tid = det["tag_id"]

            # On calcule où le robot est par rapport au tag
            T_cam_tag = rodrigues_to_matrix(det["rvec"], det["tvec"])
            T_tag_cam = np.linalg.inv(T_cam_tag)
            T_cam_robot = self.T_cam_robot
            T_tag_robot = T_tag_cam @ T_cam_robot

            # Position du robot dans le repère du tag
            t_robot_in_tag = T_tag_robot[:3, 3]
            # On place le tag à (0,0,0) et le robot à sa position relative
            self.x = float(-t_robot_in_tag[0])
            self.y = float(-t_robot_in_tag[1])
            self.z = 0.0
            self.yaw = 0.0  # on suppose que le robot regarde vers le tag au démarrage
            self.initialized = True

            # Le tag est exactement à l'origine du monde
            self.map.add_or_update(tid, 0.0, 0.0, 0.0, 0.0, is_confirmed_view=True)
            self.traj.append((self.x, self.y))

            print(f"[SLAM] ORIGINE : tag ID{tid} fixé à (0,0,0)")
            print(f"[SLAM] Robot démarré @ ({self.x:.1f}, {self.y:.1f}, 0.0)")
            return True

        # ─── Localisation par tags connus ──────────────────────────────────
        if len(known_dets) > 0:
            poses = []
            for det in known_dets:
                p = self._compute_robot_pose_from_tag(det, det["tag_id"])
                if p is not None:
                    poses.append(p)

            if len(poses) == 0:
                return False

            xs = [p[0] for p in poses]
            ys = [p[1] for p in poses]
            zs = [p[2] for p in poses]
            yaws = [p[3] for p in poses]

            self.x   = float(np.median(xs))
            self.y   = float(np.median(ys))
            self.z   = float(np.median(zs))
            self.yaw = mean_angle(yaws)
            self.initialized = True
            self.traj.append((self.x, self.y))

            # Cartographie des inconnus
            for det in unknown_dets:
                tid = det["tag_id"]
                tx, ty, tz, tyaw = self._compute_tag_pose_from_robot(det)
                self.map.add_or_update(tid, tx, ty, tz, tyaw)

            return True

        # ─── Tags inconnus seuls, robot initialisé → cartographie faible ───
        if self.initialized and len(unknown_dets) > 0:
            for det in unknown_dets:
                tid = det["tag_id"]
                tx, ty, tz, tyaw = self._compute_tag_pose_from_robot(det)
                self.map.add_or_update(tid, tx, ty, tz, tyaw, is_confirmed_view=False)
            return False

        return False

    def dead_reckoning(self, vx, vy, omega, dt):
        if dt <= 0:
            return
        self.yaw += omega * dt
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.traj.append((self.x, self.y))


# ─────────────────────────────────────────────────────────────────────────────
# AFFICHAGE CARTE 2D
# ─────────────────────────────────────────────────────────────────────────────

def draw_map(tracker, w=640, h=480, scale=2.0):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    ox, oy = w // 2, h // 2

    # Grille
    for i in range(-500, 501, 50):
        x = ox + int(i * scale)
        cv2.line(img, (x, 0), (x, h), (25, 25, 25), 1)
        y = oy - int(i * scale)
        cv2.line(img, (0, y), (w, y), (25, 25, 25), 1)

    # Origine
    cv2.circle(img, (ox, oy), 3, (255, 255, 255), -1)
    cv2.putText(img, "ORIGINE", (ox+5, oy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200,200,200), 1)

    # Tags
    for tid, t in tracker.map.tags.items():
        px = ox + int(t["x"] * scale)
        py = oy - int(t["y"] * scale)
        conf = t["conf"]

        if conf >= 1.0:
            color = (0, 255, 0)
        elif conf >= 0.5:
            color = (0, 165, 255)
        else:
            color = (0, 0, 255)

        sz = max(4, int(APRILTAG_SIZE_CM * scale * 0.5))
        cv2.rectangle(img, (px-sz, py-sz), (px+sz, py+sz), color, 1)

        fx = int(px + sz*2 * np.cos(t["yaw"]))
        fy = int(py - sz*2 * np.sin(t["yaw"]))
        cv2.arrowedLine(img, (px, py), (fx, fy), color, 2, tipLength=0.3)

        label = f"ID{tid}" if t["views"] >= TAG_CONFIRM_VIEWS else f"ID{tid}?{t['views']}"
        cv2.putText(img, label, (px+6, py-6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

    # Trajectoire
    pts = [(ox + int(x*scale), oy - int(y*scale)) for x, y in tracker.traj]
    for i in range(1, len(pts)):
        cv2.line(img, pts[i-1], pts[i], (200, 200, 200), 1)

    # Robot
    if tracker.initialized:
        rx = ox + int(tracker.x * scale)
        ry = oy - int(tracker.y * scale)
        yaw = tracker.yaw
        L = 14
        tip  = (int(rx + L*2.2*np.cos(yaw)), int(ry - L*2.2*np.sin(yaw)))
        left = (int(rx + L*np.cos(yaw+2.4)), int(ry - L*np.sin(yaw+2.4)))
        rght = (int(rx + L*np.cos(yaw-2.4)), int(ry - L*np.sin(yaw-2.4)))
        cv2.fillConvexPoly(img, np.array([tip, left, rght]), (255, 120, 0))

        fov = np.radians(50)
        for a in (-fov, fov):
            fx = int(rx + 70*np.cos(yaw+a))
            fy = int(ry - 70*np.sin(yaw+a))
            cv2.line(img, (rx, ry), (fx, fy), (60, 60, 60), 1)

    # Infos
    status = "LOCALISÉ" if tracker.initialized else "INIT..."
    color = (0, 255, 0) if tracker.initialized else (0, 0, 255)
    lines = [
        f"ROBOT {status}",
        f"X={tracker.x:.1f} Y={tracker.y:.1f} Z={tracker.z:.1f} cm",
        f"Yaw={np.degrees(tracker.yaw):.1f}°",
        f"Tags : {len(tracker.map.tags)}",
    ]
    for i, line in enumerate(lines):
        cv2.putText(img, line, (10, 20 + i*18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    return img


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print("\n[INFO] AprilTag SLAM — VERSION CORRIGÉE")
    print("  Repère caméra→robot corrigé (Z_cam = X_robot)")
    print(f"  Pitch caméra : {CAM_PITCH_DEG}°")
    print("  Tags projetés sur le sol (Z=0)")
    print("  Q = quit | R = reset yaw\n")

    arduino = ArduinoReader()
    tag_map = TagMapSLAM()
    had_map = tag_map.load()

    cap_thread = VideoCaptureThread(CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT)
    detector = AprilTagDetector()
    tracker = RobotTrackerSLAM(tag_map)

    time.sleep(0.3)
    fps_q = deque(maxlen=30)
    t_prev = time.perf_counter()

    try:
        while True:
            ret, frame = cap_thread.read()
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

            localized = tracker.update(detections, t_now)

            if not localized:
                # Dead reckoning depuis IMU Arduino (format IMU,yaw,omega_z,...)
                _, omega_z = arduino.get()
                tracker.dead_reckoning(0.0, 0.0, float(np.radians(omega_z)), dt)

            # Overlay caméra
            for d in detections:
                c = d["corners"].astype(np.int32)
                tid = d["tag_id"]
                is_known = tag_map.is_known(tid)
                color = (0, 255, 0) if is_known else (0, 165, 255)
                cv2.polylines(frame, [c.reshape(-1, 1, 2)], True, color, 1)
                cx, cy = int(np.mean(c[:,0])), int(np.mean(c[:,1]))
                label = f"ID{tid}" if is_known else f"ID{tid} NEW"
                cv2.putText(frame, label, (cx+5, cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

            cv2.rectangle(frame, (0, 0), (420, 28), (0, 0, 0), -1)
            info = f"FPS:{fps:.1f} | Tags:{len(detections)} | X:{tracker.x:.0f} Y:{tracker.y:.0f} Yaw:{np.degrees(tracker.yaw):.0f}°"
            cv2.putText(frame, info, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

            map_img = draw_map(tracker, w=640, h=480, scale=2.0)

            cv2.imshow("Camera", frame)
            cv2.imshow("SLAM MAP", map_img)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('r'):
                arduino.reset_yaw()
                print("[YAW] Reset")

    finally:
        arduino.stop()
        cap_thread.stop()
        cv2.destroyAllWindows()
        tag_map.save()
        print("\n[INFO] Terminé. Carte sauvegardée.")


if __name__ == "__main__":
    main()