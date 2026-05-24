"""
============================================================================
  APRILTAG ROBOT LOCALIZATION — RASPBERRY PI
  Localisation absolue par balises fixes + odométrie entre balises.

  PRINCIPE :
  • Tu crées une carte (TAG_MAP) avec la position exacte de chaque tag.
  • Le robot se localise instantanément dès qu'il voit un tag.
  • Quand aucun tag n'est visible, l'odométrie prend le relais.
  • Affichage d'une carte 2D temps réel.

  CONFIG :
  1. Remplir TAG_MAP avec les coordonnées de TES tags (x,y,z,yaw_deg).
  2. Ajuster CAM_FORWARD_CM / CAM_HEIGHT_CM selon ton montage physique.
  3. Brancher tes encodeurs de roues dans update_dead_reckoning() (optionnel).
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
# CONFIG CARTE & ROBOT
# ─────────────────────────────────────────────────────────────────────────────

# === CARTE DES TAGS ===
# Format : {id: (x_cm, y_cm, z_cm, yaw_deg)}
# yaw_deg = orientation du tag dans le monde (0° = tag regarde vers +X)
# Exemple : couloir de 3 mètres avec un tag tous les 150 cm
TAG_MAP = {
    0: (0.0,   0.0,  0.0,   0.0),   # Origine
    1: (150.0, 0.0,  0.0,   0.0),   # 1.5m devant
    2: (300.0, 0.0,  0.0,   0.0),   # 3m devant
    3: (150.0, 100.0, 0.0,  90.0),  # à droite du couloir, tourné
}

# === OFFSET CAMÉRA → ROBOT ===
# La caméra est rarement au centre du robot.
# Ces valeurs décalent la pose caméra pour obtenir la pose robot.
CAM_FORWARD_CM     = 15.0   # caméra est X cm devant le centre robot
CAM_HEIGHT_CM      = 25.0   # caméra est Z cm au-dessus du sol
CAM_YAW_OFFSET_DEG = 0.0    # si caméra de travers par rapport au robot

# === CAMÉRA ===
CAMERA_INDEX = 0
FRAME_WIDTH  = 640
FRAME_HEIGHT = 480

# === INTRINSÈQUES — chargées depuis data/camera_calibration/camera_calibration.json ===
_CAL_FILE = os.path.join(os.path.dirname(__file__), "data", "camera_calibration", "camera_calibration.json")
if os.path.exists(_CAL_FILE):
    with open(_CAL_FILE) as _f:
        _cal = json.load(_f)
    CAM_MATRIX  = np.array(_cal["camera_matrix"],  dtype=np.float32)
    DIST_COEFFS = np.array(_cal["dist_coeffs"],     dtype=np.float32)
    APRILTAG_SIZE_CM = float(_cal.get("apriltag_size_cm", APRILTAG_SIZE_CM))
else:
    CAM_MATRIX = np.array([
        [828.395, 0.0,     337.460],
        [0.0,     812.656, 213.622],
        [0.0,     0.0,     1.0]
    ], dtype=np.float32)
    DIST_COEFFS = np.array([[-1.436, 14.760, -0.00570, 0.0543, -37.111]], dtype=np.float32)

APRILTAG_DICT    = cv2.aruco.DICT_4X4_250
APRILTAG_SIZE_CM = 10.0  # peut être écrasé par le JSON
MAX_DISTANCE_M   = 5.0
SERIAL_BAUD      = 115200


# ─────────────────────────────────────────────────────────────────────────────
# LECTURE IMU ARDUINO (format: IMU,yaw,omega_z,ax,ay,az)
# ─────────────────────────────────────────────────────────────────────────────

class ArduinoReader:
    def __init__(self, port=None, baud=SERIAL_BAUD):
        self.yaw_deg  = 0.0
        self.omega_z  = 0.0
        self.stopped  = False
        self.lock     = threading.Lock()
        if port is None:
            ports = list(serial.tools.list_ports.comports())
            cands = [p.device for p in ports
                     if any(x in p.description for x in ['Arduino','USB','Serial'])
                     or 'ACM' in p.device or 'ttyUSB' in p.device]
            if not cands:
                print("[ARDUINO] Pas de port détecté — dead reckoning désactivé")
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
# UTILITAIRES MATHÉMATIQUES
# ─────────────────────────────────────────────────────────────────────────────

def rodrigues_to_matrix(rvec, tvec):
    """Convertit rvec/tvec (OpenCV) en matrice 4x4 de transformation."""
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3]  = tvec.flatten()
    return T

def yaw_from_matrix(R):
    """Extrait le yaw (rad) d'une matrice 3x3, axe Z projeté sur le sol (X,Y)."""
    # Axe Z de la caméra (avant) projeté sur le plan monde XY
    dx = R[0, 2]
    dy = R[1, 2]
    return np.arctan2(dy, dx)

def mean_angle(angles_rad):
    """Moyenne robuste d'angles (évite le problème 359° vs 1°)."""
    if len(angles_rad) == 0:
        return 0.0
    s = np.mean(np.sin(angles_rad))
    c = np.mean(np.cos(angles_rad))
    return np.arctan2(s, c)


# ─────────────────────────────────────────────────────────────────────────────
# THREAD CAMÉRA
# ─────────────────────────────────────────────────────────────────────────────

class VideoCaptureThread:
    def __init__(self, src=0, width=640, height=480):
        import sys
        if sys.platform == 'win32':
            self.cap = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        else:
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
# DÉTECTEUR ARUCO (léger, même que précédemment)
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
            if diag < 8:
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
# CARTE & LOCALISATION
# ─────────────────────────────────────────────────────────────────────────────

class TagMap:
    """Stocke les balises fixes avec leur pose dans le monde."""
    def __init__(self, tag_dict):
        self.tags = {}  # {id: (x,y,z,yaw_deg)}
        for tid, data in tag_dict.items():
            x, y, z, yaw_deg = data
            yaw = np.radians(yaw_deg)
            R = np.array([
                [np.cos(yaw), -np.sin(yaw), 0],
                [np.sin(yaw),  np.cos(yaw), 0],
                [0, 0, 1]
            ], dtype=np.float64)
            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = R
            T[:3, 3]  = [x, y, z]
            self.tags[tid] = {"T_world_tag": T, "yaw": yaw, "pos": np.array([x, y, z])}


class RobotTracker:
    """
    Estime la pose (x,y,z,yaw) du robot dans le monde.
    - Quand un tag est vu : localisation absolue (reset).
    - Quand rien n'est vu : dead reckoning (odométrie).
    """
    def __init__(self, tag_map):
        self.tag_map = tag_map
        self.x = 0.0      # cm
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0    # rad
        self.confidence = 0.0   # 1.0 = tag vu, décroît sinon
        self.last_tag_time = 0.0
        self.history = deque(maxlen=5)

    def update_from_detections(self, detections, t_now):
        """Calcule la pose robot à partir des tags visibles."""
        poses = []

        for det in detections:
            tid = det["tag_id"]
            if tid not in self.tag_map.tags:
                continue  # Tag inconnu, on l'ignore (ou on peut l'ajouter en SLAM)

            # 1. Pose du tag dans la caméra (OpenCV)
            T_cam_tag = rodrigues_to_matrix(det["rvec"], det["tvec"])

            # 2. Pose de la caméra dans le tag
            T_tag_cam = np.linalg.inv(T_cam_tag)

            # 3. Pose du tag dans le monde (carte)
            T_world_tag = self.tag_map.tags[tid]["T_world_tag"]

            # 4. Pose de la caméra dans le monde
            T_world_cam = T_world_tag @ T_tag_cam

            R_wc = T_world_cam[:3, :3]
            t_wc = T_world_cam[:3, 3]

            # 5. Orientation de la caméra (yaw)
            yaw_cam = yaw_from_matrix(R_wc)

            # 6. Offset caméra → robot
            yaw_robot = yaw_cam + np.radians(CAM_YAW_OFFSET_DEG)
            cr = np.cos(yaw_robot)
            sr = np.sin(yaw_robot)

            x_robot = t_wc[0] - CAM_FORWARD_CM * cr
            y_robot = t_wc[1] - CAM_FORWARD_CM * sr
            z_robot = t_wc[2] - CAM_HEIGHT_CM

            poses.append((x_robot, y_robot, z_robot, yaw_robot))

        if len(poses) == 0:
            self.confidence *= 0.95  # décroît progressivement
            return False

        # Fusion multi-tag : médiane sur la position, moyenne circulaire sur yaw
        xs = np.array([p[0] for p in poses])
        ys = np.array([p[1] for p in poses])
        zs = np.array([p[2] for p in poses])
        yaws = np.array([p[3] for p in poses])

        self.x     = float(np.median(xs))
        self.y     = float(np.median(ys))
        self.z     = float(np.median(zs))
        self.yaw   = mean_angle(yaws)
        self.confidence = 1.0
        self.last_tag_time = t_now

        self.history.append((self.x, self.y, self.yaw))
        return True

    def update_dead_reckoning(self, vx, vy, omega, dt):
        """
        Met à jour la pose par odométrie quand aucun tag n'est visible.
        vx, vy : cm/s dans le frame robot
        omega  : rad/s (taux de rotation / yaw rate)
        """
        if dt <= 0:
            return
        self.yaw += omega * dt
        c = np.cos(self.yaw)
        s = np.sin(self.yaw)
        # Passage du frame robot au frame monde
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt


# ─────────────────────────────────────────────────────────────────────────────
# AFFICHAGE CARTE 2D (TOP-DOWN)
# ─────────────────────────────────────────────────────────────────────────────

def draw_map(tracker, tag_map, detections, w=640, h=480, scale=2.0):
    """
    Dessine une carte vue du dessus.
    scale = pixels par cm (2.0 = 1cm = 2px)
    """
    img = np.zeros((h, w, 3), dtype=np.uint8)
    ox, oy = w // 2, h // 2  # origine monde au centre de l'image

    # Grille
    for i in range(-500, 501, 50):
        x = ox + int(i * scale)
        cv2.line(img, (x, 0), (x, h), (30, 30, 30), 1)
        y = oy - int(i * scale)
        cv2.line(img, (0, y), (w, y), (30, 30, 30), 1)

    # Tags fixes (verts)
    for tid, data in tag_map.tags.items():
        x, y, z = data["pos"]
        yaw = data["yaw"]
        px = ox + int(x * scale)
        py = oy - int(y * scale)

        # Carré 10cm = 20px
        sz = int(APRILTAG_SIZE_CM * scale)
        p1 = (px - sz//2, py - sz//2)
        p2 = (px + sz//2, py + sz//2)
        cv2.rectangle(img, p1, p2, (0, 200, 0), 1)

        # Flèche orientation
        fx = int(px + sz * np.cos(yaw))
        fy = int(py - sz * np.sin(yaw))
        cv2.arrowedLine(img, (px, py), (fx, fy), (0, 255, 0), 2, tipLength=0.3)
        cv2.putText(img, f"ID{tid}", (px + 8, py - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    # Tags actuellement vus (cyan)
    for det in detections:
        tid = det["tag_id"]
        if tid in tag_map.tags:
            x, y, z = tag_map.tags[tid]["pos"]
            px = ox + int(x * scale)
            py = oy - int(y * scale)
            cv2.circle(img, (px, py), 4, (255, 255, 0), -1)

    # Robot (triangle bleu)
    rx = ox + int(tracker.x * scale)
    ry = oy - int(tracker.y * scale)
    yaw = tracker.yaw
    L = 12  # demi-longueur triangle en pixels
    ptip = (int(rx + L*2.0 * np.cos(yaw)), int(ry - L*2.0 * np.sin(yaw)))
    pleft = (int(rx + L * np.cos(yaw + 2.5)), int(ry - L * np.sin(yaw + 2.5)))
    pright = (int(rx + L * np.cos(yaw - 2.5)), int(ry - L * np.sin(yaw - 2.5)))
    cv2.fillConvexPoly(img, np.array([ptip, pleft, pright]), (255, 100, 0))

    # Cône de vision (gris)
    fov = np.radians(45)
    for angle in (-fov, fov):
        fx = int(rx + 80 * np.cos(yaw + angle))
        fy = int(ry - 80 * np.sin(yaw + angle))
        cv2.line(img, (rx, ry), (fx, fy), (80, 80, 80), 1)

    # Infos texte
    status = "TAG VU" if tracker.confidence > 0.8 else "ODOMETRIE"
    color = (0, 255, 0) if tracker.confidence > 0.8 else (0, 0, 255)
    lines = [
        f"ROBOT {status}",
        f"X={tracker.x:.1f}  Y={tracker.y:.1f}  Z={tracker.z:.1f} cm",
        f"Yaw={np.degrees(tracker.yaw):.1f}°",
        f"Conf={tracker.confidence:.2f}",
    ]
    for i, line in enumerate(lines):
        cv2.putText(img, line, (10, 20 + i*18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    return img


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print("\n[INFO] AprilTag Robot Localization")
    print("  Tags connus :", list(TAG_MAP.keys()))
    print("  Appuie sur Q pour quitter, R pour reset yaw\n")

    arduino = ArduinoReader()  # lecture IMU série
    cap_thread = VideoCaptureThread(CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT)
    tag_detector = AprilTagDetector()
    tag_map    = TagMap(TAG_MAP)
    tracker    = RobotTracker(tag_map)

    time.sleep(0.3)
    fps_q = deque(maxlen=30)
    t_prev = time.perf_counter()

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
        detections = tag_detector.detect(gray)

        # ─── LOCALISATION ──────────────────────────────────────────────────
        tag_seen = tracker.update_from_detections(detections, t_now)

        if not tag_seen:
            # Dead reckoning depuis IMU Arduino (format IMU,yaw,omega_z,...)
            yaw_deg, omega_z = arduino.get()
            vx    = 0.0      # cm/s — pas d'encodeurs sur ce robot
            vy    = 0.0
            omega = float(np.radians(omega_z))  # rad/s depuis gyro
            tracker.update_dead_reckoning(vx, vy, omega, dt)

        # ─── AFFICHAGE CAMÉRA ─────────────────────────────────────────────
        for d in detections:
            c = d["corners"].astype(np.int32)
            cv2.polylines(frame, [c.reshape(-1, 1, 2)], True, (0, 255, 0), 1)
            tid = d["tag_id"]
            cx, cy = int(np.mean(c[:,0])), int(np.mean(c[:,1]))
            cv2.putText(frame, f"ID{tid}", (cx+5, cy-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)

        cv2.putText(frame, f"FPS:{fps:.1f}", (8,20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        # ─── AFFICHAGE CARTE 2D ──────────────────────────────────────────
        map_img = draw_map(tracker, tag_map, detections, w=640, h=480, scale=2.0)

        cv2.imshow("Camera", frame)
        cv2.imshow("MAP", map_img)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        if key == ord('r'):
            arduino.reset_yaw()
            print("[YAW] Reset")

    arduino.stop()
    cap_thread.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()