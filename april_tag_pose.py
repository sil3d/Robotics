"""
============================================================================
  APRILTAG 3D POSE — RASPBERRY PI (FLUIDE + ROBUSTE + INFOS COMPLÈTES)
  
  Corrections :
  • Détection à CHAQUE frame (le vrai lag venait du NLM/CLAHE, pas d'ArUco)
  • Paramètres détecteur plus agressifs pour ne pas perdre les tags
  • Seuil de taille pixel baissé (8 px)
  • Historique avec "prédiction" : garde la dernière pose 0.5s si tag perdu
  • Overlay informatif complet mais sans effets coûteux
============================================================================
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import time
import threading
import json
import os
from collections import deque

# ─────────────────────────────────────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────────────────────────────────────
CAMERA_INDEX       = 0
FRAME_WIDTH        = 640
FRAME_HEIGHT       = 480
TARGET_FPS         = 30

APRILTAG_DICT      = aruco.DICT_APRILTAG_36H11
APRILTAG_SIZE_CM   = 10.0

# ─── INTRINSÈQUES — chargées depuis data/camera_calibration/camera_calibration.json ─────────────────
_CAL_FILE = os.path.join(os.path.dirname(__file__), "data", "camera_calibration", "camera_calibration.json")
if os.path.exists(_CAL_FILE):
    with open(_CAL_FILE) as _f:
        _cal = json.load(_f)
    CAM_MATRIX       = np.array(_cal["camera_matrix"], dtype=np.float32)
    DIST_COEFFS      = np.array(_cal["dist_coeffs"],   dtype=np.float32)
    APRILTAG_SIZE_CM = float(_cal.get("apriltag_size_cm", APRILTAG_SIZE_CM))
else:
    CAM_MATRIX  = np.array([
        [828.395, 0.0,     337.460],
        [0.0,     812.656, 213.622],
        [0.0,     0.0,     1.0]
    ], dtype=np.float32)
    DIST_COEFFS = np.array([[-1.436, 14.760, -0.00570, 0.0543, -37.111]], dtype=np.float32)

MAX_DISTANCE_M     = 5.0
ENABLE_CONTRAST    = True
MIN_TAG_PX_DIAG    = 8       # ↓ plus permissif (avant 12)
PREDICT_LOST_MS    = 500     # garde la pose 500ms après disparition


# ─────────────────────────────────────────────────────────────────────────────
# THREAD CAMÉRA (anti-lag buffer USB)
# ─────────────────────────────────────────────────────────────────────────────
class VideoCaptureThread:
    def __init__(self, src=0, width=640, height=480):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.ret = False
        self.frame = None
        self.stopped = False
        self.lock = threading.Lock()

        th = threading.Thread(target=self._update, daemon=True)
        th.start()

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
# DÉTECTEUR ROBUSTE
# ─────────────────────────────────────────────────────────────────────────────
class AprilTagPoseEstimator:
    def __init__(self, tag_size_cm=APRILTAG_SIZE_CM):
        self.tag_dict   = aruco.getPredefinedDictionary(APRILTAG_DICT)
        self.tag_params = aruco.DetectorParameters()

        # Paramètres intermédiaires : détectent loin sans tuer le CPU
        self.tag_params.adaptiveThreshWinSizeMin   = 3
        self.tag_params.adaptiveThreshWinSizeMax   = 43   # ↑ meilleur pour petits tags
        self.tag_params.adaptiveThreshWinSizeStep  = 5
        self.tag_params.adaptiveThreshConstant     = 9
        self.tag_params.minMarkerPerimeterRate     = 0.015  # ↓ plus permissif
        self.tag_params.polygonalApproxAccuracyRate = 0.04
        self.tag_params.minCornerDistanceRate      = 0.03
        self.tag_params.minDistanceToBorder        = 2
        self.tag_params.cornerRefinementMethod      = aruco.CORNER_REFINE_APRILTAG
        self.tag_params.cornerRefinementWinSize     = 4
        self.tag_params.cornerRefinementMaxIterations = 15  # compromis
        self.tag_params.cornerRefinementMinAccuracy  = 0.01
        self.tag_params.perspectiveRemovePixelPerCell = 4

        self.detector = aruco.ArucoDetector(self.tag_dict, self.tag_params)

        half = tag_size_cm / 2.0
        self.obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

        self.history = {}   # {tid: {"q": deque, "last_seen": t, "last_pose": {...}}}
        self.max_dist_cm = MAX_DISTANCE_M * 100.0

    def _preprocess(self, gray):
        if not ENABLE_CONTRAST:
            return gray
        # ~0.3 ms, booste le contraste local sans tuer le CPU
        return cv2.convertScaleAbs(gray, alpha=1.25, beta=-15)

    @staticmethod
    def _rvec_to_euler(rvec):
        R, _ = cv2.Rodrigues(rvec)
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        if sy < 1e-6:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0
        else:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        return np.array([x, y, z])

    def detect(self, gray, t_now):
        proc = self._preprocess(gray)
        corners, ids, rejected = self.detector.detectMarkers(proc)
        seen_ids = set()

        # ─── Prédiction : on garde les tags récemment vus même si absents ──
        predictions = []
        for tid, h in self.history.items():
            if tid not in [int(i) for i in (ids.flatten() if ids is not None else [])]:
                if (t_now - h["last_seen"]) * 1000.0 < PREDICT_LOST_MS:
                    # Tag "prédit" : on réaffiche sa dernière pose connue
                    predictions.append(h["last_pose"])

        if ids is None:
            return predictions, len(rejected)

        for i, tid in enumerate(ids.flatten()):
            tid = int(tid)
            c = corners[i][0]

            diag = float(np.linalg.norm(c[0] - c[2]))
            if diag < MIN_TAG_PX_DIAG:
                continue

            ok, rvec, tvec = cv2.solvePnP(
                self.obj_points, c,
                CAM_MATRIX, DIST_COEFFS,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not ok:
                continue

            tvec = tvec.flatten()
            dist = float(np.linalg.norm(tvec))
            if dist > self.max_dist_cm:
                continue

            # ─── Historique + lissage ──────────────────────────────────────
            if tid not in self.history:
                self.history[tid] = {
                    "q": deque(maxlen=5),
                    "last_seen": t_now,
                    "last_tvec": tvec.copy(),
                    "last_pose": None
                }

            h = self.history[tid]
            h["q"].append(tvec.copy())
            h["last_seen"] = t_now

            # Médiane pour stabilité
            med_tvec = np.median(np.array(h["q"]), axis=0)

            # Rejet de saut brutal
            jump = np.linalg.norm(med_tvec - h["last_tvec"])
            if jump > 25.0 and len(h["q"]) >= 3:
                med_tvec = h["last_tvec"]

            h["last_tvec"] = med_tvec.copy()

            rvec = rvec.flatten()
            euler = np.degrees(self._rvec_to_euler(rvec))

            pose = {
                "tag_id": tid,
                "tvec": med_tvec,
                "rvec": rvec,
                "euler": euler,
                "dist_cm": float(np.linalg.norm(med_tvec)),
                "corners": c,
                "px": int(diag),
                "predicted": False,
            }
            h["last_pose"] = pose
            seen_ids.add(tid)
            predictions.append(pose)

        # Nettoyage vieux historiques
        for tid in list(self.history.keys()):
            if (t_now - self.history[tid]["last_seen"]) * 1000.0 > PREDICT_LOST_MS * 3:
                del self.history[tid]

        return predictions, len(rejected)


# ─────────────────────────────────────────────────────────────────────────────
# AFFICHAGE COMPLET MAIS OPTIMISÉ
# ─────────────────────────────────────────────────────────────────────────────
def draw_overlay(frame, dets, fps, rejected, det_ms):
    h, w = frame.shape[:2]

    # Barre d'info en haut (noir opaque, pas de transparence)
    cv2.rectangle(frame, (0, 0), (420, 28), (0, 0, 0), -1)
    cv2.putText(frame,
                f"FPS:{fps:.1f} Det:{det_ms:.1f}ms Rej:{rejected} | Q=quit",
                (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

    for d in dets:
        c = d["corners"].astype(np.int32)
        cx = int(np.mean(c[:, 0]))
        cy = int(np.mean(c[:, 1]))

        # Couleur : vert si détecté, orange si prédit (perdu momentanément)
        color = (0, 165, 255) if d.get("predicted") else (0, 255, 0)

        cv2.polylines(frame, [c.reshape(-1, 1, 2)], True, color, 1)
        cv2.circle(frame, (cx, cy), 3, color, -1)

        t = d["tvec"]
        e = d["euler"]

        # Texte groupé (5 putText au lieu de 10)
        lines = [
            f"ID{d['tag_id']}  {d['dist_cm']:.1f}cm  {d['px']}px",
            f"t=({t[0]:+.1f},{t[1]:+.1f},{t[2]:+.1f})",
            f"YPR=({e[0]:+.1f},{e[1]:+.1f},{e[2]:+.1f})",
        ]

        # Fond noir derrière le texte pour lisibilité (rectangle opaque rapide)
        txt_h = 14
        max_w = max(len(l) for l in lines) * 7
        x0, y0 = cx + 8, cy - 50
        cv2.rectangle(frame, (x0 - 2, y0 - 10), (x0 + max_w, y0 + len(lines)*txt_h + 2), (0, 0, 0), -1)

        for j, line in enumerate(lines):
            cv2.putText(frame, line, (x0, y0 + j * txt_h),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

    cv2.drawMarker(frame, (w // 2, h // 2), (120, 120, 120),
                   cv2.MARKER_CROSS, 16, 1)


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────
def main():
    print("\n[INFO] AprilTag RPi — Fluide + Robuste + Infos complètes")
    print(f"  Résolution: {FRAME_WIDTH}x{FRAME_HEIGHT}")
    print(f"  Distance max: {MAX_DISTANCE_M} m")
    print(f"  Prédiction perte: {PREDICT_LOST_MS} ms")
    print("  Q = quit\n")

    cap_thread = VideoCaptureThread(CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT)
    estimator = AprilTagPoseEstimator()
    time.sleep(0.3)

    fps_q = deque(maxlen=30)
    t_prev = time.perf_counter()

    while True:
        ret, frame = cap_thread.read()
        if not ret or frame is None:
            time.sleep(0.005)
            continue

        t_now = time.perf_counter()
        fps_q.append(t_now)
        fps = len(fps_q) / (fps_q[-1] - fps_q[0]) if len(fps_q) > 1 else 0.0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        t0 = time.perf_counter()
        dets, rejected = estimator.detect(gray, t_now)
        det_ms = (time.perf_counter() - t0) * 1000.0

        draw_overlay(frame, dets, fps, rejected, det_ms)

        cv2.imshow("RPi AprilTag", frame)
        if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
            break

    cap_thread.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()