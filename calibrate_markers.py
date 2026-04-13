"""
=============================================================================
  ARUCO MARKER CALIBRATION TOOL — v2
  - Loads reference images from professor
  - Identifies markers by image comparison (template + feature matching)
  - Auto-adjusts coordinates from pose estimation
  - Saves marker IDs, positions, and patches to JSON
  - Self-corrects if given coordinates are inaccurate
=============================================================================

  SETUP:
    1. Create a folder called "reference_markers/" next to this script
    2. Put the professor's marker images inside, named like:
         marker_1.png   marker_2.png   marker_3.png  ...
       (the number in the filename = the marker ID)
    3. Optionally create "reference_markers.json" with known positions:
         {"markers": {"1": {"x_cm": 0, "y_cm": 150}, "2": {"x_cm": 200, "y_cm": 50}}}

  RUN:
    python calibrate_markers.py

  KEYS:
    SPACE = record visible markers (auto-identify + measure position)
    A     = auto-adjust all confirmed against reference
    S     = save to JSON
    L     = load reference JSON
    R     = reset buffers
    Q     = quit
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import json
import time
import os
from collections import deque

# ─────────────────────────────────────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────────────────────────────────────
CAMERA_INDEX    = 0
FRAME_WIDTH     = 640
FRAME_HEIGHT    = 480
MARKER_SIZE_CM  = 10.0

CAM_MATRIX      = np.array([
    [800,   0, 320],
    [  0, 800, 240],
    [  0,   0,   1]
], dtype=np.float32)
DIST_COEFFS     = np.zeros((5, 1), dtype=np.float32)

# Folders / files
REFERENCE_DIR       = "reference_markers"
REFERENCE_JSON      = "reference_markers.json"
OUTPUT_JSON         = "marker_positions.json"
OUTPUT_PATCHES      = "marker_patches"

# Stabilisation
STABLE_FRAMES       = 30
POSITION_TOLERANCE_CM = 5.0

# Identification thresholds
TEMPLATE_MIN_SCORE  = 0.65    # template matching minimum (0-1)
ORB_MIN_MATCHES     = 8       # minimum feature matches for confirmation


class MarkerIdentifier:
    """
    Identifies ArUco markers by comparing detected patches
    against reference images using two methods:
      1. Template matching (normalized cross-correlation)
      2. ORB feature matching (rotation/scale invariant)
    """

    def __init__(self, ref_dir):
        self.references = {}   # {marker_id: {"image": np.ndarray, "gray": np.ndarray, "orb_kp": list, "orb_des": np.ndarray}}
        self.orb = cv2.ORB_create(nfeatures=500)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self._load_references(ref_dir)

    def _load_references(self, ref_dir):
        if not os.path.isdir(ref_dir):
            print(f"[WARN] Reference directory '{ref_dir}' not found — image identification disabled")
            return

        for fname in sorted(os.listdir(ref_dir)):
            if not fname.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp')):
                continue

            # Extract ID from filename: marker_1.png -> 1
            base = os.path.splitext(fname)[0]
            parts = base.replace("-", "_").split("_")
            marker_id = None
            for part in parts:
                if part.isdigit():
                    marker_id = int(part)
                    break

            if marker_id is None:
                print(f"  [WARN] Cannot extract marker ID from '{fname}' — skipping")
                continue

            path = os.path.join(ref_dir, fname)
            img = cv2.imread(path)
            if img is None:
                print(f"  [WARN] Cannot read '{path}' — skipping")
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            kp, des = self.orb.detectAndCompute(gray, None)

            self.references[marker_id] = {
                "image": img,
                "gray": gray,
                "path": path,
                "orb_kp": kp,
                "orb_des": des,
            }
            print(f"  [LOAD] Marker {marker_id} from '{fname}' ({img.shape[1]}x{img.shape[0]})")

        if self.references:
            print(f"[INFO] Loaded {len(self.references)} reference images")
        else:
            print("[WARN] No reference images found — identification by image disabled")

    def identify(self, patch_gray):
        """
        Identify a marker patch against all references.
        Returns (marker_id, confidence, method) or (None, 0, None).
        """
        if not self.references:
            return None, 0.0, None

        best_id = None
        best_score = 0.0
        best_method = None

        for mid, ref in self.references.items():
            # Method 1: Template matching
            score_tmpl = self._template_match(patch_gray, ref["gray"])

            # Method 2: ORB feature matching
            score_orb = self._orb_match(patch_gray, ref)

            # Take the best of both
            score = max(score_tmpl, score_orb)
            method = "template" if score_tmpl >= score_orb else "orb"

            if score > best_score:
                best_score = score
                best_id = mid
                best_method = method

        if best_score >= TEMPLATE_MIN_SCORE:
            return best_id, best_score, best_method
        return None, best_score, best_method

    def _template_match(self, patch_gray, ref_gray):
        if patch_gray.shape[0] < ref_gray.shape[0] or patch_gray.shape[1] < ref_gray.shape[1]:
            # Resize reference to fit patch
            ref_resized = cv2.resize(ref_gray, (patch_gray.shape[1], patch_gray.shape[0]))
        else:
            ref_resized = ref_gray

        result = cv2.matchTemplate(patch_gray, ref_resized, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, _ = cv2.minMaxLoc(result)
        return float(max_val)

    def _orb_match(self, patch_gray, ref):
        if ref["orb_des"] is None or len(ref["orb_kp"]) == 0:
            return 0.0

        kp, des = self.orb.detectAndCompute(patch_gray, None)
        if des is None or len(kp) < 4:
            return 0.0

        try:
            matches = self.bf.match(ref["orb_des"], des)
            matches = sorted(matches, key=lambda m: m.distance)

            if len(matches) < ORB_MIN_MATCHES:
                return 0.0

            # Score based on top matches (lower distance = better)
            avg_dist = np.mean([m.distance for m in matches[:ORB_MIN_MATCHES]])
            max_dist = 60.0  # typical max for good matches
            score = max(0.0, 1.0 - (avg_dist / max_dist))
            return float(score)
        except Exception:
            return 0.0


class MarkerCalibrator:

    def __init__(self, ref_dir=REFERENCE_DIR):
        self.aruco_dict   = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.detector     = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        half = MARKER_SIZE_CM / 2.0
        self.obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

        self.identifier = MarkerIdentifier(ref_dir)

        self.buffers = {}
        self.confirmed = {}
        self.reference = {}

        # Last identification results for display: {marker_id: (identified_id, confidence, method)}
        self.last_identities = {}

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        results = []

        if ids is None:
            return results

        for i, mid in enumerate(ids.flatten()):
            c = corners[i][0]
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points, c, CAM_MATRIX, DIST_COEFFS,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not success:
                continue

            tvec = tvec.flatten()
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))

            results.append({
                "id": int(mid),
                "corners": c,
                "tvec": tvec,
                "rvec": rvec.flatten(),
                "center_px": (cx, cy),
            })
        return results

    def update_buffer(self, detections):
        for det in detections:
            mid = det["id"]
            if mid not in self.buffers:
                self.buffers[mid] = deque(maxlen=STABLE_FRAMES)
            self.buffers[mid].append(det["tvec"].copy())

    def get_stable_position(self, marker_id):
        if marker_id not in self.buffers or len(self.buffers[marker_id]) < STABLE_FRAMES:
            return None
        buf = np.array(self.buffers[marker_id])
        return np.mean(buf, axis=0)

    def is_stable(self, marker_id):
        return marker_id in self.buffers and len(self.buffers[marker_id]) >= STABLE_FRAMES

    def identify_marker(self, frame, corners):
        """Extract patch and identify against reference images."""
        pts = corners.astype(int)
        x, y, w, h = cv2.boundingRect(pts)
        margin = 5
        x = max(0, x - margin)
        y = max(0, y - margin)
        w = min(frame.shape[1] - x, w + 2 * margin)
        h = min(frame.shape[0] - y, h + 2 * margin)
        patch = frame[y:y+h, x:x+w]
        patch_gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)

        identified_id, confidence, method = self.identifier.identify(patch_gray)
        return identified_id, confidence, method, patch

    def auto_adjust(self, marker_id, measured_tvec):
        if marker_id not in self.reference:
            return measured_tvec, False, "No reference — using measured"

        ref = self.reference[marker_id]
        ref_pos = np.array([ref.get("x_cm", 0), ref.get("y_cm", 0), ref.get("z_cm", 0)])
        measured_pos = np.array([measured_tvec[0], measured_tvec[1], measured_tvec[2]])

        diff = np.linalg.norm(measured_pos - ref_pos)

        if diff > POSITION_TOLERANCE_CM:
            return measured_tvec, True, f"Auto-adjusted (diff={diff:.1f} cm)"
        else:
            blended = 0.7 * ref_pos + 0.3 * measured_pos
            return blended, False, f"Reference OK (diff={diff:.1f} cm) — blended"

    def save_json(self):
        data = {
            "calibration_date": time.strftime("%Y-%m-%d %H:%M:%S"),
            "marker_size_cm": MARKER_SIZE_CM,
            "camera_matrix": CAM_MATRIX.tolist(),
            "dist_coeffs": DIST_COEFFS.tolist(),
            "markers": {}
        }

        for mid, info in self.confirmed.items():
            data["markers"][str(mid)] = {
                "x_cm": round(info["x_cm"], 2),
                "y_cm": round(info["y_cm"], 2),
                "z_cm": round(info["z_cm"], 2),
                "adjusted": info.get("adjusted", False),
                "adjust_reason": info.get("adjust_reason", ""),
                "identified_by": info.get("identified_by", "aruco_id"),
                "id_confidence": round(info.get("id_confidence", 0), 3),
                "patch_file": info.get("patch_file", ""),
                "reference_file": info.get("reference_file", ""),
                "sample_count": STABLE_FRAMES,
            }

        with open(OUTPUT_JSON, "w") as f:
            json.dump(data, f, indent=2)

        print(f"\n[SAVE] {len(self.confirmed)} markers saved to {OUTPUT_JSON}")
        return OUTPUT_JSON

    def load_reference_json(self, path):
        if not os.path.exists(path):
            print(f"[INFO] No reference JSON at {path} — starting fresh")
            return
        with open(path, "r") as f:
            data = json.load(f)
        for mid_str, info in data.get("markers", {}).items():
            self.reference[int(mid_str)] = info
        print(f"[INFO] Loaded {len(self.reference)} reference positions from {path}")


def draw_ui(frame, calibrator, detections, fps):
    h, w = frame.shape[:2]
    overlay = frame.copy()
    pad = 10
    lh = 20

    # ── Left panel ───────────────────────────────────────────────────────
    lines_left = [
        ("=== ARUCO CALIBRATION v2 ===", (0, 220, 220)),
        (f"FPS: {fps:.1f}", (255, 255, 255)),
        ("", (255, 255, 255)),
        ("KEYS:", (0, 220, 220)),
        ("  SPACE = record markers", (255, 255, 255)),
        ("  A     = auto-adjust all", (255, 255, 255)),
        ("  S     = save JSON", (255, 255, 255)),
        ("  L     = load ref JSON", (255, 255, 255)),
        ("  R     = reset buffers", (255, 255, 255)),
        ("  Q     = quit", (255, 255, 255)),
        ("", (255, 255, 255)),
        (f"Refs loaded: {len(calibrator.identifier.references)} images", (200, 200, 0)),
        (f"Confirmed: {len(calibrator.confirmed)} markers", (0, 255, 0)),
    ]

    y = pad
    for text, color in lines_left:
        cv2.putText(frame, text, (pad, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        y += lh

    # ── Right panel: confirmed ───────────────────────────────────────────
    y = pad
    x_right = w - 300
    panel_h = 30 + min(len(calibrator.confirmed), 10) * lh
    cv2.rectangle(overlay, (x_right, y), (w - pad, y + panel_h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

    cv2.putText(frame, "CONFIRMED MARKERS", (x_right + 5, y + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 220), 1)
    y += 30
    for mid, info in sorted(calibrator.confirmed.items()):
        status = "ADJ" if info.get("adjusted") else "OK"
        color = (0, 140, 255) if info.get("adjusted") else (0, 255, 0)
        text = f"ID {mid}: ({info['x_cm']:.0f},{info['y_cm']:.0f},{info['z_cm']:.0f}) [{status}]"
        cv2.putText(frame, text, (x_right + 5, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
        y += lh

    # ── Detection labels ─────────────────────────────────────────────────
    for det in detections:
        mid = det["id"]
        cx, cy = det["center_px"]
        tvec = det["tvec"]

        if calibrator.is_stable(mid):
            ring_color = (0, 255, 0)
            stability = f"STABLE"
        else:
            count = len(calibrator.buffers.get(mid, []))
            ring_color = (0, 165, 255)
            stability = f"BUFFERING {count}/{STABLE_FRAMES}"

        pts = det["corners"].astype(int)
        cv2.polylines(frame, [pts.reshape(-1, 1, 2)], True, ring_color, 2)

        info_lines = [
            f"ArUco ID: {mid}",
            f"Pos: ({tvec[0]:.1f}, {tvec[1]:.1f}, {tvec[2]:.1f}) cm",
            stability,
        ]

        # Show image identification result
        if mid in calibrator.last_identities:
            ident_id, ident_conf, ident_method = calibrator.last_identities[mid]
            if ident_id is not None:
                info_lines.append(f"Ref match: ID={ident_id} ({ident_conf*100:.0f}%)")
                info_lines.append(f"Method: {ident_method}")
                if ident_id != mid:
                    info_lines.append(f"⚠ MISMATCH! ArUco={mid} vs Ref={ident_id}")
                    ring_color = (0, 0, 255)
            else:
                info_lines.append(f"No ref match ({ident_conf*100:.0f}%)")

        draw_info_box(frame, (cx - 90, cy - 100), info_lines, ring_color)


def draw_info_box(frame, origin, lines, color):
    x, y = origin
    x = max(5, x)
    y = max(50, y)
    pad = 4
    lh = 15
    box_w = 230
    box_h = len(lines) * lh + pad * 2

    overlay = frame.copy()
    cv2.rectangle(overlay, (x, y), (x + box_w, y + box_h), (10, 10, 10), -1)
    cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)

    for i, line in enumerate(lines):
        c = (0, 0, 255) if "MISMATCH" in str(line) else color
        cv2.putText(frame, str(line), (x + pad, y + pad + (i + 1) * lh),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, c, 1, cv2.LINE_AA)


def main():
    calibrator = MarkerCalibrator(REFERENCE_DIR)
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {CAMERA_INDEX}")
        return

    print(f"\n[INFO] Camera opened")
    print("[INFO] ARUCO CALIBRATION TOOL v2")
    print("  SPACE = record visible markers")
    print("  A     = auto-adjust against reference")
    print("  S     = save to JSON")
    print("  L     = load reference JSON")
    print("  R     = reset buffers")
    print("  Q     = quit")
    print()

    fps_times = deque(maxlen=30)
    os.makedirs(OUTPUT_PATCHES, exist_ok=True)

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        fps_times.append(time.perf_counter())
        fps = (len(fps_times) - 1) / (fps_times[-1] - fps_times[0]) if len(fps_times) > 1 else 0

        detections = calibrator.detect(frame)
        calibrator.update_buffer(detections)

        # Run image identification on each detection
        calibrator.last_identities.clear()
        for det in detections:
            ident_id, ident_conf, ident_method, _ = calibrator.identify_marker(frame, det["corners"])
            calibrator.last_identities[det["id"]] = (ident_id, ident_conf, ident_method)

        draw_ui(frame, calibrator, detections, fps)

        cx, cy = FRAME_WIDTH // 2, FRAME_HEIGHT // 2
        cv2.drawMarker(frame, (cx, cy), (200, 200, 200), cv2.MARKER_CROSS, 20, 1)

        cv2.imshow("ARUCO CALIBRATION", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:
            break

        elif key == ord(' '):
            recorded = 0
            for det in detections:
                mid = det["id"]
                if not calibrator.is_stable(mid):
                    print(f"  [WARN] Marker {mid} not stable — skipping")
                    continue

                stable_tvec = calibrator.get_stable_position(mid)
                adjusted_tvec, was_adjusted, reason = calibrator.auto_adjust(mid, stable_tvec)

                # Image identification
                ident_id, ident_conf, ident_method, patch = calibrator.identify_marker(frame, det["corners"])

                # Save patch
                patch_path = os.path.join(OUTPUT_PATCHES, f"aruco_{mid}.png")
                cv2.imwrite(patch_path, patch)

                ref_file = ""
                if ident_id is not None and ident_id in calibrator.identifier.references:
                    ref_file = calibrator.identifier.references[ident_id]["path"]

                calibrator.confirmed[mid] = {
                    "x_cm": float(adjusted_tvec[0]),
                    "y_cm": float(adjusted_tvec[1]),
                    "z_cm": float(adjusted_tvec[2]),
                    "adjusted": was_adjusted,
                    "adjust_reason": reason,
                    "identified_by": ident_method if ident_method else "aruco_id",
                    "id_confidence": float(ident_conf),
                    "patch_file": patch_path,
                    "reference_file": ref_file,
                }
                recorded += 1
                id_info = f"ref_match={ident_id}({ident_conf*100:.0f}%)" if ident_id else "no_ref_match"
                print(f"  [RECORD] Marker {mid}: x={adjusted_tvec[0]:.1f} y={adjusted_tvec[1]:.1f} z={adjusted_tvec[2]:.1f} | {id_info} | {reason}")

            if recorded:
                print(f"  [OK] {recorded} marker(s) recorded")
            else:
                print("  [WARN] No stable markers to record")

        elif key == ord('a'):
            print("\n[AUTO-ADJUST] Checking all confirmed markers...")
            for mid in list(calibrator.confirmed.keys()):
                if mid in calibrator.buffers and len(calibrator.buffers[mid]) >= 5:
                    buf = np.array(calibrator.buffers[mid])
                    measured = np.mean(buf, axis=0)
                    adjusted, was_adjusted, reason = calibrator.auto_adjust(mid, measured)
                    calibrator.confirmed[mid]["x_cm"] = float(adjusted[0])
                    calibrator.confirmed[mid]["y_cm"] = float(adjusted[1])
                    calibrator.confirmed[mid]["z_cm"] = float(adjusted[2])
                    calibrator.confirmed[mid]["adjusted"] = was_adjusted
                    calibrator.confirmed[mid]["adjust_reason"] = reason
                    print(f"  Marker {mid}: {reason}")
                else:
                    print(f"  Marker {mid}: not visible — skipping")

        elif key == ord('s'):
            if calibrator.confirmed:
                calibrator.save_json()
            else:
                print("  [WARN] No confirmed markers to save")

        elif key == ord('l'):
            ref_path = input("  Reference JSON path (default: reference_markers.json): ").strip()
            if not ref_path:
                ref_path = REFERENCE_JSON
            calibrator.load_reference_json(ref_path)

        elif key == ord('r'):
            calibrator.buffers.clear()
            print("  [RESET] All buffers cleared")

    cap.release()
    cv2.destroyAllWindows()

    if calibrator.confirmed:
        calibrator.save_json()
        print("[INFO] Auto-saved on exit")


if __name__ == "__main__":
    main()
