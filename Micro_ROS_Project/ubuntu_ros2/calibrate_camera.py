"""
============================================================================
  CAMERA CALIBRATION TOOL
  Uses OpenCV chessboard calibration with AprilTag as fallback.
  Produces camera_matrix and dist_coeffs for accurate 3D pose estimation.

  HOW TO RUN:
      python calibrate_camera.pyq

  STEPS:
      1. Print the chessboard pattern (see CHESSBOARD cfg below)
      2. Hold the chessboard flat, capture ≥15 images from different angles
         (SPACE to capture)
      3. Press 'c' to compute calibration once you have ≥15 images
      4. The calibrated values are printed and saved to data/camera_calibration/camera_calibration.json

  KEYS:
      SPACE = capture a calibration frame
      C     = compute calibration (need ≥15 frames)
      V     = visualize reprojection errors
      R     = reset all captured frames
      Q     = quit
============================================================================
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import json
import time
import os
from pathlib import Path

# ─────────────────────────────────────────────────────────────────────────────
# CONFIG — update these to match your setup
# ─────────────────────────────────────────────────────────────────────────────
CAMERA_INDEX       = 0  # Change this if multiple cameras
FRAME_WIDTH        = 640  # Lower res = faster framerate
FRAME_HEIGHT       = 480
OUTPUT_JSON        = os.path.join(os.path.dirname(__file__), "..", "..", "data", "camera_calibration", "camera_calibration.json")

# Chessboard parameters (print from OpenCV docs or generate programmatically)
CHESSBOARD_COLS    = 9      # inner corners
CHESSBOARD_ROWS    = 6      # inner corners
SQUARE_SIZE_CM     = 2.0    # physical size of one square in cm

# AprilTag parameters (used as alternative/verification marker)
USE_APRILTAG      = True
APRILTAG_DICT     = aruco.DICT_APRILTAG_36H11
APRILTAG_SIZE_CM  = 10.0    # physical tag side length in cm

# Calibration thresholds
MIN_CALIB_FRAMES  = 15      # minimum frames to attempt calibration
MAX_CALIB_FRAMES  = 50      # cap stored frames
GOOD_FRAMES_KEEP  = 25      # keep best N frames by reprojection error


class CameraCalibrator:
    def __init__(self):
        self.frames         = []        # captured frames (grayscales)
        self.object_points  = []        # 3D world points for each frame
        self.image_points   = []        # 2D image points for each frame
        self.frame_errors   = []        # avg reprojection error per frame
        self.tag_detector   = None
        self.tag_obj_points = None
        if USE_APRILTAG:
            self._setup_april_tag()

    def _setup_april_tag(self):
        self.tag_dict   = aruco.getPredefinedDictionary(APRILTAG_DICT)
        self.tag_params = aruco.DetectorParameters()
        self.tag_detector = aruco.ArucoDetector(self.tag_dict, self.tag_params)
        half = APRILTAG_SIZE_CM / 2.0
        self.tag_obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

    def _create_chessboard_world_points(self):
        objp = np.zeros((CHESSBOARD_COLS * CHESSBOARD_ROWS, 3), np.float32)
        objp[:, :2] = np.mgrid[0:CHESSBOARD_COLS, 0:CHESSBOARD_ROWS].T.reshape(-1, 2)
        objp *= SQUARE_SIZE_CM
        return objp

    def find_chessboard(self, gray):
        found, corners = cv2.findChessboardCorners(gray, (CHESSBOARD_COLS, CHESSBOARD_ROWS))
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        return found, corners

    def find_april_tags(self, gray):
        corners, ids, rejected = self.tag_detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return [], []
        tag_corners = [c[0] for c in corners]
        tag_ids = ids.flatten().tolist()
        return tag_corners, tag_ids

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = {"chessboard": False, "apriltag": False, "corners": [], "ids": []}

        # Try chessboard first
        found_cb, corners_cb = self.find_chessboard(gray)
        if found_cb:
            detections["chessboard"] = True
            detections["corners"] = corners_cb
            detections["ids"] = list(range(len(corners_cb)))
            return detections

        # Try AprilTag
        if USE_APRILTAG:
            tag_corners, tag_ids = self.find_april_tags(gray)
            if tag_ids:
                detections["apriltag"] = True
                detections["corners"] = tag_corners
                detections["ids"] = tag_ids
                return detections

        return detections

    def capture_calibration_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dets = self.process_frame(frame)
        if not dets["corners"]:
            return False, "No pattern detected"

        corners = dets["corners"]

        if dets["chessboard"]:
            objp = self._create_chessboard_world_points()
            corners_arr = np.array(corners, dtype=np.float32)
            if len(corners_arr.shape) == 3:
                corners_arr = corners_arr.reshape(-1, 1, 2)
            self.frames.append(gray)
            self.image_points.append(corners_arr)
            self.object_points.append(objp.astype(np.float32))
        elif dets["apriltag"]:
            corners_arr = np.array(corners, dtype=np.float32)
            if len(corners_arr.shape) == 2:
                corners_arr = corners_arr.reshape(1, -1, 2)
            obj_pts_all = []
            for _ in range(len(corners)):
                obj_pts_all.append(self.tag_obj_points.copy().astype(np.float32))
            self.frames.append(gray)
            self.image_points.append(corners_arr)
            self.object_points.append(obj_pts_all)

        status = f"Captured ({len(self.frames)}/{MIN_CALIB_FRAMES})"
        return True, status

    def calibrate(self):
        if len(self.image_points) < MIN_CALIB_FRAMES:
            return None, None, None, f"Need ≥{MIN_CALIB_FRAMES} frames, got {len(self.image_points)}"

        obj_points_valid = []
        img_points_valid = []

        for i, (obj_pts, img_pts) in enumerate(zip(self.object_points, self.image_points)):
            if img_pts is None or len(img_pts) == 0:
                continue

            if isinstance(obj_pts, list):
                all_obj = np.vstack(obj_pts)
            else:
                all_obj = obj_pts

            if isinstance(img_pts, list):
                all_img = np.vstack(img_pts)
            else:
                all_img = img_pts.reshape(-1, 1, 2)

            if len(all_obj) > 0 and len(all_img) > 0:
                obj_points_valid.append(all_obj.astype(np.float32))
                img_points_valid.append(all_img.astype(np.float32))

        if len(obj_points_valid) < MIN_CALIB_FRAMES:
            return None, None, None, f"Only {len(obj_points_valid)} valid frames"

        ret, cam_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            obj_points_valid, img_points_valid,
            (FRAME_WIDTH, FRAME_HEIGHT),
            None, None
        )

        if not ret:
            return None, None, None, "calibrateCamera failed"

        # Compute per-frame reprojection errors
        errors = []
        for i, (obj_pts, img_pts, rvec, tvec) in enumerate(zip(
                obj_points_valid, img_points_valid, rvecs, tvecs)):
            img_pts_projected, _ = cv2.projectPoints(obj_pts, rvec, tvec, cam_matrix, dist_coeffs)
            img_pts_projected = img_pts_projected.reshape(-1, 2)
            img_pts_actual = img_pts.reshape(-1, 2)
            errors.append(np.linalg.norm(img_pts_projected - img_pts_actual))

        mean_error = float(np.mean(errors))
        return cam_matrix, dist_coeffs, mean_error, f"Calibration done! Mean error: {mean_error:.2f} px"

    def save_calibration(self, cam_matrix, dist_coeffs, mean_error):
        os.makedirs(os.path.dirname(OUTPUT_JSON), exist_ok=True)
        data = {
            "calibration_date": time.strftime("%Y-%m-%d %H:%M:%S"),
            "frame_width": FRAME_WIDTH,
            "frame_height": FRAME_HEIGHT,
            "camera_matrix": cam_matrix.tolist(),
            "dist_coeffs": dist_coeffs.tolist(),
            "mean_reprojection_error_px": round(mean_error, 4),
            "frames_used": len(self.image_points),
            "apriltag_size_cm": APRILTAG_SIZE_CM,
        }
        with open(OUTPUT_JSON, "w") as f:
            json.dump(data, f, indent=2)
        return data


def draw_overlay(frame, calibrator, dets, fps, error_text=""):
    h, w = frame.shape[:2]
    overlay = frame.copy()
    pad = 10
    lh = 18

    lines = [
        "=== CAMERA CALIBRATION ===",
        f"FPS: {fps:.1f}",
        f"Frames captured: {len(calibrator.frames)}/{MIN_CALIB_FRAMES} (min needed)",
        "",
        "KEYS:",
        "  SPACE = capture frame",
        "  C     = calibrate (≥15 frames)",
        "  V     = show reprojection",
        "  R     = reset",
        "  Q     = quit",
    ]
    for i, text in enumerate(lines):
        color = (0, 220, 220) if i == 0 else (255, 255, 255)
        cv2.putText(frame, text, (pad, pad + i * lh), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    if dets["chessboard"]:
        cv2.drawChessboardCorners(frame, (CHESSBOARD_COLS, CHESSBOARD_ROWS), dets["corners"], True)
        status = "CHESSBOARD DETECTED"
        sc = (0, 255, 0)
    elif dets["apriltag"]:
        for c in dets["corners"]:
            cv2.polylines(frame, [c.reshape(-1, 1, 2).astype(int)], True, (0, 255, 0), 2)
        status = f"APRILTAG DETECTED ({len(dets['corners'])} tag(s))"
        sc = (0, 255, 0)
    else:
        status = "No pattern — show chessboard or AprilTag"
        sc = (0, 165, 255)

    cv2.putText(frame, status, (pad, h - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, sc, 1)

    if error_text:
        cv2.putText(frame, error_text, (pad, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1)

    cx, cy = w // 2, h // 2
    cv2.drawMarker(frame, (cx, cy), (200, 200, 200), cv2.MARKER_CROSS, 20, 1)


def list_cameras():
    print("\n[INFO] Searching for available cameras...")
    available = []
    for i in range(2):  # Only check 0 and 1 - skip ffmpeg errors
        try:
            cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)  # Use DirectShow for Windows
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    available.append(i)
                    print(f"  Camera {i}: AVAILABLE")
                cap.release()
        except Exception as e:
            print(f"  Camera {i}: error - {e}")
    if not available:
        print("  No cameras found!")
        return []
    print(f"\n  Found {len(available)} working camera(s): {available}")
    return available

def main():
    calibrator = CameraCalibrator()

    available = list_cameras()
    if not available:
        print("[ERROR] No cameras found. Check if webcam is connected.")
        return

    # Try CAMERA_INDEX first, then try other cameras
    global CAMERA_INDEX
    cap = None
    camera_found = False

    for try_idx in [CAMERA_INDEX] + [i for i in available if i != CAMERA_INDEX]:
        for attempt in range(3):
            print(f"  Trying camera {try_idx}, attempt {attempt + 1}...")
            cap = cv2.VideoCapture(try_idx, cv2.CAP_DSHOW)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    print(f"  [OK] Camera {try_idx} is working!")
                    CAMERA_INDEX = try_idx
                    camera_found = True
                    break
                cap.release()
                time.sleep(0.5)
        if camera_found:
            break

    if not camera_found:
        print(f"[ERROR] Cannot open any camera. Available: {available}")
        return

    print("\n[INFO] Camera Calibration Tool")
    print(f"  Resolution: {FRAME_WIDTH}x{FRAME_HEIGHT}")
    print(f"  Min frames needed: {MIN_CALIB_FRAMES}")
    print(f"  Pattern: chessboard ({CHESSBOARD_COLS}x{CHESSBOARD_ROWS}) or AprilTag 36H11")
    print()

    fps_times = []
    last_error = ""
    show_reprojection = False
    reproject_frame = None
    frame_count = 0
    display_skip = 2

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        frame_count += 1

        # Only detect pattern every 3 frames
        if frame_count % 3 == 1:
            dets = calibrator.process_frame(frame)
        else:
            dets = {"chessboard": False, "apriltag": False, "corners": [], "ids": []}

        # Only display every display_skip frames to improve performance
        if frame_count % display_skip == 0:
            t_now = time.perf_counter()

            # Update FPS counter every 30 frames
            if len(fps_times) >= 30:
                fps_times = fps_times[-29:]
            if fps_times:
                fps = len(fps_times) / (t_now - fps_times[0])
            else:
                fps = 0
            fps_times.append(t_now)

            if show_reprojection and reproject_frame is not None:
                frame = reproject_frame.copy()
                reproject_frame = None

            draw_overlay(frame, calibrator, dets, fps, last_error)
            cv2.imshow("CALIBRATION", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:
                break
            elif key == ord(' '):
                ok, msg = calibrator.capture_calibration_frame(frame)
                last_error = msg
                if ok:
                    print(f"  [OK] {msg}")
                else:
                    print(f"  [WARN] {msg}")
            elif key == ord('c'):
                cam_mat, dist_coef, mean_err, msg = calibrator.calibrate()
                last_error = msg
                print(f"\n[MESSAGE] {msg}")
                if cam_mat is not None:
                    print(f"\n  Camera matrix (fx, fy, cx, cy):")
                    print(f"  {cam_mat.tolist()}")
                    print(f"\n  Distortion coeffs:")
                    print(f"  {dist_coef.tolist()}")
                    data = calibrator.save_calibration(cam_mat, dist_coef, mean_err)
                    print(f"\n  [SAVE] Saved to {OUTPUT_JSON}")
                    print("\n  Paste these into main.py Config:")
                    print(f"    CAM_MATRIX = np.array({cam_mat.tolist()}, dtype=np.float32)")
                    print(f"    DIST_COEFFS = np.array({dist_coef.tolist()}, dtype=np.float32)")
                    print()
            elif key == ord('v'):
                show_reprojection = True
            elif key == ord('r'):
                calibrator.frames.clear()
                calibrator.image_points.clear()
                calibrator.object_points.clear()
                last_error = ""
                print("  [RESET] All frames cleared")
        else:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
