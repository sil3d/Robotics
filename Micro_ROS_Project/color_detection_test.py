#!/usr/bin/env python3
"""
===========================================================================
  COLOR DETECTION TOOL - OPTIMIZED FOR RASPBERRY PI 4B
  v4 - AprilTag 36h11 (standard for robotics, more robust)
===========================================================================

  Changes v4 vs v3:
    - AprilTag: DICT_APRILTAG_36H11 (standard, matches project docs)
    - AprilTag: uses AprilTagPoseEstimator from april_tag_pose.py
    - AprilTag: prediction keeps last pose 500ms after tag lost
    - AprilTag: median filtering for stability

  Controls:
      Q = Quit
      S = Save screenshot
      M = Toggle mask view (debug)
      A = Toggle AprilTag overlay

===========================================================================
"""

import cv2
import numpy as np
import time
import sys
import os

# Add parent dir to path to import april_tag_pose
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from april_tag_pose import AprilTagPoseEstimator, VideoCaptureThread

# AprilTag size in meters
APRILTAG_SIZE_M = 0.10  # 10 cm

# ===========================================================================
# GLOBAL CONFIG
# ===========================================================================

PROCESS_WIDTH  = 320
PROCESS_HEIGHT = 240

CAMERA_INDEX  = 1
CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480

MIN_PIXEL_COUNT = 200   # minimum pixels to declare a valid detection
FOCAL_LENGTH    = 414.2 # scaled focal length matching PROCESS_WIDTH
BOX_WIDTH_MM    = 100   # reference object real width (mm)

FRAME_SKIP = 2          # process every Nth frame (1 = every frame)

# ===========================================================================
# HSV COLOUR RANGES  (v3 — tuned + cyan gap fixed)
# ===========================================================================

# RED — two ranges because hue wraps around 0/180
# Saturation min=130 excludes most skin (palm 20-80, fingertip 80-120)
RED_L1 = np.array([0,   150, 80],  dtype=np.uint8)
RED_U1 = np.array([10,  255, 240], dtype=np.uint8)
RED_L2 = np.array([170, 150, 80],  dtype=np.uint8)
RED_U2 = np.array([180, 255, 240], dtype=np.uint8)

# GREEN — upper hue capped at 85 (was 80, avoids overlap with blue at 90)
GREEN_L = np.array([40, 50, 50],  dtype=np.uint8)
GREEN_U = np.array([85, 255, 255], dtype=np.uint8)

# BLUE — hue starts at 90 (was 100) to capture cyan; S min=60 (was 40)
# Specular highlights are handled by HIGHLIGHT mask below, not by widening S.
BLUE_L = np.array([100,  100, 50],  dtype=np.uint8)
BLUE_U = np.array([125, 255, 255], dtype=np.uint8)

# Near-white pixels caused by specular reflections on blue objects
HIGHLIGHT_L = np.array([0,   0,  230], dtype=np.uint8)
HIGHLIGHT_U = np.array([179, 40, 255], dtype=np.uint8)

# ===========================================================================
# ENVIRONMENT MARKER IDs (ArUco)
# ===========================================================================
HOME_ID             = 12   # marker at the home position
MANUFACTURING_ID    = 3   # (if needed)
STATION_A_ID        = 9  # for green boxes (Station A)
STATION_B_ID        = 6  # for blue boxes (Station B)

# Known global positions (x, y, theta) of each marker – example in metres and radians.
# This allows the robot to compute its own pose when it sees a marker.
'''MARKER_POSITIONS = {
    HOME_ID:          (0.0, 0.0, 0.0),
    MANUFACTURING_ID: (0.0, 1.0, 0.0),
    STATION_A_ID:     (2.0, 1.5, 0.0),
    STATION_B_ID:     (2.0, -1.5, 0.0),
}'''

# Distance threshold to stop (teacher: station is 15cm from marker)
STOP_DISTANCE_CM = 15
STOP_DISTANCE_M  = STOP_DISTANCE_CM / 100.0   # 0.15 m

# ===========================================================================
# COLOUR DETECTOR
# ===========================================================================

class ColorDetector:
    """
    RPi-optimised HSV colour detector.

    Pipeline per colour
    -------------------
    1. inRange(HSV)          → raw binary mask
    2. OPEN (3×3)            → removes small noise blobs
    3. CLOSE (7×7)           → fills gaps (reflection holes for blue)
    4. _get_box_info()       → largest contour, shape validation, bbox

    Blue extra step (between 1 and 2):
      subtract highlight mask → exclude specular white pixels before closing
    """

    def __init__(self):
        self.frame_count = 0
        # Pre-allocated HSV buffer — avoids per-frame malloc
        self.hsv = np.zeros((PROCESS_HEIGHT, PROCESS_WIDTH, 3), dtype=np.uint8)
        # Morphological kernels (pre-allocated once)
        self.open_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

    # ------------------------------------------------------------------
    def _clean_mask(self, mask):
        """Nettoyage identique pour les 3 couleurs."""
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.open_kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.close_kernel, iterations=2)
        return mask

    # ------------------------------------------------------------------
    def detect(self, frame):
        """
        Detect red / green / blue in *frame*.
        Returns result dict, or None on skipped frames.
        """
        self.frame_count += 1
        if self.frame_count % FRAME_SKIP != 0:
            return None

        # Resize once → process at low resolution
        frame_small = cv2.resize(frame, (PROCESS_WIDTH, PROCESS_HEIGHT),
                                 interpolation=cv2.INTER_LINEAR)
        cv2.cvtColor(frame_small, cv2.COLOR_BGR2HSV, dst=self.hsv)

        result = {
            'detected':    [],
            'red':   False, 'green':  False, 'blue':  False,
            'red_count': 0, 'green_count': 0, 'blue_count': 0,
            'red_box': None, 'green_box': None, 'blue_box': None,
            'frame_id': self.frame_count,
        }

        # ── RED ──────────────────────────────────────────────────────────
        rm1      = cv2.inRange(self.hsv, RED_L1, RED_U1)
        rm2      = cv2.inRange(self.hsv, RED_L2, RED_U2)
        red_mask = self._clean_mask(cv2.bitwise_or(rm1, rm2))

        red_count = cv2.countNonZero(red_mask)
        result['red_count'] = red_count
        if red_count > MIN_PIXEL_COUNT:
            result['red'] = True
            result['detected'].append('red')
            self._get_box_info(red_mask, 'red', result)

        # ── GREEN ─────────────────────────────────────────────────────────
        green_raw = cv2.inRange(self.hsv, GREEN_L, GREEN_U)
        green_mask  = self._clean_mask(green_raw)
        green_count = cv2.countNonZero(green_mask)
        result['green_count'] = green_count
        if green_count > MIN_PIXEL_COUNT:
            result['green'] = True
            result['detected'].append('green')
            self._get_box_info(green_mask, 'green', result)

        # ── BLUE ──────────────────────────────────────────────────────────
        # 1. Raw mask
        blue_raw = cv2.inRange(self.hsv, BLUE_L, BLUE_U)
        # 2. Exclude specular highlights (near-white pixels from light glare)
        highlight   = cv2.inRange(self.hsv, HIGHLIGHT_L, HIGHLIGHT_U)
        blue_no_hl  = cv2.bitwise_and(blue_raw, cv2.bitwise_not(highlight))
        # 3. Clean (large closing fills the holes the highlights left behind)
        blue_mask   = self._clean_mask(blue_no_hl)

        blue_count = cv2.countNonZero(blue_mask)
        result['blue_count'] = blue_count
        if blue_count > MIN_PIXEL_COUNT:
            result['blue'] = True
            result['detected'].append('blue')
            self._get_box_info(blue_mask, 'blue', result)

        return result

    # ------------------------------------------------------------------
    def _get_box_info(self, mask, color_name, result):
        """
        Find the largest *valid* contour and store bbox + distance.

        Shape filter
        ------------
        aspect_ratio > 3.0  → elongated (finger, pen, cable) → skip
        solidity     < 0.40 → irregular blob (noise)          → skip
        """
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        best = None  # (area, x, y, pw, ph)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_PIXEL_COUNT:
                continue
            x, y, pw, ph = cv2.boundingRect(cnt)
            if pw < 5 or ph < 5:
                continue

            aspect   = max(pw, ph) / (min(pw, ph) + 1e-9)
            solidity = area / (pw * ph + 1e-9)

            if aspect > 3.0 or solidity < 0.40:
                continue

            if best is None or area > best[0]:
                best = (area, x, y, pw, ph)

        if best is None:
            return

        _, x, y, pw, ph = best
        distance  = (BOX_WIDTH_MM * FOCAL_LENGTH) / pw

        result[f'{color_name}_box'] = {
            'x': int(x), 'y': int(y),
            'pixel_w':    int(pw),
            'pixel_h':    int(ph),
            'width_mm':   round((pw * distance) / FOCAL_LENGTH, 1),
            'height_mm':  round((ph * distance) / FOCAL_LENGTH, 1),
            'distance_m': round(distance / 1000, 3),
            'center_x':   int(x + pw // 2),
            'center_y':   int(y + ph // 2),
        }


# ===========================================================================
# APRILTAG DETECTOR — Uses AprilTagPoseEstimator from april_tag_pose.py
# ===========================================================================
# AprilTagPoseEstimator features:
#   - DICT_APRILTAG_36H11 (standard for robotics)
#   - Prediction keeps last pose 500ms after tag lost
#   - Median filtering for stability
#   - Optimized parameters for Raspberry Pi
# ===========================================================================

# Note: AprilTagPoseEstimator is imported from april_tag_pose.py
# Use VideoCaptureThread from april_tag_pose for threaded camera capture

# ===========================================================================
# CAMERA
# ===========================================================================

class Camera:
    """Multi-backend camera (V4L2 → DirectShow → auto)."""

    def __init__(self, index=0):
        for backend, name in [
            (cv2.CAP_V4L2,  "V4L2"),
            (cv2.CAP_DSHOW, "DirectShow"),
            (cv2.CAP_ANY,   "Auto"),
        ]:
            cap = cv2.VideoCapture(index, backend)
            if cap.isOpened():
                self.cap = cap
                print(f"[OK] Camera opened with {name}")
                break
        else:
            raise RuntimeError(f"Cannot open camera {index}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FPS, 15)

        w   = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h   = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        print(f"[OK] Resolution: {w}x{h} @ {fps} FPS")

    def read(self):
        return self.cap.read()

    def release(self):
        if self.cap:
            self.cap.release()


# ===========================================================================
# MASK DEBUG WINDOW
# ===========================================================================

class MaskWindow:
    """Small HSV mask previews for debugging (disabled by default)."""

    W, H = 120, 90

    def show(self, hsv_small):
        rm1 = cv2.resize(cv2.inRange(hsv_small, RED_L1, RED_U1),   (self.W, self.H))
        rm2 = cv2.resize(cv2.inRange(hsv_small, RED_L2, RED_U2),   (self.W, self.H))
        gm  = cv2.resize(cv2.inRange(hsv_small, GREEN_L, GREEN_U), (self.W, self.H))
        bm  = cv2.resize(cv2.inRange(hsv_small, BLUE_L,  BLUE_U),  (self.W, self.H))
        hl  = cv2.resize(cv2.inRange(hsv_small, HIGHLIGHT_L, HIGHLIGHT_U),
                         (self.W, self.H))

        red_m  = cv2.bitwise_or(rm1, rm2)
        blue_m = cv2.bitwise_and(bm, cv2.bitwise_not(hl))

        def colorise(m, bgr):
            img = np.zeros((self.H, self.W, 3), dtype=np.uint8)
            img[m > 0] = bgr
            return img

        cv2.imshow("Mask RED",   colorise(red_m,  (0, 0, 255)))
        cv2.imshow("Mask GREEN", colorise(gm,     (0, 255, 0)))
        cv2.imshow("Mask BLUE",  colorise(blue_m, (255, 80, 0)))


# ===========================================================================
# DRAWING HELPERS
# ===========================================================================

_COLOR_BGR = {'red': (0, 0, 255), 'green': (0, 200, 0), 'blue': (255, 80, 0)}


def draw_color_boxes(display, color_result):
    for name in ('red', 'green', 'blue'):
        if not color_result.get(name):
            continue
        box = color_result.get(f'{name}_box')
        if box is None:
            continue
        bgr          = _COLOR_BGR[name]
        x, y, pw, ph = box['x'], box['y'], box['pixel_w'], box['pixel_h']
        cv2.rectangle(display, (x, y), (x + pw, y + ph), bgr, 1)
        cv2.putText(display,
                    f"{box['width_mm']:.0f}x{box['height_mm']:.0f}mm",
                    (x, max(y - 3, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, bgr, 1)
        cv2.putText(display,
                    f"{box['distance_m']:.2f}m",
                    (x, y + ph + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, bgr, 1)
        cv2.circle(display, (box['center_x'], box['center_y']), 3, bgr, -1)


def draw_aruco_tags(display, tags):
    for tag in tags:
        corners = tag['corners']
        tx = int(np.mean(corners[:, 0]))
        ty = int(np.mean(corners[:, 1]))
        # Draw marker outline
        pts = corners.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(display, [pts], True, (0, 255, 255), 1)
        cv2.circle(display, (tx, ty), 3, (0, 255, 255), -1)
        dist_m = tag['dist_cm'] / 100.0
        cv2.putText(display,
                    f"ID:{tag['tag_id']} {dist_m:.2f}m",
                    (tx + 5, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)


def draw_status(display, detected, fps):
    if detected:
        label  = 'DET: ' + '+'.join(c.upper() for c in detected)
        lcolor = (0, 255, 0)
        lw     = 180
    else:
        label  = "NO DETECTION"
        lcolor = (128, 128, 128)
        lw     = 110
    cv2.rectangle(display, (0, 0), (lw, 18), (0, 0, 0), -1)
    cv2.putText(display, label, (3, 13),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, lcolor, 1)
    cv2.putText(display, f"FPS:{fps}", (PROCESS_WIDTH - 55, 13),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

from enum import Enum

class RobotState(Enum):
    IDLE = 1
    SEARCHING_BOX = 2          # looking for a coloured box
    GOING_TO_STATION = 3       # moving toward the selected station marker
    DROPPING_BOX = 4
    RETURNING_HOME = 5

# ===========================================================================
# MAIN
# ===========================================================================

def main():

    print("\n" + "=" * 56)
    print("  COLOR DETECTION v4 - AprilTag 36h11")
    print("=" * 56)
    print(f"  Process res  : {PROCESS_WIDTH}x{PROCESS_HEIGHT}")
    print(f"  Frame skip   : every {FRAME_SKIP} frames")
    print(f"  Min pixels   : {MIN_PIXEL_COUNT}")
    print(f"  AprilTag     : DICT_APRILTAG_36H11")
    print("=" * 56)
    print("  Q=Quit  S=Save  M=Masks  A=AprilTag toggle")
    print("=" * 56 + "\n")

    color_det  = ColorDetector()
    april_tag_estimator = AprilTagPoseEstimator(tag_size_cm=APRILTAG_SIZE_M * 100)
    mask_win   = MaskWindow()

    try:
        camera = Camera(CAMERA_INDEX)
    except RuntimeError as e:
        print(f"[ERROR] {e}")
        return

    show_masks  = False
    show_aruco  = True

    fps         = 0
    frame_count = 0
    last_fps_t  = time.time()
    color_result = None

    state = RobotState.SEARCHING_BOX
    target_station_id = None

    WIN = "Robot Vision v3"

    while True:
        ret, frame = camera.read()
        if not ret:
            time.sleep(0.01)
            continue

        # FPS counter
        frame_count += 1
        now = time.time()
        if now - last_fps_t >= 1.0:
            fps         = frame_count
            frame_count = 0
            last_fps_t  = now

        # ── Color detection ───────────────────────────────────────────
        new_result = color_det.detect(frame)
        if new_result is not None:
            color_result = new_result

        if color_result is None:
            cv2.waitKey(1)
            continue


        # ── Build display frame (resize ONCE, reuse for gray) ─────────
        display    = cv2.resize(frame, (PROCESS_WIDTH, PROCESS_HEIGHT))

        # ── AprilTag: convert already-resized display to gray ────────────
        #   resize → gray  (faster than  gray → resize)
        if show_aruco:
            gray_small = cv2.cvtColor(display, cv2.COLOR_BGR2GRAY)
            t_now = time.time()
            tags, _ = april_tag_estimator.detect(gray_small, t_now)
             # ========== STATE MACHINE LOGIC (paste the whole block here) ==========
            # ---- State machine logic ----
            if state == RobotState.SEARCHING_BOX:
                if color_result and color_result.get('detected'):
                    blue_n  = color_result.get('blue_count', 0)
                    green_n = color_result.get('green_count', 0)

                    # Choisit la couleur avec le PLUS de pixels (la plus sure)
                    if green_n > blue_n and green_n > MIN_PIXEL_COUNT:
                        target_station_id = STATION_A_ID
                        print(f"[STATE] Green box -> Station A (ID {target_station_id})")
                        state = RobotState.GOING_TO_STATION
                    elif blue_n > green_n and blue_n > MIN_PIXEL_COUNT:
                        target_station_id = STATION_B_ID
                        print(f"[STATE] Blue box -> Station B (ID {target_station_id})")
                        state = RobotState.GOING_TO_STATION

            elif state == RobotState.GOING_TO_STATION:
                # Look for the target station marker
                station_found = False
                for tag in tags:
                    if tag['tag_id'] == target_station_id:
                        distance_m = tag['dist_cm'] / 100.0  # convert cm to m
                        station_found = True
                        # Draw extra info
                        cv2.putText(display, f"Station distance: {distance_m*100:.1f}cm",
                                    (10, PROCESS_HEIGHT-40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,0), 1)
                        
                        if distance_m <= STOP_DISTANCE_M:
                            print(f"[STATE] Reached station at {distance_m*100:.1f}cm → dropping box")
                            state = RobotState.DROPPING_BOX
                            # Optional: send command to drop box (e.g., GPIO servo)
                        else:
                            # Here you would send movement commands to the robot
                            print(f"Moving toward station: {distance_m*100:.1f}cm left")
                        break
                if not station_found:
                    cv2.putText(display, f"Searching for station marker ID {target_station_id}",
                                (10, PROCESS_HEIGHT-20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)

            elif state == RobotState.DROPPING_BOX:
                # Simulate drop (wait 1 second, then go home)
                cv2.putText(display, "DROPPING BOX...", (PROCESS_WIDTH//2-50, PROCESS_HEIGHT//2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
                if not hasattr(state, 'drop_counter'):
                    state.drop_counter = 0
                state.drop_counter += 1
                if state.drop_counter > 30:  # ~2 seconds at 15 fps
                    del state.drop_counter
                    print("[STATE] Box dropped, returning home")
                    target_station_id = HOME_ID
                    state = RobotState.RETURNING_HOME

            elif state == RobotState.RETURNING_HOME:
                home_found = False
                for tag in tags:
                    if tag['tag_id'] == HOME_ID:
                        distance_m = tag['dist_cm'] / 100.0  # convert cm to m
                        cv2.putText(display, f"Home distance: {distance_m*100:.1f}cm",
                                    (10, PROCESS_HEIGHT-40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,0), 1)
                        if distance_m <= STOP_DISTANCE_M:
                            print("[STATE] Back home. Ready for next box.")
                            state = RobotState.SEARCHING_BOX
                            target_station_id = None
                        else:
                            print(f"Moving home: {distance_m*100:.1f}cm left")
                        home_found = True
                        break
                if not home_found:
                    cv2.putText(display, "Searching for Home marker...", (10, PROCESS_HEIGHT-20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
            
            # Draw all markers (original yellow outlines)
            draw_aruco_tags(display, tags)
        
        draw_color_boxes(display, color_result)
        draw_status(display, color_result.get('detected', []), fps)
        cv2.imshow(WIN, display)

        

        # ── Optional mask debug ───────────────────────────────────────
        if show_masks:
            hsv_small = cv2.cvtColor(display, cv2.COLOR_BGR2HSV)
            mask_win.show(hsv_small)

        # ── Key handling ──────────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord('s'):
            ts    = time.strftime("%Y%m%d_%H%M%S")
            det   = color_result.get('detected', [])
            cstr  = '+'.join(c.upper() for c in det) or 'NONE'
            fname = f"detection_{cstr}_{ts}.jpg"
            cv2.imwrite(fname, display)
            print(f"[SAVE] {fname}")

        elif key == ord('m'):
            show_masks = not show_masks
            if not show_masks:
                for n in ("Mask RED", "Mask GREEN", "Mask BLUE"):
                    cv2.destroyWindow(n)
            print(f"[INFO] Masks: {'ON' if show_masks else 'OFF'}")

        elif key in (ord('a'), ord('A')):
            show_aruco = not show_aruco
            print(f"[INFO] ArUco: {'ON' if show_aruco else 'OFF'}")

    
    camera.release()
    cv2.destroyAllWindows()
    print("\n[OK] Done")


if __name__ == '__main__':
    main()