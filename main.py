"""
=============================================================================
  AUTONOMOUS MOBILE ROBOT — Computer Vision Pipeline
  Author  : [prince gildas]
  Target  : PC (dev/test)  →  Raspberry Pi 4 (production)
  Python  : 3.10+
  Deps    : pip install opencv-python opencv-contrib-python numpy
=============================================================================

  HOW TO RUN (PC):
      python robot_vision.py

  HOW TO RUN (Raspberry Pi):
      python robot_vision.py --camera 0
      (same script, just change camera index if needed)

  WINDOW LAYOUT:
      [RAW STREAM]   — pure camera feed, no drawings
      [CV PIPELINE]  — all detections, bounding boxes, overlays, FPS

=============================================================================
  LINES MARKED WITH  # [ROBOT]  are for real robot integration
  They are COMMENTED OUT while running on PC.
  Uncomment them when deploying on Raspberry Pi.
=============================================================================
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import time
import argparse
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple, List

# ─────────────────────────────────────────────────────────────────────────────
# [ROBOT] — ROS2 imports (uncomment on Raspberry Pi)
# ─────────────────────────────────────────────────────────────────────────────
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from std_msgs.msg import String, Bool


# =============================================================================
#  CONFIGURATION — tweak these values without touching the logic
# =============================================================================
class Config:
    # ── Camera ────────────────────────────────────────────────────────────────
    CAMERA_INDEX        = 0          # 0 = default webcam / USB cam
    FRAME_WIDTH         = 640
    FRAME_HEIGHT        = 480
    TARGET_FPS          = 30

    # ── ArUco ─────────────────────────────────────────────────────────────────
    # Known marker positions in the environment (marker_id: (x_cm, y_cm))
    MARKER_POSITIONS    = {
        1: (0,   150),   # Manufacturing station
        2: (200, 50),    # Station A
        3: (0,   50),    # Station B
        4: (100, 0),     # Home
        5: (50,  150),   # Wall reference left
        6: (150, 150),   # Wall reference right
    }
    MARKER_SIZE_CM      = 10.0       # physical side length of each marker

    # ── Camera intrinsics (UNCALIBRATED defaults — calibrate for real robot!)
    # Replace with output of cv2.calibrateCamera() for accurate pose estimation
    FOCAL_LENGTH_PX     = 800        # approximate, update after calibration
    # Camera matrix (fx, fy, cx, cy)
    CAM_MATRIX          = np.array([
        [800,   0, 320],
        [  0, 800, 240],
        [  0,   0,   1]
    ], dtype=np.float32)
    DIST_COEFFS         = np.zeros((5, 1), dtype=np.float32)  # assume no distortion on PC

    # ── Color detection ───────────────────────────────────────────────────────
    # HSV ranges — tune these under your actual lighting conditions!
    RED_HSV_LOWER1      = np.array([0,   100, 100])  # red wraps around in HSV
    RED_HSV_UPPER1      = np.array([10,  255, 255])
    RED_HSV_LOWER2      = np.array([160, 100, 100])
    RED_HSV_UPPER2      = np.array([180, 255, 255])
    GREEN_HSV_LOWER     = np.array([40,  80,  80])
    GREEN_HSV_UPPER     = np.array([80,  255, 255])
    COLOR_MIN_AREA_PX   = 1500       # ignore tiny blobs (noise)

    # ── Visual servoing thresholds (for real robot navigation) ────────────────
    CENTER_TOLERANCE_PX = 30         # pixel offset considered "centered"
    APPROACH_DISTANCE_CM= 25         # stop at this distance from box
    PICKUP_DISTANCE_CM  = 12         # close gripper at this distance

    # ── FPS display ───────────────────────────────────────────────────────────
    FPS_SAMPLE_SIZE     = 30         # rolling average over N frames

    # ── Window names ──────────────────────────────────────────────────────────
    WIN_RAW             = "RAW STREAM"
    WIN_CV              = "CV PIPELINE"


# =============================================================================
#  DATA STRUCTURES
# =============================================================================
@dataclass
class MarkerDetection:
    marker_id   : int
    corners     : np.ndarray
    tvec        : np.ndarray    # translation vector (x, y, z in cm)
    rvec        : np.ndarray    # rotation vector
    distance_cm : float
    x_offset_px : float         # horizontal offset from frame center
    center_px   : Tuple[int, int]

@dataclass
class ColorDetection:
    color       : str           # "RED" or "GREEN" or "NONE"
    area_px     : int
    bbox        : Optional[Tuple[int, int, int, int]]  # x, y, w, h
    center_px   : Optional[Tuple[int, int]]
    confidence  : float         # 0.0 – 1.0

@dataclass
class RobotState:
    """Mirrors the task_manager state machine"""
    state           : str   = "IDLE"
    # Possible states:
    # IDLE → NAVIGATE_TO_MANUFACTURING → DETECT_COLOR → PICKUP
    # → NAVIGATE_TO_STORAGE → DROP → NAVIGATE_HOME → IDLE
    box_color       : str   = "NONE"
    target_marker   : int   = 4          # start at Home (marker 4)
    estimated_x_cm  : float = 0.0
    estimated_y_cm  : float = 0.0
    estimated_yaw   : float = 0.0


# =============================================================================
#  ARUCO DETECTOR
# =============================================================================
class ArucoDetector:
    """
    Detects ArUco markers and estimates robot pose.
    Uses DICT_4X4_50 (good balance between robustness and capacity).
    """

    def __init__(self):
        self.aruco_dict   = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.detector     = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # 3D object points of a single marker (used for pose estimation)
        half = Config.MARKER_SIZE_CM / 2.0
        self.obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

    def detect(self, frame: np.ndarray) -> List[MarkerDetection]:
        """
        Detect all markers in frame.
        Returns list of MarkerDetection sorted by distance (closest first).
        """
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        detections = []
        frame_cx   = frame.shape[1] // 2  # horizontal center of frame

        if ids is None:
            return detections

        for i, marker_id in enumerate(ids.flatten()):
            c = corners[i][0]  # shape (4, 2) — corner pixel coordinates

            # Pose estimation using solvePnP
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points, c,
                Config.CAM_MATRIX, Config.DIST_COEFFS,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not success:
                continue

            tvec = tvec.flatten()  # (x, y, z) in cm
            rvec = rvec.flatten()

            distance_cm = float(np.linalg.norm(tvec))  # Euclidean distance

            # Center of marker in pixel space
            center_px = (
                int(np.mean(c[:, 0])),
                int(np.mean(c[:, 1]))
            )
            x_offset_px = center_px[0] - frame_cx   # negative = left, positive = right

            detections.append(MarkerDetection(
                marker_id   = int(marker_id),
                corners     = c,
                tvec        = tvec,
                rvec        = rvec,
                distance_cm = distance_cm,
                x_offset_px = x_offset_px,
                center_px   = center_px,
            ))

        detections.sort(key=lambda d: d.distance_cm)
        return detections

    @staticmethod
    def estimate_robot_pose(detection: MarkerDetection) -> Tuple[float, float, float]:
        """
        Estimate robot's global position from a single marker detection.
        Returns (x_cm, y_cm, yaw_deg) in the global map frame.

        How it works:
            We know the marker's global position from Config.MARKER_POSITIONS.
            tvec gives us the robot→marker vector in camera frame.
            We invert it to get marker→robot, then add to the marker's global pos.

        NOTE: This is a simplified 2D projection.
              For production use, implement full SE3 transformation.
        """
        mid = detection.marker_id
        if mid not in Config.MARKER_POSITIONS:
            return (0.0, 0.0, 0.0)

        marker_gx, marker_gy = Config.MARKER_POSITIONS[mid]

        # tvec = (x_right, y_down, z_forward) in camera frame
        # For a forward-facing camera:
        #   robot is BEHIND the marker by z, offset laterally by x
        robot_x = marker_gx - detection.tvec[2]   # z = depth forward
        robot_y = marker_gy - detection.tvec[0]   # x = lateral offset

        # Yaw from rotation vector (simplified — around Y axis)
        yaw_deg = float(np.degrees(detection.rvec[1]))

        return (robot_x, robot_y, yaw_deg)


# =============================================================================
#  COLOR DETECTOR
# =============================================================================
class ColorDetector:
    """
    Detects RED or GREEN boxes using HSV masking.
    Returns bounding box and confidence (% of frame area covered).
    """

    def detect(self, frame: np.ndarray) -> ColorDetection:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red mask (two ranges because red wraps around H=0/180 in HSV)
        mask_r1  = cv2.inRange(hsv, Config.RED_HSV_LOWER1, Config.RED_HSV_UPPER1)
        mask_r2  = cv2.inRange(hsv, Config.RED_HSV_LOWER2, Config.RED_HSV_UPPER2)
        mask_red = cv2.bitwise_or(mask_r1, mask_r2)

        # Green mask
        mask_green = cv2.inRange(hsv, Config.GREEN_HSV_LOWER, Config.GREEN_HSV_UPPER)

        # Morphological cleanup — remove noise
        kernel     = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask_red   = cv2.morphologyEx(mask_red,   cv2.MORPH_OPEN,  kernel)
        mask_red   = cv2.morphologyEx(mask_red,   cv2.MORPH_CLOSE, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN,  kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

        red_area   = int(cv2.countNonZero(mask_red))
        green_area = int(cv2.countNonZero(mask_green))
        total_px   = frame.shape[0] * frame.shape[1]

        best_color = "NONE"
        best_area  = 0
        best_mask  = None

        if red_area > Config.COLOR_MIN_AREA_PX and red_area >= green_area:
            best_color = "RED"
            best_area  = red_area
            best_mask  = mask_red
        elif green_area > Config.COLOR_MIN_AREA_PX:
            best_color = "GREEN"
            best_area  = green_area
            best_mask  = mask_green

        if best_mask is None:
            return ColorDetection("NONE", 0, None, None, 0.0)

        # Largest contour → bounding box
        contours, _ = cv2.findContours(best_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return ColorDetection(best_color, best_area, None, None, 0.0)

        largest   = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)
        center    = (x + w // 2, y + h // 2)
        confidence = min(1.0, best_area / (total_px * 0.15))  # normalised 0–1

        return ColorDetection(
            color      = best_color,
            area_px    = best_area,
            bbox       = (x, y, w, h),
            center_px  = center,
            confidence = confidence,
        )


# =============================================================================
#  NAVIGATION COMMANDS (Visual Servoing)
# =============================================================================
class NavigationCommands:
    """
    Converts visual observations into velocity commands.
    On PC: just prints/returns the command dict.
    On Raspberry Pi: publishes to /cmd_vel ROS2 topic.
    """

    def __init__(self):
        # [ROBOT] self.pub = node.create_publisher(Twist, '/cmd_vel', 10)
        pass

    def compute_alignment_command(self, x_offset_px: float, distance_cm: float) -> dict:
        """
        Visual servoing toward a marker.
        Returns {'linear_x': float, 'angular_z': float}
        """
        cmd = {'linear_x': 0.0, 'angular_z': 0.0}

        # Step 1 — Rotate to center the marker horizontally
        if abs(x_offset_px) > Config.CENTER_TOLERANCE_PX:
            # Proportional control: larger offset → faster rotation
            angular = -0.003 * x_offset_px   # gain tuned for 640px width
            cmd['angular_z'] = float(np.clip(angular, -0.5, 0.5))

        # Step 2 — Once centered, approach
        elif distance_cm > Config.APPROACH_DISTANCE_CM:
            cmd['linear_x'] = 0.15   # m/s forward

        # Step 3 — Very close → slow approach for pickup
        elif distance_cm > Config.PICKUP_DISTANCE_CM:
            cmd['linear_x'] = 0.05

        # Step 4 — In pickup range → STOP
        else:
            cmd['linear_x']  = 0.0
            cmd['angular_z'] = 0.0

        return cmd

    def send_command(self, cmd: dict):
        """
        PC: print only.
        Raspberry Pi: publish Twist to /cmd_vel.
        """
        # ── PC debug output ────────────────────────────────────────────────
        # (remove or comment this print in production — too noisy)
        # print(f"  CMD → lin={cmd['linear_x']:.2f}  ang={cmd['angular_z']:.2f}")

        # ── [ROBOT] ROS2 publish ──────────────────────────────────────────
        # twist = Twist()
        # twist.linear.x  = cmd['linear_x']
        # twist.angular.z = cmd['angular_z']
        # self.pub.publish(twist)
        pass

    def stop(self):
        self.send_command({'linear_x': 0.0, 'angular_z': 0.0})


# =============================================================================
#  OVERLAY RENDERER — draws everything on the CV window
# =============================================================================
class Renderer:
    # Color palette (BGR)
    C_RED     = (0,   50,  220)
    C_GREEN   = (0,  200,   60)
    C_BLUE    = (220, 80,    0)
    C_YELLOW  = (0,  220,  220)
    C_WHITE   = (255, 255, 255)
    C_BLACK   = (0,     0,   0)
    C_ORANGE  = (0,  140,  255)
    C_CYAN    = (255, 200,   0)
    C_GRAY    = (160, 160,  160)

    def __init__(self, frame_w: int, frame_h: int):
        self.fw = frame_w
        self.fh = frame_h

    # ── ArUco overlays ────────────────────────────────────────────────────────
    def draw_markers(self, frame: np.ndarray, detections: List[MarkerDetection]):
        for d in detections:
            pts = d.corners.astype(int)

            # Draw border around marker
            cv2.polylines(frame, [pts.reshape(-1, 1, 2)], True, self.C_CYAN, 2)

            # Corner dots
            for pt in pts:
                cv2.circle(frame, tuple(pt), 5, self.C_YELLOW, -1)

            # Axes (X=red, Y=green, Z=blue) — shows orientation
            cv2.drawFrameAxes(
                frame,
                Config.CAM_MATRIX, Config.DIST_COEFFS,
                d.rvec.reshape(3, 1), d.tvec.reshape(3, 1),
                Config.MARKER_SIZE_CM * 0.5
            )

            # Info box above the marker
            cx, cy = d.center_px
            label_lines = [
                f"ID: {d.marker_id}",
                f"Dist: {d.distance_cm:.1f} cm",
                f"X off: {d.x_offset_px:+.0f} px",
                f"tvec: ({d.tvec[0]:.1f}, {d.tvec[1]:.1f}, {d.tvec[2]:.1f})",
            ]
            self._draw_info_box(frame, (cx - 60, cy - 90), label_lines, self.C_CYAN)

            # Center crosshair
            cv2.drawMarker(frame, d.center_px, self.C_YELLOW,
                           cv2.MARKER_CROSS, 15, 2)

    # ── Color detection overlays ──────────────────────────────────────────────
    def draw_color_detection(self, frame: np.ndarray, det: ColorDetection):
        if det.color == "NONE" or det.bbox is None:
            return

        color_bgr = self.C_RED if det.color == "RED" else self.C_GREEN
        x, y, w, h = det.bbox

        # Bounding box
        cv2.rectangle(frame, (x, y), (x + w, y + h), color_bgr, 3)

        # Corner accents (industrial look)
        corner_len = 15
        for (cx, cy, dx, dy) in [
            (x,     y,     1,  1),
            (x+w,   y,    -1,  1),
            (x,     y+h,   1, -1),
            (x+w,   y+h,  -1, -1),
        ]:
            cv2.line(frame, (cx, cy), (cx + dx*corner_len, cy), color_bgr, 3)
            cv2.line(frame, (cx, cy), (cx, cy + dy*corner_len), color_bgr, 3)

        # Confidence bar
        bar_w  = int(w * det.confidence)
        cv2.rectangle(frame, (x, y + h + 5), (x + w, y + h + 15), self.C_GRAY, -1)
        cv2.rectangle(frame, (x, y + h + 5), (x + bar_w, y + h + 15), color_bgr, -1)

        # Label
        label = f"{det.color}  {det.confidence*100:.0f}%  {det.area_px}px"
        cv2.putText(frame, label, (x, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

        # Center dot
        if det.center_px:
            cv2.circle(frame, det.center_px, 6, color_bgr, -1)

    # ── HUD — top-left panel ──────────────────────────────────────────────────
    def draw_hud(self, frame: np.ndarray, fps: float,
                 state: RobotState, n_markers: int, color_det: ColorDetection,
                 nav_cmd: dict):

        panel_lines = [
            ("─── ROBOT VISION HUD ───",     self.C_CYAN),
            (f"FPS       : {fps:5.1f}",       self.C_WHITE),
            (f"State     : {state.state}",    self.C_YELLOW),
            (f"Box color : {state.box_color}",
             self.C_RED if state.box_color == "RED" else
             self.C_GREEN if state.box_color == "GREEN" else self.C_GRAY),
            (f"Markers   : {n_markers} visible",  self.C_WHITE),
            (f"Robot pos : ({state.estimated_x_cm:.0f}, {state.estimated_y_cm:.0f}) cm",
             self.C_WHITE),
            (f"Yaw       : {state.estimated_yaw:.1f}°",   self.C_WHITE),
            ("─── NAV CMD ─────────────",     self.C_CYAN),
            (f"linear_x  : {nav_cmd['linear_x']:+.3f} m/s",  self.C_ORANGE),
            (f"angular_z : {nav_cmd['angular_z']:+.3f} r/s",  self.C_ORANGE),
        ]

        x0, y0 = 10, 10
        pad    = 6
        lh     = 20
        total_h = len(panel_lines) * lh + pad * 2
        total_w = 260

        # Background
        overlay = frame.copy()
        cv2.rectangle(overlay, (x0 - pad, y0 - pad),
                      (x0 + total_w, y0 + total_h), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)

        for i, (text, color) in enumerate(panel_lines):
            cv2.putText(frame, text,
                        (x0, y0 + i * lh + lh),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)

    # ── Center crosshair on frame ─────────────────────────────────────────────
    def draw_crosshair(self, frame: np.ndarray):
        cx, cy = self.fw // 2, self.fh // 2
        cv2.line(frame, (cx - 20, cy), (cx + 20, cy), self.C_WHITE, 1)
        cv2.line(frame, (cx, cy - 20), (cx, cy + 20), self.C_WHITE, 1)
        cv2.circle(frame, (cx, cy), 40, self.C_GRAY, 1)

    # ── Visual servoing guidance arrow ────────────────────────────────────────
    def draw_alignment_guide(self, frame: np.ndarray, x_offset_px: float):
        """Shows which direction robot needs to rotate to center on target."""
        cx, cy = self.fw // 2, self.fh - 50
        if abs(x_offset_px) < Config.CENTER_TOLERANCE_PX:
            # Centered — green circle
            cv2.circle(frame, (cx, cy), 18, self.C_GREEN, 3)
            cv2.putText(frame, "ALIGNED", (cx - 30, cy + 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.C_GREEN, 1)
        else:
            # Arrow pointing in correction direction
            arrow_len = int(min(abs(x_offset_px) * 0.5, 80))
            direction = -1 if x_offset_px > 0 else 1
            end_x     = cx + direction * arrow_len
            cv2.arrowedLine(frame, (cx, cy), (end_x, cy), self.C_ORANGE, 3, tipLength=0.3)
            cv2.putText(frame, f"OFFSET {x_offset_px:+.0f}px",
                        (cx - 50, cy + 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.C_ORANGE, 1)

    # ── Mini map (top-right) ──────────────────────────────────────────────────
    def draw_minimap(self, frame: np.ndarray, robot: RobotState,
                     detections: List[MarkerDetection]):
        """
        Draws a simple 2D top-down map of the environment.
        Known marker positions + estimated robot position.
        """
        MAP_SIZE  = 160
        MAP_SCALE = MAP_SIZE / 220.0   # 220cm is the map width
        mx0       = self.fw - MAP_SIZE - 10
        my0       = 10

        # Background
        overlay = frame.copy()
        cv2.rectangle(overlay, (mx0, my0), (mx0 + MAP_SIZE, my0 + MAP_SIZE),
                      (25, 25, 25), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        cv2.rectangle(frame, (mx0, my0), (mx0 + MAP_SIZE, my0 + MAP_SIZE),
                      self.C_CYAN, 1)
        cv2.putText(frame, "MAP", (mx0 + 5, my0 + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.C_CYAN, 1)

        visible_ids = {d.marker_id for d in detections}

        # Draw known markers
        for mid, (gx, gy) in Config.MARKER_POSITIONS.items():
            px = mx0 + int(gx * MAP_SCALE)
            py = my0 + MAP_SIZE - int(gy * MAP_SCALE)
            color = self.C_YELLOW if mid in visible_ids else self.C_GRAY
            cv2.rectangle(frame, (px - 4, py - 4), (px + 4, py + 4), color, -1)
            cv2.putText(frame, str(mid), (px + 6, py + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

        # Draw robot
        rx = mx0 + int(robot.estimated_x_cm * MAP_SCALE)
        ry = my0 + MAP_SIZE - int(robot.estimated_y_cm * MAP_SCALE)
        rx = int(np.clip(rx, mx0 + 4, mx0 + MAP_SIZE - 4))
        ry = int(np.clip(ry, my0 + 4, my0 + MAP_SIZE - 4))
        cv2.circle(frame, (rx, ry), 6, self.C_GREEN, -1)

        # Heading arrow
        yaw_rad = np.radians(robot.estimated_yaw)
        arrow_end = (
            rx + int(14 * np.cos(yaw_rad)),
            ry - int(14 * np.sin(yaw_rad))
        )
        cv2.arrowedLine(frame, (rx, ry), arrow_end, self.C_GREEN, 2, tipLength=0.4)

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _draw_info_box(self, frame, origin, lines, color):
        x, y   = origin
        pad    = 4
        lh     = 16
        box_w  = 200
        box_h  = len(lines) * lh + pad * 2

        overlay = frame.copy()
        cv2.rectangle(overlay, (x, y), (x + box_w, y + box_h), (10, 10, 10), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        for i, line in enumerate(lines):
            cv2.putText(frame, line, (x + pad, y + pad + (i + 1) * lh),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)


# =============================================================================
#  MAIN PIPELINE
# =============================================================================
class RobotVisionPipeline:

    def __init__(self):
        self.cap       = self._init_camera()
        self.aruco     = ArucoDetector()
        self.color     = ColorDetector()
        self.nav       = NavigationCommands()
        self.renderer  = Renderer(Config.FRAME_WIDTH, Config.FRAME_HEIGHT)
        self.state     = RobotState()
        self.fps_times = deque(maxlen=Config.FPS_SAMPLE_SIZE)

        # [ROBOT] rclpy.init()
        # [ROBOT] self.node = rclpy.create_node('robot_vision_node')
        # [ROBOT] self.nav  = NavigationCommands(self.node)

    def _init_camera(self) -> cv2.VideoCapture:
        cap = cv2.VideoCapture(Config.CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  Config.FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Config.FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS,          Config.TARGET_FPS)

        if not cap.isOpened():
            raise RuntimeError(
                f"Cannot open camera index {Config.CAMERA_INDEX}. "
                "Try --camera 1 or check USB connection."
            )
        print(f"[INFO] Camera opened: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}×"
              f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} "
              f"@ {int(cap.get(cv2.CAP_PROP_FPS))}fps")
        return cap

    def _compute_fps(self) -> float:
        now = time.perf_counter()
        self.fps_times.append(now)
        if len(self.fps_times) < 2:
            return 0.0
        elapsed = self.fps_times[-1] - self.fps_times[0]
        return (len(self.fps_times) - 1) / elapsed if elapsed > 0 else 0.0

    def _update_robot_state(self, detections: List[MarkerDetection],
                            color_det: ColorDetection):
        """
        Simplified state machine update.
        In production this is a full ROS2 node (task_manager).
        """
        # Update color if detected with high confidence
        if color_det.confidence > 0.5:
            self.state.box_color = color_det.color

        # Update pose from the closest visible marker
        if detections:
            x, y, yaw = ArucoDetector.estimate_robot_pose(detections[0])
            self.state.estimated_x_cm = x
            self.state.estimated_y_cm = y
            self.state.estimated_yaw  = yaw

        # ── [ROBOT] Full state machine transitions ────────────────────────
        # if self.state.state == "IDLE":
        #     self.state.state = "NAVIGATE_TO_MANUFACTURING"
        #     self.state.target_marker = 1
        # elif self.state.state == "NAVIGATE_TO_MANUFACTURING":
        #     if detections and detections[0].marker_id == 1:
        #         if detections[0].distance_cm < Config.APPROACH_DISTANCE_CM:
        #             self.state.state = "DETECT_COLOR"
        # elif self.state.state == "DETECT_COLOR":
        #     if color_det.confidence > 0.7:
        #         self.state.box_color = color_det.color
        #         self.state.state     = "PICKUP"
        # elif self.state.state == "PICKUP":
        #     if detections and detections[0].distance_cm < Config.PICKUP_DISTANCE_CM:
        #         self.nav.stop()
        #         # [ROBOT] trigger gripper servo via /gripper_cmd topic
        #         self.state.state = "NAVIGATE_TO_STORAGE"
        #         self.state.target_marker = 2 if self.state.box_color == "RED" else 3

    def run(self):
        print("[INFO] Starting vision pipeline. Press Q to quit.")
        cv2.namedWindow(Config.WIN_RAW, cv2.WINDOW_NORMAL)
        cv2.namedWindow(Config.WIN_CV,  cv2.WINDOW_NORMAL)
        cv2.resizeWindow(Config.WIN_RAW, Config.FRAME_WIDTH, Config.FRAME_HEIGHT)
        cv2.resizeWindow(Config.WIN_CV,  Config.FRAME_WIDTH, Config.FRAME_HEIGHT)

        # Position windows side by side
        cv2.moveWindow(Config.WIN_RAW, 50,  100)
        cv2.moveWindow(Config.WIN_CV,  Config.FRAME_WIDTH + 80, 100)

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("[WARN] Frame grab failed — retrying…")
                time.sleep(0.05)
                continue

            fps = self._compute_fps()

            # ── RAW window: unmodified frame ──────────────────────────────
            raw_frame = frame.copy()
            cv2.putText(raw_frame, "RAW STREAM", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
            cv2.imshow(Config.WIN_RAW, raw_frame)

            # ── CV PIPELINE ───────────────────────────────────────────────
            cv_frame = frame.copy()

            # 1. ArUco detection
            detections  = self.aruco.detect(cv_frame)

            # 2. Color detection
            color_det   = self.color.detect(cv_frame)

            # 3. Compute navigation command (toward closest visible marker)
            nav_cmd = {'linear_x': 0.0, 'angular_z': 0.0}
            if detections:
                closest = detections[0]
                nav_cmd = self.nav.compute_alignment_command(
                    closest.x_offset_px, closest.distance_cm
                )
                self.nav.send_command(nav_cmd)

            # 4. Update robot state
            self._update_robot_state(detections, color_det)

            # 5. Render all overlays onto cv_frame
            self.renderer.draw_crosshair(cv_frame)
            self.renderer.draw_markers(cv_frame, detections)
            self.renderer.draw_color_detection(cv_frame, color_det)
            self.renderer.draw_hud(cv_frame, fps, self.state,
                                   len(detections), color_det, nav_cmd)
            self.renderer.draw_minimap(cv_frame, self.state, detections)

            if detections:
                self.renderer.draw_alignment_guide(cv_frame, detections[0].x_offset_px)

            # Window title with live stats
            cv2.setWindowTitle(Config.WIN_CV,
                f"CV PIPELINE  |  FPS: {fps:.1f}  |  "
                f"Markers: {len(detections)}  |  "
                f"Color: {color_det.color}  |  "
                f"State: {self.state.state}")

            cv2.imshow(Config.WIN_CV, cv_frame)

            # ── Key handling ──────────────────────────────────────────────
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:   # Q or ESC to quit
                break
            elif key == ord('s'):               # S → save a screenshot
                fname = f"capture_{int(time.time())}.png"
                cv2.imwrite(fname, cv_frame)
                print(f"[INFO] Saved {fname}")
            elif key == ord('r'):               # R → reset robot state
                self.state = RobotState()
                print("[INFO] Robot state reset")

        self.cleanup()

    def cleanup(self):
        print("[INFO] Cleaning up…")
        self.cap.release()
        cv2.destroyAllWindows()
        # [ROBOT] rclpy.shutdown()


# =============================================================================
#  ENTRY POINT
# =============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Robot Vision Pipeline")
    parser.add_argument("--camera", type=int, default=Config.CAMERA_INDEX,
                        help="Camera device index (default: 0)")
    args = parser.parse_args()
    Config.CAMERA_INDEX = args.camera

    pipeline = RobotVisionPipeline()
    pipeline.run()