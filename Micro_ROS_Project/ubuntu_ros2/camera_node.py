#!/usr/bin/env python3
"""
 ===========================================================================
   CAMERA NODE - AprilTag Detection + Color Detection
 ===========================================================================

 ROS2 node that:
 1. Captures camera frames
 2. Detects AprilTag markers using OpenCV
 3. Estimates 6DOF pose of each marker
 4. Publishes marker poses to /aruco_detections
 5. Also runs color detection for box color

 PUBLISHERS:
  - /aruco_detections (geometry_msgs/msg/PoseArray) - detected markers
  - /box_color (std_msgs/msg/String) - "red", "green", or "none"

 SUBSCRIBERS:
  - /image_raw (sensor_msgs/msg/Image) - camera frames

 PARAMETERS:
  - camera_index: camera device index (default: 0)
  - tag_size: physical tag size in meters (default: 0.1)
  - camera_matrix: path to calibration JSON

 ===========================================================================
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import json
import os

# ─────────────────────────────────────────────────────────────────────────────
# APRILTAG CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────
APRILTAG_DICT = aruco.DICT_APRILTAG_36H11
APRILTAG_SIZE_M = 0.10  # 10cm tag

# Camera calibration (will be loaded from JSON or use defaults)
DEFAULT_CAM_MATRIX = np.array([
    [828.4, 0, 337.5],
    [0, 812.7, 213.6],
    [0, 0, 1]
], dtype=np.float32)

DEFAULT_DIST_COEFFS = np.array([
    [-1.44, 14.76, -0.006, 0.054, -37.11]
], dtype=np.float32)

# ─────────────────────────────────────────────────────────────────────────────
# BOX DETECTION CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────
# Known box physical dimensions (in meters) for size-based distance estimation
BOX_WIDTH = 0.10   # 10cm
BOX_HEIGHT = 0.08  # 8cm
BOX_DEPTH = 0.06   # 6cm

# Focal length from camera calibration (pixels)
# This is approximated from the camera matrix [0,0] element
FOCAL_LENGTH = 828.4

# ─────────────────────────────────────────────────────────────────────────────
# COLOR DETECTION CONFIG (HSV for red, green, and blue boxes)
# ─────────────────────────────────────────────────────────────────────────────
COLOR_RANGES = {
    'red': {
        'lower1': np.array([0, 100, 100]),
        'upper1': np.array([10, 255, 255]),
        'lower2': np.array([170, 100, 100]),
        'upper2': np.array([180, 255, 255])
    },
    'green': {
        'lower': np.array([40, 50, 50]),
        'upper': np.array([80, 255, 255])
    },
    'blue': {
        'lower': np.array([100, 50, 50]),
        'upper': np.array([130, 255, 255])
    }
}


# ─────────────────────────────────────────────────────────────────────────────
# BOX DETECTOR CLASS
# ─────────────────────────────────────────────────────────────────────────────
class BoxDetector:
    """Detects box orientation and estimates distance"""

    def __init__(self):
        # Known box dimensions (for size-based distance estimation)
        self.box_width = BOX_WIDTH
        self.box_height = BOX_HEIGHT
        self.box_depth = BOX_DEPTH
        self.focal_length = FOCAL_LENGTH

    def detect_orientation(self, frame):
        """
        Detect if box is horizontal or vertical using contour analysis.

        Returns:
            tuple: (orientation_str, distance_meters)
                orientation: 'horizontal', 'vertical', or 'unknown'
                distance: estimated distance in meters (0.0 if unknown)
        """
        return self._detect_orientation(frame)

    def _detect_orientation(self, frame):
        """
        Detect box orientation using edge detection and contour analysis.

        Steps:
        1. Convert to HSV and apply color mask (red/green/blue detected box)
        2. Find contours of the colored region
        3. Calculate bounding box of largest contour
        4. Compute aspect ratio (width/height)
        5. If aspect ratio > 1.5: horizontal (lying down)
           If aspect ratio < 0.67: vertical (standing up)
           Else: ambiguous or square
        """
        # Get center ROI where box is expected
        h, w = frame.shape[:2]
        roi = frame[int(h*0.35):int(h*0.75), int(w*0.25):int(w*0.75)]

        # Convert to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Combine masks for detected colors
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for color_name, ranges in COLOR_RANGES.items():
            if color_name == 'red':
                mask |= cv2.bitwise_or(
                    cv2.inRange(hsv, ranges['lower1'], ranges['upper1']),
                    cv2.inRange(hsv, ranges['lower2'], ranges['upper2'])
                )
            else:
                mask |= cv2.inRange(hsv, ranges['lower'], ranges['upper'])

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return 'unknown', 0.0

        # Find largest contour (should be the box)
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < 500:  # Too small
            return 'unknown', 0.0

        # Get bounding rectangle
        x, y, bw, bh = cv2.boundingRect(largest)

        # Aspect ratio determines orientation
        aspect_ratio = bw / bh if bh > 0 else 1.0

        # Determine orientation based on aspect ratio
        if aspect_ratio > 1.5:
            orientation = 'horizontal'
        elif aspect_ratio < 0.67:
            orientation = 'vertical'
        else:
            orientation = 'unknown'

        # Estimate distance based on box pixel size
        distance = self._estimate_distance_from_pixels(bw, self.box_width)

        return orientation, distance

    def _estimate_distance_from_pixels(self, pixel_size, actual_size):
        """
        Estimate distance using known object size and pixel projection.

        Uses similar triangles: distance = (real_size * focal_length) / pixel_size
        """
        if pixel_size > 0:
            distance = (actual_size * self.focal_length) / pixel_size
            return round(distance, 3)
        return 0.0

    def estimate_distance(self, frame, tag_pose=None):
        """
        Estimate distance to box.

        Uses:
        - AprilTag distance if available (most accurate)
        - Box pixel size as fallback
        - Known box dimensions
        """
        if tag_pose is not None:
            return self._estimate_distance_from_tag(tag_pose)

        # Fallback to pixel-based estimation
        _, distance = self._detect_orientation(frame)
        return distance

    def _estimate_distance_from_tag(self, tag_pose):
        """
        Estimate distance to box using AprilTag position.

        If AprilTag is near the box, use tag distance as box distance.
        """
        if tag_pose is None:
            return 0.0

        # Tag position from camera
        tz = tag_pose.position.z  # distance from camera to tag

        # Box is assumed to be near the tag (on same surface)
        # Add estimated offset
        box_offset = 0.05  # 5cm in front of tag
        distance = tz + box_offset

        return round(distance, 3)


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Declare parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('tag_size', 0.10)
        self.declare_parameter('calibration_file', '')

        self.camera_index = self.get_parameter('camera_index').value
        self.tag_size = self.get_parameter('tag_size').value
        cal_file = self.get_parameter('calibration_file').value

        # Load calibration
        self.cam_matrix = DEFAULT_CAM_MATRIX
        self.dist_coeffs = DEFAULT_DIST_COEFFS
        if cal_file and os.path.exists(cal_file):
            self._load_calibration(cal_file)

        self.get_logger().info(f'Camera Node started (tag_size={self.tag_size}m)')

        # ROS interfaces
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(APRILTAG_DICT)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Publishers
        self.aruco_pub = self.create_publisher(PoseArray, '/aruco_detections', 10)
        self.color_pub = self.create_publisher(String, '/box_color', 10)
        self.box_info_pub = self.create_publisher(String, '/box_info', 10)

        # Box detector for orientation and distance
        self.box_detector = BoxDetector()

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

        # Tag object points (for pose estimation)
        half = self.tag_size / 2.0
        self.tag_obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

        # Camera capture (for testing without ROS image transport)
        self.cap = None
        self._init_camera()

        # Timer for polling camera (30 Hz)
        self.timer = self.create_timer(0.033, self.poll_camera)

        self.latest_frame = None

    def _load_calibration(self, filepath):
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            self.cam_matrix = np.array(data['camera_matrix'], dtype=np.float32)
            self.dist_coeffs = np.array(data['dist_coeffs'], dtype=np.float32)
            self.get_logger().info(f'Calibration loaded from {filepath}')
        except Exception as e:
            self.get_logger().warn(f'Failed to load calibration: {e}')

    def _init_camera(self):
        self.cap = cv2.VideoCapture(self.camera_index)
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info(f'Camera {self.camera_index} opened')
        else:
            self.get_logger().error(f'Cannot open camera {self.camera_index}')

    def _detect_april_tags(self, gray):
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            return [], [], []

        # Estimate pose for each tag
        poses = []
        tag_ids = ids.flatten().tolist()

        for i, tag_id in enumerate(tag_ids):
            corners_single = corners[i]
            success, rvec, tvec = cv2.solvePnP(
                self.tag_obj_points, corners_single,
                self.cam_matrix, self.dist_coeffs)

            if success:
                pose = Pose()
                pose.position.x = float(tvec[0])
                pose.position.y = float(tvec[1])
                pose.position.z = float(tvec[2])

                # Convert rotation vector to quaternion
                rot_matrix, _ = cv2.Rodrigues(rvec)
                q = self._rotation_to_quaternion(rot_matrix)
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]

                poses.append(pose)

        return poses, tag_ids, corners

    def _rotation_to_quaternion(self, rot_matrix):
        """Convert rotation matrix to quaternion [x,y,z,w]"""
        trace = np.trace(rot_matrix)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (rot_matrix[2, 1] - rot_matrix[1, 2]) * s
            qy = (rot_matrix[0, 2] - rot_matrix[2, 0]) * s
            qz = (rot_matrix[1, 0] - rot_matrix[0, 1]) * s
        else:
            if rot_matrix[0, 0] > rot_matrix[1, 1] and rot_matrix[0, 0] > rot_matrix[2, 2]:
                s = 2.0 * np.sqrt(1.0 + rot_matrix[0, 0] - rot_matrix[1, 1] - rot_matrix[2, 2])
                qw = (rot_matrix[2, 1] - rot_matrix[1, 2]) / s
                qx = 0.25 * s
                qy = (rot_matrix[0, 1] + rot_matrix[1, 0]) / s
                qz = (rot_matrix[0, 2] + rot_matrix[2, 0]) / s
            elif rot_matrix[1, 1] > rot_matrix[2, 2]:
                s = 2.0 * np.sqrt(1.0 + rot_matrix[1, 1] - rot_matrix[0, 0] - rot_matrix[2, 2])
                qw = (rot_matrix[0, 2] - rot_matrix[2, 0]) / s
                qx = (rot_matrix[0, 1] + rot_matrix[1, 0]) / s
                qy = 0.25 * s
                qz = (rot_matrix[1, 2] + rot_matrix[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + rot_matrix[2, 2] - rot_matrix[0, 0] - rot_matrix[1, 1])
                qw = (rot_matrix[1, 0] - rot_matrix[0, 1]) / s
                qx = (rot_matrix[0, 2] + rot_matrix[2, 0]) / s
                qy = (rot_matrix[1, 2] + rot_matrix[2, 1]) / s
                qz = 0.25 * s

        norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
        return [qx/norm, qy/norm, qz/norm, qw/norm]

    def _detect_color(self, frame):
        """
        Detect if there's a red, green, or blue box in the center region.
        Also returns bounding box pixel dimensions for size calculation.

        Returns:
            tuple: (color_name, pixel_width, pixel_height, contour_area)
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Check center region (box is expected in front of robot)
        h, w = frame.shape[:2]
        center_roi = hsv[int(h*0.35):int(h*0.75), int(w*0.25):int(w*0.75)]

        best_color = 'none'
        best_pixel_width = 0
        best_pixel_height = 0
        best_area = 0

        for color_name, ranges in COLOR_RANGES.items():
            if color_name == 'red':
                mask = cv2.bitwise_or(
                    cv2.inRange(center_roi, ranges['lower1'], ranges['upper1']),
                    cv2.inRange(center_roi, ranges['lower2'], ranges['upper2'])
                )
            else:
                mask = cv2.inRange(center_roi, ranges['lower'], ranges['upper'])

            # Find contours to get bounding box
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Get largest contour
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)

                if area > 500 and area > best_area:
                    # Get bounding rectangle
                    x, y, bw, bh = cv2.boundingRect(largest)
                    best_color = color_name
                    best_pixel_width = bw
                    best_pixel_height = bh
                    best_area = area

        return best_color, best_pixel_width, best_pixel_height, best_area

    def _detect_box_info(self, frame, tag_pose=None):
        """
        Combined detection: color, orientation, distance, real-world dimensions.

        Returns dict with:
        - color: 'red', 'green', 'blue', or 'none'
        - orientation: 'horizontal', 'vertical', 'unknown'
        - distance: estimated distance in meters
        - width_mm: real-world width in millimeters
        - height_mm: real-world height in millimeters
        - pixel_width: detected pixel width of bounding box
        - pixel_height: detected pixel height of bounding box
        - confidence: detection confidence 0-1
        """
        # Get color AND pixel dimensions from detection
        color, pixel_width, pixel_height, area = self._detect_color(frame)

        # Calculate real-world dimensions using focal length and distance
        width_mm = 0.0
        height_mm = 0.0
        distance = 0.0

        if pixel_width > 0 and pixel_height > 0:
            # First estimate distance from pixel dimensions (average)
            avg_pixel = (pixel_width + pixel_height) / 2.0
            # Use box_width as reference real-world dimension (10cm)
            distance = (BOX_WIDTH * FOCAL_LENGTH) / avg_pixel if avg_pixel > 0 else 0.0

            # Now calculate real dimensions
            if distance > 0:
                width_mm = (pixel_width * distance) / FOCAL_LENGTH * 1000.0  # convert to mm
                height_mm = (pixel_height * distance) / FOCAL_LENGTH * 1000.0

                # Apply threshold for stable reading (min 10 pixels)
                if pixel_width < 10 or pixel_height < 10:
                    width_mm = 0.0
                    height_mm = 0.0

        # Detect orientation using BoxDetector
        orientation = 'unknown'
        if color != 'none':
            # Use aspect ratio from pixel dimensions
            if pixel_width > 0 and pixel_height > 0:
                aspect = pixel_width / pixel_height
                if aspect > 1.5:
                    orientation = 'horizontal'
                elif aspect < 0.67:
                    orientation = 'vertical'

        # If we have AprilTag pose, use it for more accurate distance
        if tag_pose is not None:
            distance = self.box_detector.estimate_distance(frame, tag_pose)

        # Compute confidence
        confidence = 0.5  # Base confidence
        if color != 'none':
            confidence += 0.2
        if pixel_width > 50:  # Large enough detection
            confidence += 0.15
        if orientation != 'unknown':
            confidence += 0.15

        return {
            'color': color,
            'orientation': orientation,
            'distance': round(distance, 3),
            'width_mm': round(width_mm, 1),
            'height_mm': round(height_mm, 1),
            'pixel_width': int(pixel_width),
            'pixel_height': int(pixel_height),
            'confidence': min(1.0, confidence)
        }

    def poll_camera(self):
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        self.latest_frame = frame

        # Process AprilTag detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        poses, tag_ids, corners = self._detect_april_tags(gray)

        # Publish AprilTag poses
        if poses:
            msg = PoseArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            for pose in poses:
                msg.poses.append(pose)
            self.aruco_pub.publish(msg)

        # Get box info (color, orientation, distance)
        tag_pose = poses[0] if poses else None
        box_info = self._detect_box_info(frame, tag_pose)

        # Publish box color (legacy)
        color_msg = String()
        color_msg.data = box_info['color']
        self.color_pub.publish(color_msg)

        # Publish comprehensive box info
        box_info_msg = String()
        box_info_msg.data = json.dumps(box_info)
        self.box_info_pub.publish(box_info_msg)

        # Debug visualization (draw on frame)
        if poses and corners is not None:
            aruco.drawDetectedMarkers(frame, corners, np.array(tag_ids))

        # Show frame (for debugging on monitor)
        cv2.imshow('Camera Node', frame)
        cv2.waitKey(1)

    def image_callback(self, msg):
        """ROS image callback (when using image_transport)"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()