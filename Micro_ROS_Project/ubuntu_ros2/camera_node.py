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
import sys

# Add project root to path to import color_detection_test
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)
from color_detection_test import ColorDetector, ArucoDetector, PROCESS_WIDTH, PROCESS_HEIGHT

# ─────────────────────────────────────────────────────────────────────────────
# APRILTAG CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────
APRILTAG_DICT = cv2.aruco.DICT_4X4_250
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

        # NEW: Use optimized ColorDetector and ArucoDetector from color_detection_test
        self.color_detector = ColorDetector()
        self.aruco_detector = ArucoDetector()

        # Publishers
        self.aruco_pub = self.create_publisher(PoseArray, '/aruco_detections', 10)
        self.color_pub = self.create_publisher(String, '/box_color', 10)
        self.box_info_pub = self.create_publisher(String, '/box_info', 10)

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

    def _detect_april_tags(self, gray_small):
        """Use new ArucoDetector from color_detection_test."""
        tags = self.aruco_detector.detect(gray_small)
        
        if not tags:
            return [], [], []
        
        # Convert to PoseArray format
        poses = []
        tag_ids = []
        corners = []
        
        for tag in tags:
            pose = Pose()
            # tvec is [x, y, z] in tag dict
            tvec = tag['tvec']
            pose.position.x = float(tvec[0])
            pose.position.y = float(tvec[1])
            pose.position.z = float(tvec[2])
            
            # Convert rotation vector to quaternion
            rvec = tag['rvec']
            rot_matrix, _ = cv2.Rodrigues(rvec)
            q = self._rotation_to_quaternion(rot_matrix)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            
            poses.append(pose)
            tag_ids.append(tag['id'])
            corners.append(tag['corners'])
        
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

    def _convert_color_result_to_box_info(self, color_result, tag_pose=None):
        """
        Convert new ColorDetector result to old box_info format for compatibility.
        
        ColorDetector returns: {'detected': ['red'], 'red': True, 'red_box': {...}, ...}
        box_info format: {'color': 'red', 'orientation': 'horizontal', 'distance': 0.5, ...}
        """
        # Get primary detected color
        detected = color_result.get('detected', [])
        color = detected[0] if detected else 'none'
        
        # Get box info for that color
        box = color_result.get(f'{color}_box') if color != 'none' else None
        
        if box is None:
            return {
                'color': 'none',
                'orientation': 'unknown',
                'distance': 0.0,
                'width_mm': 0.0,
                'height_mm': 0.0,
                'pixel_width': 0,
                'pixel_height': 0,
                'confidence': 0.5
            }
        
        # Calculate orientation from aspect ratio
        pixel_w = box.get('pixel_w', 0)
        pixel_h = box.get('pixel_h', 0)
        aspect = pixel_w / pixel_h if pixel_h > 0 else 1.0
        
        if aspect > 1.5:
            orientation = 'horizontal'
        elif aspect < 0.67:
            orientation = 'vertical'
        else:
            orientation = 'unknown'
        
        # Get distance from tag_pose if available (more accurate)
        distance = box.get('distance_m', 0.0)
        if tag_pose is not None:
            # Override with AprilTag distance if available
            tz = tag_pose.position.z  # distance from camera to tag
            distance = tz + 0.05  # 5cm offset
        
        # Calculate confidence
        confidence = 0.5
        if color != 'none':
            confidence += 0.2
        if pixel_w > 50:
            confidence += 0.15
        if orientation != 'unknown':
            confidence += 0.15
            
        return {
            'color': color,
            'orientation': orientation,
            'distance': round(distance, 3),
            'width_mm': box.get('width_mm', 0.0),
            'height_mm': box.get('height_mm', 0.0),
            'pixel_width': pixel_w,
            'pixel_height': pixel_h,
            'confidence': min(1.0, confidence)
        }

    def poll_camera(self):
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        self.latest_frame = frame

        # NEW: Resize frame for faster processing (320x240)
        # This matches the PROCESS_WIDTH/HEIGHT from color_detection_test
        frame_small = cv2.resize(frame, (PROCESS_WIDTH, PROCESS_HEIGHT))
        gray_small = cv2.cvtColor(frame_small, cv2.COLOR_BGR2GRAY)

        # Process AprilTag detection using new ArucoDetector
        poses, tag_ids, corners = self._detect_april_tags(gray_small)

        # Publish AprilTag poses
        if poses:
            msg = PoseArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            for pose in poses:
                msg.poses.append(pose)
            self.aruco_pub.publish(msg)

        # NEW: Use ColorDetector for box detection (optimized)
        tag_pose = poses[0] if poses else None
        color_result = self.color_detector.detect(frame_small)
        box_info = self._convert_color_result_to_box_info(color_result, tag_pose)

        # Publish box color (legacy)
        color_msg = String()
        color_msg.data = box_info['color']
        self.color_pub.publish(color_msg)

        # Publish comprehensive box info
        box_info_msg = String()
        box_info_msg.data = json.dumps(box_info)
        self.box_info_pub.publish(box_info_msg)

        # Debug visualization (draw on display frame - full size)
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