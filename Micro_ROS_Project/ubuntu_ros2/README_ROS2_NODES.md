# ROS2 Nodes Documentation
## Micro-ROS Robot - Raspberry Pi Software

**Version:** 1.0.0
**Date:** May 2026

---

## Table of Contents

1. [Overview](#1-overview)
2. [Node Architecture](#2-node-architecture)
3. [camera_node](#3-camera_node)
4. [localization_node](#4-localization_node)
5. [navigation_node](#5-navigation_node)
6. [task_manager_node](#6-task_manager_node)
7. [Launch File](#7-launch-file)
8. [Topic Reference](#8-topic-reference)
9. [Parameter Reference](#9-parameter-reference)

---

## 1. Overview

The ROS2 layer on the Raspberry Pi handles all high-level computation:

- AprilTag detection and pose estimation
- Color detection for warehouse boxes
- Robot pose estimation using sensor fusion
- Navigation with PID waypoint following
- Mission state machine management

All nodes are Python-based for rapid development and testing.

### Node Summary

| Node | Purpose | Input Topics | Output Topics |
|------|---------|--------------|---------------|
| `camera_node` | AprilTag + color detection | (camera hardware) | `/aruco_detections`, `/box_color` |
| `localization_node` | Pose estimation | `/aruco_detections`, `/imu_data` | `/robot_pose` |
| `navigation_node` | PID waypoint following | `/robot_pose`, `/waypoint` | `/cmd_vel` |
| `task_manager_node` | Mission state machine | `/box_color`, `/navigation_status` | `/waypoint`, `/gripper_cmd` |

### System Requirements

| Requirement | Version |
|-------------|---------|
| OS | Ubuntu 22.04 (Jammy) |
| ROS2 | Humble Hawksbill |
| Python | 3.8+ |
| OpenCV | 4.x |
| NumPy | 1.x |

---

## 2. Node Architecture

### 2.1 Communication Graph

```
                    ┌─────────────────────────────────────┐
                    │           CAMERA (USB)               │
                    │         (OpenCV VideoCapture)        │
                    └──────────────────┬──────────────────┘
                                       │
                                       ▼
                    ┌─────────────────────────────────────┐
                    │           camera_node                 │
                    │  • AprilTag detection (aruco)        │
                    │  • 6DOF pose estimation (solvePnP)    │
                    │  • HSV color detection               │
                    │                                     │
                    │  PUBLISHES:                         │
                    │    → /aruco_detections (PoseArray)   │
                    │    → /box_color (String)             │
                    └──────────────────┬───────────────────┘
                                       │
                        ┌──────────────┴──────────────┐
                        │                               │
                        ▼                               ▼
          ┌─────────────────────────┐    ┌─────────────────────────┐
          │   localization_node     │    │    task_manager_node    │
          │                         │    │                         │
          │ SUBSCRIBES:             │    │ SUBSCRIBES:             │
          │   ← /aruco_detections   │    │   ← /box_color          │
          │   ← /imu_data           │    │   ← /navigation_status  │
          │                         │    │                         │
          │ PUBLISHES:              │    │ PUBLISHES:              │
          │   → /robot_pose         │    │   → /waypoint            │
          └───────────┬────────────┘    │   → /gripper_cmd        │
                      │                  │   → /task_status         │
                      │                  │                         │
                      │                  │ SERVICES:               │
                      │                  │   ← /start_task          │
                      ▼                  │   ← /cancel_task         │
          ┌─────────────────────────┐    └────────────┬────────────┘
          │    navigation_node      │                 │
          │                         │                 │
          │ SUBSCRIBES:             │                 │
          │   ← /robot_pose         │                 │
          │   ← /waypoint           │                 │
          │                         │                 │
          │ PUBLISHES:              │                 │
          │   → /cmd_vel             │─────────────────┘
          │   → /navigation_status  │
          └───────────┬─────────────┘
                      │
                      ▼
          ┌─────────────────────────────────────────┐
          │           micro_ros_agent               │
          │  (UDP port 8888)                         │
          │                                         │
          │  BRIDGES TO:                            │
          │    → /imu_data (from ESP32)             │
          │    → /ultrasonic_data (from ESP32)      │
          │    ← /cmd_vel (to ESP32)                │
          │    ← /gripper_cmd (to ESP32)           │
          └────────────────────┬────────────────────┘
                               │
                               │ UDP
                               ▼
                    ┌──────────────────────┐
                    │        ESP32         │
                    │    (micro-ROS)       │
                    └──────────────────────┘
```

### 2.2 Timing Diagram

```
CAMERA (30Hz)          IMU (50Hz)           NAVIGATION (20Hz)
    │                       │                      │
    ▼                       ▼                      ▼
┌───────┐              ┌───────┐            ┌───────┐
│Detect │              │Read   │            │PID    │
│Tags   │              │IMU    │            │Control│
└───┬───┘              └───┬───┘            └───┬───┘
    │                      │                    │
    ▼                      │                    ▼
/aruco_detect               │              /cmd_vel
    │                      │                    │
    │    ┌─────────────────┴─────────────────┐  │
    ▼    ▼                                     ▼
┌───────────┐                            ┌───────────┐
│localization│                            │   ESP32    │
│  Update   │                            │   Motor   │
│  Pose     │                            │   Control │
└─────┬─────┘                            └───────────┘
      │
      ▼
/robot_pose
      │
      ▼
(navigation subscribes)
```

---

## 3. camera_node

**File:** `camera_node.py`
**Package:** `micro_ros_robot`
**Purpose:** Detect AprilTag markers and box colors from camera feed

### 3.1 Functionality

The camera_node performs three main functions:

1. **AprilTag Detection**
   - Uses OpenCV's aruco module with DICT_APRILTAG_36H11
   - Detects multiple tags simultaneously
   - Estimates 6DOF pose for each tag using solvePnP

2. **6DOF Pose Estimation**
   - Uses camera calibration parameters
   - Converts rotation vectors to quaternions
   - Publishes pose array with all detected tags

3. **Color Detection**
   - HSV color space for robust detection
   - Detects red and green boxes in center ROI
   - Publishes color as string

### 3.2 ROS Interface

**Publishers:**
| Topic | Type | Description |
|-------|------|-------------|
| `/aruco_detections` | geometry_msgs/PoseArray | All detected AprilTag poses |
| `/box_color` | std_msgs/String | "red", "green", or "none" |

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_index` | int | 0 | Video device index |
| `tag_size` | float | 0.10 | Physical tag size in meters |
| `calibration_file` | string | "" | Path to `data/camera_calibration/camera_calibration.json` |

### 3.3 AprilTag Detection Algorithm

```python
def _detect_april_tags(self, gray):
    # 1. Create detector with predefined dictionary
    detector = aruco.ArucoDetector(
        aruco.getPredefinedDictionary(APRILTAG_DICT),
        aruco.DetectorParameters()
    )

    # 2. Detect markers in grayscale image
    corners, ids, rejected = detector.detectMarkers(gray)

    # 3. For each detected tag, estimate pose
    poses = []
    for i, tag_id in enumerate(ids):
        # Solve PnP: find rotation and translation
        success, rvec, tvec = cv2.solvePnP(
            self.tag_obj_points,   # 3D tag corners
            corners[i],             # 2D image points
            self.cam_matrix,       # Camera intrinsics
            self.dist_coeffs       # Distortion
        )

        # Convert rotation vector to quaternion
        pose = self._rotation_to_quaternion(
            cv2.Rodrigues(rvec)[0]
        )

        # Store pose message
        poses.append(pose)

    return poses, ids.flatten().tolist()
```

### 3.4 Color Detection Algorithm

```python
def _detect_color(self, frame):
    # 1. Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 2. Define center region of interest (box expected here)
    h, w = frame.shape[:2]
    center_roi = hsv[int(h*0.4):int(h*0.7), int(w*0.3):int(w*0.7)]

    # 3. Check each color range
    for color_name, ranges in COLOR_RANGES.items():
        mask = cv2.inRange(center_roi, ranges['lower'], ranges['upper'])
        if cv2.countNonZero(mask) > 500:
            return color_name

    return 'none'

# HSV Color Ranges
COLOR_RANGES = {
    'red': {
        'lower': np.array([0, 100, 100]),
        'upper': np.array([10, 255, 255])
    },
    'green': {
        'lower': np.array([40, 50, 50]),
        'upper': np.array([80, 255, 255])
    }
}
```

### 3.5 Camera Calibration

The node uses camera calibration data for accurate pose estimation:

```python
# Default calibration (fallback if no JSON)
DEFAULT_CAM_MATRIX = np.array([
    [828.4, 0, 337.5],
    [0, 812.7, 213.6],
    [0, 0, 1]
], dtype=np.float32)

DEFAULT_DIST_COEFFS = np.array([
    [-1.44, 14.76, -0.006, 0.054, -37.11]
], dtype=np.float32)
```

To use custom calibration:
```bash
ros2 run micro_ros_robot camera_node --ros-args -p calibration_file:=/path/to/calibration.json
```

### 3.6 AprilTag Object Points

The 3D coordinates of tag corners (for pose estimation):

```python
# Tag is a square in XY plane, Z=0
# Half size = tag_size / 2
half = self.tag_size / 2.0
self.tag_obj_points = np.array([
    [-half,  half, 0],  # Top-left
    [ half,  half, 0],  # Top-right
    [ half, -half, 0],  # Bottom-right
    [-half, -half, 0]   # Bottom-left
], dtype=np.float32)
```

---

## 4. localization_node

**File:** `localization_node.py`
**Package:** `micro_ros_robot`
**Purpose:** Estimate robot pose from AprilTag detections and IMU

### 4.1 Functionality

The localization_node fuses multiple sensors to estimate robot position:

1. **AprilTag Pose Tracking**
   - Uses detected tag positions to triangulate robot location
   - Each detection provides relative position to a known marker

2. **IMU Orientation Correction**
   - Uses gyroscope for yaw angle
   - Fills gaps between visual detections

3. **Pose Smoothing**
   - Low-pass filter to reduce noise
   - Handles detection failures gracefully

### 4.2 ROS Interface

**Subscribers:**
| Topic | Type | Description |
|-------|------|-------------|
| `/aruco_detections` | geometry_msgs/PoseArray | Detected tag poses |
| `/imu_data` | geometry_msgs/Accel | IMU yaw and acceleration |
| `/cmd_result` | std_msgs/String | Command feedback (debug) |

**Publishers:**
| Topic | Type | Description |
|-------|------|-------------|
| `/robot_pose` | geometry_msgs/Pose | Estimated robot (x, y, theta) |

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `marker_map` | dict | {0:[0,0,0], 1:[1.5,0,0], ...} | Tag ID → world position |
| `robot_frame` | string | "map" | Coordinate frame ID |

### 4.3 Pose Estimation Algorithm

```python
def aruco_callback(self, msg: PoseArray):
    if len(msg.poses) == 0:
        return

    # For each detected tag:
    for i, tag_pose in enumerate(msg.poses):
        # 1. Extract translation from camera to tag
        tx = tag_pose.position.x
        ty = tag_pose.position.y
        tz = tag_pose.position.z

        # 2. Extract rotation (quaternion to matrix)
        qx, qy, qz, qw = tag_pose.orientation
        rot_matrix = self._quaternion_to_rotation_matrix(qx, qy, qz, qw)

        # 3. Transform to world frame
        # camera_to_tag vector rotated by tag orientation
        world_to_tag = rot_matrix @ np.array([tx, ty, tz])

        # 4. Compute robot position
        # Robot is "behind" the tag (camera is at front of robot)
        tag_dist = np.linalg.norm([tx, ty, tz])
        if tag_dist < 0.01:
            continue

        # Direction from tag to robot (opposite of camera-to-tag)
        direction = -world_to_tag / tag_dist

        # Robot world position = marker + direction * distance
        robot_pos = marker_world + direction * tag_dist

    # 5. Update filter
    self.robot_x = robot_pos[0]
    self.robot_y = robot_pos[1]
```

### 4.4 Marker Map Configuration

The marker map defines the world position of each AprilTag:

```yaml
marker_map:
  '0': [0.0, 0.0, 0.0]     # Home position
  '1': [1.5, 0.0, 0.0]     # Manufacturing station
  '2': [0.0, 1.5, 0.0]    # Storage A
  '3': [1.5, 1.5, 0.0]    # Storage B
```

These coordinates are in meters relative to the world origin.

### 4.5 IMU Integration

```python
def imu_callback(self, msg):
    # Yaw from gyroscope integration
    # linear.x contains yaw in degrees
    self.robot_yaw = math.radians(msg.linear.x)
```

The IMU provides:
- Yaw angle (from gyro integration)
- Angular velocity (for rate estimation)
- Linear accelerations (for tilt detection - future)

### 4.6 Pose Publication

```python
def publish_pose(self):
    pose_msg = Pose()
    pose_msg.position.x = self.robot_x
    pose_msg.position.y = self.robot_y
    pose_msg.position.z = 0.0

    # Convert yaw to quaternion
    q = self._yaw_to_quaternion(self.robot_yaw)
    pose_msg.orientation.x = q[0]
    pose_msg.orientation.y = q[1]
    pose_msg.orientation.z = q[2]
    pose_msg.orientation.w = q[3]

    self.pose_pub.publish(pose_msg)
```

---

## 5. navigation_node

**File:** `navigation_node.py`
**Package:** `micro_ros_robot`
**Purpose:** Navigate robot to waypoints using PID control

### 5.1 Functionality

The navigation_node implements a PID-based waypoint following controller:

1. **Waypoint Tracking**
   - Receives target waypoints from task manager
   - Computes distance and angle to waypoint

2. **PID Control**
   - Separate PIDs for linear and angular velocity
   - Proportional, Integral, Derivative terms

3. **Obstacle Awareness**
   - Angle threshold prevents moving when not facing waypoint
   - (Future: incorporate ultrasonic data for obstacle avoidance)

### 5.2 ROS Interface

**Subscribers:**
| Topic | Type | Description |
|-------|------|-------------|
| `/robot_pose` | geometry_msgs/Pose | Current robot position |
| `/waypoint` | geometry_msgs/Point | Target waypoint (x, y, theta) |

**Publishers:**
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity command to ESP32 |
| `/navigation_status` | geometry_msgs/Point | Status (x, y, z=reached_flag) |

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_linear_speed` | float | 0.20 | Max forward speed (m/s) |
| `max_angular_speed` | float | 1.5 | Max turn rate (rad/s) |
| `pid_linear_kp` | float | 2.0 | Linear Kp |
| `pid_linear_ki` | float | 0.1 | Linear Ki |
| `pid_linear_kd` | float | 0.5 | Linear Kd |
| `pid_angular_kp` | float | 3.0 | Angular Kp |
| `pid_angular_ki` | float | 0.2 | Angular Ki |
| `pid_angular_kd` | float | 1.0 | Angular Kd |
| `waypoint_threshold` | float | 0.15 | Distance to consider reached (m) |
| `angle_threshold` | float | 0.2 | Angle threshold for orientation (rad) |

### 5.3 PID Controller Implementation

```python
class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, current, target):
        error = target - current

        # Proportional term
        p = self.kp * error

        # Integral term (with anti-windup)
        self.integral += error * self.dt
        self.integral = max(-50, min(50, self.integral))
        i = self.ki * self.integral

        # Derivative term
        d = self.kd * (error - self.prev_error) / self.dt
        self.prev_error = error

        # Total output
        output = p + i + d

        # Clamp to limits
        return max(-self.output_limit, min(self.output_limit, output))
```

### 5.4 Control Algorithm

```python
def control_loop(self):
    # 1. Compute distance to waypoint
    dx = self.target_x - self.current_x
    dy = self.target_y - self.current_y
    dist = math.sqrt(dx*dx + dy*dy)

    # 2. Compute angle to waypoint
    angle_to_waypoint = math.atan2(dy, dx)

    # 3. Compute relative angle (robot vs waypoint)
    relative_angle = self._normalize_angle(angle_to_waypoint - self.current_yaw)

    # 4. Check if waypoint reached
    if dist < self.waypoint_threshold:
        self.waypoint_active = False
        self._publish_vel(0.0, 0.0)
        return

    # 5. PID control for angular velocity (turn first)
    angular_vel = self.pid_angular.compute(0, relative_angle)

    # 6. Only move forward if facing waypoint (within 30°)
    angle_ok = abs(relative_angle) < math.radians(30)
    if angle_ok:
        linear_vel = self.pid_linear.compute(0, dist)
    else:
        linear_vel = 0.0

    # 7. Publish velocity command
    self._publish_vel(linear_vel, angular_vel)
```

### 5.5 Velocity Mapping

The navigation_node outputs standard Twist messages:

```python
def _publish_vel(self, linear_x, angular_z):
    msg = Twist()
    msg.linear.x = max(-self.max_lin, min(self.max_lin, linear_x))
    msg.angular.z = max(-self.max_ang, min(self.max_ang, angular_z))
    self.cmd_vel_pub.publish(msg)
```

These are then received by the ESP32 which converts to motor PWM.

### 5.6 PID Tuning Guide

**Linear Velocity PID (forward/backward):**
- **Kp too high:** Robot overshoots and oscillates
- **Kp too low:** Robot too slow to reach waypoint
- **Ki too high:** Oscillations increase (integral windup)
- **Kd too high:** Jerky movements

**Angular Velocity PID (turning):**
- **Kp too high:** Robot spins wildly
- **Kp too low:** Robot takes too long to turn
- **Ki too high:** Constant oscillation
- **Kd too high:** Response too damped

**Recommended starting values:**
```yaml
pid_linear_kp: 2.0
pid_linear_ki: 0.1
pid_linear_kd: 0.5

pid_angular_kp: 3.0
pid_angular_ki: 0.2
pid_angular_kd: 1.0
```

---

## 6. task_manager_node

**File:** `task_manager_node.py`
**Package:** `micro_ros_robot`
**Purpose:** Execute the complete warehouse mission as a state machine

### 6.1 Functionality

The task_manager_node implements a state machine that controls the robot's mission:

1. **State Management**
   - 8 states: IDLE, GO_TO_MFG, DETECT_COLOR, PICK_BOX, GO_TO_STORAGE, DEPOSIT_BOX, RETURN_HOME, TASK_COMPLETE

2. **Mission Control**
   - Starts mission on `/start_task` service call
   - Navigates through waypoints
   - Controls gripper based on color detection

3. **Event Handling**
   - Waypoint reached events from navigation
   - Color detection events from camera
   - Cancel requests via `/cancel_task`

### 6.2 ROS Interface

**Subscribers:**
| Topic | Type | Description |
|-------|------|-------------|
| `/box_color` | std_msgs/String | Detected color ("red"/"green"/"none") |
| `/navigation_status` | geometry_msgs/Point | Waypoint reached (z=1.0) |
| `/robot_pose` | geometry_msgs/Pose | Current position (logging) |

**Publishers:**
| Topic | Type | Description |
|-------|------|-------------|
| `/waypoint` | geometry_msgs/Point | Target position for navigation |
| `/gripper_cmd` | std_msgs/String | "open" or "close" |
| `/task_status` | std_msgs/String | Current state info |

**Services:**
| Service | Type | Description |
|---------|------|-------------|
| `/start_task` | std_srvs/Empty | Start new mission |
| `/cancel_task` | std_srvs/Empty | Cancel and return home |

### 6.3 State Machine Definition

```python
class TaskState:
    IDLE = "IDLE"
    GO_TO_MFG = "GO_TO_MFG"
    DETECT_COLOR = "DETECT_COLOR"
    PICK_BOX = "PICK_BOX"
    GO_TO_STORAGE = "GO_TO_STORAGE"
    DEPOSIT_BOX = "DEPOSIT_BOX"
    RETURN_HOME = "RETURN_HOME"
    TASK_COMPLETE = "TASK_COMPLETE"
```

### 6.4 Mission Sequence

```
┌─────────────────────────────────────────────────────────────────────┐
│                         MISSION SEQUENCE                            │
└─────────────────────────────────────────────────────────────────────┘

[START] → /start_task service called

    │
    ▼
┌─────────┐   waypoint reached    ┌─────────────┐
│GO_TO_MFG│─────────────────────▶│DETECT_COLOR │
└─────────┘                      └──────┬──────┘
                                        │
                           color detected │
                                        ▼
                               ┌─────────────┐
                               │  PICK_BOX   │───► close gripper
                               └──────┬──────┘
                                      │
                               gripper closed │
                                      ▼
                    ┌────────────────────────────────────┐
                    │           GO_TO_STORAGE             │
                    │                                    │
                    │  if color == "red"  → Storage A     │
                    │  if color == "green" → Storage B     │
                    └─────────────────┬──────────────────┘
                                      │
                           waypoint reached │
                                      ▼
                               ┌─────────────┐
                               │ DEPOSIT_BOX │───► open gripper
                               └──────┬──────┘
                                      │
                                box deposited │
                                      ▼
                               ┌─────────────┐
                               │ RETURN_HOME │
                               └──────┬──────┘
                                      │
                              home reached │
                                      ▼
                               ┌─────────────┐
                               │TASK_COMPLETE│
                               └──────┬──────┘
                                      │
                              auto-transition │
                                      ▼
                               ┌─────────────┐
                               │    IDLE     │◄────────[END]
                               └─────────────┘
```

### 6.5 Waypoint Definitions

```python
# Station positions
home_pos = [0.0, 0.0, 0.0]
mfg_pos = [1.5, 0.0, 0.0]
storage_a = [0.0, 1.5, 0.0]  # Red box
storage_b = [1.5, 1.5, 0.0]  # Green box
```

### 6.6 Service Callbacks

```python
def start_task_callback(self, request, response):
    if self.task_active:
        return response  # Already running

    self.get_logger().info('START TASK - Going to Manufacturing')
    self.task_active = True
    self.current_state = TaskState.GO_TO_MFG
    self.detected_color = 'none'
    self._send_waypoint(self.mfg_pos[0], self.mfg_pos[1], self.mfg_pos[2])
    return response

def cancel_task_callback(self, request, response):
    self.get_logger().info('CANCEL TASK')
    self.task_active = False
    self.current_state = TaskState.IDLE
    self._send_waypoint(self.home_pos[0], self.home_pos[1], self.home_pos[2])
    return response
```

### 6.7 Event-Driven Transitions

```python
def _on_waypoint_reached(self):
    if self.current_state == TaskState.GO_TO_MFG:
        self.current_state = TaskState.DETECT_COLOR
        self._wait_and_transition(TaskState.PICK_BOX, 3.0)

    elif self.current_state == TaskState.GO_TO_STORAGE:
        self.current_state = TaskState.DEPOSIT_BOX
        self._gripper_open()  # Release box
        self._wait_and_transition(TaskState.RETURN_HOME, 2.0)

    elif self.current_state == TaskState.RETURN_HOME:
        self.current_state = TaskState.TASK_COMPLETE
        self.task_active = False

def color_callback(self, msg):
    color = msg.data.lower()
    if color in ['red', 'green']:
        self.detected_color = color
        self.get_logger().info(f'Box color detected: {color.upper()}')
```

---

## 7. Launch File

**File:** `robot_bringup.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera Node
        Node(
            package='micro_ros_robot',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'camera_index': 0,
                'tag_size': 0.10,
            }]
        ),

        # Localization Node
        Node(
            package='micro_ros_robot',
            executable='localization_node',
            name='localization_node',
            output='screen',
            parameters=[{
                'marker_map': {
                    '0': [0.0, 0.0, 0.0],
                    '1': [1.5, 0.0, 0.0],
                    '2': [0.0, 1.5, 0.0],
                    '3': [1.5, 1.5, 0.0],
                },
            }]
        ),

        # Navigation Node
        Node(
            package='micro_ros_robot',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[{
                'max_linear_speed': 0.20,
                'max_angular_speed': 1.5,
                'waypoint_threshold': 0.15,
            }]
        ),

        # Task Manager Node
        Node(
            package='micro_ros_robot',
            executable='task_manager_node',
            name='task_manager_node',
            output='screen',
            parameters=[{
                'home_position': [0.0, 0.0, 0.0],
                'manufacturing_position': [1.5, 0.0, 0.0],
                'storage_a_position': [0.0, 1.5, 0.0],
                'storage_b_position': [1.5, 1.5, 0.0],
            }]
        ),
    ])
```

### 7.1 Running the Launch File

```bash
# Standard launch
ros2 launch micro_ros_robot robot_bringup.launch.py

# With custom parameters
ros2 launch micro_ros_robot robot_bringup.launch.py camera_index:=1

# With remapping
ros2 launch micro_ros_robot robot_bringup.launch.py \
    camera_node:=camera \
    navigation_node:=nav
```

---

## 8. Topic Reference

### 8.1 Complete Topic List

| Topic | Type | Publisher | Subscriber | Rate |
|-------|------|-----------|------------|------|
| `/aruco_detections` | PoseArray | camera_node | localization_node | 30Hz |
| `/box_color` | String | camera_node | task_manager_node | 30Hz |
| `/cmd_vel` | Twist | navigation_node | ESP32 | 20Hz |
| `/cmd_result` | String | ESP32 | (debug) | event |
| `/gripper_cmd` | String | task_manager_node | ESP32 | event |
| `/imu_data` | Accel | ESP32 | localization_node | 50Hz |
| `/navigation_status` | Point | navigation_node | task_manager_node | event |
| `/robot_pose` | Pose | localization_node | navigation_node | 20Hz |
| `/task_status` | String | task_manager_node | (debug) | 10Hz |
| `/ultrasonic_data` | Point | ESP32 | (debug) | 5Hz |
| `/waypoint` | Point | task_manager_node | navigation_node | event |

### 8.2 Message Definitions

#### geometry_msgs/Twist (cmd_vel)
```yaml
linear:
  x: 0.2    # m/s, range [-0.3, 0.3]
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5    # rad/s, range [-2.0, 2.0]
```

#### geometry_msgs/Pose (robot_pose)
```yaml
position:
  x: 0.5    # meters
  y: 1.2
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.707  # sin(yaw/2)
  w: 0.707  # cos(yaw/2)
```

#### geometry_msgs/PoseArray (aruco_detections)
```yaml
poses:  # Array of detected tag poses
  -
    position:
      x: 0.5   # distance from camera to tag (meters)
      y: 0.3
      z: 1.0
    orientation:
      x: 0.1   # quaternion
      y: 0.2
      z: 0.3
      w: 0.9
```

---

## 9. Parameter Reference

### 9.1 camera_node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_index` | int | 0 | Video device (0=/dev/video0) |
| `tag_size` | float | 0.10 | AprilTag side length (meters) |
| `calibration_file` | string | "" | Path to `data/camera_calibration/camera_calibration.json` |

### 9.2 localization_node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `marker_map` | dict | see below | Tag ID to world position mapping |
| `robot_frame` | string | "map" | TF frame for robot pose |

Default marker_map:
```python
{
    '0': [0.0, 0.0, 0.0],
    '1': [1.5, 0.0, 0.0],
    '2': [0.0, 1.5, 0.0],
    '3': [1.5, 1.5, 0.0],
}
```

### 9.3 navigation_node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_linear_speed` | float | 0.20 | Max forward speed (m/s) |
| `max_angular_speed` | float | 1.5 | Max turn rate (rad/s) |
| `pid_linear_kp` | float | 2.0 | Linear proportional gain |
| `pid_linear_ki` | float | 0.1 | Linear integral gain |
| `pid_linear_kd` | float | 0.5 | Linear derivative gain |
| `pid_angular_kp` | float | 3.0 | Angular proportional gain |
| `pid_angular_ki` | float | 0.2 | Angular integral gain |
| `pid_angular_kd` | float | 1.0 | Angular derivative gain |
| `waypoint_threshold` | float | 0.15 | Distance to consider reached (m) |
| `angle_threshold` | float | 0.2 | Angle threshold for orientation (rad) |

### 9.4 task_manager_node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `home_position` | list | [0.0, 0.0, 0.0] | Home station (x, y, theta) |
| `manufacturing_position` | list | [1.5, 0.0, 0.0] | MFG station |
| `storage_a_position` | list | [0.0, 1.5, 0.0] | Storage A (red box) |
| `storage_b_position` | list | [1.5, 1.5, 0.0] | Storage B (green box) |

---

**Document Version:** 1.0.0
**Last Updated:** May 2026
**Author:** Robotics Team