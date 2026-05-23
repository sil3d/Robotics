# ============================================================================
#   MICRO-ROS ROBOT PROJECT - Complete ROS2 Implementation
# ============================================================================

Complete autonomous mobile robot using ROS2 + micro-ROS distributed architecture.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        RASPBERRY PI (ROS2 Humble)                   │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐               │
│  │camera_node   │  │color_detector│  │localization  │               │
│  │/aruco_detect│  │ /box_color   │  │ /robot_pose  │               │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘               │
│         │                 │                 │                        │
│         └────────────┬────┴────────────────┘                        │
│                      ▼                                             │
│              ┌──────────────┐   ┌──────────────┐                   │
│              │navigation    │   │ task_manager │                   │
│              │ /cmd_vel     │   │ state machine│                   │
│              └──────┬───────┘   └──────────────┘                   │
│                     │                                              │
│  ┌──────────────────┴──────────────────────────────────────────┐  │
│  │                    micro_ros_agent (UDP:8888)               │  │
│  └──────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
                              │
                          UDP│8888
                              │
┌─────────────────────────────────────────────────────────────────────┐
│                      ESP32 (micro-ROS)                             │
├─────────────────────────────────────────────────────────────────────┤
│  Subscribe: /cmd_vel (Twist), /gripper_cmd (String)                │
│  Publish:   /imu_data, /ultrasonic_data, /cmd_result                │
└─────────────────────────────────────────────────────────────────────┘
```

## Project Structure

```
Micro_ROS_Project/
├── esp32_firmware/
│   └── micro_ros_esp32Robot.ino   # ESP32 firmware (micro-ROS)
│
├── ubuntu_ros2/
│   ├── micro_ros_robot/           # ROS2 Python package
│   │   ├── launch/
│   │   │   └── robot_bringup.launch.py
│   │   ├── scripts/
│   │   │   ├── camera_node.py
│   │   │   ├── localization_node.py
│   │   │   ├── navigation_node.py
│   │   │   └── task_manager_node.py
│   │   └── package.xml
│   │
│   ├── flask_ros_bridge.py       # Debug web interface
│   ├── data/
│   │   └── camera_calibration/camera_calibration.json
│   └── README.md                 # This file
│
└── README.md
```

## ROS2 Nodes

### camera_node
- Subscribes: `/image_raw` (camera frames)
- Publishes: `/aruco_detections` (PoseArray), `/box_color` (String)
- Detects AprilTag markers and estimates 6DOF pose
- HSV color detection for red/green boxes

### localization_node
- Subscribes: `/aruco_detections`, `/imu_data`
- Publishes: `/robot_pose` (Pose)
- Fuses AprilTag pose + IMU yaw for pose estimation

### navigation_node
- Subscribes: `/robot_pose`, `/waypoint`
- Publishes: `/cmd_vel` (Twist)
- PID controller for waypoint following

### task_manager_node
- Subscribes: `/box_color`, `/navigation_status`, `/robot_pose`
- Publishes: `/waypoint`, `/gripper_cmd`
- State machine: IDLE → GO_TO_MFG → DETECT_COLOR → PICK → GO_TO_STORAGE → DEPOSIT → RETURN_HOME

## Station Map

```
        Storage B (1.5, 1.5)
            │
            │        Storage A (0, 1.5)
            │            │
    ────────┼───────────────────────────── Y
            │            │
    MFG     │         Home
    (1.5, 0)│        (0, 0)
            │
            └───────────────────────────── X
```

## Topic Summary

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/image_raw` | sensor_msgs/Image | camera_node out | Camera frames |
| `/aruco_detections` | geometry_msgs/PoseArray | camera_node out | AprilTag poses |
| `/box_color` | std_msgs/String | camera_node out | "red"/"green"/"none" |
| `/robot_pose` | geometry_msgs/Pose | localization_node out | Robot (x,y,theta) |
| `/cmd_vel` | geometry_msgs/Twist | navigation_node out → ESP32 | Velocity commands |
| `/waypoint` | geometry_msgs/Point | task_manager → navigation | Target (x,y,theta) |
| `/gripper_cmd` | std_msgs/String | task_manager → ESP32 | "open"/"close" |
| `/imu_data` | geometry_msgs/Accel | ESP32 → localization | Yaw, omega, accel |
| `/ultrasonic_data` | geometry_msgs/Point | ESP32 | 4x US distances |

## Web Interface (Flask)

A Flask debug dashboard is available for monitoring and control:

```bash
# Start the web interface
python3 flask_ros_bridge.py
```

Open browser at: `http://localhost:5000`

### Video Streams

| Stream | URL | Description |
|--------|-----|-------------|
| **Robot Map** | `/map_feed` | SLAM visualization with position, trajectory, ultrasonic sensors |
| **IMU 3D** | `/imu3d_feed` | Real-time 3D IMU orientation |
| **Camera Feed** | `/camera_feed` | Raw camera from `/image_raw` (~30 FPS) |

### Features

- Real-time sensor health monitoring
- Mission state display (LSTM enabled/disabled)
- Manual controls (WASD, gripper, emergency stop)
- REST API for external control

## Installation

### ESP32 Firmware
1. Open `esp32_firmware/micro_ros_esp32Robot.ino` in Arduino IDE
2. Install libraries:
   - micro_ros_arduino
   - DFRobot_BMI160
   - ESP32Servo
3. Flash to ESP32

### Ubuntu ROS2
```bash
# Create workspace
mkdir -p ~/micro_ros_ws/src
cd ~/micro_ros_ws/src

# Clone micro-ROS packages
git clone -b humble https://github.com/micro-ROS/micro_ros_msgs.git
git clone -b humble https://github.com/micro-ROS/micro_ros_agent.git
git clone -b humble https://github.com/micro-ROS/micro_ros_utilities.git

# Copy robot package
cp -r /path/to/ubuntu_ros2/micro_ros_robot src/

# Build
cd ~/micro_ros_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# Install Python dependencies
pip3 install opencv-python numpy cv_bridge image_transport
```

## Running

```bash
# Terminal 1: Start micro-ROS agent
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Terminal 2: Start all robot nodes
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch micro_ros_robot robot_bringup.launch.py

# Or run nodes individually:
ros2 run micro_ros_robot camera_node
ros2 run micro_ros_robot localization_node
ros2 run micro_ros_robot navigation_node
ros2 run micro_ros_robot task_manager_node

# Start task via service
ros2 service call /start_task std_srvs/Empty "{}"
```

## Debug

```bash
# List topics
ros2 topic list

# Echo IMU
ros2 topic echo /imu_data

# Echo robot pose
ros2 topic echo /robot_pose

# Manually send waypoint
ros2 topic pub /waypoint geometry_msgs/Point "{x: 1.5, y: 0.0, z: 0.0}" -1

# Manually send velocity (for testing)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" -1
```

## ESP32 Wiring

```
Motors (Differential Drive):
  Motor A: IN1=27, IN2=26, ENA=14
  Motor B: IN3=13, IN4=12, ENB=4

Servo/Gripper: PIN=19

IMU BMI160: SDA=21, SCL=22

Ultrasonics (4x):
  Front: TRIG=5, ECHO=34
  Back:  TRIG=2, ECHO=35
  Left:  TRIG=15, ECHO=32
  Right: TRIG=33, ECHO=25
```

## Velocity Control

ESP32 receives `/cmd_vel` (geometry_msgs/Twist):
- `linear.x` = forward/backward speed (m/s)
- `angular.z` = yaw rate (rad/s)

Differential drive mapping:
```
v_left  = linear.x - angular.z * (wheel_dist / 2)
v_right = linear.x + angular.z * (wheel_dist / 2)
```

Max values:
- Linear: 0.3 m/s
- Angular: 2.0 rad/s