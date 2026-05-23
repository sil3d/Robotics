# ============================================================================
#   MICRO-ROS AUTONOMOUS MOBILE ROBOT PROJECT
# ============================================================================

```
    ██████╗ ██████╗ ███████╗ █████╗  ██████╗██╗  ██╗
    ██╔══██╗██╔══██╗██╔════╝██╔══██╗██╔════╝██║  ██║
    ██████╔╝██████╔╝█████╗  ███████║██║     ███████║
    ██╔══██╗██╔══██╗██╔══╝  ██╔══██║██║     ██╔══██║
    ██████╔╝██║  ██║███████╗██║  ██║╚██████╗██║  ██║
    ╚═════╝ ╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝ ╚═════╝╚═╝  ╚═╝
```

An autonomous mobile robot using distributed ROS2 + micro-ROS architecture.
Complete warehouse automation system with AprilTag SLAM, color detection,
and mission management.

## ═══════════════════════════════════════════════════════════════════════════
## TABLE OF CONTENTS
## ═══════════════════════════════════════════════════════════════════════════

1.  [Project Overview](#1-project-overview)
2.  [System Architecture](#2-system-architecture)
3.  [Hardware Setup](#3-hardware-setup)
4.  [Installation](#4-installation)
5.  [Running the Robot](#5-running-the-robot)
6.  [Project Structure](#6-project-structure)
7.  [ROS2 Topics and Messages](#7-ros2-topics-and-messages)
8.  [Node Descriptions](#8-node-descriptions)
9.  [Calibration](#9-calibration)
10. [Web Interface (Flask)](#10-web-interface-flask)
11. [Troubleshooting](#11-troubleshooting)
12. [Development](#12-development)
13. [Enhanced Features (v2.0)](#13-enhanced-features-v20)
14. [New Topics and Services](#14-new-topics-and-services)
15. [File Structure (v2.0)](#15-file-structure-v20)

## ═══════════════════════════════════════════════════════════════════════════
## 1. PROJECT OVERVIEW
## ═══════════════════════════════════════════════════════════════════════════

### 1.1 Project Description

This project implements an autonomous mobile robot for warehouse operations using
a distributed computing architecture based on ROS2 and micro-ROS.

The robot performs the following mission:
1. Start from **Home** position
2. Navigate to **Manufacturing Station**
3. Detect box color (red or green)
4. Pick up the box using a gripper mechanism
5. Deliver to correct storage:
   - **Red box** → Storage A (x=0, y=1.5)
   - **Green box** → Storage B (x=1.5, y=1.5)
6. Return to **Home** position

### 1.2 Key Features

| Feature | Implementation |
|---------|----------------|
| Distributed Architecture | ESP32 (low-level) + Raspberry Pi (high-level) |
| Communication | ROS2 + micro-ROS over UDP |
| Localization | AprilTag visual markers + IMU fusion |
| Navigation | PID waypoint following |
| Color Detection | HSV-based red/green detection |
| Gripper Control | Servo-based mechanical gripper |
| Web Interface | Flask debug dashboard (port 5000) |

### 1.3 Technical Specifications

| Specification | Value |
|---------------|-------|
| Max Linear Speed | 0.3 m/s |
| Max Angular Speed | 2.0 rad/s |
| IMU Update Rate | 50 Hz |
| Ultrasonic Update Rate | 5 Hz |
| Camera Resolution | 640x480 |
| AprilTag Family | 36h11 |
| AprilTag Size | 10 cm |

### 1.4 Environment Layout

```
    Y (meters)
    │
    │   ┌──────────────────────────────┐
    │   │                              │
    │   │    Storage B    Storage A    │
    │   │    (1.5, 1.5)    (0, 1.5)    │
    │   │        ●──────────●          │
    │   │        │          │          │
    │   │        │          │          │
    │   │        │          │          │
    │   │        │          │          │
    │   │   MFG──●          │          │
    │   │  (1.5, 0)         │          │
    │   │        │          │          │
    │   │        │          │          │
    │   │   Home●───────────●──────────│
    │   │  (0,0)                    X   │
    │   └──────────────────────────────┘
    └─────────────────────────────────────
```

**Station Coordinates:**
- Home: (0.0, 0.0, 0.0)
- Manufacturing (MFG): (1.5, 0.0, 0.0)
- Storage A (Red box): (0.0, 1.5, 0.0)
- Storage B (Green box): (1.5, 1.5, 0.0)

## ═══════════════════════════════════════════════════════════════════════════
## 2. SYSTEM ARCHITECTURE
## ═══════════════════════════════════════════════════════════════════════════

### 2.1 High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         RASPBERRY PI 4 (ROS2 Humble)                    │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    CAMERA_NODE                                  │   │
│  │  • Captures frames from USB camera                              │   │
│  │  • Detects AprilTag 36h11 markers                               │   │
│  │  • Estimates 6DOF pose for each marker                         │   │
│  │  • Performs HSV color detection (red/green)                     │   │
│  │                                                                 │   │
│  │  PUBLISHES:                                                     │   │
│  │    → /aruco_detections (geometry_msgs/PoseArray)                │   │
│  │    → /box_color (std_msgs/String) "red"/"green"/"none"         │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                 │                                      │
│                                 ▼                                      │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                 LOCALIZATION_NODE                               │   │
│  │  • Subscribes to /aruco_detections and /imu_data                │   │
│  │  • Maintains robot pose (x, y, theta)                          │   │
│  │  • Uses AprilTag poses + IMU yaw for pose estimation            │   │
│  │  • Publishes robot position in map frame                        │   │
│  │                                                                 │   │
│  │  PUBLISHES:                                                     │   │
│  │    → /robot_pose (geometry_msgs/Pose)                           │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                 │                                      │
│                                 ▼                                      │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                  NAVIGATION_NODE                                │   │
│  │  • PID waypoint following controller                            │   │
│  │  • Subscribes to /robot_pose and /waypoint                      │   │
│  │  • Computes linear and angular velocity commands                │   │
│  │  • Handles obstacle avoidance (future)                          │   │
│  │                                                                 │   │
│  │  PUBLISHES:                                                     │   │
│  │    → /cmd_vel (geometry_msgs/Twist)                             │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                 │                                      │
│                                 ▼                                      │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                 TASK_MANAGER_NODE                                │   │
│  │  • Mission state machine                                        │   │
│  │  • Subscribes to /box_color, /navigation_status                 │   │
│  │  • Manages complete mission sequence                            │   │
│  │  • Controls gripper via /gripper_cmd                            │   │
│  │                                                                 │   │
│  │  PUBLISHES:                                                     │   │
│  │    → /waypoint (geometry_msgs/Point)                            │   │
│  │    → /gripper_cmd (std_msgs/String) "open"/"close"              │   │
│  │                                                                 │   │
│  │  SERVICES:                                                      │   │
│  │    ← /start_task (std_srvs/Empty)                               │   │
│  │    ← /cancel_task (std_srvs/Empty)                              │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                   MICRO_ROS_AGENT                                │   │
│  │  • Bridges ESP32 micro-ROS messages to ROS2                      │   │
│  │  • UDP server on port 8888                                      │   │
│  │  • Receives /imu_data, /ultrasonic_data from ESP32              │   │
│  │  • Publishes /cmd_vel, /gripper_cmd to ESP32                    │   │
│  └─────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
                                  │
                                  │ UDP (port 8888)
                                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                              ESP32 (micro-ROS)                          │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                      MOTOR CONTROL                              │   │
│  │  • Differential drive (2 motors)                                │   │
│  │  • Subscribes to /cmd_vel (Twist)                               │   │
│  │  • PWM output: 30kHz, 8-bit resolution                          │   │
│  │  • Velocity mapping: linear.x → forward, angular.z → turn       │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                         IMU (BMI160)                            │   │
│  │  • 6-axis IMU (3-axis accel + 3-axis gyro)                      │   │
│  │  • I2C communication (SDA=21, SCL=22)                           │   │
│  │  • Gyro calibration at startup                                  │   │
│  │  • Publishes: yaw, omega_z, accel_x/y/z                         │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    ULTRASONIC SENSORS (4x)                      │   │
│  │  • Front, Back, Left, Right                                     │   │
│  │  • Round-robin measurement at 5Hz                                │   │
│  │  • Valid range: 5-150 cm                                         │   │
│  │  • 5 samples averaged per reading                                │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    GRIPPER (SERVO)                              │   │
│  │  • Subscribes to /gripper_cmd (String)                          │   │
│  │  • "open" → 0°, "close" → 90°                                   │   │
│  │  • Servo pin: 19                                                │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  PUBLISHES:                                                             │
│    → /imu_data (geometry_msgs/Accel)        - 50Hz                      │
│    → /ultrasonic_data (geometry_msgs/Point) - 5Hz                       │
│    → /cmd_result (std_msgs/String)          - on command               │
│                                                                         │
│  SUBSCRIBES:                                                            │
│    ← /cmd_vel (geometry_msgs/Twist)                                     │
│    ← /gripper_cmd (std_msgs/String)                                    │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Data Flow Diagram

```
┌──────────┐     ┌────────────┐     ┌────────────────┐     ┌─────────────┐
│  Camera  │────▶│   AprilTag │     │               │     │  Differential│
│  (USB)   │     │  Detection │     │               │     │   Drive     │
└──────────┘     └─────┬──────┘     │               │     └──────┬──────┘
                      │            │               │            │
                      ▼            ▼               ▼            │
               ┌──────────┐  ┌───────────┐  ┌───────────┐       │
               │ Color    │  │  Robot    │  │   PID     │       │
               │ Detection│  │   Pose    │  │  Control  │       │
               └────┬─────┘  └─────┬─────┘  └─────┬─────┘       │
                   │              │              │              │
                   ▼              ▼              ▼              │
            ┌──────────┐    ┌───────────┐  ┌───────────┐        │
            │ /box_color│    │/robot_pose│  │ /cmd_vel   │───────┘
            └──────────┘    └───────────┘  └───────────┘
```

### 2.3 State Machine

```
                    ┌─────────────────┐
                    │                 │
                    │     IDLE       │◄──────────────────────┐
                    │                 │                       │
                    │  Waiting for    │                       │
                    │  /start_task    │                       │
                    │                 │                       │
                    └────────┬────────┘                       │
                             │                                │
                             │ /start_task                    │
                             ▼                                │
                    ┌─────────────────┐                       │
                    │                 │                       │
                    │   GO_TO_MFG     │──────────────────────►│
                    │                 │   (cancel)            │
                    │  Navigate to    │                       │
                    │  MFG station    │                       │
                    │                 │                       │
                    └────────┬────────┘                       │
                             │                                │
                             │ waypoint reached              │
                             ▼                                │
                    ┌─────────────────┐                       │
                    │                 │                       │
                    │  DETECT_COLOR   │──────────────────────►│
                    │                 │   (cancel)           │
                    │  Camera detects │                       │
                    │  red or green   │                       │
                    │                 │                       │
                    └────────┬────────┘                       │
                             │                                │
                             │ color detected                │
                             ▼                                │
                    ┌─────────────────┐                       │
                    │                 │                       │
                    │   PICK_BOX      │──────────────────────►│
                    │                 │   (cancel)           │
                    │  Close gripper  │                       │
                    │  (grab box)     │                       │
                    │                 │                       │
                    └────────┬────────┘                       │
                             │                                │
                             │ gripper closed                 │
                             ▼                                │
                    ┌─────────────────┐                       │
                    │                 │                       │
                    │  GO_TO_STORAGE  │──────────────────────►│
                    │                 │   (cancel)           │
                    │  Navigate to:   │                       │
                    │  Storage A=red   │                       │
                    │  Storage B=green │                       │
                    │                 │                       │
                    └────────┬────────┘                       │
                             │                                │
                             │ waypoint reached              │
                             ▼                                │
                    ┌─────────────────┐                       │
                    │                 │                       │
                    │  DEPOSIT_BOX    │──────────────────────►│
                    │                 │   (cancel)           │
                    │  Open gripper   │                       │
                    │  (release box)  │                       │
                    │                 │                       │
                    └────────┬────────┘                       │
                             │                                │
                             │ box deposited                 │
                             ▼                                │
                    ┌─────────────────┐                       │
                    │                 │                       │
                    │  RETURN_HOME    │──────────────────────►│
                    │                 │   (cancel)           │
                    │  Navigate to    │                       │
                    │  Home position  │                       │
                    │                 │                       │
                    └────────┬────────┘                       │
                             │                                │
                             │ home reached                  │
                             ▼                                │
                    ┌─────────────────┐                       │
                    │                 │                       │
                    │ TASK_COMPLETE  │───────────────────────┘
                    │                 │   (auto-return to IDLE)
                    │  Mission done!  │
                    │                 │
                    └─────────────────┘
```

## ═══════════════════════════════════════════════════════════════════════════
## 3. HARDWARE SETUP
## ═══════════════════════════════════════════════════════════════════════════

### 3.1 ESP32 Pinout

```
    ┌────────────────────────┐
    │         ESP32          │
    │                        │
    │  Vin ──────────────────┼─── 5V (from USB or 5V supply)
    │  GND ──────────────────┼─── GND
    │                        │
    │  GPIO 21 ──────────────┼─── I2C SDA (to IMU)
    │  GPIO 22 ──────────────┼─── I2C SCL (to IMU)
    │                        │
    │  ─── MOTOR A (Left) ───│
    │  GPIO 27 ──────────────┼─── IN1 (direction)
    │  GPIO 26 ──────────────┼─── IN2 (direction)
    │  GPIO 14 ──────────────┼─── ENA (PWM speed)
    │                        │
    │  ─── MOTOR B (Right) ───│
    │  GPIO 13 ──────────────┼─── IN3 (direction)
    │  GPIO 12 ──────────────┼─── IN4 (direction)
    │  GPIO  4 ──────────────┼─── ENB (PWM speed)
    │                        │
    │  ─── GRIPPER SERVO ────│
    │  GPIO 19 ──────────────┼─── Servo Signal
    │                        │
    │  ─── ULTRASONIC 1 (Front) ───│
    │  GPIO  5 ──────────────┼─── TRIG
    │  GPIO 34 ──────────────┼─── ECHO
    │                        │
    │  ─── ULTRASONIC 2 (Back) ────│
    │  GPIO  2 ──────────────┼─── TRIG
    │  GPIO 35 ──────────────┼─── ECHO
    │                        │
    │  ─── ULTRASONIC 3 (Left) ────│
    │  GPIO 15 ──────────────┼─── TRIG
    │  GPIO 32 ──────────────┼─── ECHO
    │                        │
    │  ─── ULTRASONIC 4 (Right) ───│
    │  GPIO 33 ──────────────┼─── TRIG
    │  GPIO 25 ──────────────┼─── ECHO
    │                        │
    └────────────────────────┘
```

### 3.2 Motor Driver Connection (L298N or similar)

```
ESP32              Motor Driver         Motors
──────             ─────────────        ──────
GPIO 27 ──────────► IN1                 Motor A +
GPIO 26 ──────────► IN2                 Motor A -
GPIO 14 ──────────► ENA (PWM)          Motor A Enable

GPIO 13 ──────────► IN3                 Motor B +
GPIO 12 ──────────► IN4                 Motor B -
GPIO  4 ──────────► ENB (PWM)           Motor B Enable

GND ──────────────► GND                 (common ground)
Vin ──────────────► +12V (if using external supply)
```

### 3.3 IMU Connection (BMI160)

```
ESP32              BMI160 Module
──────             ─────────────
GPIO 21 ──────────► SDA
GPIO 22 ──────────► SCL
3.3V  ────────────► VCC
GND  ─────────────► GND
```

### 3.4 Ultrasonic Sensor Connection (HC-SR04)

```
ESP32              HC-SR04
──────             ────────
GPIO 5 ────────────► TRIG (Front)
GPIO 34 ────────────► ECHO (Front)

GPIO 2 ────────────► TRIG (Back)
GPIO 35 ────────────► ECHO (Back)

GPIO 15 ────────────► TRIG (Left)
GPIO 32 ────────────► ECHO (Left)

GPIO 33 ────────────► TRIG (Right)
GPIO 25 ────────────► ECHO (Right)

3.3V ──────────────► VCC
GND  ──────────────► GND
```

### 3.5 Servo/Gripper Connection

```
ESP32              Servo
──────             ─────
GPIO 19 ───────────► Signal (orange/yellow)
3.3V ──────────────► VCC (red)
GND  ──────────────► GND (brown)
```

### 3.6 Power Requirements

| Component | Voltage | Current (typical) |
|-----------|---------|-------------------|
| ESP32 Dev Kit | 5V | 500mA |
| Motors (2x) | 6-12V | 1-2A (stall) |
| IMU (BMI160) | 3.3V | 5mA |
| Ultrasonics (4x) | 5V | 60mA |
| Servo | 5-6V | 500mA |

**Recommended:** Use a 7.4V LiPo battery with BEC for regulated 5V/6V output.

### 3.7 Mechanical Build

```
    ┌─────────────────────────────────────┐
    │           TOP VIEW                   │
    │                                      │
    │  ┌─────┐                   ┌─────┐  │
    │  │US-L │                   │US-R │  │
    │  └─────┘                   └─────┘  │
    │                                      │
    │           ┌───────┐                  │
    │           │ Camera│                  │
    │           │ (USB) │                  │
    │           └───────┘                  │
    │                                      │
    │  ┌─────┐                   ┌─────┐  │
    │  │US-F │     ┌─────┐      │US-B │  │
    │  └─────┘     │ESP32│      └─────┘  │
    │              └─────┘                 │
    │  ┌─────┐                   ┌─────┐  │
    │  │Motor│                   │Motor│  │
    │  │  A  │                   │  B  │  │
    │  └─────┘                   └─────┘  │
    │                                      │
    │         ┌──────────────┐            │
    │         │   Battery    │            │
    │         │  (underneath)│            │
    │         └──────────────┘            │
    │                                      │
    └─────────────────────────────────────┘
```

## ═══════════════════════════════════════════════════════════════════════════
## 4. INSTALLATION
## ═══════════════════════════════════════════════════════════════════════════

### 4.1 ESP32 Firmware Installation

#### 4.1.1 Required Libraries

Install these libraries in your Arduino IDE (Sketch → Include Library → Manage Libraries):

| Library | Version | Purpose |
|---------|---------|---------|
| micro_ros_arduino | latest | micro-ROS communication |
| DFRobot_BMI160 | latest | IMU sensor |
| ESP32Servo | latest | Servo control |

#### 4.1.2 Installing micro_ros_arduino

```bash
# Navigate to your Arduino libraries folder
cd ~/Arduino/libraries

# Clone micro-ROS Arduino library
git clone https://github.com/micro-ROS/micro_ros_arduino.git

# Restart Arduino IDE
```

#### 4.1.3 ESP32 Board Configuration

1. Open `esp32_firmware/micro_ros_esp32Robot.ino` in Arduino IDE
2. Select Tools → Board → ESP32 Dev Module
3. Select the correct port (e.g., /dev/ttyUSB0 or COM3)
4. Configure:
   - Upload Speed: 115200
   - CPU Frequency: 240MHz
   - Flash Size: 4MB (or appropriate for your board)

#### 4.1.4 Flashing the Firmware

1. Connect ESP32 via USB
2. Press and hold BOOT button, then press EN button for 1 second
   (puts ESP32 in download mode)
3. Click Upload in Arduino IDE
4. Wait for "Hard resetting via RTS pin..." message
5. Open Serial Monitor (115200 baud) to see startup messages

#### 4.1.5 Expected Serial Output

```
ets Jun  8 2016 00:22:57
rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
...
[CALIB] Gyro Z, ne bouge pas...
[CALIB] bias=-1.23
[ROS2] ESP32 ready!
[ROS2] Topics: /imu_data /ultrasonic_data /cmd_vel /gripper_cmd
```

### 4.2 Ubuntu ROS2 Installation

#### 4.2.1 Prerequisites

- Ubuntu 22.04 (Jammy) or 20.04 (Focal)
- ROS 2 Humble Hawksbill installed
- Python 3.8+

#### 4.2.2 Install ROS2 Humble

```bash
# Set locale
locale  # Check for UTF-8

# Enable universe repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe

# Install ROS2
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

sudo apt update
sudo apt install ros-humble-desktop

# Source ROS2 in your bashrc (optional)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install dependencies
sudo apt install python3-pip python3-rosdep
sudo rosdep init
rosdep update
```

#### 4.2.3 Install micro-ROS Packages

```bash
# Create workspace
mkdir -p ~/micro_ros_ws/src
cd ~/micro_ros_ws/src

# Clone micro-ROS repositories (humble branch)
git clone -b humble https://github.com/micro-ROS/micro_ros_msgs.git
git clone -b humble https://github.com/micro-ROS/micro_ros_agent.git
git clone -b humble https://github.com/micro-ROS/micro_ros_utilities.git

# Build workspace
cd ~/micro_ros_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# Verify installation
ros2 run micro_ros_agent micro_ros_agent --help
```

#### 4.2.4 Install Robot Package

```bash
# Copy robot package to workspace
cp -r /path/to/Micro_ROS_Project/ubuntu_ros2/micro_ros_robot ~/micro_ros_ws/src/

# Install Python dependencies
pip3 install opencv-python numpy cv-bridge sensor-msgs geometry-msgs std-msgs std-srvs

# Build robot package
cd ~/micro_ros_ws
colcon build --packages-select micro_ros_robot
source install/setup.bash

# Verify nodes are available
ros2 pkg list | grep micro_ros_robot
```

### 4.3 Camera Calibration Data

If you have a pre-calibrated camera, place `camera_calibration.json` at:
```bash
cp camera_calibration.json ../data/camera_calibration/camera_calibration.json
```

Otherwise, follow the calibration procedure in Section 9.

### 4.4 Network Configuration

ESP32 and Raspberry Pi must be on the same network. The micro-ROS agent
communicates via UDP on port 8888.

```
┌─────────────┐         ┌─────────────────┐         ┌─────────────┐
│   ESP32     │◄───────►│     Network      │◄───────►│ Raspberry Pi│
│  (WiFi/ETH) │  UDP    │   (Router)       │  UDP    │  (ROS2)     │
│  Port 8888  │         │                  │  Port 8888 │
└─────────────┘         └─────────────────┘         └─────────────┘
```

## ═══════════════════════════════════════════════════════════════════════════
## 5. RUNNING THE ROBOT
## ═══════════════════════════════════════════════════════════════════════════

### 5.1 Quick Start

```bash
# Terminal 1: Start micro-ROS agent
source /opt/ros/humble/setup.bash
source ~/micro_ros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Terminal 2: Launch all robot nodes
source /opt/ros/humble/setup.bash
source ~/micro_ros_ws/install/setup.bash
ros2 launch micro_ros_robot robot_bringup.launch.py
```

### 5.2 Launch File Options

The launch file starts all nodes with default parameters. To customize:

```bash
# Run with specific camera index
ros2 launch micro_ros_robot robot_bringup.launch.py camera_index:=1

# Run with debug output
ros2 launch micro_ros_robot robot_bringup.launch.py log_level:=debug
```

### 5.3 Starting a Mission

```bash
# After all nodes are running, start the mission
ros2 service call /start_task std_srvs/Empty "{}"

# Monitor task status
ros2 topic echo /task_status
```

### 5.4 Manual Control (for testing)

```bash
# Send velocity command (for testing motor wiring)
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" -1

# Send gripper command
ros2 topic pub /gripper_cmd std_msgs/String "{data: 'open'}" -1
ros2 topic pub /gripper_cmd std_msgs/String "{data: 'close'}" -1

# Navigate to specific waypoint
ros2 topic pub /waypoint geometry_msgs/Point "{x: 1.5, y: 0.0, z: 0.0}" -1
```

### 5.5 Monitoring

```bash
# List all active topics
ros2 topic list

# Echo IMU data
ros2 topic echo /imu_data

# Echo robot pose
ros2 topic echo /robot_pose

# Echo AprilTag detections
ros2 topic echo /aruco_detections

# Echo box color detection
ros2 topic echo /box_color

# Echo navigation status
ros2 topic echo /navigation_status

# Check node health
ros2 node list
ros2 node info /camera_node
ros2 node info /localization_node
ros2 node info /navigation_node
ros2 node info /task_manager
```

## ═══════════════════════════════════════════════════════════════════════════
## 6. PROJECT STRUCTURE
## ═══════════════════════════════════════════════════════════════════════════

```
Micro_ROS_Project/
│
├── esp32_firmware/
│   ├── micro_ros_esp32Robot.ino    # Complete ESP32 firmware
│   └── README_ESP32.md             # ESP32-specific documentation
│
├── ubuntu_ros2/
│   ├── micro_ros_robot/            # ROS2 Python package
│   │   ├── package.xml              # Package manifest
│   │   ├── setup.py                 # Setup script
│   │   ├── launch/
│   │   │   └── robot_bringup.launch.py  # Main launch file
│   │   └── scripts/
│   │       ├── camera_node.py       # AprilTag + color detection
│   │       ├── localization_node.py # Pose estimation
│   │       ├── navigation_node.py   # PID waypoint following
│   │       └── task_manager_node.py # Mission state machine
│   │
│   ├── data/
│   │   ├── camera_calibration/camera_calibration.json
│   │   ├── reference_markers/reference_markers.json
│   │   ├── marker_positions/marker_positions.json
│   │   ├── marker_map_auto/marker_map_auto.json
│   │   ├── tags/tags.json
│   │   └── tags_slam/tags_slam.json
│   ├── flask_ros_bridge.py           # Debug web interface
│   ├── README.md                     # This file
│   └── NOTES.md                      # Development notes
│
└── docs/
    ├── ARCHITECTURE.md              # System architecture details
    ├── CALIBRATION.md               # Camera calibration guide
    ├── NAVIGATION.md                # Navigation algorithm details
    └── TASK_MANAGER.md              # State machine documentation
```

## ═══════════════════════════════════════════════════════════════════════════
## 7. ROS2 TOPICS AND MESSAGES
## ═══════════════════════════════════════════════════════════════════════════

### 7.1 Topic Summary Table

| Topic | Message Type | Direction | Rate | Description |
|-------|--------------|-----------|------|-------------|
| `/image_raw` | sensor_msgs/Image | internal | 30Hz | Camera frames |
| `/aruco_detections` | geometry_msgs/PoseArray | camera_node → localization | 30Hz | Detected tag poses |
| `/box_color` | std_msgs/String | camera_node → task_manager | 30Hz | "red"/"green"/"none" |
| `/robot_pose` | geometry_msgs/Pose | localization → navigation | 20Hz | Robot (x,y,theta) |
| `/cmd_vel` | geometry_msgs/Twist | navigation → ESP32 | 20Hz | Velocity commands |
| `/waypoint` | geometry_msgs/Point | task_manager → navigation | event | Target position |
| `/gripper_cmd` | std_msgs/String | task_manager → ESP32 | event | "open"/"close" |
| `/imu_data` | geometry_msgs/Accel | ESP32 → localization | 50Hz | Yaw, omega, accel |
| `/ultrasonic_data` | geometry_msgs/Point | ESP32 → debug | 5Hz | 4x US distances |
| `/navigation_status` | geometry_msgs/Point | navigation → task_manager | event | Waypoint reached |
| `/task_status` | std_msgs/String | task_manager → debug | 10Hz | Current state |
| `/cmd_result` | std_msgs/String | ESP32 → debug | event | Command feedback |

### 7.2 Message Definitions

#### /cmd_vel (geometry_msgs/Twist)
```yaml
linear:
  x: 0.2          # Forward speed (m/s), range: [-0.3, 0.3]
  y: 0.0          # (unused for differential drive)
  z: 0.0          # (unused)
angular:
  x: 0.0          # (unused)
  y: 0.0          # (unused)
  z: 0.5          # Yaw rate (rad/s), range: [-2.0, 2.0]
```

#### /robot_pose (geometry_msgs/Pose)
```yaml
position:
  x: 0.5          # X position in map (meters)
  y: 1.2          # Y position in map (meters)
  z: 0.0          # (always 0 for 2D robot)
orientation:
  x: 0.0          # Quaternion x
  y: 0.0          # Quaternion y
  z: 0.707        # Quaternion z (sin(theta/2))
  w: 0.707        # Quaternion w (cos(theta/2))
```

#### /imu_data (geometry_msgs/Accel)
```yaml
linear:
  x: 45.3         # Yaw angle (degrees), 0-360
  y: -0.12        # Angular velocity around Z (rad/s)
  z: 0.0          # (not used)
angular:
  x: 0.15         # Acceleration X (m/s^2)
  y: -0.02        # Acceleration Y (m/s^2)
  z: 9.81         # Acceleration Z (m/s^2) - gravity when level
```

#### /ultrasonic_data (geometry_msgs/Point)
```yaml
x: 25.5          # Front sensor (cm), -1 = no reading
y: 45.2          # Back sensor (cm), -1 = no reading
z: 30.1          # Left sensor (cm), -1 = no reading
w: 0.0           # Right sensor (cm), -1 = no reading (stored in z for legacy)
```

#### /box_color (std_msgs/String)
```yaml
data: "red"      # "red", "green", or "none"
```

## ═══════════════════════════════════════════════════════════════════════════
## 8. NODE DESCRIPTIONS
## ═══════════════════════════════════════════════════════════════════════════

### 8.1 camera_node

**Purpose:** Detect AprilTag markers and box color from camera feed.

**Inputs:**
- USB Camera (OpenCV VideoCapture)
- Camera calibration parameters

**Outputs:**
- `/aruco_detections` - Array of detected marker poses
- `/box_color` - Detected color of box in front of robot

**Algorithm:**
1. Capture frame from camera at 30Hz
2. Convert to grayscale
3. Detect AprilTag markers using aruco.ArucoDetector
4. For each detected tag, estimate 6DOF pose using cv2.solvePnP
5. Convert rotation vector to quaternion
6. Publish pose array
7. Perform HSV color detection in center ROI
8. Publish color ("red"/"green"/"none")

**AprilTag Configuration:**
- Dictionary: DICT_APRILTAG_36H11
- Physical size: 10cm (configurable)
- Detection threshold: adjustable in DetectorParameters

**Color Detection (HSV):**
- Red: H: 0-10, S: 100-255, V: 100-255
- Green: H: 40-80, S: 50-255, V: 50-255
- Threshold: 500+ pixels to confirm detection

### 8.2 localization_node

**Purpose:** Estimate robot pose from AprilTag detections and IMU.

**Inputs:**
- `/aruco_detections` - Detected tag poses from camera
- `/imu_data` - IMU yaw angle
- Marker map (configured via parameters)

**Outputs:**
- `/robot_pose` - Estimated robot position (x, y, theta)

**Algorithm:**
1. Maintain marker map: ID → (x, y, z) world coordinates
2. For each AprilTag detection:
   - Extract translation and rotation from pose
   - Transform to world frame using tag orientation
   - Compute robot position relative to tag
3. Fuse multiple tag measurements for robustness
4. Use IMU yaw for orientation correction
5. Apply low-pass filter to smooth pose

**Marker Map Configuration:**
```yaml
marker_map:
  '0': [0.0, 0.0, 0.0]    # Home
  '1': [1.5, 0.0, 0.0]    # Manufacturing
  '2': [0.0, 1.5, 0.0]    # Storage A
  '3': [1.5, 1.5, 0.0]    # Storage B
```

### 8.3 navigation_node

**Purpose:** Navigate robot to waypoints using PID control.

**Inputs:**
- `/robot_pose` - Current robot position
- `/waypoint` - Target waypoint from task manager

**Outputs:**
- `/cmd_vel` - Velocity commands to ESP32

**PID Controller:**

For linear velocity (forward/backward):
- Kp: 2.0 (proportional gain)
- Ki: 0.1 (integral gain)
- Kd: 0.5 (derivative gain)

For angular velocity (turn):
- Kp: 3.0
- Ki: 0.2
- Kd: 1.0

**Algorithm:**
1. Compute distance to waypoint: d = √((x_w - x_r)² + (y_w - y_r)²)
2. Compute angle to waypoint: θ = atan2(y_w - y_r, x_w - x_r)
3. Compute relative angle: φ = θ - robot_yaw (normalized to [-π, π])
4. If angle error > 30°, stop and rotate (don't move forward)
5. Compute angular velocity using PID: ω = PID(0, φ)
6. Compute linear velocity using PID: v = PID(0, d)
7. Publish velocity command

**Waypoint threshold:** 0.15 meters (waypoint considered reached)
**Angle threshold:** 0.2 radians (for final orientation)

### 8.4 task_manager_node

**Purpose:** Execute the complete warehouse mission as a state machine.

**States:**
1. IDLE - Waiting for start command
2. GO_TO_MFG - Navigate to manufacturing station
3. DETECT_COLOR - Wait for camera to detect box color
4. PICK_BOX - Close gripper to grab box
5. GO_TO_STORAGE - Navigate to correct storage (A or B)
6. DEPOSIT_BOX - Open gripper to release box
7. RETURN_HOME - Navigate back to home position
8. TASK_COMPLETE - Mission finished

**Inputs:**
- `/box_color` - Detected color from camera_node
- `/navigation_status` - Waypoint reached confirmation
- `/robot_pose` - Current position (for logging)

**Outputs:**
- `/waypoint` - Target position for navigation_node
- `/gripper_cmd` - "open" or "close" commands

**Services:**
- `/start_task` - Start a new mission
- `/cancel_task` - Cancel current mission and return home

**Mission Sequence:**
```
start_task → GO_TO_MFG → DETECT_COLOR → PICK_BOX
           → GO_TO_STORAGE (A or B based on color)
           → DEPOSIT_BOX → RETURN_HOME → TASK_COMPLETE → IDLE
```

## ═══════════════════════════════════════════════════════════════════════════
## 9. CALIBRATION
## ═══════════════════════════════════════════════════════════════════════════

### 9.1 Camera Calibration

Camera calibration is required for accurate AprilTag pose estimation.
The calibration script `calibrate_camera.py` uses chessboard pattern.

#### Equipment Needed
- Chessboard printout (9x6 inner corners, 2cm square size)
- Good lighting
- Camera mounted on robot

#### Calibration Steps

1. **Print chessboard:**
   Generate or print a 9x6 chessboard pattern. Square size: 2.0 cm

2. **Run calibration script:**
   ```bash
   cd ~/micro_ros_ws/src/micro_ros_robot
   python3 ../../../calibrate_camera.py
   ```

3. **Capture frames:**
   - Hold chessboard flat
   - Capture ≥15 images from different angles
   - Press SPACE to capture
   - Press C to compute calibration

4. **Collected frames should include:**
   - Tilt variations (±30°)
   - Distance variations (0.5m to 2m)
   - Rotation variations (full 360°)
   - Position in all corners of frame

5. **Results:**
   - Mean reprojection error < 1.0 px is good
   - Camera matrix and distortion coefficients saved
   - JSON file: `data/camera_calibration/camera_calibration.json`

#### Calibration Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Pattern | 9x6 inner corners | Chessboard size |
| Square size | 2.0 cm | Physical size |
| Min frames | 15 | Minimum for calibration |
| Max frames | 50 | Maximum stored |
| Resolution | 640x480 | Camera capture |

### 9.2 IMU Calibration

IMU calibration happens automatically at ESP32 startup:
- Gyroscope Z-axis bias is calculated
- 200 samples collected over 1 second
- Robot must be stationary during calibration

**Expected output:**
```
[CALIB] Gyro Z, ne bouge pas...
[CALIB] bias=-1.23
```

If bias is unstable (>10), check IMU wiring or sensor.

### 9.3 Ultrasonic Sensor Calibration

The ultrasonic sensors are calibrated in firmware:
- Speed of sound: 0.034 cm/μs (at ~20°C)
- Valid range: 5-150 cm
- 5 samples averaged, outliers rejected

For precise distance, verify speed of sound adjustment for your environment:
```cpp
// In micro_ros_esp32Robot.ino
float speed_of_sound = 0.034;  // Adjust for temperature
```

## ═══════════════════════════════════════════════════════════════════════════
## 10. WEB INTERFACE (Flask)
## ═══════════════════════════════════════════════════════════════════════════

The Flask web interface provides a debug dashboard for monitoring robot state,
visualizing sensor data, and viewing live video streams.

### 10.1 Starting the Interface

```bash
cd ~/micro_ros_ws/src/micro_ros_robot
python3 flask_ros_bridge.py
```

Open browser at: `http://localhost:5000`

### 10.2 Video Streams

Three video streams are available:

| Stream | Endpoint | Description |
|--------|----------|-------------|
| **Robot Map** | `/map_feed` | SLAM map with robot position, trajectory, and ultrasonic sensors |
| **IMU 3D** | `/imu3d_feed` | Real-time 3D visualization of IMU orientation |
| **Camera Feed** | `/camera_feed` | Raw camera feed from `/image_raw` topic (~30 FPS) |

### 10.3 Dashboard Features

- **Sensor Health**: Real-time status of IMU, AprilTag detection, ultrasonics
- **Localization Confidence**: Visual confidence bar for pose estimation
- **Mission State**: Current mission state, target color, target tag
- **LSTM Status**: Enable/disable LSTM assistant, view prediction confidence
- **Manual Controls**: WASD movement, gripper open/close, emergency stop

### 10.4 REST API Endpoints

```bash
# Get robot state
curl http://localhost:5000/state

# Send velocity command
curl -X POST http://localhost:5000/cmd/1  # Forward

# Send mission control (LSTM)
curl -X POST http://localhost:5000/mission_ctrl \
  -H "Content-Type: application/json" \
  -d '{"action":"toggle_lstm"}'
```

## ═══════════════════════════════════════════════════════════════════════════
## 11. TROUBLESHOOTING
## ═══════════════════════════════════════════════════════════════════════════

### 11.1 ESP32 Issues

#### Problem: "i2c init fail" in Serial Monitor
**Cause:** IMU not responding
**Solutions:**
1. Check I2C wiring (SDA=21, SCL=22)
2. Verify IMU power (3.3V)
3. Run I2C scanner to find device address
4. Check for wire breakage

#### Problem: Motors not responding
**Cause:** PWM or direction pins not configured
**Solutions:**
1. Verify motor driver wiring
2. Check ENA/ENB PWM signals with oscilloscope
3. Test motor with simple Arduino sketch first

#### Problem: ESP32 not connecting to WiFi
**Cause:** Wrong credentials or network issues
**Solutions:**
1. Check SSID and password in sketch
2. Verify ESP32 can reach router
3. Try with static IP instead of DHCP

### 11.2 ROS2 Issues

#### Problem: "Failed to find executor" error
**Cause:** Python nodes missing dependencies
**Solutions:**
```bash
pip3 install opencv-python numpy cv-bridge sensor-msgs geometry-msgs std-msgs std-srvs
```

#### Problem: Nodes not communicating
**Cause:** Topic mismatch or QoS issue
**Solutions:**
1. Check `ros2 topic list` shows expected topics
2. Verify message types match
3. Use `ros2 topic info /topic_name` to check

#### Problem: micro_ros_agent not receiving from ESP32
**Cause:** Network or protocol mismatch
**Solutions:**
1. Verify ESP32 and Pi on same network
2. Check UDP port (8888) is not blocked
3. Run `ros2 topic echo /imu_data` to see if data arrives

### 11.3 Navigation Issues

#### Problem: Robot circles instead of going straight
**Cause:** Wheel speed imbalance or PID tuning
**Solutions:**
1. Calibrate left/right motor PWM values
2. Increase angular Kp in navigation_node
3. Check for mechanical issues (wheel alignment)

#### Problem: Robot overshoots waypoints
**Cause:** PID gains too high
**Solutions:**
1. Reduce Kp for both linear and angular
2. Increase Kd (derivative damping)
3. Lower max speed

#### Problem: Robot doesn't face waypoint before moving
**Cause:** Angle threshold too high or PID issue
**Solutions:**
1. Check angle threshold parameter (default: 30°)
2. Reduce angle threshold to 20°
3. Verify robot yaw from IMU is correct

### 11.4 AprilTag Detection Issues

#### Problem: Tags not detected
**Cause:** Camera settings, lighting, or tag quality
**Solutions:**
1. Ensure good lighting (no shadows)
2. Increase camera exposure
3. Check tag size matches configuration (10cm default)
4. Verify camera calibration is loaded

#### Problem: Pose estimation inaccurate
**Cause:** Camera not calibrated or wrong tag size
**Solutions:**
1. Recalibrate camera with chessboard
2. Verify tag physical size in config
3. Use higher resolution camera

## ═══════════════════════════════════════════════════════════════════════════
## 12. DEVELOPMENT
## ═══════════════════════════════════════════════════════════════════════════

### 12.1 Adding New Nodes

To add a new ROS2 node:

1. Create Python file in `micro_ros_robot/scripts/`:
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # Add publishers/subscribers

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2. Update `setup.py`:
```python
entry_points={
    'console_scripts': [
        'my_node = micro_ros_robot.my_node:main',
    ],
},
```

3. Add to launch file if needed

### 12.2 Debug Tools

```bash
# Record all topics to bag file
ros2 bag record -a

# Replay bag file
ros2 bag play bag_name/

# Plot topic data
ros2 topic echo /imu_data --format csv > data.csv
```

### 12.3 Adding AprilTag Markers

To add markers to the environment:

1. Print AprilTag 36h11 markers
2. Measure marker positions in world frame
3. Update marker_map in localization_node parameters

### 12.4 Modifying PID Values

To tune PID controllers, open `navigation_node.py` and modify parameters:

```python
# For more aggressive control
self.pid_linear = PIDController(kp=3.0, ki=0.2, kd=0.8, output_limit=0.3)
self.pid_angular = PIDController(kp=4.0, ki=0.3, kd=1.5, output_limit=2.0)
```

### 12.5 Future Improvements

- [ ] Add obstacle avoidance using ultrasonic data
- [ ] Implement SLAM for unknown environments
- [ ] Add path planning with obstacle maps
- [ ] Implement multi-robot coordination
- [ ] Add battery monitoring
- [ ] Implement emergency stop

## ═══════════════════════════════════════════════════════════════════════════
## 13. ENHANCED FEATURES (v2.0)
## ═══════════════════════════════════════════════════════════════════════════

### 13.1 Fault Tolerance

The system now includes comprehensive fault tolerance:

**RobotState** (`robot_state.py`):
- Maintains belief about robot pose with confidence metric
- Last Known Good State (LKGS) for each sensor
- Automatic fallback when sensors fail

**SensorManager** (`sensor_manager.py`):
- Real-time sensor health monitoring
- Automatic failure detection (timeout, noise, stuck)
- Median Absolute Deviation filtering for noisy sensors

**FaultRecovery** (`fault_recovery.py`):
- State machine: NORMAL → DEGRADED → FAULT_RECOVERY → ABORT
- Automatic retry with fallback strategies
- Manual reset capability

### 13.2 Smart Gripper

The gripper now uses ultrasonic sensors for verified picking:

**Calibration:**
1. Place box against closed gripper
2. `ros2 service call /calibrate_gripper std_srvs/Empty "{}"`
3. Robot learns max open distance

**Verified Pickup:**
1. Pre-pick: ultrasonic verifies box position
2. Close gripper
3. Post-pick: ultrasonic confirms box between jaws
4. Retry once on failure, then abort

### 13.3 Obstacle Detection

Navigation now uses ultrasonic for obstacle avoidance:

```bash
# Emergency stop
ros2 service call /emergency_stop std_srvs/Empty "{}"
```

Obstacles within 20cm: immediate stop
Obstacles within 50cm: slow to 50% speed

### 13.4 Auto-Scan Mode

Robot can autonomously scan for AprilTags:

```bash
# Scan for 4 markers
ros2 service call /start_scan std_srvs/Empty "{expected_count: 4}"
```

Robot rotates slowly, accumulates detections, builds marker map.

### 13.5 Box Orientation Detection

Camera now detects box orientation (horizontal/vertical):

```bash
# Watch box info
ros2 topic echo /box_info
# {"color": "red", "orientation": "horizontal", "distance": 0.45, "confidence": 0.8}
```

### 13.6 Confidence-Weighted Localization

Pose estimation now includes confidence:

```bash
# Watch localization confidence
ros2 topic echo /localization_confidence
# 0.85 (decays when AprilTag not detected)
```

Lower confidence → slower navigation speed

## ═══════════════════════════════════════════════════════════════════════════
## 14. NEW TOPICS AND SERVICES
## ═══════════════════════════════════════════════════════════════════════════

| Topic/Service | Type | Description |
|---------------|------|-------------|
| `/sensor_health` | std_msgs/String | JSON sensor status |
| `/localization_confidence` | Float32 | Pose confidence 0-1 |
| `/robot_state_status` | std_msgs/String | State summary |
| `/robot_mode` | std_msgs/String | NORMAL/DEGRADED/FAULT/ABORT |
| `/box_info` | std_msgs/String | Color+orientation+distance |
| `/scan_status` | std_msgs/String | Auto-scan progress |
| `/gripper_status` | std_msgs/String | Gripper state JSON |
| `/calibrate_gripper` | service | Trigger gripper calibration |
| `/gripper_pick` | service | Execute verified pick |
| `/start_scan` | service | Start auto-scan |
| `/emergency_stop` | service | Immediate stop |

## ═══════════════════════════════════════════════════════════════════════════
## 15. FILE STRUCTURE (v2.0)
## ═══════════════════════════════════════════════════════════════════════════

```
Micro_ROS_Project/
├── esp32_firmware/
│   └── micro_ros_esp32Robot.ino   # Enhanced: gripper feedback, tilt, health
│
├── ubuntu_ros2/
│   ├── micro_ros_robot/
│   │   └── scripts/
│   │       ├── robot_state.py          # NEW: Central state management
│   │       ├── sensor_manager.py        # NEW: Sensor monitoring
│   │       ├── fault_recovery.py       # NEW: Fault handling state machine
│   │       ├── smart_gripper.py         # NEW: Verified gripper control
│   │       ├── auto_scan.py            # NEW: Autonomous scanning
│   │       ├── camera_node.py          # Enhanced: BoxDetector
│   │       ├── localization_node.py    # Enhanced: confidence tracking
│   │       ├── navigation_node.py      # Enhanced: obstacle detection
│   │       └── task_manager_node.py    # Enhanced: integration
│   │
│   ├── test_robot_state.py              # NEW: Unit tests
│   ├── test_sensor_manager.py           # NEW: Unit tests
│   ├── test_fault_recovery.py           # NEW: Unit tests
│   ├── test_smart_gripper.py            # NEW: Unit tests
│   │
│   └── README.md                         # Enhanced
│
├── docs/
│   └── CALIBRATION.md                   # Enhanced: gripper calibration
│
├── SPEC.md                              # NEW: Full specification
└── README.md                           # Enhanced: v2.0 features
```

## ═══════════════════════════════════════════════════════════════════════════

**Project:** Micro-ROS Autonomous Mobile Robot
**Version:** 2.0.0
**Date:** May 2026
**License:** MIT