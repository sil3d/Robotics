# ESP32 + Raspberry Pi Autonomous Robot

Autonomous warehouse robot using ESP32 (low-level control) + Raspberry Pi (high-level AI).
SLAM localization with AprilTags, A* pathfinding, color-based cube sorting.

## Quick Start

```bash
# Flash ESP32 firmware via Arduino IDE
# Then on RPi/PC:
cd Micro_ROS_Project/ubuntu_ros2
python3 mission_engine.py
```

## Architecture

```
┌─────────────────────────────────────────────┐
│          Raspberry Pi / PC (ROS2)           │
│                                             │
│  mission_engine.py ← state machine + A*    │
│    ├── RobotTracker (SLAM)                  │
│    ├── ArduinoReader (IMU via USB)          │
│    ├── AprilTagDetector (camera)            │
│    ├── ColorDetector (blue/green cubes)     │
│    └── LSTMAssistant (advisory)             │
│                                             │
│  task_manager_node.py ← ROS2 ↔ engine      │
│  navigation_node.py ← PID waypoint follower│
│  flask_ros_bridge.py ← Web UI (port 5000)  │
│  camera_node.py ← ROS2 camera node         │
└──────────────────┬──────────────────────────┘
                   │ USB Serial
┌──────────────────┴──────────────────────────┐
│              ESP32 (micro-ROS)              │
│  PID motors │ BMI160 IMU │ 4x Ultrasons    │
│  Servo gripper │ WiFi AP                    │
└─────────────────────────────────────────────┘
```

## Mission Cycle

```
HOME(12) → Manufacture(3) → Station B(6)/A(9) → Manufacture(3) → Station A(9)/B(6) → HOME(12)
```

2 cubes per cycle (blue + green), A* pathfinding between 12 AprilTag waypoints.

## Project Structure

| Directory | Description |
|-----------|-------------|
| `Micro_ROS_Project/` | Main robot: ESP32 firmware + ROS2 nodes + mission engine |
| `Micro_ROS_Project/ubuntu_ros2/mission_engine.py` | Core state machine with SLAM + A* |
| `Micro_ROS_Project/ubuntu_ros2/navigation_node.py` | ROS2 PID waypoint follower (reads `robot_config.json`) |
| `Micro_ROS_Project/ubuntu_ros2/task_manager_node.py` | ROS2 bridge between mission engine and firmware |
| `Micro_ROS_Project/esp32_firmware/` | ESP32 Arduino firmware |
| `test_PID_auto/` | PID tuning + RL drive assist (Flask web UI) |
| `data/` | Config files: calibration, tag maps, reference markers, **robot_config.json** (global PID/trims/navigation) |
| `auto_detetc_tag_arduino.py` | SLAM library: ArduinoReader, AprilTagDetector, RobotTracker |
| `color_detection_test.py` | HSV color detector (blue/green/red) |
| `april_tag_pose.py` | Standalone AprilTag 3D pose visualizer |

## Documentation

- [AGENT.md](AGENT.md) — Guide for AI agents working on this codebase
- [CHANGELOG.md](CHANGELOG.md) — Version history
- [Micro_ROS_Project/README.md](Micro_ROS_Project/README.md) — Full ROS2 architecture docs
- [test_PID_auto/README.md](test_PID_auto/README.md) — PID + RL drive assist docs
- [test_PID_auto/GUIDE_REGLAGE.md](test_PID_auto/GUIDE_REGLAGE.md) — PID tuning guide

## Tests

```bash
cd Micro_ROS_Project/ubuntu_ros2
python test_pid_controller.py   # 6 tests PIDController
python test_astar_path.py       # 8 tests A* pathfinding
python test_fault_recovery.py
python test_robot_state.py
python test_sensor_manager.py
python test_smart_gripper.py
```

## Key Specs

| Spec | Value |
|------|-------|
| AprilTag dictionary | `cv2.aruco.DICT_4X4_250` |
| Tag count | 12 markers |
| Cube colors | Blue (cyan) + Green |
| Pathfinding | A* (graphe complet) |
| Localization | Camera + IMU + Optical Flow (SLAM) |
| Camera backend (Windows) | `cv2.CAP_DSHOW` |
| PID defaults | Kp=4.0 Ki=0.02 Kd=0.7 (in `data/robot_config.json`) |
| Motor minPWM | 55 (both motors) |
| Obstacle stop | 8 cm (unified: `STOP_DIST_CM` = `obstacle_threshold_near`) |
| Navigation PID | Linear Kp=2.0 Ki=0.1 Kd=0.5 / Angular Kp=3.0 Ki=0.2 Kd=1.0 |
| Max cmd_vel | linear=0.20 m/s, angular=1.50 rad/s |
| Version | 8.0 |

## Author

**Prince Gildas Mbama Kombila**
