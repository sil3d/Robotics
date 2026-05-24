# AGENT.md — Guide for AI Agents

## Project Overview

This is an **autonomous warehouse robot** project using ESP32 + Raspberry Pi + ROS2. The robot picks up colored cubes (blue/green) at a manufacturing station and delivers them to the correct drop station using AprilTag SLAM localization and A* pathfinding.

## Repository Structure

```
Robotics/
├── auto_detetc_tag_arduino.py    # SLAM: ArduinoReader, AprilTagDetector, TagMapSLAM, RobotTracker
├── april_tag_pose.py             # Standalone AprilTag 3D pose estimator (RPi/PC)
├── april_tag_slam.py             # Standalone SLAM with optical flow + IMU
├── april_tag_localisation.py     # Tag localisation with IMU
├── calibrate_camera.py           # Camera calibration (chessboard)
├── calibrate_markers.py          # Marker calibration tool
├── color_detection_test.py       # HSV color detector (blue/green/red) + ArucoDetector
├── hsv_tuner.py                  # HSV tuning tool
├── data/
│   ├── camera_calibration/camera_calibration.json
│   ├── reference_markers/reference_markers.json   # 12 tags: roles + positions
│   ├── tags_slam/tags_slam.json                   # SLAM map (positions from scan)
│   ├── robot_config.json                          # Global config: PID/trims/minPWM/ramp/ia_trims
│   └── marker_positions/
├── Micro_ROS_Project/
│   ├── esp32_firmware/micro_ros_esp32Robot.ino    # ESP32 firmware (PID + IMU + US + gripper)
│   └── ubuntu_ros2/
│       ├── mission_engine.py      # Main state machine (SLAM + A* + missions)
│       ├── navigation_node.py     # ROS2 PID waypoint follower (reads robot_config.json)
│       ├── task_manager_node.py   # ROS2 bridge mission engine ↔ firmware
│       ├── camera_node.py         # ROS2 camera node
│       ├── flask_ros_bridge.py    # Web UI bridge
│       ├── calibrate_camera.py    # Camera calibration for ROS2
│       ├── launch_micro_ros.sh    # Launch script
│       ├── lstm_assistant.py      # LSTM advisory system
│       ├── test_pid_controller.py # Unit tests: PIDController
│       └── test_astar_path.py     # Unit tests: astar_path + path_to_waypoints
├── test_PID_auto/
│   ├── app.py                     # Flask PID control + RL drive assist
│   ├── arduino/arduino.ino        # ESP32 PID firmware
│   └── templates/index.html       # Web UI
└── README.md                      # Main project README
```

## Key Architecture Decisions

### Localization (Camera + IMU)
- **AprilTag dictionary**: `cv2.aruco.DICT_4X4_250` (ALL files must use this)
- **Camera backend on Windows**: `cv2.CAP_DSHOW` (DirectShow) — avoids blocking
- **Tracker**: `RobotTracker` from `auto_detetc_tag_arduino.py` fuses:
  - Tag detection (absolute position when tag visible)
  - Optical Flow (dead reckoning between tags)
  - IMU yaw (orientation from BMI160)
- **Units**: Tracker uses **cm**, mission_engine converts to **meters** (`/ 100.0`)

### Tag Layout (12 markers)
| Tag ID | Role | Position (cm) |
|--------|------|---------------|
| 12 | HOME | (0, 0) |
| 3 | Manufacture (pickup) | (0, 70) |
| 6 | Station B (blue drop) | (60, 55) |
| 9 | Station A (green drop) | (-60, 30) |
| 1,2 | North wall | left/right |
| 4,7,11 | West wall | top → bottom |
| 5,6,8,10 | East wall | top → bottom |

### Mission Cycle
```
SCAN_360 (SLAM) → NAVIGATE_WAYPOINT (A* → Manufacture)
  → DETECT_CUBE (blue or green) → CLOSE_GRIPPER
  → NAVIGATE_WAYPOINT (A* → Station B or A) → RELEASE
  → NAVIGATE_WAYPOINT (A* → Manufacture) → [2nd cube]
  → NAVIGATE_WAYPOINT (A* → HOME) → RECORD → new cycle
```

### A* Pathfinding
- **Graphe complet** of all 12 tags (every tag connected to every other)
- Weight = Euclidean distance
- Waypoint-by-waypoint navigation with absolute localization

## Rules for AI Agents

1. **AprilTag dictionary**: ALWAYS use `cv2.aruco.DICT_4X4_250`. Never use `DICT_APRILTAG_36H11` or `DICT_4X4_50`.
2. **Camera on Windows**: ALWAYS use `cv2.CAP_DSHOW` backend. Default backend blocks.
3. **Colors**: There are TWO cube colors: **blue** (cyan) and **green**. Cyan IS blue.
4. **Language**: Code comments in English. User communication in French.
5. **Don't break imports**: `auto_detetc_tag_arduino.py` exports `ArduinoReader`, `AprilTagDetector`, `TagMapSLAM`, `RobotTracker`, `build_T_robot_cam`.
6. **Units**: Tracker = cm, mission_engine = meters. Always convert with `/100.0`.
7. **Arduino is optional**: `_DummyArduino` fallback if not connected. Never crash on missing Arduino.
8. **data/ folder**: All JSON configs go in `data/` subfolders. Never hardcode positions — load from JSON.
9. **Global robot config**: `data/robot_config.json` is the single source of truth for PID/trims/minPWM/ramp/ia_trims/**navigation**. Both `test_PID_auto/app.py`, `flask_ros_bridge.py`, and `navigation_node.py` load it at startup. Never hardcode these values.
10. **Yaw source**: `task_manager_node` uses SLAM tracker yaw when `tracker.initialized`, IMU yaw only as fallback. Never let both override each other concurrently.
11. **cmd_vel arbitrage**: Only `task_manager_node` (mission) and `navigation_node` (waypoint) publish on `/cmd_vel`. `flask_ros_bridge /velocity` is blocked (HTTP 409) while a mission is running.
12. **PID dt**: `PIDController.compute()` uses real `time.perf_counter()` dt. Do not hardcode `dt=0.05`.
13. **Firmware scaling**: `cmdVelCallback` divisors are `0.20` (linear) and `1.50` (angular) — matching `navigation_node` limits. Do not change one without the other.

## test_PID_auto Subsystem

This is the **manual control + RL drive assist** subsystem, separate from the autonomous mission engine.

### Architecture
```
Flask app (app.py) → WebSocket → ESP32 (arduino.ino)
     ↕                              ↕
  Web UI (index.html)          Motors + IMU + Ultrasons
     ↕
  RL Agent (industrial_ai.py)
```

### Key Files
- `test_PID_auto/app.py` — Flask server, handles WiFi/USB connection to ESP32, IA loop
- `test_PID_auto/arduino/arduino.ino` — ESP32 firmware: PID yaw, ramp acceleration, ultrasons, gripper, WiFi AP
- `test_PID_auto/templates/index.html` — Web UI with D-Pad, speed slider, PID tuning, IA toggle, trajectory view
- `test_PID_auto/industrial_ai.py` — UnifiedRLAgent (Deep RL DDPG), PC training + Pi inference

### ESP32 WiFi AP
- SSID: `ROBOT_WIFI`
- Password: `robot1234`
- IP: `192.168.4.1`
- WebSocket: `ws://192.168.4.1/ws`

### Motor Control (L298N)
- PWM range: 0–255 (8-bit)
- `speedMax = 255` (hardcoded in firmware)
- `MOTOR_A_MINPWM / MOTOR_B_MINPWM = 55` (minimum to start on ground — 35 is too low, works only in air)
- Ramp acceleration: `rampSpeed=250 PWM/sec`, `rampBrake=350`, `rampNeutral=200`

### JSON Commands
```json
{"t":"dir","x":0.0,"y":0.6}                          // x: rotation, y: speed (-1 to 1)
{"t":"cfg","ykp":4,"yki":0.02,"ykd":0.7}             // PID tuning
{"t":"cfg","ta":0,"tb":0,"ma":55,"mb":55}             // trims + MINPWM per motor
{"t":"ia","tl":3.5,"tr":-1.2,"rbst":-0.15}           // IA corrections
{"t":"save"}                                          // Save to EEPROM
```

### Global Config (robot_config.json)
```json
{
  "pid":     {"kp": 4.0, "ki": 0.02, "kd": 0.7},
  "trims":   {"a": 0.0, "b": 0.0},
  "minpwm":  {"a": 55.0, "b": 55.0},
  "ramp":    {"speed": 80.0, "brake": 120.0, "neutral": 200.0},
  "ia_trims":{"L": 0.0, "R": 0.0, "boost": 0.0},
  "navigation": {
    "max_linear_speed": 0.20, "max_angular_speed": 1.50,
    "pid_linear_kp": 2.0, "pid_linear_ki": 0.1, "pid_linear_kd": 0.5,
    "pid_angular_kp": 3.0, "pid_angular_ki": 0.2, "pid_angular_kd": 1.0,
    "obstacle_threshold_near": 0.08, "obstacle_threshold_far": 0.15
  }
}
```
Read by `test_PID_auto/app.py` at startup. Written on Save. Read by `flask_ros_bridge.py` and pushed to `/robot_cfg` at boot. Section `navigation` read by `navigation_node.py` via `_apply_robot_config()` at node startup.

### Speed Control UI
- Speed slider (10–100%) → `sendDirection()` sends `{t:"dir", x, y}`
- D-Pad: rotation is **independent** from speed (rotRatio = 40% combined, 50% alone)
- Changing slider while holding a key **re-sends** the command immediately

### IA (Deep RL)
- PC: training mode (PyTorch, background training thread active)
- Pi: inference-only mode — `_is_raspberry_pi()` detects ARM → `INFERENCE_ONLY=True` → training thread disabled to free CPU
- Save frequency: every 300 steps (~15s at 20Hz) to reduce SD card wear (was 50 steps = 2.5s)
- Learns: trim_L/R (motor compensation), ramp_boost (acceleration profile)
- Model saved to: `drive_assist_model.pt` (full float32)
- Export for Pi: `drive_assist_rpi_int8.pt` (INT8 TorchScript, generated by `export_rpi.py`)
- **Auto-save on Save button**: 4 steps in sequence: EEPROM → robot_config.json → .pt → INT8 export (background)

### Common Issues
- `torch.load weights_only=True` fails with old models → use `weights_only=False`
- Robot doesn't start on ground: increase MINPWM (`{"t":"cfg","ma":55,"mb":55}`)
- WiFi connection timeout: ESP32 WiFi AP is separate from PC network — connect PC to `ROBOT_WIFI`
- `drive_assist_rpi_int8.pt` missing on Pi: click Save in `test_PID_auto` UI (auto-exports) or run `python export_rpi.py` manually
- Config not applied to micro-ROS firmware: check `robot_config.json` exists and `flask_ros_bridge.py` was restarted

## Common Pitfalls

- `cv2.CAP_DSHOW` doesn't appear in Pyright stubs — it's a false positive, works at runtime
- `build_T_robot_cam()` is called inside `RobotTracker.__init__()` — the import is needed transitively
- `_step_scan()` calls `self.tag_map.tags.clear()` only if the map is empty — if tags are already known (prior map), they are preserved and merged
- The tracker resets to (0,0) after scan — tag positions are relative to scan origin
- `NAVIGATE_TAG`, `NAVIGATE_DROP`, `BACK_HOME` have been **removed** from `State` — all navigation goes through `NAVIGATE_WAYPOINT`
- Motor MINPWM 35 is too low to start on ground — use 55 minimum
- `_step_detect_cube()` has a 15s timeout → `State.ERROR` if no cube found; `_detect_cube_start` is reset by `_step_open_gripper()`
- `PIDController` anti-windup limit is dynamic: `min(5.0, output_limit / ki)` — not a fixed ±50
- `task_manager_node` IMU callbacks are throttled to 20 Hz — don't call `engine.update_sensors()` at raw IMU rate (100Hz)

## Running

### Autonomous Mission (SLAM + A*)
```bash
# ESP32: flash Micro_ROS_Project/esp32_firmware/micro_ros_esp32Robot.ino
# RPi/PC:
cd Micro_ROS_Project/ubuntu_ros2
python3 mission_engine.py
# or via launch script:
./launch_micro_ros.sh
```

### Manual Control + IA Training
```bash
# ESP32: flash test_PID_auto/arduino/arduino.ino
# Connect PC to WiFi "ROBOT_WIFI" / robot1234
cd test_PID_auto
python app.py
# → Open http://localhost:5000
```

**NOTE**: The two firmwares are NOT interchangeable. micro_ros firmware uses Serial, test_PID_auto uses WiFi AP.
