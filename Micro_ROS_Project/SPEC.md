# SPEC.md - Enhanced Micro-ROS Robot System Specification

## Context

This document describes the enhanced fault-tolerant and intelligent control system added to the Micro-ROS autonomous mobile robot project.

## 1. System Architecture

### 1.1 Overview

The enhanced system adds:
- **RobotState**: Central state management with Last Known Good State (LKGS)
- **SensorManager**: Real-time sensor health monitoring and failure detection
- **FaultRecovery**: State machine for graceful degradation and recovery
- **SmartGripperController**: Sensor-verified gripper operations
- **BoxDetector**: Box orientation and distance detection
- **AutoScanMode**: Autonomous AprilTag mapping

### 1.2 Component Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    TASK_MANAGER_NODE                        │
│  - State machine with FAULT_RECOVERY, ABORT states           │
│  - Integrates all components                                 │
└───────────┬─────────────────────────────────────────────────┘
            │
    ┌───────┴───────┐
    │               │
┌───▼───────┐   ┌────▼────┐
│RobotState │   │FaultRec │
│- LKGS     │   │- Retry  │
│- Confidence   │- Fallback│
└───┬───────┘   └────┬────┘
    │               │
┌───▼─────────┐ ┌───▼────────────┐
│Localization  │ │Navigation     │
│Node          │ │- Obstacle det │
│- Confidence  │ │- Speed mod   │
│- LKGS pose   │ │- Speed mod   │
└───┬──────────┘ └────────────────┘
         │              │
         │    ┌─────────┴─────────┐
         │    │                   │
    ┌────▼────▼────┐    ┌────────▼────────┐
    │SensorManager  │    │ SmartGripper     │
    │- IMU health   │    │ - Calibration    │
    │- US variance  │    │ - Verified pick  │
    │- Confidence   │    │ - Grab detect    │
    └───────────────┘    └──────────────────┘
```

## 2. New Components

### 2.1 RobotState (robot_state.py)

**Purpose:** Central state management for robot belief.

**Features:**
- Pose (x, y, theta) with confidence metric (0.0-1.0)
- Last Known Good State (LKGS) for each sensor
- Gripper state (position, has_box, confidence)
- Fault state tracking

**Key Methods:**
- `update_pose(x, y, theta, confidence)` - Update with LKGS
- `get_last_known_good_pose()` - Retrieve LKGS when current fails
- `update_sensor_health(name, healthy, confidence)` - Track sensor health
- `get_overall_confidence()` - Weighted average from all sensors

### 2.2 SensorManager (sensor_manager.py)

**Purpose:** Real-time sensor monitoring and failure detection.

**Monitors:**
- IMU: timeout (100ms), drift detection
- Ultrasonics: stuck sensor, noise (MAD filter), out of range
- Camera/AprilTag: detection timeout, rejection rate

**Publishes:** `/sensor_health` (JSON)

**Confidence Calculation:**
```
IMU: 30% weight
Ultrasonics: 30% weight
AprilTag: 40% weight
```

### 2.3 FaultRecovery (fault_recovery.py)

**Purpose:** Handle sensor failures with recovery strategies.

**States:**
- NORMAL → DEGRADED (confidence < 0.7)
- DEGRADED → FAULT_RECOVERY (confidence < 0.3)
- FAULT_RECOVERY → NORMAL (recovery success)
- FAULT_RECOVERY → ABORT (max retries exceeded)

**Recovery Actions:**
| Fault | Retry | Fallback | Abort After |
|-------|-------|----------|-------------|
| IMU timeout | 2s delay | AprilTag-only | 3 retries |
| AprilTag timeout | 3s delay | IMU dead reckoning | 3 retries |
| Ultrasonic stuck | Skip | Ignore sensor | 5 retries |
| Navigation error | 1s delay | Use LKGS | 2 retries |

### 2.4 SmartGripperController (smart_gripper.py)

**Purpose:** Intelligent gripper with sensor verification.

**Calibration:**
1. User places box against closed gripper
2. Trigger `/calibrate_gripper` service
3. Ultrasonic measures distance = max_open

**Verified Pickup:**
1. Pre-pick: ultrasonic verifies box position (5-30cm)
2. Close gripper
3. Post-pick: ultrasonic confirms box still between jaws
4. If fail: retry once, then abort

**States:**
- OPEN → CLOSING → CLOSED → OPENING → OPEN
- ERROR on failed pickup

### 2.5 BoxDetector (camera_node.py enhancement)

**Purpose:** Detect box orientation and estimate distance.

**Features:**
- Contour analysis for horizontal/vertical detection
- AprilTag-based distance estimation
- Pixel size fallback for distance

**Output:** `/box_info` (JSON)
```json
{
  "color": "red",
  "orientation": "horizontal",
  "distance": 0.45,
  "confidence": 0.8
}
```

### 2.6 AutoScanMode (auto_scan.py)

**Purpose:** Autonomous room scanning for AprilTag mapping.

**Trigger:** `/start_scan` service (expects marker count)

**Behavior:**
1. Rotate robot slowly (0.3 rad/s)
2. Accumulate AprilTag detections with robot pose
3. Build world positions from multiple views
4. Save to `data/marker_map_auto/marker_map_auto.json`

**Parameters:**
- `expected_marker_count`: User specifies number
- `scan_timeout`: 30 seconds
- `confidence_threshold`: 3 detections to confirm marker

## 3. Enhanced Existing Nodes

### 3.1 navigation_node.py - Obstacle Detection

**New Subscriptions:**
- `/ultrasonic_data` - Front obstacle detection

**New Parameters:**
- `obstacle_threshold_near`: 0.20m (stop)
- `obstacle_threshold_far`: 0.50m (slow to 50%)
- `obstacle_slowdown_factor`: 0.5

**New Service:**
- `/emergency_stop` - Immediate stop

### 3.2 localization_node.py - Confidence Tracking

**New Features:**
- Pose confidence with decay
- IMU-only mode when AprilTag fails
- Last Known Good State (LKGS) usage
- Sensor fusion weighting

**New Publisher:**
- `/localization_confidence` (Float32)

### 3.3 task_manager_node.py - Integration

**New States:**
- AUTO_SCAN - Room scan before mission
- VERIFY_PICKUP - SmartGripper verification
- GRIPPER_RETRY - Retry failed grip
- FAULT_RECOVERY - Fault handling
- ABORT - Manual intervention needed

**New Subscriptions:**
- `/sensor_health` - Fault detection
- `/robot_state_status` - State updates
- `/scan_status` - Auto-scan completion

## 4. ESP32 Firmware Enhancements

**New Features:**
1. **Gripper position tracking** - `gripper_commanded` variable
2. **Tilt detection** - `tilt_pitch` calculated from accelerometer
3. **Ultrasonic variance tracking** - `usVariance[4]` EWMA
4. **Sensor health publisher** - `/sensor_health` at 1Hz

**Message Changes:**
- `/imu_data.linear.z` = tilt_pitch (degrees, -90 to 90)
- `/sensor_health` = JSON with us_variance, gripper_pos, imu_healthy

## 5. Topic Summary

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/sensor_health` | std_msgs/String | SensorManager → all | JSON health status |
| `/localization_confidence` | std_msgs/Float32 | localization → all | Pose confidence 0-1 |
| `/robot_state_status` | std_msgs/String | RobotState → all | State summary |
| `/recovery_status` | std_msgs/String | FaultRecovery → all | Recovery status |
| `/robot_mode` | std_msgs/String | FaultRecovery → all | NORMAL/DEGRADED/FAULT/ABORT |
| `/box_info` | std_msgs/String | camera_node | Color+orientation+distance |
| `/scan_status` | std_msgs/String | AutoScan → all | Scan progress |
| `/gripper_status` | std_msgs/String | SmartGripper → all | Gripper state JSON |

## 6. New Services

| Service | Type | Description |
|---------|------|-------------|
| `/calibrate_gripper` | std_srvs/Empty | Trigger gripper calibration |
| `/gripper_pick` | std_srvs/Empty | Execute verified pick |
| `/gripper_release` | std_srvs/Empty | Release box |
| `/start_scan` | std_srvs/Empty | Start auto-scan (param: expected_count) |
| `/reset_recovery` | std_srvs/Empty | Reset from abort |
| `/emergency_stop` | std_srvs/Empty | Immediate stop |

## 7. Usage Examples

### 7.1 Start Mission with Auto-Scan
```bash
# Start auto-scan with 4 expected markers
ros2 service call /start_scan std_srvs/Empty "{expected_count: 4}"

# Start mission
ros2 service call /start_task std_srvs/Empty "{}"
```

### 7.2 Calibrate Gripper
```bash
# Place box against gripper, then call
ros2 service call /calibrate_gripper std_srvs/Empty "{}"
```

### 7.3 Monitor System Health
```bash
# Watch sensor health
ros2 topic echo /sensor_health

# Watch recovery status
ros2 topic echo /robot_mode

# Watch localization confidence
ros2 topic echo /localization_confidence
```

### 7.4 Emergency Stop
```bash
ros2 service call /emergency_stop std_srvs/Empty "{}"
```

## 8. Configuration Parameters

### 8.1 Fault Recovery
- `confidence_threshold_degraded`: 0.7 (→ DEGRADED)
- `confidence_threshold_recovery`: 0.3 (→ FAULT_RECOVERY)
- `max_recovery_attempts`: 3
- `lkgs_max_age_imu`: 10s
- `lkgs_max_age_apriltag`: 30s

### 8.2 Navigation
- `obstacle_threshold_near`: 0.20m
- `obstacle_threshold_far`: 0.50m
- `obstacle_slowdown_factor`: 0.5

### 8.3 AutoScan
- `expected_marker_count`: 4
- `scan_rotation_speed`: 0.3 rad/s
- `scan_timeout`: 30s
- `confidence_threshold`: 3 detections

### 8.4 SmartGripper
- `pick_verify_timeout`: 2s
- `max_retries`: 1
- `box_distance_min`: 0.05m
- `box_distance_max`: 0.30m