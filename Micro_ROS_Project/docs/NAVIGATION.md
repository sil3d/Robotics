# Navigation System Documentation
## Micro-ROS Autonomous Mobile Robot

**Version:** 1.0.0
**Date:** May 2026

---

## Table of Contents

1. [Overview](#1-overview)
2. [Differential Drive Kinematics](#2-differential-drive-kinematics)
3. [PID Control Theory](#3-pid-control-theory)
4. [Navigation Algorithm](#4-navigation-algorithm)
5. [Parameter Tuning](#5-parameter-tuning)
6. [Waypoint Management](#6-waypoint-management)
7. [Obstacle Handling](#7-obstacle-handling)
8. [Error Recovery](#8-error-recovery)

---

## 1. Overview

The navigation system uses PID (Proportional-Integral-Derivative) control to navigate
the robot from its current position to a target waypoint.

### 1.1 System Components

```
┌─────────────────────────────────────────────────────────────┐
│                    NAVIGATION SYSTEM                         │
│                                                             │
│  ┌─────────────┐    ┌──────────────┐    ┌─────────────────┐  │
│  │ Waypoint   │───▶│   PID        │───▶│   cmd_vel       │  │
│  │ (x, y)    │    │  Controller │    │   (Twist)       │  │
│  └─────────────┘    └──────────────┘    └─────────────────┘  │
│                          │                      │             │
│                          ▼                      ▼             │
│                   ┌──────────────┐    ┌─────────────────┐  │
│                   │   Robot      │◀───│   robot_pose    │  │
│                   │   State     │    │   (feedback)    │  │
│                   └──────────────┘    └─────────────────┘  │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 Control Loop

```python
while waypoint_active:
    # 1. Get current state
    current_pose = read_robot_pose()

    # 2. Compute errors
    distance_error = compute_distance(current_pose, waypoint)
    angle_error = compute_angle(current_pose, waypoint)

    # 3. PID control
    angular_vel = pid_angular.compute(0, angle_error)
    linear_vel = pid_linear.compute(0, distance_error)

    # 4. Apply velocity
    if abs(angle_error) < ANGLE_THRESHOLD:
        publish_cmd_vel(linear_vel, angular_vel)
    else:
        publish_cmd_vel(0, angular_vel)  # Rotate only
```

---

## 2. Differential Drive Kinematics

### 2.1 Robot Model

```
            Robot
         ┌────────┐
         │   ▲    │  ← forward direction
         │  ╱ ╲   │
         │ L   R  │  ← wheels (Left, Right)
         │   ▼    │
         └────────┘

    Wheel separation: L (wheelbase length)
```

### 2.2 Forward Kinematics

**Input:** Wheel velocities (vL, vR)
**Output:** Robot velocities (v, ω)

```python
def forward_kinematics(vL, vR, wheel_dist):
    """
    Convert wheel velocities to robot velocities.

    v = (vL + vR) / 2          # linear velocity
    ω = (vR - vL) / L           # angular velocity
    """
    v = (vL + vR) / 2.0
    omega = (vR - vL) / wheel_dist
    return v, omega
```

### 2.3 Inverse Kinematics

**Input:** Robot velocities (v, ω)
**Output:** Wheel velocities (vL, vR)

```python
def inverse_kinematics(v, omega, wheel_dist):
    """
    Convert robot velocities to wheel velocities.

    vL = v - ω * (L/2)
    vR = v + ω * (L/2)
    """
    vL = v - omega * (wheel_dist / 2.0)
    vR = v + omega * (wheel_dist / 2.0)
    return vL, vR
```

### 2.4 ESP32 Implementation

```cpp
void handleVelocity(float lin_x, float ang_z) {
    // Differential drive
    float v_left  = lin_x - ang_z * (WHEEL_DIST / 2.0);
    float v_right = lin_x + ang_z * (WHEEL_DIST / 2.0);

    // Map to PWM
    motorASpd = int((v_left  / max_v) * 255.0);
    motorBSpd = int((v_right / max_v) * 255.0);
}
```

---

## 3. PID Control Theory

### 3.1 PID Formula

```
u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt

Where:
  u(t)  = control output
  e(t)  = error (setpoint - measured)
  Kp    = proportional gain
  Ki    = integral gain
  Kd    = derivative gain
```

### 3.2 Each Term Explained

| Term | Effect | Problem if Too High | Problem if Too Low |
|------|--------|---------------------|-------------------|
| **P** (Proportional) | Responds to current error | Oscillation, overshoot | Slow response, steady-state error |
| **I** (Integral) | Eliminates steady-state error | Integral windup, oscillation | Can't eliminate offset |
| **D** (Derivative) | Dampens response | Amplifies noise | Weak damping |

### 3.3 Discrete Implementation

```python
class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.prev_error = 0.0
        self.integral = 0.0
        self.dt = 0.05  # 20ms control loop

    def compute(self, current, target):
        error = target - current

        # Proportional
        p = self.kp * error

        # Integral with anti-windup
        self.integral += error * self.dt
        self.integral = max(-50, min(50, self.integral))  # Clamp
        i = self.ki * self.integral

        # Derivative
        derivative = (error - self.prev_error) / self.dt
        d = self.kd * derivative
        self.prev_error = error

        # Total output
        output = p + i + d

        # Clamp to limits
        return max(-self.output_limit, min(self.output_limit, output))
```

### 3.4 Anti-Windup

Integral windup occurs when the controller keeps integrating while the
actuator is saturated:

```
                    │
output ─────────────┤ max
                    │
        ┌───────────┘
        │    ∫e(t) grows even though output is clamped
        │
        └──────────────────────────────────► time
                    ▲
                    │
                    error is still positive, integral keeps growing
```

**Solution:** Clamp integral term to prevent windup:
```python
self.integral = max(-50, min(50, self.integral))
```

---

## 4. Navigation Algorithm

### 4.1 Waypoint Following State Machine

```
         ┌──────────────────────────────────────────────┐
         │                                              │
         ▼                                              │
    ┌─────────┐   distance < threshold    ┌────────────────────┐
    │APPROACH │────────────────────────▶│   ROTATE_TO_GOAL   │
    └─────────┘                         └────────────────────┘
         ▲                                      │
         │                                      │
         │              angle > threshold      │
         │                                      │
         │◀─────────────────────────────────────┘
         │
         │  angle < threshold
         │
         ▼
    ┌─────────┐
    │ MOVE    │───────────────────────────► [GOAL_REACHED]
    └─────────┘
```

### 4.2 Approach Phase

When far from waypoint:
1. Compute angle to waypoint
2. If angle error > threshold: rotate in place
3. If angle error < threshold: move forward while correcting

```python
def control_loop(self):
    # Distance and angle to waypoint
    dx = self.target_x - self.current_x
    dy = self.target_y - self.current_y
    dist = sqrt(dx² + dy²)
    angle_to_waypoint = atan2(dy, dx)

    # Relative angle
    relative_angle = normalize_angle(angle_to_waypoint - self.current_yaw)

    if dist < self.waypoint_threshold:
        # Waypoint reached!
        self.waypoint_active = False
        self.publish_vel(0, 0)
        return

    # Rotate towards waypoint if misaligned
    if abs(relative_angle) > self.angle_threshold:
        angular_vel = self.pid_angular.compute(0, relative_angle)
        linear_vel = 0  # Don't move forward while rotating
    else:
        # Aligned - move forward
        angular_vel = self.pid_angular.compute(0, relative_angle)
        linear_vel = self.pid_linear.compute(0, dist)

    self.publish_vel(linear_vel, angular_vel)
```

### 4.3 Rotation to Goal

When close to waypoint:
1. Stop forward motion
2. Rotate to final orientation
3. Confirm arrival

```python
def rotate_to_goal(self):
    target_yaw = self.target_theta  # Final orientation
    angle_error = normalize_angle(target_yaw - self.current_yaw)

    if abs(angle_error) < self.angle_threshold:
        return True  # Rotation complete

    angular_vel = self.pid_angular.compute(0, angle_error)
    self.publish_vel(0, angular_vel)
    return False
```

### 4.4 Publishing Velocity Commands

```python
def publish_vel(self, linear_x, angular_z):
    msg = Twist()
    msg.linear.x = max(-self.max_lin, min(self.max_lin, linear_x))
    msg.angular.z = max(-self.max_ang, min(self.max_ang, angular_z))
    self.cmd_vel_pub.publish(msg)
```

---

## 5. Parameter Tuning

### 5.1 Default Parameters

```yaml
max_linear_speed: 0.20   # m/s
max_angular_speed: 1.5  # rad/s

# Linear PID (distance control)
pid_linear_kp: 2.0
pid_linear_ki: 0.1
pid_linear_kd: 0.5

# Angular PID (rotation control)
pid_angular_kp: 3.0
pid_angular_ki: 0.2
pid_angular_kd: 1.0

waypoint_threshold: 0.15  # meters
angle_threshold: 0.2       # radians (~11°)
```

### 5.2 Tuning Procedure

**Step 1: Zero PID, Test Raw Response**
```yaml
pid_linear_kp: 0
pid_linear_ki: 0
pid_linear_kd: 0
```

Robot should not move at all (or very slowly).

**Step 2: Tune Angular (Turning)**

1. Set Kp=1, Ki=0, Kd=0
2. Give 45° angle command
3. Observe: Does it turn? Oscillates? Overshoots?

Tuning checklist:
- **Overshoots and oscillates** → Reduce Kp
- **Doesn't reach target** → Increase Kp
- **Steady-state error** → Add small Ki
- **Jerky motion** → Reduce Kd or increase Kp slightly

**Step 3: Tune Linear (Forward)**

1. Set Kp=1, Ki=0, Kd=0
2. Give 0.5m distance command
3. Robot should move forward and stop

Tuning checklist:
- **Overshoots waypoint** → Reduce Kp
- **Too slow** → Increase Kp
- **Oscillation at stop** → Increase Kd
- **Can't reach target** → Add small Ki

### 5.3 Tuning Cheat Sheet

| Problem | Kp | Ki | Kd |
|---------|-----|-----|-----|
| Doesn't reach target | ↑ | ↑ | - |
| Overshoots | ↓ | ↓ | ↑ |
| Oscillation | ↓ | ↓ | ↑ |
| Slow response | ↑ | - | - |
| Jerky | ↓ | - | ↓ |
| Never settles | ↓ | ↓ | - |
| Steady-state error | - | ↑ | - |

### 5.4 Safe Parameter Ranges

```python
# Never exceed these during tuning
SAFE_LIMITS = {
    'max_linear_speed': (0.05, 0.40),   # m/s
    'max_angular_speed': (0.5, 3.0),   # rad/s
    'pid_linear_kp': (0.1, 10.0),
    'pid_linear_ki': (0.0, 1.0),
    'pid_linear_kd': (0.0, 5.0),
    'pid_angular_kp': (0.1, 20.0),
    'pid_angular_ki': (0.0, 2.0),
    'pid_angular_kd': (0.0, 10.0),
}
```

---

## 6. Waypoint Management

### 6.1 Waypoint Message Format

```yaml
# geometry_msgs/Point
x: 1.5      # Target X position (meters)
y: 0.0      # Target Y position (meters)
z: 0.0      # Target orientation (radians)
            # Note: z carries theta, not a Z coordinate
```

### 6.2 Task Manager → Navigation

```python
# In task_manager_node.py
def send_waypoint(self, x, y, theta=0.0):
    msg = Point()
    msg.x = x
    msg.y = y
    msg.z = theta  # z is used for target angle
    self.waypoint_pub.publish(msg)
```

### 6.3 Waypoint Sequence

```
Waypoint 1        Waypoint 2        Waypoint 3
    │                  │                  │
    ▼                  ▼                  ▼
(0,0,0) ────────▶ (1.5,0,0) ────────▶ (1.5,1.5,0)
 Home              MFG              Storage B
```

### 6.4 Waypoint Reached Detection

```python
def is_waypoint_reached(self):
    dx = self.target_x - self.current_x
    dy = self.target_y - self.current_y
    dist = sqrt(dx*dx + dy*dy)

    if dist < self.waypoint_threshold:
        return True
    return False
```

### 6.5 Final Orientation

When waypoint is reached, robot may need to rotate to final orientation:

```python
def rotate_to_final_orientation(self):
    target = self.target_theta

    if target is None:
        return True  # No final orientation needed

    current = self.current_yaw
    error = normalize_angle(target - current)

    if abs(error) < self.angle_threshold:
        return True  # Orientation achieved

    # Rotate to target
    angular_vel = self.pid_angular.compute(0, error)
    self.publish_vel(0, angular_vel)
    return False
```

---

## 7. Obstacle Handling

### 7.1 Current Implementation

The basic navigation does NOT include obstacle avoidance.
It assumes:
- Clear paths between waypoints
- Static environment
- AprilTag markers visible for localization

### 7.2 Ultrasonic Integration (Future)

Planned integration with ultrasonic sensors:

```python
def check_obstacles(self, us_distances):
    """
    Check ultrasonic readings for obstacles.

    us_distances = [front, back, left, right]  # cm

    Returns: (has_obstacle, direction)
    """
    thresholds = {
        'front': 20.0,   # cm
        'back': 15.0,
        'left': 15.0,
        'right': 15.0,
    }

    if us_distances[0] < thresholds['front']:
        return True, 'front'
    # ... check other directions

    return False, None
```

### 7.3 Simple Avoidance Behavior

```python
def obstacle_avoidance(self, us_distances):
    if us_distances[0] < 20.0:  # Obstacle ahead
        # Stop and rotate
        self.publish_vel(0, self.max_angular * 0.5)  # Turn left
        return True
    return False
```

### 7.4 Full Avoidance State Machine

```
┌─────────────────────────────────────────┐
│            NAVIGATE_TO_WAYPOINT          │
└─────────────────────────────────────────┘
                   │
                   ▼
         ┌─────────────────────┐
         │  OBSTACLE_AHEAD?   │
         └──────────┬──────────┘
                    │
         ┌──────────┴──────────┐
         │Yes                  │No
         ▼                     ▼
┌─────────────────┐   ┌─────────────────────┐
│  STOP_MOTORS   │   │    MOVE_TO_WAYPOINT │
└────────┬────────┘   └─────────────────────┘
         │
         ▼
┌─────────────────┐
│  AVOID_LEFT    │◀──────────────────┐
│  (rotate + move)│                  │
└────────┬────────┘                  │
         │                             │
         ▼                             │
┌─────────────────┐                   │
│ CLEAR_PATH?     │──No───────────────┘
└────────┬────────┘
         │Yes
         ▼
┌─────────────────────────────┐
│    RESUME_NAVIGATION        │
└─────────────────────────────┘
```

---

## 8. Error Recovery

### 8.1 Common Failure Modes

| Failure | Detection | Recovery |
|---------|-----------|----------|
| Can't reach waypoint | Timeout (>30s) | Cancel, return home |
| Rotation timeout | >10s rotating | Continue anyway |
| Pose jumps | Large dx,dy between frames | Use last good pose |
| Stuck on obstacle | US shows <5cm for >5s | Reverse, try another path |

### 8.2 Timeout Handling

```python
WAYPOINT_TIMEOUT = 30.0  # seconds

def check_timeout(self):
    elapsed = time.time() - self.waypoint_start_time
    if elapsed > WAYPOINT_TIMEOUT:
        self.get_logger().warn(f'Waypoint timeout ({elapsed:.1f}s)')
        return True  # Timeout occurred
    return False
```

### 8.3 Recovery Behaviors

```python
def recover_from_failure(self):
    """
    Called when normal navigation fails.
    """
    # Option 1: Return home
    self.send_waypoint(self.home_pos[0], self.home_pos[1], 0)
    self.current_state = "RECOVERY_HOME"

    # Option 2: Try alternate path
    # (Would require path planning)

    # Option 3: Wait and retry
    # (If obstacle was temporary)
```

### 8.4 Health Monitoring

```python
def check_navigation_health(self):
    """
    Monitor navigation health.
    Returns: (healthy, message)
    """
    # Check if getting robot pose updates
    if self.pose_stale:
        return False, "Robot pose stale"

    # Check if waypoint is reachable
    if self.distance_to_waypoint > 5.0:
        return False, "Waypoint too far"

    # Check for oscillation
    if self.is_oscillating():
        return False, "Navigation oscillating"

    return True, "OK"
```

---

## Appendix: Algorithm Reference

### A.1 Angle Normalization

```python
def normalize_angle(angle):
    """
    Normalize angle to [-π, π] range.
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
```

### A.2 Quaternion to Yaw

```python
def quaternion_to_yaw(qx, qy, qz, qw):
    """
    Extract yaw angle from quaternion.
    Assumes rotation around Z axis (standard 2D robot).
    """
    # yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy**2 + qz**2)
    return math.atan2(siny_cosp, cosy_cosp)
```

### A.3 Yaw to Quaternion

```python
def yaw_to_quaternion(yaw):
    """
    Convert yaw angle to quaternion.
    Returns: (qx, qy, qz, qw)
    """
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))
```

### A.4 Complete Control Loop

```python
def control_loop(self):
    # 1. Compute errors
    dx = self.target_x - self.current_x
    dy = self.target_y - self.current_y
    dist = math.sqrt(dx*dx + dy*dy)

    # 2. Target angle
    target_angle = math.atan2(dy, dx)

    # 3. Angle error
    angle_error = self.normalize_angle(target_angle - self.current_yaw)

    # 4. Check if waypoint reached
    if dist < self.waypoint_threshold:
        self.on_waypoint_reached()
        return

    # 5. PID control
    if abs(angle_error) > self.angle_threshold:
        # Rotate in place
        angular_vel = self.pid_angular.compute(0, angle_error)
        linear_vel = 0.0
    else:
        # Move forward with heading correction
        angular_vel = self.pid_angular.compute(0, angle_error)
        linear_vel = self.pid_linear.compute(0, dist)

    # 6. Publish
    self.publish_vel(linear_vel, angular_vel)
```

---

**Document Version:** 1.0.0
**Last Updated:** May 2026
**Author:** Robotics Team