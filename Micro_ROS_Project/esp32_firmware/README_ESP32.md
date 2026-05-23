# ESP32 Firmware Documentation
## Micro-ROS Robot Controller

**Version:** 1.0.0
**Date:** May 2026
**File:** `micro_ros_esp32Robot.ino`

---

## Table of Contents

1. [Overview](#1-overview)
2. [Hardware Requirements](#2-hardware-requirements)
3. [Pin Configuration](#3-pin-configuration)
4. [ROS2 Topics](#4-ros2-topics)
5. [Velocity Control](#5-velocity-control)
6. [IMU Integration](#6-imu-integration)
7. [Ultrasonic Sensors](#7-ultrasonic-sensors)
8. [Gripper Control](#8-gripper-control)
9. [Installation](#9-installation)
10. [Troubleshooting](#10-troubleshooting)
11. [Source Code](#11-source-code)

---

## 1. Overview

The ESP32 runs the low-level control layer of the robot using micro-ROS.
It handles:
- Motor velocity control via differential drive
- IMU data acquisition (BMI160)
- Ultrasonic distance measurement (4 sensors)
- Gripper servo control

The ESP32 communicates with the Raspberry Pi via UDP using the micro-ROS protocol.

### Key Features

| Feature | Specification |
|---------|---------------|
| Processor | ESP32 (dual-core @ 240MHz) |
| Communication | micro-ROS over UDP (port 8888) |
| PWM Resolution | 8-bit (0-255) |
| PWM Frequency | 30kHz for motors |
| IMU Rate | 50Hz |
| Ultrasonic Rate | 5Hz (round-robin) |

### Boot Sequence

```
1. Serial.begin(115200)
2. Configure motor pins (output mode)
3. Configure ultrasonic pins
4. Initialize servo
5. Initialize IMU (BMI160 via I2C)
6. Calibrate gyroscope (stationary for 1s)
7. Setup micro-ROS transports (WiFi/UDP)
8. Create ROS2 node, publishers, subscribers, timers
9. Enter main loop
```

### Expected Serial Output

```
ets Jun  8 2016 00:22:57
rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
...
[CALIB] Gyro Z, ne bouge pas...
[CALIB] bias=-1.23
[ROS2] ESP32 ready!
[ROS2] Topics: /imu_data /ultrasonic_data /cmd_vel /gripper_cmd
```

---

## 2. Hardware Requirements

### 2.1 Required Components

| Component | Model | Quantity | Purpose |
|-----------|-------|----------|---------|
| Microcontroller | ESP32 Dev Kit | 1 | Main controller |
| Motor Driver | L298N or similar | 1 | H-bridge for motors |
| IMU | BMI160 | 1 | 6-axis accelerometer/gyroscope |
| Ultrasonic | HC-SR04 | 4 | Distance measurement |
| Servo | SG90 or similar | 1 | Gripper control |
| Motors | DC geared motors | 2 | Robot locomotion |
| Camera | USB webcam | 1 | AprilTag detection (on Pi) |

### 2.2 Optional Components

| Component | Purpose |
|-----------|---------|
| OLED Display | Debug information |
| RGB LED | Status indication |
| Battery Monitor | Power tracking |

### 2.3 Power Budget

```
ESP32 Dev Kit:     5V @ 500mA = 2.5W
Motors (2x):        6-12V @ 1-2A = 6-24W (stall)
IMU:               3.3V @ 5mA = 16.5mW
Ultrasonics (4x):   5V @ 60mA = 300mW
Servo:             5-6V @ 500mA = 2.5W
---------------------------
Total (idle):      ~3W
Total (running):   ~10W
Total (peak):      ~30W
```

**Recommendation:** Use a 7.4V 3000mAh LiPo battery with a 5V/3A BEC regulator.

---

## 3. Pin Configuration

### 3.1 GPIO Pin Map

```
+------------------+----------------------------------------+
| GPIO             | Function                               |
+------------------+----------------------------------------+
| 4  | ENB          | Motor B PWM speed (LEDC channel 1)   |
| 5  | US1_TRIG      | Ultrasonic 1 (Front) trigger         |
| 12 | MB_IN4        | Motor B direction (IN4)              |
| 13 | MB_IN3        | Motor B direction (IN3)              |
| 14 | MA_EN         | Motor A PWM speed (LEDC channel 0)    |
| 15 | US3_TRIG      | Ultrasonic 3 (Left) trigger          |
| 19 | SERVO         | Servo signal (gripper)               |
| 21 | I2C_SDA       | IMU BMI160 SDA                        |
| 22 | I2C_SCL       | IMU BMI160 SCL                        |
| 25 | US4_ECHO      | Ultrasonic 4 (Right) echo            |
| 26 | MA_IN2        | Motor A direction (IN2)              |
| 27 | MA_IN1        | Motor A direction (IN1)              |
| 32 | US3_ECHO      | Ultrasonic 3 (Left) echo            |
| 33 | US4_TRIG      | Ultrasonic 4 (Right) trigger         |
| 34 | US1_ECHO      | Ultrasonic 1 (Front) echo (input)    |
| 35 | US2_ECHO      | Ultrasonic 2 (Back) echo (input)      |
| 2  | US2_TRIG      | Ultrasonic 2 (Back) trigger          |
+------------------+----------------------------------------+
```

### 3.2 Motor Control Wiring

```
                    ESP32                   L298N Module              Motor
                   --------                 -----------              ------
GPIO 27 (MA_IN1) ────────────────►  IN1
GPIO 26 (MA_IN2) ────────────────►  IN2
GPIO 14 (MA_EN)  ────────────────►  ENA  ◄── PWM
                                                        └───► Motor A (+)
                                                        └───► Motor A (-)

GPIO 13 (MB_IN3) ────────────────►  IN3
GPIO 12 (MB_IN4) ────────────────►  IN4
GPIO  4 (MB_EN)  ────────────────►  ENB  ◄── PWM
                                                        └───► Motor B (+)
                                                        └───► Motor B (-)

GND               ────────────────►  GND  (common ground)
Vin (7.4V)        ────────────────►  +12V (motor power)
```

### 3.3 IMU Wiring (I2C)

```
ESP32                   BMI160 Module
------                  -------------
GPIO 21 (SDA)  ────────────►  SDA
GPIO 22 (SCL)  ────────────►  SCL
3.3V           ────────────►  VCC
GND            ────────────►  GND

I2C Address: 0x69 (default for BMI160)
```

### 3.4 Ultrasonic Wiring

```
ESP32           HC-SR04 (Front)
------          --------------
GPIO 5  (TRIG)  ──────────►  TRIG
GPIO 34 (ECHO)  ◄──────────  ECHO
5V             ────────────►  VCC
GND            ────────────►  GND

ESP32           HC-SR04 (Back)
------          -------------
GPIO 2  (TRIG)  ──────────►  TRIG
GPIO 35 (ECHO)  ◄──────────  ECHO
5V             ────────────►  VCC
GND            ────────────►  GND

ESP32           HC-SR04 (Left)
------          -------------
GPIO 15 (TRIG)  ──────────►  TRIG
GPIO 32 (ECHO)  ◄──────────  ECHO
5V             ────────────►  VCC
GND            ────────────►  GND

ESP32           HC-SR04 (Right)
------          -------------
GPIO 33 (TRIG)  ──────────►  TRIG
GPIO 25 (ECHO)  ◄──────────  ECHO
5V             ────────────►  VCC
GND            ────────────►  GND
```

### 3.5 Servo/Gripper Wiring

```
ESP32                   SG90 Servo
------                  ----------
GPIO 19 (SERVO) ────────►  Signal (orange)
5V             ────────────►  VCC (red)
GND            ────────────►  GND (brown)

Note: Use a separate 5V supply for servo if drawing > 500mA.
The ESP32's internal regulator may overheat with servo load.
```

---

## 4. ROS2 Topics

### 4.1 Publishers (ESP32 → Raspberry Pi)

#### `/imu_data` (geometry_msgs/Accel)
```cpp
// Published at 50Hz (every 20ms)
imu_msg.linear.x  = yaw_angle;      // Degrees, 0-360
imu_msg.linear.y  = omega_z;        // rad/s (yaw rate)
imu_msg.linear.z  = 0;              // unused
imu_msg.angular.x = accel_x;        // m/s^2
imu_msg.angular.y = accel_y;        // m/s^2
imu_msg.angular.z = accel_z;        // m/s^2
```

#### `/ultrasonic_data` (geometry_msgs/Point)
```cpp
// Published at 5Hz (every 200ms)
us_msg.x = front_distance;   // cm, -1 = no reading
us_msg.y = back_distance;    // cm, -1 = no reading
us_msg.z = left_distance;    // cm, -1 = no reading
us_msg.w = right_distance;   // cm, -1 = no reading (note: stored in z)
```

#### `/cmd_result` (std_msgs/String)
```cpp
// Published on each command received
result_msg.data = "vel lin=0.20 ang=0.50"   // or "GRIPPER OPEN", etc.
```

### 4.2 Subscribers (ESP32 ← Raspberry Pi)

#### `/cmd_vel` (geometry_msgs/Twist)
```cpp
// Differential drive velocity control
linear.x  → forward/backward speed (m/s), clamped to [-0.3, 0.3]
angular.z → yaw rate (rad/s), clamped to [-2.0, 2.0]

// Conversion to motor PWM:
// v_left  = linear.x - angular.z * (wheel_dist / 2)
// v_right = linear.x + angular.z * (wheel_dist / 2)
// PWM = (velocity / max_velocity) * 255
```

#### `/gripper_cmd` (std_msgs/String)
```cpp
// Gripper position control
if (data == "open")  → servo.write(0);   // 0 degrees
if (data == "close") → servo.write(90);  // 90 degrees
```

---

## 5. Velocity Control

### 5.1 Differential Drive Kinematics

The robot uses differential drive (2 motor wheels):
```
        ω
       ───►
    ┌───────┐
    │       │  v_right = v + ω × (L/2)
    │  ◄─── │  v_left  = v - ω × (L/2)
    │       │
    └───────┘
```

Where:
- `v` = linear velocity (m/s)
- `ω` = angular velocity (rad/s)
- `L` = wheelbase distance (meters between wheels)

### 5.2 Velocity Mapping Constants

```cpp
#define MAX_LINEAR_X   0.30   // m/s  (max forward speed)
#define MAX_ANGULAR_Z  2.00   // rad/s (max turn rate)
#define WHEEL_DIST     0.10   // m     (axle length)
```

### 5.3 Motor PWM Mapping

```cpp
void handleVelocity(float lin_x, float ang_z) {
    // 1. Clamp inputs to valid ranges
    lin_x = constrain(lin_x, -MAX_LINEAR_X, MAX_LINEAR_X);
    ang_z = constrain(ang_z, -MAX_ANGULAR_Z, MAX_ANGULAR_Z);

    // 2. Differential drive inverse kinematics
    float v_left  = lin_x - ang_z * (WHEEL_DIST / 2.0);
    float v_right = lin_x + ang_z * (WHEEL_DIST / 2.0);

    // 3. Map to PWM (0-255)
    // max_v = MAX_LINEAR + MAX_ANGULAR * (L/2) = 0.3 + 2.0 * 0.05 = 0.4
    float max_v = MAX_LINEAR_X + MAX_ANGULAR_Z * (WHEEL_DIST / 2.0);
    motorASpd = int((v_left  / max_v) * 255.0);
    motorBSpd = int((v_right / max_v) * 255.0);

    // 4. Clamp to PWM range
    motorASpd = constrain(motorASpd, -255, 255);
    motorBSpd = constrain(motorBSpd, -255, 255);
}
```

### 5.4 PWM Configuration

```cpp
#define PWM_FREQ  30000   // Hz (30kHz - inaudible)
#define PWM_RES   8       // bits (256 levels)
```

The LEDC peripheral generates PWM:
- Channel 0: Motor A (GPIO 14 / ENA)
- Channel 1: Motor B (GPIO 4 / ENB)

### 5.5 Direction Control

Each motor has 2 direction pins (IN1, IN2 or IN3, IN4):

| IN1 | IN2 | Motor Output |
|-----|-----|-------------|
| LOW | LOW | Brake (stop) |
| LOW | HIGH | Forward |
| HIGH | LOW | Reverse |
| HIGH | HIGH | Brake (stop) |

```cpp
void setMotor(char m, int s) {
    // m = 'A' or 'B'
    // s = -255 to +255 (negative = reverse)

    if (s > 0) {
        // Forward
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        ledcWrite(en, s);  // PWM speed
    } else if (s < 0) {
        // Reverse
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        ledcWrite(en, -s);  // PWM speed
    } else {
        // Stop
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        ledcWrite(en, 0);
    }
}
```

---

## 6. IMU Integration

### 6.1 BMI160 Specifications

| Parameter | Value |
|-----------|-------|
| Interface | I2C (400kHz max) |
| I2C Address | 0x69 |
| Accelerometer | 16-bit, ±2g to ±16g |
| Gyroscope | 16-bit, ±125°/s to ±2000°/s |
| Sample Rate | Up to 1.6kHz |

### 6.2 Data Conversion

```cpp
// Accelerometer (16-bit signed, ±2g range)
// LSB = 16384 LSB/g
float fax = accel[0] * 9.80665 / 16384.0;  // m/s^2

// Gyroscope (16-bit signed, ±250°/s range)
// LSB = 131 LSB/(°/s)
float fgz = (gyro[2] - bias_gz) / 131.0;  // rad/s
```

### 6.3 Yaw Integration

The yaw angle is computed by integrating the gyroscope Z-axis:

```cpp
// In publishIMU():
dt = (now - lastIMUTime) / 1000.0;  // seconds
if (dt > 0 && dt < 0.1) {
    yaw += fgz * dt;  // Integrate
}
while (yaw >= 360.0) yaw -= 360.0;
while (yaw < 0.0)    yaw += 360.0;
```

### 6.4 Gyro Calibration

At startup, 200 gyroscope samples are averaged to compute a bias:

```cpp
void calibrateGyro() {
    Serial.println("[CALIB] Gyro Z, ne bouge pas...");
    long sum = 0;
    for (int i = 0; i < 200; i++) {
        int16_t gyro[3];
        if (bmi160.getGyroData(gyro) == BMI160_OK) {
            sum += gyro[2];
        }
        delay(5);
    }
    bias_gz = sum / 200.0;
    Serial.print("[CALIB] bias="); Serial.println(bias_gz);
}
```

**Important:** Keep the robot still during startup calibration!

### 6.5 I2C Communication

```cpp
#include <Wire.h>

// Initialize I2C on default pins (SDA=21, SCL=22)
Wire.begin();

// Initialize BMI160
DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;

while (bmi160.I2cInit(i2c_addr) != BMI160_OK) {
    Serial.println("i2c init fail");
    delay(1000);
}
```

---

## 7. Ultrasonic Sensors

### 7.1 HC-SR04 Specifications

| Parameter | Value |
|-----------|-------|
| Measuring Range | 2-400 cm |
| Accuracy | ±3mm (ideal conditions) |
| Measuring Angle | 15 degrees |
| Trigger Pulse | 10μs |
| Measurement Cycle | 60ms minimum |

### 7.2 Measurement Process

```cpp
// 1. Send trigger pulse
digitalWrite(TRIG, LOW);
delayMicroseconds(4);
digitalWrite(TRIG, HIGH);
delayMicroseconds(10);
digitalWrite(TRIG, LOW);

// 2. Measure echo pulse width
long duration = pulseIn(ECHO, HIGH, 25000);  // timeout 25ms

// 3. Convert to distance
float distance = duration * 0.034 / 2.0;  // cm (at 20°C)
```

### 7.3 Round-Robin Scheduling

Each ultrasonic is measured in sequence to avoid cross-talk:

```cpp
void readUSRoundRobin() {
    // Cycle through sensors 0-3
    switch (currentUS) {
        case 0: trig=US1_TRIG; echo=US1_ECHO; break;  // Front
        case 1: trig=US2_TRIG; echo=US2_ECHO; break;  // Back
        case 2: trig=US3_TRIG; echo=US3_ECHO; break;  // Left
        default: trig=US4_TRIG; echo=US4_ECHO; break;  // Right
    }

    // Take 5 measurements, filter outliers
    for (int i = 0; i < 5; i++) {
        // Trigger and read...
        if (valid >= 3) {
            usValues[currentUS] = median;  // Use median
        } else {
            usValues[currentUS] = -1.0;  // No valid reading
        }

    currentUS = (currentUS + 1) % 4;  // Next sensor
}
```

### 7.4 Filtering Algorithm

```cpp
// Take 5 samples, reject outliers, average middle values
float vals[5];
int valid = 0;

// Collect samples
for (int i = 0; i < 5; i++) {
    long d = pulseIn(echo, HIGH, 25000);
    if (d > 0) {
        float dist = d * 0.034 / 2.0;
        if (dist >= 5.0 && dist <= 150.0) {  // Valid range
            vals[valid++] = dist;
        }
    }
    delay(25);
}

// Use median if 3+ valid, else -1
usValues[currentUS] = (valid == 0) ? -1.0 : vals[valid / 2];
```

### 7.5 Timing

```
Timer interval: 200ms (5Hz)

200ms ┬──► Measure US0 (Front) - takes ~60ms
      │
      ├────► Measure US1 (Back) - takes ~60ms
      │
      ├────► Measure US2 (Left) - takes ~60ms
      │
      └────► Measure US3 (Right) - takes ~60ms

      └──► Next cycle (wait remaining time)
```

---

## 8. Gripper Control

### 8.1 SG90 Servo Specifications

| Parameter | Value |
|-----------|-------|
| Operating Voltage | 4.8-6V |
| Torque | 2.5 kg·cm (4.8V) |
| Speed | 0.12s/60° (4.8V) |
| Angle Range | 0-180° |

### 8.2 Gripper States

| Command | Angle | Action |
|---------|-------|--------|
| "open" | 0° | Fully open, ready to receive box |
| "close" | 90° | Fully closed, gripping box |

### 8.3 Servo Control

```cpp
#include <ESP32Servo.h>

Servo gripper;

// In setup():
gripper.attach(SERVO_PIN);  // GPIO 19
gripper.write(0);  // Start fully open

// In gripperCallback():
if (cmd == 'o' || cmd == 'O') {
    gripper.write(0);   // Open position
} else if (cmd == 'c' || cmd == 'C') {
    gripper.write(90);  // Close position
}
```

### 8.4 Mechanical Design

The gripper is a simple claw mechanism:
- Servo rotation 0°: claws open (parallel)
- Servo rotation 90°: claws closed (gripping)

**Tip:** Add rubber grips to claw tips to improve box hold.

---

## 9. Installation

### 9.1 Arduino IDE Setup

1. **Install ESP32 Board Support:**
   - File → Preferences → Additional Board Manager URLs:
     ```
     https://dl.espressif.com/dl/package_esp32_index.json
     ```
   - Tools → Board → Board Manager → ESP32 → Install

2. **Install Required Libraries:**
   - Sketch → Include Library → Manage Libraries
   - Search and install:
     - `micro_ros_arduino` (from GitHub: micro-ROS/micro_ros_arduino)
     - `DFRobot_BMI160`
     - `ESP32Servo`

3. **Configure Board:**
   - Tools → Board → ESP32 Dev Module
   - Tools → Upload Speed → 115200
   - Tools → CPU Frequency → 240MHz
   - Tools → Flash Size → 4MB (or appropriate)

### 9.2 Flashing the Firmware

1. Connect ESP32 to computer via USB
2. Open `micro_ros_esp32Robot.ino` in Arduino IDE
3. Select correct port (Tools → Port)
4. Press and hold BOOT button, click EN button, release
   (This puts ESP32 in download mode)
5. Click Upload
6. Wait for "Hard resetting via RTS pin..."
7. Open Serial Monitor (115200 baud) to verify

### 9.3 Network Configuration

The ESP32 connects to the micro-ROS agent via UDP.
Configure the agent IP in the micro-ros_arduino library or via:

```cpp
// In your sketch or platformio.ini
#define MICROROS_AGENT_IP IPAddress(192, 168, 1, 100)
```

### 9.4 Verifying Operation

After flashing, check Serial Monitor for:
```
[CALIB] Gyro Z, ne bouge pas...
[CALIB] bias=-1.23
[ROS2] ESP32 ready!
[ROS2] Topics: /imu_data /ultrasonic_data /cmd_vel /gripper_cmd
```

---

## 10. Troubleshooting

### 10.1 IMU Issues

**Symptom:** "i2c init fail" repeated in Serial Monitor

**Causes:**
1. IMU not powered (check 3.3V connection)
2. I2C wiring issue (SDA/SCL swapped)
3. IMU address conflict
4. Wire break

**Solutions:**
1. Verify 3.3V and GND at IMU module
2. Check SDA→SDA, SCL→SCL connections
3. Run I2C scanner to find device address

```cpp
// I2C Scanner Code
#include <Wire.h>
void setup() {
    Serial.begin(115200);
    Wire.begin();
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found: 0x");
            Serial.println(address, HEX);
        }
    }
}
```

### 10.2 Motor Issues

**Symptom:** Motors not spinning or spinning in wrong direction

**Causes:**
1. ENA/ENB PWM not configured
2. IN1/IN2 or IN3/IN4 wired incorrectly
3. Motor driver not powered

**Solutions:**
1. Check LEDC configuration (GPIO 14 and 4)
2. Swap IN1/IN2 wires to reverse direction
3. Verify motor driver has power (check voltage at motor driver)

**Test individual motors:**
```cpp
// Add to setup() temporarily
setMotor('A', 100);  // Should spin forward
delay(1000);
setMotor('A', -100); // Should spin reverse
delay(1000);
setMotor('A', 0);
```

### 10.3 Communication Issues

**Symptom:** ESP32 not appearing in `ros2 topic list`

**Causes:**
1. Network connectivity (WiFi)
2. Firewall blocking UDP port 8888
3. Wrong agent IP configured

**Solutions:**
1. Check ESP32 Serial Monitor for WiFi status
2. Verify Pi and ESP32 on same network
3. Test with `ping` from Pi to ESP32
4. Check UDP port: `sudo netstat -ulnp | grep 8888`

### 10.4 Calibration Issues

**Symptom:** Robot drifts or circles while trying to go straight

**Causes:**
1. Motor speed imbalance
2. Gyro bias not computed correctly
3. Wheel alignment issue

**Solutions:**
1. Increase or decrease motor PWM for slower wheel
2. Recalibrate gyro (keep robot still at startup)
3. Check wheel alignment and axle straightness

---

## 11. Source Code

The complete source code is in `micro_ros_esp32Robot.ino`.

### Code Structure

```cpp
// ═══════════════════════════════════════════════════════════════
// INCLUDES
// ═══════════════════════════════════════════════════════════════
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/node.h>
#include <geometry_msgs/msg/accel.h>
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <Wire.h>
#include <DFRobot_BMI160.h>
#include <ESP32Servo.h>

// ═══════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════
#define MA_IN1 27  // Motor A direction
#define MA_IN2 26
#define MA_EN  14  // Motor A PWM (LEDC channel 0)
#define MB_IN3 13  // Motor B direction
#define MB_IN4 12
#define MB_EN  4   // Motor B PWM (LEDC channel 1)
#define SERVO_PIN 19
#define US1_TRIG 5
#define US1_ECHO 34
// ... (all pin definitions)

// ═══════════════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════════════
#define PWM_FREQ     30000
#define PWM_RES      8
#define IMU_INTERVAL 20    // ms (50Hz)
#define US_INTERVAL  200    // ms (5Hz)
#define MAX_LINEAR_X 0.30  // m/s
#define MAX_ANGULAR_Z 2.00 // rad/s
#define WHEEL_DIST   0.10  // meters

// ═══════════════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ═══════════════════════════════════════════════════════════════
DFRobot_BMI160 bmi160;
Servo gripper;
rcl_timer_t timer_imu, timer_us;
float yaw = 0.0;
float bias_gz = 0.0;
float usValues[4] = {-1,-1,-1,-1};
int motorASpd = 0, motorBSpd = 0;
char result[80];  // For result messages

// ... (full implementation in source file)
```

---

**Document Version:** 1.0.0
**Last Updated:** May 2026
**Author:** Robotics Team