/*
 ===========================================================================
   ESP32 MICRO-ROS FIRMWARE — Robot complet (PID yaw + Rampe + Odom + SLAM)
   Port complet de test_PID_auto/arduino/arduino.ino sur couche Micro-ROS.
 ===========================================================================
 TOPICS PUBLISHERS:
   /imu_data        (geometry_msgs/Accel)  — yaw, omega, accel
   /ultrasonic_data (geometry_msgs/Point)  — 4 distances cm
   /odom_data       (geometry_msgs/Point)  — posX, posY, yaw
   /cmd_result      (std_msgs/String)      — feedback commandes
   /sensor_health   (std_msgs/String)      — JSON santé capteurs

 TOPICS SUBSCRIBERS:
   /cmd_vel     (geometry_msgs/Twist)  — linear.x (m/s), angular.z (rad/s)
   /gripper_cmd (std_msgs/String)      — angle entier "90" ou o/c
   /robot_cfg   (std_msgs/String)      — JSON config PID/trims/rampe

 SERIAL (115200) :
   Émission : IMU,yaw,omega,ax,ay,az  (compatible scripts Python AprilTag)
   Réception: commande "R" = reset yaw
 ===========================================================================
*/

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
#include <PID_v1.h>
#include <EEPROM.h>

// ─── PINS ───────────────────────────────────
#define MA_IN1    27
#define MA_IN2    26
#define MA_EN     14
#define MB_IN3    13
#define MB_IN4    12
#define MB_EN     4
#define SERVO_PIN 19
#define US1_TRIG  5
#define US1_ECHO  34
#define US2_TRIG  2
#define US2_ECHO  35
#define US3_TRIG  15
#define US3_ECHO  32
#define US4_TRIG  33
#define US4_ECHO  25

// ─── CONFIG ─────────────────────────────────
#define PWM_FREQ        30000
#define PWM_RES         8
#define IMU_INTERVAL    20
#define US_INTERVAL     60
#define TELEM_INTERVAL  50
#define EEPROM_SIZE     256
#define EEPROM_MAGIC    0xDEADBEEF
#define EEPROM_ADDR     0
#define US_MAX_DIST     400.0f

// ─── MOTEUR CONFIG ──────────────────────────
bool  INVERSER_GAUCHE_DROITE = false;
float MOTOR_A_TRIM    = 0;
float MOTOR_B_TRIM    = 0;
float MOTOR_A_MINPWM  = 35;
float MOTOR_B_MINPWM  = 35;
float speedMax        = 255;

// ─── COMMANDE DIRECTION ─────────────────────
// dirY : -1=arrière, 0=stop, 1=avant
// dirX : -1=gauche,  0=droit, 1=droite
float dirX = 0, dirY = 0;
int   gripperAngle = 90;

// ─── ODOMÉTRIE ──────────────────────────────
float posX = 0, posY = 0;
float wheelBase  = 0.15f;   // m entre roues
float pwmToSpeed = 0.0047f; // PWM → m/s
unsigned long lastOdomTime = 0;

// ─── IMU ────────────────────────────────────
DFRobot_BMI160 bmi160;
Servo gripper;
float yaw = 0, yaw_rate = 0;
int16_t accelX = 0, accelY = 0, accelZ = 0;
float bias_gz = 0;
bool imuReady = false;
unsigned long lastIMUTime = 0;

// ─── PID YAW ────────────────────────────────
double yawSetpoint = 0, yawInput, yawOutput;
double yawKp = 4.0, yawKi = 0.08, yawKd = 0.6;
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, yawKp, yawKi, yawKd, DIRECT);
bool yawLocked = false;
float lockedYaw = 0;

// ─── RAMPE ──────────────────────────────────
float currentSpeedL = 0, currentSpeedR = 0;
float rampSpeed   = 250.0f;
float rampBrake   = 350.0f;
float rampNeutral = 200.0f;

// ─── ULTRASONS ──────────────────────────────
struct UltrasonicSensor {
  uint8_t trigPin, echoPin;
  float distance;
  bool responding;
  unsigned long lastUpdate;
};
UltrasonicSensor usSensors[4] = {
  {US1_TRIG, US1_ECHO, 0, false, 0},
  {US2_TRIG, US2_ECHO, 0, false, 0},
  {US3_TRIG, US3_ECHO, 0, false, 0},
  {US4_TRIG, US4_ECHO, 0, false, 0}
};
bool  usEnabled    = true;
float usSpeedLimit = 1.0f;

// ─── TÉLÉMÉTRIE ─────────────────────────────
float currentOutputL = 0, currentOutputR = 0;
float currentTargetL = 0, currentTargetR = 0;

// ─── MICRO-ROS ──────────────────────────────
rclc_executor_t executor;
rclc_node_t     node;

rcl_publisher_t     imu_pub, us_pub, odom_pub, result_pub, health_pub;
rcl_subscription_t  vel_sub, gripper_sub, cfg_sub;

geometry_msgs__msg__Accel  imu_msg;
geometry_msgs__msg__Point  us_msg;
geometry_msgs__msg__Point  odom_msg;
std_msgs__msg__String      result_msg;
std_msgs__msg__String      health_msg;
geometry_msgs__msg__Twist  cmd_vel_msg;
std_msgs__msg__String      gripper_cmd_msg;
std_msgs__msg__String      cfg_cmd_msg;

rcl_timer_t timer_imu, timer_us, timer_health;

char result_buf[80];

// ═══════════════════════════════════════════════════════════════
// EEPROM
// ═══════════════════════════════════════════════════════════════
struct RobotConfig {
  uint32_t magic;
  float yaw_kp, yaw_ki, yaw_kd;
  float motor_a_trim, motor_b_trim;
  float motor_a_minpwm, motor_b_minpwm;
  float ramp_speed, ramp_brake, ramp_neutral;
};

float getJsonFloat(const char* s, const char* key) {
  const char* p = strstr(s, key);
  if (!p) return 0.0f;
  p += strlen(key);
  return atof(p);
}

void saveConfig() {
  RobotConfig cfg;
  cfg.magic           = EEPROM_MAGIC;
  cfg.yaw_kp          = yawKp;  cfg.yaw_ki = yawKi;  cfg.yaw_kd = yawKd;
  cfg.motor_a_trim    = MOTOR_A_TRIM;  cfg.motor_b_trim   = MOTOR_B_TRIM;
  cfg.motor_a_minpwm  = MOTOR_A_MINPWM; cfg.motor_b_minpwm = MOTOR_B_MINPWM;
  cfg.ramp_speed      = rampSpeed;  cfg.ramp_brake = rampBrake;  cfg.ramp_neutral = rampNeutral;
  EEPROM.put(EEPROM_ADDR, cfg);
  EEPROM.commit();
  Serial.println("[EEPROM] Config sauvegardée");
}

bool loadConfig() {
  RobotConfig cfg;
  EEPROM.get(EEPROM_ADDR, cfg);
  if (cfg.magic != EEPROM_MAGIC) return false;
  yawKp = cfg.yaw_kp; yawKi = cfg.yaw_ki; yawKd = cfg.yaw_kd;
  MOTOR_A_TRIM = cfg.motor_a_trim; MOTOR_B_TRIM = cfg.motor_b_trim;
  MOTOR_A_MINPWM = cfg.motor_a_minpwm; MOTOR_B_MINPWM = cfg.motor_b_minpwm;
  rampSpeed = cfg.ramp_speed; rampBrake = cfg.ramp_brake; rampNeutral = cfg.ramp_neutral;
  yawPID.SetTunings(yawKp, yawKi, yawKd);
  Serial.printf("[EEPROM] Config chargée | PID:%.1f,%.2f,%.1f | Trims:%.0f,%.0f\n",
                yawKp, yawKi, yawKd, MOTOR_A_TRIM, MOTOR_B_TRIM);
  return true;
}

// ═══════════════════════════════════════════════════════════════
// MOTEURS
// ═══════════════════════════════════════════════════════════════
void setMotorRaw(char m, int s) {
  int in1, in2, en;
  if (m == 'A') { in1 = MA_IN1; in2 = MA_IN2; en = MA_EN; }
  else          { in1 = MB_IN3; in2 = MB_IN4; en = MB_EN; }
  int pwm = constrain(abs(s), 0, 255);
  if (s > 0)      { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else if (s < 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else            { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  }
  ledcWrite(en, pwm);
}

// ═══════════════════════════════════════════════════════════════
// IMU
// ═══════════════════════════════════════════════════════════════
void calibrateIMU() {
  Serial.println("[CALIB] IMU — ne bouge pas...");
  long sum = 0;
  int16_t gyro[3];
  for (int i = 0; i < 500; i++) {
    if (bmi160.getGyroData(gyro) == BMI160_OK) sum += gyro[2];
    delay(5);
  }
  bias_gz = sum / 500.0f;
  yaw = 0; yaw_rate = 0;
  imuReady = true;
  lastIMUTime = millis();
  Serial.printf("[CALIB] OK — bias gz=%.2f\n", bias_gz);
}

void updateIMU() {
  if (!imuReady) return;
  int16_t accel[3], gyro[3];
  if (bmi160.getAccelData(accel) != BMI160_OK) return;
  if (bmi160.getGyroData(gyro)  != BMI160_OK) return;
  accelX = accel[0]; accelY = accel[1]; accelZ = accel[2];
  unsigned long now = millis();
  float dt = (now - lastIMUTime) / 1000.0f;
  lastIMUTime = now;
  if (dt <= 0 || dt > 0.2f) dt = 0.01f;
  float fgz = (gyro[2] - bias_gz) / 131.0f;
  yaw_rate = fgz;
  yaw += fgz * dt;
  while (yaw >= 180.0f)  yaw -= 360.0f;
  while (yaw < -180.0f)  yaw += 360.0f;
}

// ═══════════════════════════════════════════════════════════════
// ULTRASONS — round-robin + limiteur progressif
// ═══════════════════════════════════════════════════════════════
float readUltrasonic(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long d = pulseIn(echo, HIGH, 25000);
  if (d == 0) return -1;
  float dist = (d * 0.0343f) / 2.0f;
  return (dist > US_MAX_DIST) ? -1 : dist;
}

void updateUltrasonics() {
  if (!usEnabled) { usSpeedLimit = 1.0f; return; }
  static uint8_t  curSensor = 0;
  static unsigned long lastRead = 0;
  unsigned long now = millis();
  if (now - lastRead < US_INTERVAL) return;
  lastRead = now;

  UltrasonicSensor& us = usSensors[curSensor];
  float dist = readUltrasonic(us.trigPin, us.echoPin);
  if (dist >= 0) { us.distance = dist; us.responding = true; us.lastUpdate = now; }
  else if (now - us.lastUpdate > 1000) { us.responding = false; }

  bool fwd = (dirY >  0.1f);
  bool bck = (dirY < -0.1f);
  if (fwd || bck) {
    int s = fwd ? 0 : 2, e = fwd ? 2 : 4;
    float minDist = 999.0f;
    bool  hasObs  = false;
    for (int i = s; i < e; i++) {
      if (usSensors[i].responding && usSensors[i].distance > 0) {
        minDist = min(minDist, usSensors[i].distance);
        hasObs = true;
      }
    }
    if (hasObs) {
      if      (minDist < 3.0f)  usSpeedLimit = 0.05f;
      else if (minDist < 15.0f) usSpeedLimit = 0.05f + (minDist - 3.0f) / 12.0f * 0.95f;
      else                      usSpeedLimit = 1.0f;
    } else { usSpeedLimit = 1.0f; }
  } else { usSpeedLimit = 1.0f; }

  curSensor = (curSensor + 1) % 4;
}

// ═══════════════════════════════════════════════════════════════
// RAMPE
// ═══════════════════════════════════════════════════════════════
float applyRamp(float cur, float tgt, float up, float dn, float dt) {
  float diff = tgt - cur;
  float maxC;
  if      (fabsf(tgt) > fabsf(cur) && tgt * cur >= 0) maxC = up * dt;
  else if (tgt * cur < 0)                              maxC = dn * dt;
  else                                                 maxC = rampNeutral * dt;
  if (fabsf(diff) <= maxC) return tgt;
  return cur + (diff > 0 ? maxC : -maxC);
}

// ═══════════════════════════════════════════════════════════════
// ODOMÉTRIE
// ═══════════════════════════════════════════════════════════════
void updateOdometry() {
  unsigned long now = millis();
  if (lastOdomTime == 0) { lastOdomTime = now; return; }
  float dt = (now - lastOdomTime) / 1000.0f;
  lastOdomTime = now;
  if (dt <= 0 || dt > 0.2f) return;
  float vLin = ((currentOutputL + currentOutputR) / 2.0f) * pwmToSpeed;
  if (fabsf(vLin) > 0.01f) {
    posX += vLin * cosf(yaw * PI / 180.0f) * dt;
    posY += vLin * sinf(yaw * PI / 180.0f) * dt;
  }
}

// ═══════════════════════════════════════════════════════════════
// CONTRÔLE MOTEURS — PID Yaw + Rampe + Trims
// ═══════════════════════════════════════════════════════════════
void updateMotors() {
  // Verrouillage cap (seulement en avant, pas en arrière)
  bool goingStraight = (fabsf(dirY) > 0.1f) && (fabsf(dirX) < 0.3f) && (dirY > 0);
  if (goingStraight && !yawLocked) {
    lockedYaw = yaw; yawLocked = true;
    yawPID.SetMode(AUTOMATIC);
  }
  if (!goingStraight && yawLocked) {
    yawLocked = false;
    yawPID.SetMode(MANUAL);
    yawOutput = 0; yawSetpoint = 0;
  }
  if (yawLocked) { yawSetpoint = lockedYaw; yawInput = yaw; yawPID.Compute(); }

  // Base moteurs
  float baseL = (dirY + dirX) * speedMax;
  float baseR = (dirY - dirX) * speedMax;

  // Limite ultrason (avant ET arrière)
  if (usEnabled && fabsf(dirY) > 0.1f && usSpeedLimit < 1.0f) {
    baseL *= usSpeedLimit;
    baseR *= usSpeedLimit;
  }

  // PID yaw
  float maxPid = speedMax * 0.40f;
  float pid = constrain((float)yawOutput, -maxPid, maxPid);
  pid = -pid;
  if (dirY < 0) pid = -pid;
  float tgtL = yawLocked ? baseL - pid : baseL;
  float tgtR = yawLocked ? baseR + pid : baseR;

  // Trims (inversés en marche arrière)
  float ts = (dirY >= 0) ? 1.0f : -1.0f;
  tgtL += MOTOR_A_TRIM * ts;
  tgtR += MOTOR_B_TRIM * ts;

  // Rampe
  static unsigned long lastRamp = 0;
  unsigned long now = millis();
  float dt = (now - lastRamp) / 1000.0f;
  lastRamp = now;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;
  currentSpeedL = applyRamp(currentSpeedL, tgtL, rampSpeed, rampBrake, dt);
  currentSpeedR = applyRamp(currentSpeedR, tgtR, rampSpeed, rampBrake, dt);

  // Anti-calage
  auto minPWM = [](float v, float m) -> float {
    if (fabsf(v) < 3.0f) return 0;
    if (v > 0 && v < m)  return m;
    if (v < 0 && v > -m) return -m;
    return v;
  };
  float outL = constrain(minPWM(currentSpeedL, MOTOR_A_MINPWM), -255, 255);
  float outR = constrain(minPWM(currentSpeedR, MOTOR_B_MINPWM), -255, 255);

  setMotorRaw(INVERSER_GAUCHE_DROITE ? 'B' : 'A', (int)outL);
  setMotorRaw(INVERSER_GAUCHE_DROITE ? 'A' : 'B', (int)outR);
  currentOutputL = outL; currentOutputR = outR;
  currentTargetL = tgtL; currentTargetR = tgtR;
}

// ═══════════════════════════════════════════════════════════════
// MICRO-ROS PUBLISHERS (timers)
// ═══════════════════════════════════════════════════════════════
void publishIMU(rcl_timer_t*, int64_t) {
  imu_msg.linear.x  = yaw;
  imu_msg.linear.y  = yaw_rate;
  imu_msg.linear.z  = 0;
  imu_msg.angular.x = accelX;
  imu_msg.angular.y = accelY;
  imu_msg.angular.z = accelZ;
  rcl_publish(&imu_pub, &imu_msg, NULL);
}

void publishUS(rcl_timer_t*, int64_t) {
  us_msg.x = usSensors[0].distance;
  us_msg.y = usSensors[1].distance;
  us_msg.z = usSensors[2].distance;
  rcl_publish(&us_pub, &us_msg, NULL);
}

void publishHealth(rcl_timer_t*, int64_t) {
  char health[300];
  snprintf(health, sizeof(health),
    "{\"imu\":%d,\"us\":[%.1f,%.1f,%.1f,%.1f],\"usr\":[%d,%d,%d,%d],"
    "\"usl\":%.2f,\"px\":%.3f,\"py\":%.3f,\"lk\":%d,\"yaw\":%.2f}",
    imuReady ? 1 : 0,
    usSensors[0].distance, usSensors[1].distance,
    usSensors[2].distance, usSensors[3].distance,
    usSensors[0].responding ? 1 : 0, usSensors[1].responding ? 1 : 0,
    usSensors[2].responding ? 1 : 0, usSensors[3].responding ? 1 : 0,
    usSpeedLimit, posX, posY, yawLocked ? 1 : 0, yaw);
  health_msg.data.data = health;
  health_msg.data.size = strlen(health);
  rcl_publish(&health_pub, &health_msg, NULL);

  // Odométrie sur topic dédié
  odom_msg.x = posX;
  odom_msg.y = posY;
  odom_msg.z = yaw;
  rcl_publish(&odom_pub, &odom_msg, NULL);
}

// ═══════════════════════════════════════════════════════════════
// MICRO-ROS SUBSCRIBERS (callbacks)
// ═══════════════════════════════════════════════════════════════
void cmdVelCallback(const void* msg_in) {
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msg_in;
  // Convertit Twist (m/s) → dirX/dirY normalisés (-1..1)
  dirY = constrain(msg->linear.x  / 0.30f, -1.0f, 1.0f);
  dirX = constrain(msg->angular.z / 2.00f, -1.0f, 1.0f);
  snprintf(result_buf, sizeof(result_buf), "vel lin=%.2f ang=%.2f",
           msg->linear.x, msg->angular.z);
  result_msg.data.data = result_buf;
  result_msg.data.size = strlen(result_buf);
  rcl_publish(&result_pub, &result_msg, NULL);
}

void gripperCallback(const void* msg_in) {
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msg_in;
  if (!msg->data.data) return;
  // Accepte angle numérique "90" ou o/O (open) / c/C (close)
  char c = msg->data.data[0];
  if (c >= '0' && c <= '9') {
    gripperAngle = atoi(msg->data.data);
  } else if (c == 'o' || c == 'O') {
    gripperAngle = 0;
  } else if (c == 'c' || c == 'C') {
    gripperAngle = 90;
  }
  gripperAngle = constrain(gripperAngle, 0, 180);
  gripper.write(gripperAngle);
  snprintf(result_buf, sizeof(result_buf), "gripper=%d", gripperAngle);
  result_msg.data.data = result_buf;
  result_msg.data.size = strlen(result_buf);
  rcl_publish(&result_pub, &result_msg, NULL);
}

void cfgCallback(const void* msg_in) {
  // JSON config: {"ykp":4.0,"yki":0.08,"ykd":0.6,"ta":0,"tb":0,"ma":35,"mb":35,"save":1}
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msg_in;
  if (!msg->data.data) return;
  const char* s = msg->data.data;

  float kp = getJsonFloat(s, "\"ykp\":"); if (kp > 0) yawKp = kp;
  float ki = getJsonFloat(s, "\"yki\":"); if (ki >= 0) yawKi = ki;
  float kd = getJsonFloat(s, "\"ykd\":"); if (kd >= 0) yawKd = kd;
  yawPID.SetTunings(yawKp, yawKi, yawKd);

  if (strstr(s, "\"ta\":")) MOTOR_A_TRIM    = getJsonFloat(s, "\"ta\":");
  if (strstr(s, "\"tb\":")) MOTOR_B_TRIM    = getJsonFloat(s, "\"tb\":");
  if (strstr(s, "\"ma\":")) MOTOR_A_MINPWM  = getJsonFloat(s, "\"ma\":");
  if (strstr(s, "\"mb\":")) MOTOR_B_MINPWM  = getJsonFloat(s, "\"mb\":");
  float rs = getJsonFloat(s, "\"rs\":"); if (rs > 10) rampSpeed   = rs;
  float rb = getJsonFloat(s, "\"rb\":"); if (rb > 10) rampBrake   = rb;
  float rn = getJsonFloat(s, "\"rn\":"); if (rn > 10) rampNeutral = rn;

  if (strstr(s, "\"reset_odom\":1")) { posX = 0; posY = 0; }
  if (strstr(s, "\"save\":1"))       saveConfig();
  if (strstr(s, "\"us_en\":0"))      usEnabled = false;
  if (strstr(s, "\"us_en\":1"))      usEnabled = true;

  Serial.printf("[CFG] PID:%.1f,%.2f,%.1f | Trims:%.0f,%.0f | MinPWM:%.0f,%.0f\n",
                yawKp, yawKi, yawKd, MOTOR_A_TRIM, MOTOR_B_TRIM,
                MOTOR_A_MINPWM, MOTOR_B_MINPWM);
}

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(100);
  EEPROM.begin(EEPROM_SIZE);

  // Moteurs
  pinMode(MA_IN1, OUTPUT); pinMode(MA_IN2, OUTPUT);
  pinMode(MB_IN3, OUTPUT); pinMode(MB_IN4, OUTPUT);
  ledcAttach(MA_EN, PWM_FREQ, PWM_RES);
  ledcAttach(MB_EN, PWM_FREQ, PWM_RES);
  setMotorRaw('A', 0); setMotorRaw('B', 0);

  // Gripper
  gripper.attach(SERVO_PIN);
  gripper.write(gripperAngle);

  // Ultrasons
  for (int i = 0; i < 4; i++) {
    pinMode(usSensors[i].trigPin, OUTPUT);
    pinMode(usSensors[i].echoPin, INPUT);
    digitalWrite(usSensors[i].trigPin, LOW);
  }

  // PID
  yawPID.SetOutputLimits(-120, 120);
  yawPID.SetSampleTime(10);
  if (!loadConfig()) Serial.println("[INFO] Valeurs par défaut");

  // IMU
  Serial.println("[TRY] IMU...");
  if      (bmi160.I2cInit(0x69) == BMI160_OK) { Serial.println("[OK] IMU 0x69"); calibrateIMU(); }
  else if (bmi160.I2cInit(0x68) == BMI160_OK) { Serial.println("[OK] IMU 0x68"); calibrateIMU(); }
  else Serial.println("[WARN] IMU non trouvée — yaw = 0");

  // ─── MICRO-ROS ─────────────────────────────────────────────
  set_microros_transports();
  rclc_support_t support;
  rclc_support_init(&support, 0, NULL, &executor);
  rclc_node_init(&node, "esp32_robot", "", &support);

  // Init messages
  geometry_msgs__msg__Accel__init(&imu_msg);
  geometry_msgs__msg__Point__init(&us_msg);
  geometry_msgs__msg__Point__init(&odom_msg);
  std_msgs__msg__String__init(&result_msg);
  std_msgs__msg__String__init(&health_msg);
  geometry_msgs__msg__Twist__init(&cmd_vel_msg);
  std_msgs__msg__String__init(&gripper_cmd_msg);
  std_msgs__msg__String__init(&cfg_cmd_msg);

  // Publishers
  rclc_publisher_init_default(&imu_pub,    &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Accel), "/imu_data");
  rclc_publisher_init_default(&us_pub,     &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "/ultrasonic_data");
  rclc_publisher_init_default(&odom_pub,   &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "/odom_data");
  rclc_publisher_init_default(&result_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,      msg, String), "/cmd_result");
  rclc_publisher_init_default(&health_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,      msg, String), "/sensor_health");

  // Subscribers
  rclc_subscription_init_default(&vel_sub,     &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),  "/cmd_vel");
  rclc_subscription_init_default(&gripper_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,      msg, String), "/gripper_cmd");
  rclc_subscription_init_default(&cfg_sub,     &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,      msg, String), "/robot_cfg");

  // Timers
  rclc_timer_init_default(&timer_imu,    &support, RCL_MS_TO_NS(IMU_INTERVAL),   publishIMU);
  rclc_timer_init_default(&timer_us,     &support, RCL_MS_TO_NS(US_INTERVAL),    publishUS);
  rclc_timer_init_default(&timer_health, &support, RCL_MS_TO_NS(1000),           publishHealth);

  // Executor
  rclc_executor_init(&executor, &support.context, 6, &allocator);
  rclc_executor_add_timer(&executor, &timer_imu);
  rclc_executor_add_timer(&executor, &timer_us);
  rclc_executor_add_timer(&executor, &timer_health);
  rclc_executor_add_subscription(&executor, &vel_sub,     &cmd_vel_msg,    &cmdVelCallback,  ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &gripper_sub, &gripper_cmd_msg,&gripperCallback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &cfg_sub,     &cfg_cmd_msg,    &cfgCallback,     ON_NEW_DATA);

  Serial.println("[ROS2] ESP32 prêt !");
}

// ═══════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
  static unsigned long lastIMU = 0, lastPID = 0, lastTelem = 0;
  unsigned long now = millis();

  if (now - lastIMU   >= 10) { updateIMU();    lastIMU  = now; }
  if (now - lastPID   >= 10) { updateMotors(); updateOdometry(); lastPID = now; }
  updateUltrasonics();

  // Commandes série (reset yaw depuis scripts Python AprilTag)
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line == "R") { yaw = 0.0f; Serial.println("[YAW] Reset"); }
  }

  // Émission IMU série (format attendu par auto_detetc_tag_arduino.py)
  if (now - lastTelem >= TELEM_INTERVAL) {
    Serial.printf("IMU,%.2f,%.4f,%d,%d,%d\n",
                  yaw, yaw_rate, accelX, accelY, accelZ);
    lastTelem = now;
  }

  // Spin Micro-ROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}