/*
  ╔══════════════════════════════════════════════════════════════════╗
  ║  ESP32 ROBOT VOITURE INTELLIGENTE — Contrôle Assisté + IA      ║
  ║  2 roues motrices + bille avant                                  ║
  ║  • Rampe d'accélération douce (anti-dérapage)                   ║
  ║  • PID yaw: maintient la trajectoire droite                     ║
  ║  • IA: compensation moteur + profil d'accélération optimal      ║
  ╚══════════════════════════════════════════════════════════════════╝
*/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DFRobot_BMI160.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include <EEPROM.h>

const char* ssid = "ROBOT_WIFI";
const char* password = "robot1234";

// ─── PINS ───
#define MA_IN1    27
#define MA_IN2    26
#define MA_EN     14
#define MB_IN3    13
#define MB_IN4    12
#define MB_EN     4
#define SERVO_PIN 19

// ─── ULTRASONS ───
#define US1_TRIG  5
#define US1_ECHO  34
#define US2_TRIG  2
#define US2_ECHO  35
#define US3_TRIG  15
#define US3_ECHO  32
#define US4_TRIG  33
#define US4_ECHO  25
#define US_MAX_DISTANCE 400
#define US_STOP_DISTANCE 3.0  // cm avant obstacle

// ─── CONFIG ───
#define EEPROM_SIZE       256
#define EEPROM_MAGIC      0xDEADBEEF
#define EEPROM_ADDR       0

bool INVERSER_GAUCHE_DROITE = false;
float MOTOR_A_TRIM = 0;
float MOTOR_B_TRIM = 0;
float MOTOR_A_MINPWM = 35;
float MOTOR_B_MINPWM = 35;
float speedMax = 255;
float dirX = 0, dirY = 0;
int gripperAngle = 90;

// ── ODOMÉTRIE ──
float posX = 0, posY = 0;
float wheelBase = 0.15;      // Distance entre roues en mètres (à ajuster selon ton robot)
float pwmToSpeed = 0.0047;   // Conversion PWM -> m/s (255 PWM ≈ 1.2 m/s)
unsigned long lastOdomTime = 0;

// ─── OBJETS ───
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
DFRobot_BMI160 bmi160;
Servo gripper;

// ─── IMU ───
float yaw = 0, yaw_rate = 0;
int16_t accelX = 0, accelY = 0, accelZ = 0;
float bias_gz = 0;
bool imuReady = false;
unsigned long lastIMUTime = 0;

// ─── PID YAW ───
double yawSetpoint = 0, yawInput, yawOutput;
double yawKp = 4.0, yawKi = 0.08, yawKd = 0.6;
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, yawKp, yawKi, yawKd, DIRECT);
bool yawLocked = false;
float lockedYaw = 0;

// ─── RAMPE D'ACCÉLÉRATION ───
// Profil S-curve: jerk limité pour pas déraper la bille
float targetSpeedL = 0, targetSpeedR = 0;
float currentSpeedL = 0, currentSpeedR = 0;
float rampSpeed = 250.0;       // PWM/sec (montée) - réactif mais pas brutal
float rampBrake = 350.0;       // PWM/sec (freinage, plus rapide)
float rampNeutral = 200.0;     // PWM/sec (retour au neutre)

// ─── VARIABLES IA GLOBALES ───
float iaRampMultiplier = 1.0;   // Compatibilité EEPROM (plus utilisé)

// ─── ULTRASONS ───
struct UltrasonicSensor {
  uint8_t trigPin;
  uint8_t echoPin;
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
bool usEnabled = true;  // Activation/désactivation globale
float usSpeedLimit = 1.0;  // 1.0 = pleine vitesse, 0.0 = arrêt

// ─── CORRECTIONS IA ───
struct IACorrections {
  float trim_L = 0;           // Compensation moteur gauche (max ±15)
  float trim_R = 0;           // Compensation moteur droit (max ±15)
  float ramp_boost = 0;       // Ajuste la rampe d'accélération (±0.5)
  bool active = false;
  unsigned long lastUpdate = 0;
};
IACorrections iaCorr;

// ─── TÉLÉMÉTRIE ───
float currentYaw = 0, currentYawRate = 0;
float currentOutputL = 0, currentOutputR = 0;
float currentTargetL = 0, currentTargetR = 0;

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
// EEPROM
// ═══════════════════════════════════════════════════════════════
struct RobotConfig {
  uint32_t magic;
  float yaw_kp, yaw_ki, yaw_kd;
  float motor_a_trim, motor_b_trim;
  float motor_a_minpwm, motor_b_minpwm;
  float ramp_speed, ramp_brake, ramp_neutral;
  float ia_trim_L, ia_trim_R;
  float ia_ramp_mult;
};

void saveConfig() {
  RobotConfig cfg;
  cfg.magic = EEPROM_MAGIC;
  cfg.yaw_kp = yawKp; cfg.yaw_ki = yawKi; cfg.yaw_kd = yawKd;
  cfg.motor_a_trim = MOTOR_A_TRIM; cfg.motor_b_trim = MOTOR_B_TRIM;
  cfg.motor_a_minpwm = MOTOR_A_MINPWM; cfg.motor_b_minpwm = MOTOR_B_MINPWM;
  cfg.ramp_speed = rampSpeed; cfg.ramp_brake = rampBrake; cfg.ramp_neutral = rampNeutral;
  cfg.ia_trim_L = iaCorr.trim_L; cfg.ia_trim_R = iaCorr.trim_R;
  cfg.ia_ramp_mult = iaRampMultiplier;
  
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
  iaCorr.trim_L = cfg.ia_trim_L; iaCorr.trim_R = cfg.ia_trim_R;
  iaRampMultiplier = cfg.ia_ramp_mult;
  
  yawPID.SetTunings(yawKp, yawKi, yawKd);
  Serial.printf("[EEPROM] Config chargée | trims IA: L=%.1f R=%.1f | ramp_mult: %.2f\n",
                iaCorr.trim_L, iaCorr.trim_R, iaRampMultiplier);
  return true;
}

// ═══════════════════════════════════════════════════════════════
// IMU
// ═══════════════════════════════════════════════════════════════
void calibrateIMU() {
  Serial.println("[CALIB] Calibration IMU...");
  long sum = 0;
  int16_t gyro[3];
  for (int i = 0; i < 500; i++) {
    if (bmi160.getGyroData(gyro) == BMI160_OK) sum += gyro[2];
    delay(5);
  }
  bias_gz = sum / 500.0;
  yaw = 0; yaw_rate = 0;
  imuReady = true;
  lastIMUTime = millis();
  Serial.printf("[CALIB] OK — bias gz=%.2f\n", bias_gz);
}

void updateIMU() {
  if (!imuReady) return;
  int16_t accel[3], gyro[3];
  if (bmi160.getAccelData(accel) != BMI160_OK) return;
  if (bmi160.getGyroData(gyro) != BMI160_OK) return;
  
  accelX = accel[0]; accelY = accel[1]; accelZ = accel[2];
  
  unsigned long now = millis();
  float dt = (now - lastIMUTime) / 1000.0;
  lastIMUTime = now;
  if (dt <= 0 || dt > 0.2f) dt = 0.01f;
  
  float fgz = (gyro[2] - bias_gz) / 131.0;
  yaw_rate = fgz;
  yaw += fgz * dt;
  while (yaw >= 180.0) yaw -= 360.0;
  while (yaw < -180.0) yaw += 360.0;
  
  currentYaw = yaw;
  currentYawRate = yaw_rate;
}

// ═══════════════════════════════════════════════════════════════
// ULTRASONS — Lecture des 4 capteurs
// ═══════════════════════════════════════════════════════════════
float readUltrasonic(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 25000);  // Timeout 25ms (~4m max)
  if (duration == 0) return -1;  // Pas de réponse
  float distance = (duration * 0.0343) / 2.0;
  return (distance > US_MAX_DISTANCE) ? -1 : distance;
}

void updateUltrasonics() {
  if (!usEnabled) {
    usSpeedLimit = 1.0;  // Pleine vitesse quand désactivé
    return;
  }
  
  static uint8_t currentSensor = 0;
  static unsigned long lastUSRead = 0;
  
  unsigned long now = millis();
  if (now - lastUSRead < 60) return;  // Cycle ~60ms entre capteurs
  lastUSRead = now;
  
  UltrasonicSensor& us = usSensors[currentSensor];
  float dist = readUltrasonic(us.trigPin, us.echoPin);
  
  if (dist >= 0) {
    us.distance = dist;
    us.responding = true;
    us.lastUpdate = now;
  } else {
    // Timeout - capteur ne répond pas
    if (now - us.lastUpdate > 1000) us.responding = false;
  }
  
  // === CORRECTION: Calculer la limite avec TOUS les capteurs de la direction ===
  // US1, US2 = AVANT (45°) | US3, US4 = ARRIÈRE (45°)
  bool isMovingForward = (dirY > 0.1);
  bool isMovingBackward = (dirY < -0.1);
  
  if (isMovingForward || isMovingBackward) {
    float minObstacleDist = 999.0;
    bool hasObstacle = false;
    
    // Sélectionner les capteurs selon la direction
    int startIdx = isMovingForward ? 0 : 2;
    int endIdx = isMovingForward ? 2 : 4;
    
    for (int i = startIdx; i < endIdx; i++) {
      if (usSensors[i].responding && usSensors[i].distance > 0) {
        minObstacleDist = min(minObstacleDist, usSensors[i].distance);
        hasObstacle = true;
      }
    }
    
    // Zone de ralentissement: 3cm à 15cm
    if (hasObstacle) {
      if (minObstacleDist < 3.0) {
        usSpeedLimit = 0.05;  // 5% vitesse max
      } else if (minObstacleDist < 15.0) {
        usSpeedLimit = 0.05 + (minObstacleDist - 3.0) / 12.0 * 0.95;
      } else {
        usSpeedLimit = 1.0;
      }
      
      // Log si on entre en zone de ralentissement
      static bool wasLimited = false;
      bool isLimited = (usSpeedLimit < 0.95);
      if (isLimited && !wasLimited) {
        const char* dir = isMovingForward ? "devant" : "derrière";
        Serial.printf("[US] Ralentissement - Obstacle %s à %.1f cm (limite: %d%%)\n", 
                      dir, minObstacleDist, (int)(usSpeedLimit * 100));
      } else if (!isLimited && wasLimited) {
        Serial.println("[US] Zone libre - Pleine vitesse");
      }
      wasLimited = isLimited;
    } else {
      usSpeedLimit = 1.0;
    }
  } else {
    // Arrêté ou changement de direction → reset immédiat de la limite
    usSpeedLimit = 1.0;
  }
  
  currentSensor = (currentSensor + 1) % 4;
}

// ═══════════════════════════════════════════════════════════════
// RAMPE D'ACCÉLÉRATION INTELLIGENTE
// ═══════════════════════════════════════════════════════════════
float applyRamp(float current, float target, float rampUp, float rampDown, float dt) {
  float diff = target - current;
  float maxChange;
  
  if (fabs(target) > fabs(current) && target * current >= 0) {
    // Accélération: utiliser rampUp (plus doux)
    maxChange = rampUp * dt;
  } else if (target * current < 0) {
    // Changement de direction: utiliser rampDown (plus rapide)
    maxChange = rampDown * dt;
  } else {
    // Décélération: utiliser rampNeutral
    maxChange = rampNeutral * dt;
  }
  
  if (fabs(diff) <= maxChange) return target;
  return current + (diff > 0 ? maxChange : -maxChange);
}

// ═══════════════════════════════════════════════════════════════
// CONTRÔLE MOTEURS — PID Yaw + Rampe + IA
// ═══════════════════════════════════════════════════════════════
void updateOdometry() {
  unsigned long now = millis();
  if (lastOdomTime == 0) { lastOdomTime = now; return; }
  float dt = (now - lastOdomTime) / 1000.0;
  lastOdomTime = now;
  if (dt <= 0 || dt > 0.2f) return;
  
  // Vitesse linéaire approximative (moyenne des deux roues)
  float vLin = ((currentOutputL + currentOutputR) / 2.0) * pwmToSpeed;
  // Vitesse angulaire depuis le yaw_rate (plus précis que la différence de roues)
  float vAng = yaw_rate * (PI / 180.0);
  
  // Intégration - vLin est négative quand on recule (currentOutput négatifs)
  float dx = vLin * cos(yaw * PI / 180.0) * dt;
  float dy = vLin * sin(yaw * PI / 180.0) * dt;
  
  // Seulement si le robot bouge (évite la dérive quand arrêté)
  if (fabs(vLin) > 0.01) {
    posX += dx;
    posY += dy;
  }
}

void updateMotors() {
  // ── Vérrouillage cap ──
  // Désactivé en marche arrière: la cinématique avec bille avant est différente
  // et le PID yaw peut causer des oscillations en reculant
  bool goingStraight = (fabs(dirY) > 0.1) && (fabs(dirX) < 0.3) && (dirY > 0);
  
  if (goingStraight && !yawLocked) {
    lockedYaw = yaw;
    yawLocked = true;
    yawPID.SetMode(AUTOMATIC);
    Serial.printf("[YAW] Verrouillage: %.2f°\n", lockedYaw);
  }
  if (!goingStraight && yawLocked) {
    yawLocked = false;
    yawPID.SetMode(MANUAL);
    yawOutput = 0;
    yawSetpoint = 0;
    Serial.println("[YAW] Déverrouillage");
  }
  
  // ── PID Yaw ──
  if (yawLocked) {
    yawSetpoint = lockedYaw;
    yawInput = yaw;
    yawPID.Compute();
  }
  
  // ── Calcul base ──
  float baseLeft = (dirY + dirX) * speedMax;
  float baseRight = (dirY - dirX) * speedMax;
  
  // ── Limite de vitesse ultrasons (ralentissement progressif) ──
  // Appliquée en avant ET en arrière selon les capteurs actifs
  if (usEnabled && fabs(dirY) > 0.1 && usSpeedLimit < 1.0) {
    baseLeft *= usSpeedLimit;
    baseRight *= usSpeedLimit;
  }
  
  // ── Application PID Yaw ──
  float maxPidInfluence = speedMax * 0.40;
  float appliedPid = constrain(yawOutput, -maxPidInfluence, maxPidInfluence);
  appliedPid = -appliedPid;
  if (dirY < 0) appliedPid = -appliedPid;
  
  float targetL = yawLocked ? baseLeft - appliedPid : baseLeft;
  float targetR = yawLocked ? baseRight + appliedPid : baseRight;
  
  // ── Corrections IA (trims uniquement, max ±8 PWM) ──
  // En marche arrière, les trims sont inversés: la roue qui était trop lente
  // en avant devient trop rapide en arrière → il faut compenser dans l'autre sens
  float trimSign = (dirY >= 0) ? 1.0 : -1.0;
  if (iaCorr.active) {
    targetL += iaCorr.trim_L * trimSign;
    targetR += iaCorr.trim_R * trimSign;
  }
  
  // ── Trims manuels ──
  targetL += MOTOR_A_TRIM * trimSign;
  targetR += MOTOR_B_TRIM * trimSign;
  
  // ── RAMPE D'ACCÉLÉRATION ──
  float effectiveRampSpeed = rampSpeed;
  float effectiveRampBrake = rampBrake;
  float effectiveRampNeutral = rampNeutral;
  
  if (iaCorr.active) {
    // L'IA peut rendre la rampe plus douce ou plus rapide (±30% max)
    float mult = 1.0 + iaCorr.ramp_boost;
    mult = constrain(mult, 0.7, 1.3);
    effectiveRampSpeed *= mult;
    effectiveRampBrake *= mult;
    effectiveRampNeutral *= mult;
  }
  
  unsigned long now = millis();
  static unsigned long lastRamp = 0;
  float dt = (now - lastRamp) / 1000.0;
  lastRamp = now;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;
  
  currentSpeedL = applyRamp(currentSpeedL, targetL, effectiveRampSpeed, effectiveRampBrake, dt);
  currentSpeedR = applyRamp(currentSpeedR, targetR, effectiveRampSpeed, effectiveRampBrake, dt);
  
  // ── Anti-calage ──
  auto applyMinPWM = [](float val, float minPWM) -> float {
    if (fabs(val) < 3.0) return 0;
    if (val > 0 && val < minPWM) return minPWM;
    if (val < 0 && val > -minPWM) return -minPWM;
    return val;
  };
  
  float outL = applyMinPWM(currentSpeedL, MOTOR_A_MINPWM);
  float outR = applyMinPWM(currentSpeedR, MOTOR_B_MINPWM);
  
  outL = constrain(outL, -255, 255);
  outR = constrain(outR, -255, 255);
  
  setMotorRaw(INVERSER_GAUCHE_DROITE ? 'B' : 'A', (int)outL);
  setMotorRaw(INVERSER_GAUCHE_DROITE ? 'A' : 'B', (int)outR);
  
  currentOutputL = outL;
  currentOutputR = outR;
  currentTargetL = targetL;
  currentTargetR = targetR;
}

// ═══════════════════════════════════════════════════════════════
// PARSER
// ═══════════════════════════════════════════════════════════════
float getF(String &s, String k) { int i = s.indexOf(k); return i < 0 ? 0 : s.substring(i+k.length()).toFloat(); }

void parseMessage(String &msg) {
  if (msg.indexOf("\"t\":\"dir\"") >= 0) {
    dirX = getF(msg, "\"x\":"); dirY = getF(msg, "\"y\":");
  }
  else if (msg.indexOf("\"t\":\"cfg\"") >= 0) {
    yawKp = getF(msg, "\"ykp\":"); yawKi = getF(msg, "\"yki\":"); yawKd = getF(msg, "\"ykd\":");
    if (yawKp > 0) yawPID.SetTunings(yawKp, yawKi, yawKd);
    MOTOR_A_TRIM = getF(msg, "\"ta\":"); MOTOR_B_TRIM = getF(msg, "\"tb\":");
    MOTOR_A_MINPWM = getF(msg, "\"ma\":"); MOTOR_B_MINPWM = getF(msg, "\"mb\":");
    
    float rs = getF(msg, "\"rs\":"); if (rs > 10) rampSpeed = rs;
    float rb = getF(msg, "\"rb\":"); if (rb > 10) rampBrake = rb;
    float rn = getF(msg, "\"rn\":"); if (rn > 10) rampNeutral = rn;
    
    Serial.printf("[CFG] PID:%.1f,%.2f,%.1f | Trims:%.0f,%.0f | MinPWM:%.0f,%.0f | Rampe:%.0f,%.0f,%.0f\n",
                  yawKp, yawKi, yawKd, MOTOR_A_TRIM, MOTOR_B_TRIM, MOTOR_A_MINPWM, MOTOR_B_MINPWM,
                  rampSpeed, rampBrake, rampNeutral);
  }
  else if (msg.indexOf("\"t\":\"ia\"") >= 0) {
    if (msg.indexOf("\"active\":0") >= 0) {
      iaCorr.active = false;
      iaCorr.trim_L = iaCorr.trim_R = iaCorr.ramp_boost = 0;
      Serial.println("[IA] Désactivée");
    } else {
      float tl = getF(msg, "\"tl\":");
      float tr = getF(msg, "\"tr\":");
      float rbst = getF(msg, "\"rbst\":");
      
      // Sécurité: si trims trop grands, c'est une divergence → ignorer
      if (fabs(tl) > 25.0 || fabs(tr) > 25.0) {
        Serial.printf("[IA] ALERTE trims excessifs reçus: %.1f %.1f → ignorés\n", tl, tr);
        iaCorr.active = false;
      } else {
        // Limites ±15 PWM pour les trims
        iaCorr.trim_L = constrain(tl, -15.0, 15.0);
        iaCorr.trim_R = constrain(tr, -15.0, 15.0);
        iaCorr.ramp_boost = constrain(rbst, -0.5, 0.5);
        iaCorr.active = true;
        iaCorr.lastUpdate = millis();
      }
    }
  }
  else if (msg.indexOf("\"t\":\"save\"") >= 0) saveConfig();
  else if (msg.indexOf("\"t\":\"grip\"") >= 0) {
    gripperAngle = (int)getF(msg, "\"a\":");
    gripper.write(gripperAngle);
  }
  else if (msg.indexOf("\"t\":\"us\"") >= 0) {
    float enabled = getF(msg, "\"en\":");
    usEnabled = (enabled > 0);
    Serial.printf("[US] Capteurs %s\n", usEnabled ? "ACTIVÉS" : "DÉSACTIVÉS");
  }
  else if (msg.indexOf("\"t\":\"reset_odom\"") >= 0) {
    posX = 0; posY = 0;
    Serial.println("[ODOM] Position reset");
  }
}

void onWs(AsyncWebSocket *srv, AsyncWebSocketClient *c, AwsEventType t, void *arg, uint8_t *data, size_t len) {
  if (t == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->opcode == WS_TEXT) {
      String msg((const char*)data, len);
      parseMessage(msg);
    }
  }
}

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200); delay(100);
  Serial.println("\n[START] Robot Voiture Intelligente — Contrôle Assisté + IA");

  EEPROM.begin(EEPROM_SIZE);
  
  pinMode(MA_IN1, OUTPUT); pinMode(MA_IN2, OUTPUT);
  pinMode(MB_IN3, OUTPUT); pinMode(MB_IN4, OUTPUT);
  ledcAttach(MA_EN, 30000, 8); ledcAttach(MB_EN, 30000, 8);
  Serial.println("[OK] Moteurs");
  
  gripper.attach(SERVO_PIN); gripper.write(gripperAngle);
  Serial.println("[OK] Gripper");

  // Init ultrasons
  for (int i = 0; i < 4; i++) {
    pinMode(usSensors[i].trigPin, OUTPUT);
    pinMode(usSensors[i].echoPin, INPUT);
    digitalWrite(usSensors[i].trigPin, LOW);
  }
  Serial.println("[OK] 4x Ultrasons");

  yawPID.SetOutputLimits(-120, 120);
  yawPID.SetSampleTime(10);

  if (!loadConfig()) {
    Serial.println("[INFO] Valeurs par défaut | Rampe: montée=" + String(rampSpeed) + 
                   " frein=" + String(rampBrake) + " neutre=" + String(rampNeutral));
  }

  Serial.println("[TRY] IMU...");
  if (bmi160.I2cInit(0x69) == BMI160_OK) { Serial.println("[OK] IMU 0x69"); calibrateIMU(); }
  else if (bmi160.I2cInit(0x68) == BMI160_OK) { Serial.println("[OK] IMU 0x68"); calibrateIMU(); }
  else { Serial.println("[WARN] IMU non trouvée"); }

  WiFi.softAP(ssid, password);
  Serial.print("[OK] WiFi AP: "); Serial.println(WiFi.softAPIP());

  ws.onEvent(onWs); server.addHandler(&ws); server.begin();
  Serial.println("[OK] WebServer");
  Serial.println("[READY] Robot prêt ! | IA désactivée par défaut");
}

// ═══════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
  static unsigned long lastIMU = 0, lastPID = 0, lastTelem = 0;
  unsigned long now = millis();

  if (now - lastIMU >= 10)  { updateIMU(); lastIMU = now; }
  if (now - lastPID >= 10)  { updateMotors(); updateOdometry(); lastPID = now; }
  updateUltrasonics();

  while (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    if (msg.startsWith("{")) parseMessage(msg);
    else if (msg == "R") {
      // Reset yaw demandé par auto_detetc_tag_arduino.py
      yaw = 0.0;
      Serial.println("[YAW] Reset via commande R");
    }
  }

  if (now - lastTelem >= 50) {
    char m[300];
    snprintf(m, sizeof(m),
             "{\"y\":%.2f,\"yr\":%.2f,\"p\":%.2f,\"ax\":%d,\"ay\":%d,\"az\":%d,\"tl\":%.0f,\"tr\":%.0f,\"ol\":%.0f,\"or\":%.0f,\"lk\":%d,\"ia\":%d,\"rs\":%.0f,\"us\":[%.1f,%.1f,%.1f,%.1f],\"usr\":[%d,%d,%d,%d],\"use\":%d,\"usl\":%.2f,\"px\":%.3f,\"py\":%.3f}",
             currentYaw, currentYawRate, (float)yawOutput,
             accelX, accelY, accelZ,
             currentTargetL, currentTargetR, currentOutputL, currentOutputR,
             yawLocked ? 1 : 0, iaCorr.active ? 1 : 0, rampSpeed,
             usSensors[0].distance, usSensors[1].distance, usSensors[2].distance, usSensors[3].distance,
             usSensors[0].responding ? 1 : 0, usSensors[1].responding ? 1 : 0,
             usSensors[2].responding ? 1 : 0, usSensors[3].responding ? 1 : 0,
             usEnabled ? 1 : 0, usEnabled ? usSpeedLimit : 1.0,
             posX, posY);
    ws.textAll(m);
    // Format IMU, pour auto_detetc_tag_arduino.py (attend: IMU,yaw,omega_z,ax,ay,az)
    Serial.printf("IMU,%.2f,%.4f,%d,%d,%d\n",
                  currentYaw, currentYawRate,
                  accelX, accelY, accelZ);
    lastTelem = now;
  }
  ws.cleanupClients();
}
