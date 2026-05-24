# 🤖 Robot Voiture 3 Roues ESP32 — Contrôle Assisté Intelligent + Ultrasons

Station de contrôle multi-plateforme (PC & Raspberry Pi 4B) pour robot différentiel à 3 roues avec :
- **4 capteurs ultrasons** (détection obstacle 3cm-4m, position 45°)
- **Rampe d'accélération douce** (anti-dérapage)
- **PID yaw** pour maintien de cap
- **IA Deep RL** qui apprend compensation + profil d'accélération optimal
- **Visualisation trajectoire** temps réel avec Kalman Filter
- **A* Pathfinding** pour navigation optimale entre les 12 AprilTags
- **SLAM** (camera + IMU + optical flow) pour localisation absolue

---

## ✨ Fonctionnalités

### 🔷 Contrôle & Sécurité
- **4 capteurs ultrasons HC-SR04:** 2 avant + 2 arrière à 45°
  - Ralentissement progressif 3-15cm (pas d'arrêt brutal)
  - Toggle ON/OFF depuis l'UI
  - Indication visuelle (🟢/🔴) de l'état des capteurs
- **Rampe d'accélération intelligente:** Montée progressive pour éviter le dérapage de la bille
- **Maintien de cap:** PID yaw verrouille la direction quand on avance tout droit

### 🧠 IA Contrôle Assisté
- **Apprentissage en temps réel** (désactivée par défaut)
- **3 corrections apprises:**
  - `trim_L/R`: Compensation moteur (max ±15 PWM)
  - `ramp_boost`: Adapte la douceur de l'accélération (±0.5 max)
- **Sauvegarde automatique:** `drive_assist_model.pt`

### 📊 Visualisation & Monitoring
- **Carte trajectoire temps réel:** Robot + chemin parcouru + grille 1m
- **Kalman Filter:** Lissage de la position (fusion IMU + odométrie moteurs)
- **Télémétrie complète:** Yaw, accel, moteurs, ultrasons, état IA
- **Console IA:** Logs temps réel de l'apprentissage

### 🔄 Compatibilité Multi-Plateforme
- **PC (Windows/Linux):** Entraînement IA complet
- **Raspberry Pi 4B:** Inférence optimisée (TorchScript INT8, ~2-5ms)
- **Détection automatique:** Adapte le mode selon la plateforme

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Interface Web (Flask)                    │
│  ┌────────────┐  ┌────────────┐  ┌────────────────────┐ │
│  │ Joystick   │  │ Graphiques │  │ Trajectoire        │ │
│  │ Virtuel    │  │ Yaw/PID    │  │ Robot + Kalman     │ │
│  └─────┬──────┘  └────────────┘  └────────────────────┘ │
└────────┼──────────────────────────────────────────────────┘
         │
    ┌────▼────┐     ┌──────────────────┐
    │ ESP32   │◄────┤ 4x Ultrasons     │
    │ WiFi AP │     │ (45° avant/arrière)
    └────┬────┘     └──────────────────┘
         │
    ┌────▼────┐     ┌──────────────┐
    │ BMI160  │     │ 2x Moteurs DC│
    │ (IMU)   │     │ + PID yaw    │
    └─────────┘     └──────────────┘
```

### Flux de données

```
ESP32 ──télémétrie──► PC/Raspberry Pi ──commandes──► ESP32
      (20Hz WebSocket)              (IA: 20Hz, Commandes: 50Hz)
```

---

## 📁 Structure

```
test_PID_auto/
├── arduino/
│   └── arduino.ino              # ESP32: PID + Ultrasons + IMU + EEPROM
├── templates/
│   └── index.html               # Interface Web complète
├── app.py                       # Flask: boucle IA + WebSocket + config globale
├── industrial_ai.py             # UnifiedRLAgent (PC/Pi compatible)
├── export_rpi.py                # Export modèle TorchScript INT8 (auto depuis Save)
├── import_rpi.py                # Test modèle optimisé
├── drive_assist_model.pt        # Modèle IA float32 (auto-généré)
├── drive_assist_rpi_int8.pt     # Modèle INT8 TorchScript pour Pi (auto-généré au Save)
└── README.md

# Config globale partagée (hors dossier, dans data/)
../data/robot_config.json        # Source de vérité unique PID/trims/minPWM/ramp/ia_trims
```

---

## 🛠️ Matériel

| Composant | Quantité | Pins ESP32 | Notes |
|-----------|----------|------------|-------|
| **ESP32 DevKit** | 1 | - | WiFi AP mode |
| **DFRobot BMI160** | 1 | I2C (SDA/SCL) | IMU 6 axes (0x68/0x69) |
| **HC-SR04** (avant gauche) | 1 | TRIG 5 / ECHO 34 | 45° avant-gauche |
| **HC-SR04** (avant droit) | 1 | TRIG 2 / ECHO 35 | 45° avant-droit |
| **HC-SR04** (arrière gauche) | 1 | TRIG 15 / ECHO 32 | 45° arrière-gauche |
| **HC-SR04** (arrière droit) | 1 | TRIG 33 / ECHO 25 | 45° arrière-droit |
| **L298N / DRV8833** | 1 | EN 14/4, IN 27/26/13/12 | Driver 2 canaux |
| **Moteurs DC** | 2 | - | 6V-12V avec réducteur |
| **Bille (caster)** | 1 | - | Roulette libre avant |
| **Servo** (optionnel) | 1 | Pin 19 | Gripper |

### Schéma de câblage Ultrasons

```
        [US1]        [US2]
       45° ╲        ╱ 45°
            ╲      ╱
    [US3]═════[ROBOT]═════[US4]
       45°                  45°
```

---

## 🚀 Installation

### ESP32 (Arduino IDE)

**Bibliothèques requises:**
- `ESPAsyncWebServer` + `AsyncTCP`
- `DFRobot_BMI160`
- `PID_v1`
- `ESP32Servo`

L'ESP32 crée le WiFi `ROBOT_WIFI` / mot de passe `robot1234`

### PC (Développement & Entraînement)

```bash
# Dépendances
pip install flask flask-socketio websocket-client pyserial numpy torch

# Lancer le serveur
python app.py
# → Ouvrir http://localhost:5000 dans le navigateur
```

### Raspberry Pi 4B (Déploiement)

```bash
# 1. Installer dépendances
pip install flask flask-socketio websocket-client pyserial numpy torch

# 2. Sur PC, exporter le modèle entraîné
python export_rpi.py
# → Génère 'drive_assist_rpi_int8.pt'

# 3. Transférer sur le Pi
scp drive_assist_rpi_int8.pt pi@raspberrypi.local:~/robot/
scp app.py pi@raspberrypi.local:~/robot/
scp industrial_ai.py pi@raspberrypi.local:~/robot/
scp -r templates pi@raspberrypi.local:~/robot/

# 4. Sur le Pi, lancer
python app.py
# → Détecte automatiquement le Pi et charge le modèle optimisé
```

---

## 🎮 Utilisation

### Interface Web

| Section | Description |
|---------|-------------|
| **🎮 Télécommande** | Joystick virtuel D-Pad + vitesse |
| **📡 Ultrasons** | 4 indicateurs + toggle ON/OFF + alerte ralentissement |
| **📊 Trajectoire** | Carte temps réel avec robot (🟢 triangle) + chemin (🔵) |
| **🧠 IA** | Toggle activation + statistiques temps réel |
| **⚙️ PID** | Réglage manuel Kp/Ki/Kd + trims moteurs |
| **⚡ Mode Auto** | Distance prédéfinie + vitesse |

### Contrôle par API (WebSocket)

```json
// Direction
{"t":"dir","x":0.0,"y":0.5}     // x: rotation (-1..1), y: vitesse (-1..1)

// Réglage PID
{"t":"cfg","ykp":4.0,"yki":0.08,"ykd":0.6}

// Activation/Désactivation ultrasons
{"t":"us","en":1}               // en: 0=off, 1=on

// Sauvegarde config EEPROM
{"t":"save"}
```

### Réglage PID Yaw

Valeurs de départ: `Kp=4.0, Ki=0.02, Kd=0.7`

| Symptôme | Ajustement |
|----------|-----------|
| Dérive non corrigée | ↑ Kp |
| Oscille gauche-droite | ↓ Kp, ↑ Kd |
| Correction lente | ↑ Kp, ↑ Ki |
| Oscillation rapide | ↓ Kp fort |

### Capteurs Ultrasons

**Comportement automatique:**
- **> 15cm:** Pleine vitesse
- **3-15cm:** Ralentissement progressif (interpolé)
- **< 3cm:** 5% vitesse max (très lent mais contrôlable)

**Alertes UI:**
- 🟢 Indicateur = capteur répond
- 🔴 Indicateur = capteur silencieux (timeout)
- 🐌 "RALENTI - XX% VITESSE" quand obstacle proche

---

## 🧠 IA Drive Assist

### Principe

L'IA utilise **Deep RL (DDPG)** avec Actor-Critic pour apprendre en temps réel :
- **State:** 10 features (yaw_error, yaw_rate, accel, moteurs, direction)
- **Action:** 3 valeurs (trim_L, trim_R, ramp_boost)
- **Reward:** Stabilité (minimiser dérive) + fluidité (pénaliser changements brusques)

### Workflow

1. **Entraînement sur PC:**
   ```bash
   python app.py
   # Activer IA, conduire le robot, l'IA apprend
   # Modèle sauvegardé dans drive_assist_model.pt
   ```

2. **Export pour Raspberry Pi:**
   ```bash
   python export_rpi.py
   # Quantification INT8 + TorchScript
   ```

3. **Déploiement sur Pi:**
   ```bash
   python app.py  # Détection auto, inférence optimisée
   ```

### Logs IA

```
[IA] Pi inference: 3.2ms | trims:(+2.5,-1.2) | ramp:-0.15
```

---

## 📊 Télémétrie

```json
{
  "y": 0.12,          // Yaw (°)
  "yr": 1.50,         // Yaw rate (°/s)
  "ax": 234, "ay": -1567, "az": 16384,  // Accélération IMU (raw)
  "tl": 150, "tr": 150,   // Target PWM (avant rampe)
  "ol": 120, "or": 118,   // Output PWM (après rampe)
  "lk": 1,            // Cap verrouillé? (0/1)
  "ia": 1,            // IA active? (0/1)
  "rs": 80,           // Rampe speed courante
  "us": [45.2, 32.1, 999.0, 120.5],      // Distances cm (4 capteurs)
  "usr": [1, 1, 0, 1],                   // Réponse capteurs (0/1)
  "use": 1,           // Ultrasons activés?
  "usl": 1.0          // Limite vitesse (0.0-1.0)
}
```

---

## 🔧 Détails Technique

### Localisation (Kalman Filter)

Fusion capteurs pour estimation position :
```
Position += Kalman(vx, vy) × dt
vx = (speed_moteurs × 0.8 + accélération_IMU × 0.2) × cos(yaw)
```

- **Précision:** ~±8-12cm (sans encodeurs)
- **Limitation:** Dérive à long terme (~10cm/10m)

### Détection Plateforme

```python
# IndustrialRLAgent détecte automatiquement:
if _is_raspberry_pi():  # ARM / aarch64 / 'raspberry pi' dans /proc/cpuinfo
    INFERENCE_ONLY = True
    # → thread d'entraînement désactivé, CPU libéré
    # → charge drive_assist_model.pt normalement
else:
    INFERENCE_ONLY = False
    # → thread background training actif
```

### Optimisations Raspberry Pi 4B

- **Quantification INT8:** Réduit mémoire et temps calcul
- **TorchScript:** Compilation statique, pas de Python overhead
- **QNNPACK:** Backend optimisé ARM NEON
- **Latence:** ~2-5ms vs ~10-20ms sur PC (non optimisé)

---

## ⚙️ Dépannage

| Problème | Solution |
|----------|----------|
| Robot tourne sans arrêt | Vérifier IMU calibrée, ajuster PID yaw |
| Bille dérape au démarrage | Diminuer rampe: `{"t":"cfg","rs":40}` |
| Ultrasons ne répondent pas | Vérifier câblage TRIG/ECHO, alim 5V stable |
| Trajectoire "saute" | Normal (précision limitée sans encodeurs) |
| IA trop lente sur Pi | Vérifier `drive_assist_rpi_int8.pt` existe |
| Erreur "Modèle non trouvé" sur Pi | Exécuter `export_rpi.py` sur PC d'abord |

---

## 📝 Changelog

### v8.0 — Production Audit Fixes
- ✅ **PID yaw wrap ±180°** corrigé dans `arduino.ino` (évite correction violente ±360°)
- ✅ **Firmware scaling** `cmdVelCallback` : diviseurs `0.30→0.20` / `2.00→1.50` (anti-saturation moteur)
- ✅ **Mode inference-only Pi** : `_is_raspberry_pi()` détecte ARM → thread training désactivé automatiquement
- ✅ **save_freq** `50→300` (~15s au lieu de 2.5s, réduit l'usure de la SD card)

### v6.0 — Global Config + Auto-Save Pipeline
- ✅ `data/robot_config.json` — source de vérité unique partagée
- ✅ Bouton Save = pipeline complet : EEPROM → JSON → .pt → INT8 export
- ✅ Feedback visuel temps réel de chaque étape dans la console UI
- ✅ Config chargée automatiquement au démarrage de Flask
- ✅ `flask_ros_bridge.py` pousse la config au boot vers l'ESP32 micro-ROS
- ✅ Correction : trims/minPWM ne sont plus écrasés lors de l'envoi PID
- ✅ Valeurs PID recommandées : Ki 0.08 → 0.02, Kd 0.6 → 0.7

### v5.0 — Multi-Plateforme + Ultrasons
- ✅ 4 capteurs ultrasons HC-SR04 (position 45°)
- ✅ Compatibilité Raspberry Pi 4B (inférence optimisée)
- ✅ Visualisation trajectoire temps réel
- ✅ Kalman Filter pour lissage position
- ✅ UnifiedRLAgent (détection auto PC/Pi)

### v4.0 — Contrôle Assisté
- IA Deep RL avec compensation moteur
- PID yaw auto-verrouillage
- Rampe d'accélération configurable

---

**Auteur:** Prince Gildas Mbama Kombila  
**Version:** 8.0 — Production Audit  
**License:** MIT
