# CHANGELOG

All notable changes to this project will be documented in this file.

## [9.0] — 2026-05-24

### Added
- **2D Map Visualization**: Real-time map at `/map_feed` with robot position, confirmed tags (green), unconfirmed (red/orange), scanning indicator (cyan)
- **Arena dimensions in config**: `robot_config.json` now includes `arena` section with width (66cm), height (47cm), and zone positions
- **Mission templates UI**: Quick buttons for 1/2/3/6 cubes or alternating color missions
- **PRECISION_DROP state**: Robot positions precisely in 15cm drop square center (2cm threshold) before releasing
- **Scan 360° improved**: Robot goes to arena center (33, 23.5cm) first, then rotates slowly at 0.25 rad/s for better tag detection
- **Drop square navigation**: 15cm square positioning with slow approach speed (0.06 m/s)

### Changed
- **Station color mapping**: Fixed to Station A (tag 9) = GREEN, Station B (tag 6) = BLUE
- **Camera backend**: Platform-specific detection — V4L2 on Linux, DirectShow on Windows
- **Gripper angle**: Synced across all projects — 0° = open, 180° = closed
- **TAG_MAP positions**: Updated with real arena measurements (66×47cm)
- **Scan behavior**: Goes to center before rotating, slower speed (0.25 rad/s)

### Fixed
- **Color-to-station mapping**: All files now consistent — green→tag9, blue→tag6
- **Platform compatibility**: `calibrate_camera.py` now works on both Windows (CAP_DSHOW) and Linux (CAP_V4L2)
- **micro-ROS includes**: Added `#include <rclc/rclc.h>` for proper type definitions
- **Map API endpoint**: `/api/map/update` receives real-time tag confirmation data from mission engine

## [8.0] — 2026-05-24

### Fixed (Production Audit)
- **[C1] /cmd_vel arbitrage**: `flask_ros_bridge.py /velocity` retourne HTTP 409 si `mission_state.running == True` — empêche l'UI de briser une mission en cours
- **[C2] PID yaw wrap ±180°**: `arduino.ino` — erreur calculée dans `[-180,+180]` avant `yawPID.Compute()` (évite saut ±360° au passage de la frontière)
- **[C3] Double source yaw**: `task_manager_node._flush_imu_to_engine()` — yaw SLAM prioritaire si `tracker.initialized`, fallback IMU sinon; throttle à 20 Hz
- **[C4] Saturation moteur asymétrique**: `micro_ros_esp32Robot.ino cmdVelCallback` — diviseurs `0.30→0.20` et `2.00→1.50` alignés sur `max_linear/max_angular` de `navigation_node`
- **[H1] navigation_node config**: `_apply_robot_config()` charge `data/robot_config.json` section `navigation` au boot et override les paramètres ROS2
- **[H2] Dead code States**: `State.NAVIGATE_TAG`, `NAVIGATE_DROP`, `BACK_HOME` supprimés de l'enum; `_step_navigate_tag()` et `_step_back_home()` retirés
- **[H3] PID dt hardcodé**: `PIDController.compute()` utilise `time.perf_counter()` pour dt réel; fallback `0.05s` si gap > 0.5s
- **[H4] Seuils obstacles incohérents**: `robot_config.json` section `navigation` ajoutée — `obstacle_threshold_near=0.08m` (8cm) aligné sur `STOP_DIST_CM` de `mission_engine`
- **[H5] /velocity UI brise mission**: Retourne 409 + message explicite si mission active
- **[M1] Anti-windup PID trop large**: Borne dynamique `min(5.0, output_limit/ki)` remplace le clamp fixe ±50
- **[M2] IA training thread sur Pi**: `industrial_ai.py` détecte ARM/Raspberry Pi → `INFERENCE_ONLY=True` → thread d'entraînement désactivé
- **[M3] IMU callbacks 100Hz**: `task_manager_node._imu_cb` bufferise, flush vers engine à 20 Hz
- **[M4] Detect cube sans timeout**: `_step_detect_cube` timeout 15s → `State.ERROR`
- **[M5] SCAN_360 efface prior map**: `tag_map.tags.clear()` seulement si la map était vide — fusionne sinon
- **[L2] save_freq I/O SD card**: `save_freq 50→300` (écriture toutes les ~15s à 20Hz au lieu de 2.5s)
- **[L3] PIDController.reset()**: `waypoint_callback` utilise `pid_linear.reset()` / `pid_angular.reset()` au lieu d'accès direct aux champs

### Added
- **`data/robot_config.json` section `navigation`**: PID linéaire/angulaire, `max_linear_speed`, `max_angular_speed`, `obstacle_threshold_near/far` — source de vérité unique pour `navigation_node`
- **`test_pid_controller.py`**: 6 tests unitaires pour `PIDController` (proportionnel, clamping, reset, anti-windup, dt fallback, dt réel)
- **`test_astar_path.py`**: 8 tests unitaires pour `astar_path` et `path_to_waypoints` (same-start, direct, grille 4 nœuds, map vide, conversion cm→m)
- **`_is_raspberry_pi()`**: Détection plateforme ARM dans `industrial_ai.py` pour mode inference-only automatique
- **`import rclpy.parameter`**: Import explicite dans `navigation_node.py` pour `_apply_robot_config()`

## [7.0] — 2026-05-24

### Added
- **Global config system**: `data/robot_config.json` — source de vérité unique pour PID/trims/minPWM/rampe/ia_trims, partagée entre `test_PID_auto` et `flask_ros_bridge`
- **Auto-save pipeline**: Le bouton Save dans `test_PID_auto` déclenche automatiquement 4 étapes en séquence : EEPROM ESP32 → robot_config.json → drive_assist_model.pt → export INT8 (background)
- **Export INT8 automatique**: `_export_int8_background()` lance `export_rpi.py` en thread et notifie l'UI via `save_status`
- **`GET /api/config`**: Nouvel endpoint dans `flask_ros_bridge.py` pour lire la config globale
- **Boot config push**: `flask_ros_bridge.py` charge `robot_config.json` au démarrage et envoie la config sur `/robot_cfg` vers l'ESP32 micro-ROS
- **Save status UI**: Console de l'UI affiche l'état de chaque étape de sauvegarde (✅/⏳/❌)

### Changed
- **`test_PID_auto/app.py`**: Variables PID/trims/minPWM initialisées depuis `robot_config.json` au lieu des valeurs hardcodées
- **`test_PID_auto/app.py`**: `handle_manual_pid` accepte et persiste aussi `ta/tb/ma/mb` (trims + minPWM)
- **`test_PID_auto/app.py`**: `handle_save_config` persiste `ia_trims` appris dans le JSON global
- **`flask_ros_bridge.py`**: `POST /api/config` persiste les changements dans `robot_config.json` en plus de publier sur `/robot_cfg`
- **`flask_ros_bridge.py`**: `MISSIONS_FILE` utilise `_DATA_ROOT` partagé (refactoring path)
- **Prior Map**: `PRIOR_MAP` dans `auto_detetc_tag_arduino.py` corrigé avec les vraies positions du scan confirmé, recentré sur ID12=(0,0). ID17 supprimé
- **`data/tags_slam/tags_slam.json`**: Mis à jour avec les positions corrigées du scan confirmé
- **PID defaults**: Ki abaissé de 0.08 à 0.02 dans `robot_config.json` (moins d'intégration, moins de dérive)
- **Kd default**: Kd abaissé de 0.6 à 0.7 dans `robot_config.json`

### Fixed
- **Flask/front-end PID field mismatch**: Front-end envoyait `kp/ki/kd`, Flask attendait `ykp/yki/ykd` → corrigé des deux côtés
- **Flask écrasait trims/minPWM**: `handle_manual_pid` envoyait seulement le PID et remettait les trims à zéro → corrigé
- **Arduino firmware `cfg` parsing**: Firmware remettait les paramètres à 0 si absents du JSON → parsé conditionnellement
- **Front-end `ia_update` écrasait PID**: Mise à jour des champs PID à "0.00" si non présents dans le payload → corrigé
- **`MOTOR_A/B_MINPWM` micro-ROS**: Défaut 35 à 55 dans `micro_ros_esp32Robot.ino` (35 trop faible au sol)
- **`mission_engine.py`**: `load_prior()` appelé après instanciation de `TagMapSLAM`
- **`color_detection_test.py`**: `NameError` `ARUCO_TAG_SIZE/CAM_MATRIX/DIST_COEFFS`, `AttributeError` Enum, `IndexError` `tvec` flatten

## [6.0] — 2026-05-23

### Added
- **A* Pathfinding**: Graphe complet des 12 tags, chemin le plus court entre n'importe quels deux tags
- **SLAM-based localization**: RobotTracker intégré dans mission_engine (camera + IMU + optical flow)
- **Scan 360° SLAM**: Cartographie automatique des 12 tags avec positions absolues
- **Waypoint navigation**: NAVIGATE_WAYPOINT state pour suivre le chemin A* waypoint par waypoint
- **Mission queue**: Cycle 2 cubes (blue → green → HOME) avec alternance LSTM
- **Missions config file**: `data/missions/missions.json` — missions configurables par l'utilisateur
- **Missions API**: `GET/POST /api/missions` — lire/modifier les missions depuis l'UI
- **Mission editor UI**: Panneau de configuration des missions dans l'interface web
- **ArduinoReader fallback**: _DummyArduino si l'Arduino n'est pas connecté
- **AGENT.md**: Guide pour les agents IA travaillant sur le codebase
- **CHANGELOG.md**: Ce fichier

### Changed
- **AprilTag dictionary**: Tous les fichiers utilisent maintenant `DICT_4X4_250` (était `DICT_APRILTAG_36H11` ou `DICT_4X4_50`)
- **Camera backend**: `cv2.CAP_DSHOW` sur Windows dans tous les fichiers (était bloquant)
- **Colors**: Cyan = bleu, deux couleurs de cubes: bleu et vert
- **Tag roles**: 12=HOME, 3=Manufacture, 6=Station B (bleu), 9=Station A (vert)
- **Navigation**: `_navigate_to()` utilise maintenant le tracker SLAM au lieu de l'odométrie dead reckoning
- **Scan**: `_step_scan()` utilise le SLAM au lieu de la rotation simple
- **Mission cycle**: 2 pickups par cycle (bleu + vert), alternance par station

### Fixed
- **Tag detection**: Dictionnaire corrigé de `DICT_APRILTAG_36H11` vers `DICT_4X4_250` dans 6 fichiers
- **Camera blocking**: Ajout de `cv2.CAP_DSHOW` dans april_tag_pose.py, april_tag_slam.py, april_tagçlocalisation.py
- **Touch conflicts**: `S` ne clash plus entre scan et reculer (scan = `C`, reculer = `S`)
- **Tag positions**: SLAM positions mises à jour avec les vraies positions du scan
- **Dead code cleanup**: Suppression des états NAVIGATE_TAG, NAVIGATE_DROP, BACK_HOME (remplacés par NAVIGATE_WAYPOINT)
- **ESP32 WiFi AP display**: `test_PID_auto/arduino/arduino.ino` affiche maintenant le SSID / IP / mot de passe au démarrage
- **Drive assist load error**: `test_PID_auto/industrial_ai.py` corrige le chargement modèle PyTorch 2.6+
- **PWM startup threshold**: `test_PID_auto/arduino/arduino.ino` augmente MINPWM 35 -> 55 pour démarrer au sol
- **Rotation / speed control**: `test_PID_auto/templates/index.html` corrige le D-Pad (rotation indépendante) et met à jour la commande quand le slider change pendant une touche

## [5.0] — 2026-05-20

### Added
- 4 capteurs ultrasons HC-SR04 (position 45°)
- Compatibilité Raspberry Pi 4B (inférence optimisée)
- Visualisation trajectoire temps réel
- Kalman Filter pour lissage position
- UnifiedRLAgent (détection auto PC/Pi)
- LSTM advisory assistant

### Changed
- Architecture distribuée ESP32 + RPi
- Communication micro-ROS via Serial USB

## [4.0] — 2026-05-15

### Added
- IA Deep RL avec compensation moteur
- PID yaw auto-verrouillage
- Rampe d'accélération configurable
- Flask web interface

## [3.0] — 2026-05-10

### Added
- AprilTag SLAM avec optical flow
- Camera + IMU fusion pour localisation
- Scan 360° automatique
- Cartographie des tags

## [2.0] — 2026-05-05

### Added
- ROS2 + micro-ROS architecture
- Camera node avec détection AprilTag
- Color detection (HSV)
- Gripper servo control

## [1.0] — 2026-05-01

### Added
- ESP32 firmware avec PID moteur
- IMU BMI160 integration
- Communication série de base
- Premier test de mouvement
