# 🤖 GUIDE DE RÉGLAGE — Robot Voiture 3 Roues Contrôle Assisté

> **Note:** Ce guide couvre le contrôle moteur PID. Pour la navigation autonome,
> le robot utilise maintenant **A* pathfinding** + **SLAM** (camera + IMU + optical flow).
> Voir `AGENT.md` pour l'architecture complète.

## Architecture

```
         Joystick Y ──→ vitesse cible (avec rampe douce)
         Joystick X ──→ rotation manuelle
                    ↓
    ┌─────────────────────────────────────┐
    │  Rampe d'accélération intelligente  │  ← Anti-dérapage bille
    │  (montée progressive des PWM)       │
    └─────────────────┬───────────────────┘
                      ↓
    ┌─────────────────────────────────────┐
    │  Si on va droit (X ≈ 0, Y ≠ 0):    │
    │   Verrouiller le cap yaw            │
    │   PID yaw corrige la dérive         │
    │     → accélère 1 roue               │
    │     → ralentit l'autre              │
    └─────────────────┬───────────────────┘
                      ↓
    ┌─────────────────────────────────────┐
    │  Corrections IA (optionnel)         │
    │   • trim_L/R: compensation moteurs  │
    │   • feedforward: anticipation       │
    │   • ramp_boost: ajuste douceur      │
    │   • accel_smooth: lissage           │
    └─────────────────┬───────────────────┘
                      ↓
      [Moteur Gauche] [Moteur Droit] + [Bille avant]
```

---

## 🔧 Étape 1: Régler la Rampe d'Accélération (CRITIQUE)

**Objectif:** La bille avant ne doit pas déraper quand tu accélères.

### Valeurs par défaut
```
ramp_speed (montée):   80 PWM/sec
ramp_brake (freinage): 150 PWM/sec  
ramp_neutral (retour): 120 PWM/sec
```

### Test
1. Pousser le joystick en avant progressivement → la voiture doit avancer sans bruit de dérapage
2. Si la bille "glisse" ou fait un bruit de frottement:
   ```json
   {"t":"cfg","rs":50,"rb":100,"rn":80}
   ```
3. Si la voiture met trop de temps à démarrer:
   ```json
   {"t":"cfg","rs":120,"rb":200,"rn":150}
   ```

### Tableau référence

| Symptôme | Solution |
|----------|----------|
| Bille dérape au démarrage | ↓ rs (ex: 80 → 50 → 30) |
| Démarrage trop lent | ↑ rs (ex: 80 → 120 → 150) |
| Freinage brutal | ↓ rb |
| Retour au neutre trop lent | ↑ rn |

---

## 🔧 Étape 2: Régler le PID Yaw (Direction)

**Objectif:** Quand tu avances tout droit, le robot ne dévie pas.

### Valeurs de départ
```
Kp = 4.0, Ki = 0.02, Kd = 0.7
```
> Ces valeurs sont chargées automatiquement depuis `data/robot_config.json` au démarrage.

### Procédure
1. Avancer tout droit avec joystick Y (sans X)
2. Observer si le robot dévie:
   - Vire à droite → augmente trim gauche ou Kp
   - Vire à gauche → augmente trim droite ou Kp

### Ajustements PID

| Symptôme | Ajustement |
|----------|----------|
| Dérive non corrigée | ↑ Kp (+1 à 2) |
| Oscille gauche-droite | ↓ Kp, ↑ Kd |
| Correction lente | ↑ Kp, ↑ Ki |
| Zigzag rapide | ↓ Kp fort, ↑ Kd |

```json
{"t":"cfg","ykp":6,"yki":0.1,"ykd":0.8}
```

---

## 🔧 Étape 3: Régler les Trims Moteurs (Anti-dérive mécanique)

Si le robot tourne toujours du même côté même avec PID:

```json
{"t":"cfg","ta":5,"tb":0}   // Moteur A 5% plus fort
```

Ajuste par pas de 2-3 jusqu'à ce que le robot avance droit sans correction PID excessive.

---

## 🔧 Étape 4: Sauvegarder

Cliquer le bouton **Save** dans l'UI (ou envoyer `{"t":"save"}`).

Cela déclenche automatiquement **4 étapes** :

| Étape | Résultat |
|-------|----------|
| ✅ EEPROM ESP32 | PID + trims + rampe persistants sur le firmware |
| ✅ `data/robot_config.json` | Config globale partagée (lue par micro-ROS au boot) |
| ✅ `drive_assist_model.pt` | Modèle IA complet sauvegardé |
| ⏳ `drive_assist_rpi_int8.pt` | Export INT8 TorchScript pour Raspberry Pi (background ~5-10s) |

---

## 🧠 Étape 5: IA Contrôle Assisté (Optionnel)

**⚠️ PRÉREQUIS: La voiture avance déjà droit en mode manuel.**

### Activation
1. Cliquer **"🧠 Activer IA Contrôle Assisté"**
2. Conduire normalement (avancer, tourner, freiner)
3. L'IA observe et apprend

### Ce que l'IA apprend

```
trim_L/R   → Compense la différence moteur (ex: moteur gauche plus faible) — max ±15 PWM
ramp_boost → Adapte la douceur de l'accélération — ±0.5 max
```

> **Note:** feedforward et accel_smooth ne font plus partie du modèle (supprimés en v6). L'IA a 3 outputs : `trim_L`, `trim_R`, `ramp_boost`.

### Lecture des logs
```
R:+1.23 | σ:0.45 | tr:(+3.0,-1.5) | ramp:-0.2 | stab:87% | mem:512
```

- **R:** Reward (stabilité + fluidité)
- **σ:** Exploration (diminue = moins d'aléatoire)
- **tr:** Trims moteurs appliqués
- **ramp:** Boost rampe (négatif = plus doux)
- **stab:** Score stabilité (%)
- **mem:** Taille du replay buffer

### Comportement attendu

**Au début (σ élevé):**
- L'IA teste des corrections aléatoires
- La conduite peut être moins bonne

**Après 5-10 minutes (σ diminue):**
- L'IA applique les corrections apprises
- La voiture devient plus stable

**Après 20+ minutes (σ bas):**
- Corrections affinées
- Sauvegarde auto du modèle

---

## 📡 Protocole JSON

### Direction (joystick)
```json
{"t":"dir","x":0.0,"y":0.5}   // x: rotation, y: vitesse (-1 à 1)
```

### Configurer Rampe d'accélération
```json
{"t":"cfg","rs":80,"rb":150,"rn":120}
// rs=ramp_speed(montée), rb=ramp_brake, rn=ramp_neutral
```

### Configurer PID Yaw
```json
{"t":"cfg","ykp":4,"yki":0.02,"ykd":0.7,"ta":0,"tb":0}
```

### Configurer Anti-calage
```json
{"t":"cfg","ma":55,"mb":55}   // PWM minimum pour éviter le calage (55 minimum au sol)
```

### Corrections IA (envoyé automatiquement)
```json
{"t":"ia","tl":3.5,"tr":-1.2,"rbst":-0.15}
```

### Désactiver IA
```json
{"t":"ia","active":0}
```

### Sauvegarder EEPROM
```json
{"t":"save"}
```

---

## ⚠️ Sécurités

- **Rampe d'accélération:** Limite la montée en puissance (pas de 0 à 100% instantané)
- **PID Yaw:** Correction max ±120 PWM (40% de la vitesse max) — wrap ±180° géré (fix v8)
- **IA Bornes:** trims ±15 PWM, ramp_boost ±0.5
- **Timeout IA:** Si pas de message IA depuis 2s → corrections désactivées
- **Fallback:** Le contrôle manuel (PID + rampe) fonctionne toujours

---

## 🎯 Valeurs recommandées par défaut

```
PID Yaw:    Kp=4.0,   Ki=0.02, Kd=0.7
Rampe:      rs=80,    rb=120,  rn=200
Trims:      A=0,      B=0
MinPWM:     A=55,     B=55
```

Ces valeurs sont dans `data/robot_config.json` — modifie-les là ou via l'UI (puis Save).

---

## 🆘 Dépannage rapide

| Problème | Cause | Solution |
|----------|-------|----------|
| Bille dérape au démarrage | Accélération trop brutale | ↓ `rs` (40-60) |
| Bille dérape en tournant | Vitesse trop haute en courbe | Réduire joystick X |
| Robot part en vrille | PID yaw trop fort | ↓ `ykp` |
| Robot ne tourne pas assez | PID yaw trop faible | ↑ `ykp` |
| Un moteur ne démarre pas | PWM minimum trop bas | ↑ `ma` ou `mb` |
| IA rend la conduite pire | Phase d'exploration | Attendre ou désactiver |

---

**Bonne route ! 🚗**

Règle d'or: **Rampe douce d'abord, PID ensuite, IA en dernier.**
