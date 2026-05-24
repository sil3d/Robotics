import os
import platform
import random
import threading
import time
from collections import deque
from dataclasses import dataclass, asdict
from typing import Optional, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim


def _is_raspberry_pi() -> bool:
    """Détecte si on tourne sur un Raspberry Pi (ARM) pour mode inference-only."""
    machine = platform.machine().lower()
    if 'arm' in machine or 'aarch64' in machine:
        return True
    try:
        with open('/proc/cpuinfo', 'r') as f:
            return 'raspberry pi' in f.read().lower()
    except Exception:
        return False


INFERENCE_ONLY = _is_raspberry_pi()


@dataclass
class DriveAssistConfig:
    """Configuration IA conservatrice et robuste."""
    # Dimensions
    state_dim: int = 10
    action_dim: int = 3           # [trim_L, trim_R, ramp_boost] (feedforward et smooth retirés pour simplifier)
    hidden_dim: int = 64
    
    # Learning
    lr_actor: float = 3e-4        # Plus rapide pour apprendre vite
    lr_critic: float = 1e-3
    gamma: float = 0.95
    tau: float = 0.005
    
    # Exploration modérée
    noise_theta: float = 0.15
    noise_sigma: float = 0.20
    noise_scale_start: float = 0.50
    noise_scale_end: float = 0.05
    noise_decay: float = 0.995
    
    # Replay
    replay_capacity: int = 5000
    batch_size: int = 32
    min_replay_size: int = 100
    train_freq: int = 4
    
    # Actions limitées (assez pour compenser, pas assez pour tourner)
    max_trim: float = 15.0        # Max ±15 PWM
    max_ramp_boost: float = 0.5   # -0.5 à +0.5
    
    # Régularisation
    l2_lambda: float = 0.005      # Pénalité L2 sur les actions
    action_smoothness_weight: float = 0.05  # Pénalité sur variation actions
    
    save_path: str = "drive_assist_model.pt"
    save_freq: int = 300  # L2: écrit ~toutes les 15s à 20Hz (anciennement 50 = 2.5s)


class ActorNetwork(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh()
        )
    def forward(self, state):
        return self.net(state)


class CriticNetwork(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )
    def forward(self, state, action):
        return self.net(torch.cat([state, action], dim=-1))


class ReplayBuffer:
    def __init__(self, capacity=5000):
        self.buffer = deque(maxlen=capacity)
        self._lock = threading.Lock()
    def push(self, state, action, reward, next_state, done):
        with self._lock:
            self.buffer.append((state, action, reward, next_state, done))
    def sample(self, batch_size):
        with self._lock:
            batch = random.sample(self.buffer, min(batch_size, len(self.buffer)))
        states, actions, rewards, next_states, dones = zip(*batch)
        return (np.array(states, dtype=np.float32), np.array(actions, dtype=np.float32),
                np.array(rewards, dtype=np.float32), np.array(next_states, dtype=np.float32),
                np.array(dones, dtype=np.float32))
    def __len__(self):
        with self._lock:
            return len(self.buffer)


class OUNoise:
    def __init__(self, size, theta=0.10, sigma=0.15):
        self.size = size; self.theta = theta; self.sigma = sigma
        self.state = np.zeros(size)
    def reset(self):
        self.state = np.zeros(self.size)
    def sample(self):
        self.state += self.theta * (-self.state) + self.sigma * np.random.randn(self.size)
        return self.state.copy()


class IndustrialRLAgent:
    """
    IA de Contrôle Assisté — VERSION CONSERVATRICE ET ROBUSTE.
    
    Principes:
    1. Petites corrections seulement (max ±8 PWM)
    2. Pénalité forte sur les actions excessives
    3. Pénalité sur la variation des actions (smoothness)
    4. Exploration très douce
    5. Régularisation L2 dans le loss
    """
    
    def __init__(self, config: Optional[DriveAssistConfig] = None):
        self.config = config or DriveAssistConfig()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"[IA v2.0 Conservatrice] Démarrage sur: {self.device}")
        
        self.actor = ActorNetwork(self.config.state_dim, self.config.action_dim, self.config.hidden_dim).to(self.device)
        self.actor_target = ActorNetwork(self.config.state_dim, self.config.action_dim, self.config.hidden_dim).to(self.device)
        self.actor_target.load_state_dict(self.actor.state_dict())
        
        self.critic = CriticNetwork(self.config.state_dim, self.config.action_dim, self.config.hidden_dim).to(self.device)
        self.critic_target = CriticNetwork(self.config.state_dim, self.config.action_dim, self.config.hidden_dim).to(self.device)
        self.critic_target.load_state_dict(self.critic.state_dict())
        
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=self.config.lr_actor, weight_decay=1e-5)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=self.config.lr_critic, weight_decay=1e-5)
        
        self.memory = ReplayBuffer(capacity=self.config.replay_capacity)
        self.noise = OUNoise(size=self.config.action_dim, theta=self.config.noise_theta, sigma=self.config.noise_sigma)
        self.noise_scale = self.config.noise_scale_start
        
        self.last_state = None
        self.last_action = None
        self.step_count = 0
        self.episode_count = 0
        self.episode_reward = 0.0
        
        # Valeurs apprises (init à 0)
        self.trim_L = 0.0
        self.trim_R = 0.0
        self.ramp_boost = 0.0
        
        self.best_reward = float('-inf')
        
        # Historique pour smoothness
        self.prev_action = np.zeros(self.config.action_dim)
        self.yaw_err_history = deque(maxlen=20)
        
        self._load_or_init()

        self._stop_training = threading.Event()
        # M2: ne pas lancer l'entraînement en arrière-plan sur Pi (inference-only)
        if INFERENCE_ONLY:
            print("[IA] Mode inference-only (Raspberry Pi détecté) — training thread désactivé")
            self._training_thread = None
        else:
            self._training_thread = threading.Thread(target=self._background_training, daemon=True)
            self._training_thread.start()
    
    def _load_or_init(self):
        path = self.config.save_path
        if os.path.exists(path):
            try:
                ckpt = torch.load(path, map_location=self.device, weights_only=False)
                self.actor.load_state_dict(ckpt['actor'])
                self.actor_target.load_state_dict(ckpt['actor_target'])
                self.critic.load_state_dict(ckpt['critic'])
                self.critic_target.load_state_dict(ckpt['critic_target'])
                self.noise_scale = ckpt.get('noise_scale', self.config.noise_scale_start)
                self.step_count = ckpt.get('step', 0)
                self.episode_count = ckpt.get('episode', 0)
                self.best_reward = ckpt.get('best_reward', float('-inf'))
                learned = ckpt.get('learned', {})
                self.trim_L = learned.get('trim_L', 0.0)
                self.trim_R = learned.get('trim_R', 0.0)
                self.ramp_boost = learned.get('ramp_boost', 0.0)
                print(f"[IA] Modèle chargé — épisodes:{self.episode_count} | trims:({self.trim_L:.1f},{self.trim_R:.1f}) | ramp:{self.ramp_boost:+.2f}")
            except Exception as e:
                print(f"[IA] Erreur chargement: {e}")
        else:
            print("[IA] Nouveau modèle (conservateur)")
    
    def _save(self, force=False):
        ckpt = {
            'actor': self.actor.state_dict(), 'actor_target': self.actor_target.state_dict(),
            'critic': self.critic.state_dict(), 'critic_target': self.critic_target.state_dict(),
            'noise_scale': self.noise_scale, 'step': self.step_count, 'episode': self.episode_count,
            'best_reward': self.best_reward,
            'learned': {'trim_L': self.trim_L, 'trim_R': self.trim_R, 'ramp_boost': self.ramp_boost},
        }
        torch.save(ckpt, self.config.save_path)
        if force:
            print(f"[IA] Sauvegardé épisode {self.episode_count}")
    
    def _compute_reward(self, yaw_error, yaw_rate, motor_L, motor_R, action, is_locked) -> float:
        """Reward simple: récompense la stabilité, pénalise la dérive et les actions excessives."""
        if not is_locked:
            # Quand on tourne manuellement: petit reward neutre
            return 0.0
        
        # Stocke historique
        self.yaw_err_history.append(yaw_error)
        
        # REWARD PRINCIPAL: stabilité du cap
        # Plus le robot va droit, plus le reward est élevé
        reward = 2.0 - abs(yaw_error) * 0.1 - abs(yaw_rate) * 0.02
        
        # Bonus si très stable
        if abs(yaw_error) < 3.0:
            reward += 2.0
        
        # Pénalité actions excessives (empêche la divergence)
        trim_mag = abs(action[0]) + abs(action[1])
        if trim_mag > 10.0:
            reward -= (trim_mag - 10.0) * 0.3  # Pénalité progressive
        
        return float(np.clip(reward, -5.0, 5.0))
    
    def get_action_and_learn(self, telemetry: dict) -> Tuple[float, float, float, str]:
        yaw = float(telemetry.get('y', 0.0))
        yaw_rate = float(telemetry.get('yr', 0.0))
        ax = float(telemetry.get('ax', 0)); ay = float(telemetry.get('ay', 0)); az = float(telemetry.get('az', 0))
        motor_L = float(telemetry.get('ol', 0.0))
        motor_R = float(telemetry.get('or', 0.0))
        is_locked = int(telemetry.get('lk', 0)) == 1
        dir_y = float(telemetry.get('dy', 0.0))
        dir_x = float(telemetry.get('dx', 0.0))
        ramp_speed = float(telemetry.get('rs', 80.0))
        
        yaw_error = 0.0
        if is_locked and 'locked_yaw' in telemetry:
            yaw_error = yaw - telemetry['locked_yaw']
            while yaw_error > 180: yaw_error -= 360
            while yaw_error < -180: yaw_error += 360
        
        # State
        state = np.array([
            np.clip(yaw_error / 45.0, -1, 1),
            np.clip(yaw_rate / 100.0, -1, 1),
            np.clip(ax / 32768.0, -1, 1),
            np.clip(ay / 32768.0, -1, 1),
            np.clip(az / 32768.0, -1, 1),
            np.clip(motor_L / 255.0, -1, 1),
            np.clip(motor_R / 255.0, -1, 1),
            np.clip(dir_y, -1, 1),
            np.clip(dir_x, -1, 1),
            np.clip(ramp_speed / 200.0, 0, 1),
        ], dtype=np.float32)
        
        # Get action from actor
        state_t = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        self.actor.eval()
        with torch.no_grad():
            action_raw = self.actor(state_t).cpu().numpy()[0]
        self.actor.train()
        
        # Add exploration noise (très doux)
        noise = self.noise.sample() * self.noise_scale
        action = np.clip(action_raw + noise, -1.0, 1.0)
        
        # Decay noise
        self.noise_scale = max(self.config.noise_scale_end, self.noise_scale * self.config.noise_decay)
        
        # Compute reward BEFORE updating (pour avoir prev_action)
        reward = self._compute_reward(yaw_error, yaw_rate, motor_L, motor_R, action, is_locked)
        self.episode_reward += reward
        
        # Store transition
        if self.last_state is not None and self.last_action is not None:
            done = (abs(yaw_error) > 45.0) if is_locked else False
            self.memory.push(self.last_state, self.last_action, reward, state, done)
            if done:
                self.episode_count += 1
                if self.episode_reward > self.best_reward:
                    self.best_reward = self.episode_reward
                    self._save(force=True)
                self.episode_reward = 0.0
                self.noise.reset()
                self.yaw_err_history.clear()
        
        # Update learned values with strong smoothing
        alpha = 0.2  # Très lisse = très conservateur
        self.trim_L = (1-alpha) * self.trim_L + alpha * action[0] * self.config.max_trim
        self.trim_R = (1-alpha) * self.trim_R + alpha * action[1] * self.config.max_trim
        self.ramp_boost = (1-alpha) * self.ramp_boost + alpha * action[2] * self.config.max_ramp_boost
        
        # Hard clamping
        self.trim_L = np.clip(self.trim_L, -self.config.max_trim, self.config.max_trim)
        self.trim_R = np.clip(self.trim_R, -self.config.max_trim, self.config.max_trim)
        self.ramp_boost = np.clip(self.ramp_boost, -self.config.max_ramp_boost, self.config.max_ramp_boost)
        
        # Update prev_action
        self.prev_action = action.copy()
        
        self.last_state = state
        self.last_action = action
        self.step_count += 1
        
        if self.step_count % self.config.save_freq == 0:
            self._save()
        
        stability = max(0, 100 - abs(yaw_error) * 5)
        log = (f"R:{reward:+.2f} | σ:{self.noise_scale:.2f} | "
               f"tr:({self.trim_L:+.1f},{self.trim_R:+.1f}) | ramp:{self.ramp_boost:+.2f} | "
               f"stab:{stability:.0f}% | mem:{len(self.memory)}")
        
        return float(self.trim_L), float(self.trim_R), float(self.ramp_boost), log
    
    def get_learned_values(self):
        return self.trim_L, self.trim_R, self.ramp_boost
    
    def reset_episode(self):
        self.episode_reward = 0.0
        self.noise.reset()
        self.prev_action = np.zeros(self.config.action_dim)
        self.yaw_err_history.clear()
    
    def _soft_update(self, target, source):
        for tp, p in zip(target.parameters(), source.parameters()):
            tp.data.copy_(self.config.tau * p.data + (1.0 - self.config.tau) * tp.data)
    
    def _train_step(self):
        if len(self.memory) < max(self.config.min_replay_size, self.config.batch_size):
            return None
        
        states, actions, rewards, next_states, dones = self.memory.sample(self.config.batch_size)
        states_t = torch.FloatTensor(states).to(self.device)
        actions_t = torch.FloatTensor(actions).to(self.device)
        rewards_t = torch.FloatTensor(rewards).unsqueeze(1).to(self.device)
        next_states_t = torch.FloatTensor(next_states).to(self.device)
        dones_t = torch.FloatTensor(dones).unsqueeze(1).to(self.device)
        
        # Critic update
        with torch.no_grad():
            next_actions = self.actor_target(next_states_t)
            target_q = self.critic_target(next_states_t, next_actions)
            target_q = rewards_t + self.config.gamma * target_q * (1 - dones_t)
        
        current_q = self.critic(states_t, actions_t)
        critic_loss = nn.MSELoss()(current_q, target_q)
        
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), max_norm=1.0)
        self.critic_optimizer.step()
        
        # Actor update with action regularization
        pred_actions = self.actor(states_t)
        # Q-value maximization
        q_value = self.critic(states_t, pred_actions)
        # Régularisation L2 sur les actions (pénalise les actions grandes)
        l2_penalty = self.config.l2_lambda * (pred_actions ** 2).mean()
        actor_loss = -q_value.mean() + l2_penalty
        
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=1.0)
        self.actor_optimizer.step()
        
        self._soft_update(self.actor_target, self.actor)
        self._soft_update(self.critic_target, self.critic)
        
        return {'critic_loss': critic_loss.item(), 'actor_loss': actor_loss.item(), 'l2_pen': l2_penalty.item()}
    
    def _background_training(self):
        while not self._stop_training.is_set():
            if len(self.memory) >= self.config.min_replay_size:
                for _ in range(self.config.train_freq):
                    self._train_step()
                    time.sleep(0.05)
            time.sleep(0.1)
    
    def get_training_stats(self):
        return {
            'episodes': self.episode_count, 'steps': self.step_count,
            'noise_scale': self.noise_scale, 'best_reward': self.best_reward,
            'memory_size': len(self.memory),
            'trim_L': self.trim_L, 'trim_R': self.trim_R,
            'ramp_boost': self.ramp_boost,
        }
    
    def stop(self):
        self._stop_training.set()
        self._save(force=True)
        if self._training_thread is not None and self._training_thread.is_alive():
            self._training_thread.join(timeout=2.0)
    def __del__(self):
        self.stop()


class IndustrialRLAgentLegacy(IndustrialRLAgent):
    def __init__(self):
        super().__init__(DriveAssistConfig())


class UnifiedRLAgent:
    """
    Agent unifié qui fonctionne sur PC et Raspberry Pi
    - PC: Utilise IndustrialRLAgent avec entraînement actif
    - Raspberry Pi: Charge le modèle TorchScript quantifié (inférence uniquement)
    """
    def __init__(self, config: Optional[DriveAssistConfig] = None):
        self.config = config or DriveAssistConfig()
        self.is_raspberry_pi = self._detect_raspberry_pi()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        if self.is_raspberry_pi:
            self._init_raspberry_pi()
        else:
            self._init_pc()
    
    def _detect_raspberry_pi(self) -> bool:
        """Détecte si on tourne sur Raspberry Pi (y compris Pi 4B)"""
        import platform
        import os
        
        # Méthode 1: Détection via platform.machine() (fonctionne sur Linux/Windows)
        machine = platform.machine().lower()
        if 'arm' in machine or 'aarch64' in machine:
            return True
        
        # Méthode 2: Détection via /proc/cpuinfo (Linux uniquement)
        # Pi 4B: BCM2711, Cortex-A72
        try:
            with open('/proc/cpuinfo', 'r') as f:
                cpuinfo = f.read()
                pi_markers = ['ARM', 'Raspberry', 'BCM2', 'BCM2711', 'Cortex-A72']
                if any(marker in cpuinfo for marker in pi_markers):
                    return True
        except:
            pass
        
        # Méthode 3: Détection via /sys/firmware/devicetree/base/model
        # C'est la méthode la plus fiable pour Pi 4B
        try:
            with open('/sys/firmware/devicetree/base/model', 'r') as f:
                model = f.read().strip().lower()
                if 'raspberry pi' in model or 'raspberrypi' in model:
                    return True
        except:
            pass
        
        # Méthode 4: Variable d'environnement (pour forcer le mode)
        if os.environ.get('RPI_MODE', '').lower() in ['1', 'true', 'yes']:
            return True
        
        return False
    
    def _get_pi_model(self) -> str:
        """Détecte le modèle spécifique de Raspberry Pi"""
        try:
            with open('/sys/firmware/devicetree/base/model', 'r') as f:
                return f.read().strip()
        except:
            pass
        
        try:
            with open('/proc/cpuinfo', 'r') as f:
                for line in f:
                    if 'Model' in line:
                        return line.split(':')[1].strip()
        except:
            pass
        
        return "Raspberry Pi (modèle inconnu)"
    
    def _init_raspberry_pi(self):
        """Initialise le modèle TorchScript pour Raspberry Pi"""
        pi_model = self._get_pi_model()
        print(f"[IA] {pi_model} détecté")
        
        # Configuration quantification optimisée pour Pi 4B
        try:
            torch.backends.quantized.engine = 'qnnpack'
            print(f"[IA] Moteur QNNPACK activé (optimisé ARM)")
        except Exception as e:
            print(f"[IA] Note: {e}")
        
        # Chargement du modèle TorchScript
        model_path = "drive_assist_rpi_int8.pt"
        if os.path.exists(model_path):
            self.scripted_model = torch.jit.load(model_path)
            self.scripted_model.eval()
            print(f"[IA] Modèle TorchScript chargé: {model_path}")
            self.training_enabled = False
        else:
            print(f"[IA] ⚠️ Modèle {model_path} non trouvé!")
            print(f"[IA]    Exécutez sur PC: python export_rpi.py")
            print(f"[IA]    Puis transférez le fichier .pt sur le Pi")
            raise FileNotFoundError(f"Modèle {model_path} requis sur Raspberry Pi")
        
        # Valeurs apprises (pas d'entraînement sur Pi)
        self.trim_L = 0.0
        self.trim_R = 0.0
        self.ramp_boost = 0.0
        self.step_count = 0
        self.episode_count = 0
    
    def _init_pc(self):
        """Initialise l'agent complet pour PC (avec entraînement)"""
        print(f"[IA] Mode PC détecté - Entraînement actif")
        self.agent = IndustrialRLAgent(self.config)
        self.training_enabled = True
    
    def get_action_and_learn(self, telemetry: dict) -> Tuple[float, float, float, str]:
        """
        Interface unifiée: retourne (trim_L, trim_R, ramp_boost, log_msg)
        """
        if self.is_raspberry_pi:
            return self._inference_pi(telemetry)
        else:
            return self.agent.get_action_and_learn(telemetry)
    
    def _inference_pi(self, telemetry: dict) -> Tuple[float, float, float, str]:
        """Inférence rapide sur Raspberry Pi"""
        import time
        start = time.perf_counter()
        
        # Extraction des features (même normalisation que sur PC)
        yaw = float(telemetry.get('y', 0.0))
        yaw_rate = float(telemetry.get('yr', 0.0))
        ax = float(telemetry.get('ax', 0))
        ay = float(telemetry.get('ay', 0))
        az = float(telemetry.get('az', 0))
        motor_L = float(telemetry.get('ol', 0.0))
        motor_R = float(telemetry.get('or', 0.0))
        dir_y = float(telemetry.get('dy', 0.0))
        dir_x = float(telemetry.get('dx', 0.0))
        ramp_speed = float(telemetry.get('rs', 80.0))
        
        # Normalisation identique à industrial_ai.py
        is_locked = int(telemetry.get('lk', 0)) == 1
        yaw_error = 0.0
        if is_locked and 'locked_yaw' in telemetry:
            yaw_error = yaw - telemetry['locked_yaw']
            while yaw_error > 180: yaw_error -= 360
            while yaw_error < -180: yaw_error += 360
        
        state = np.array([
            np.clip(yaw_error / 45.0, -1, 1),
            np.clip(yaw_rate / 100.0, -1, 1),
            np.clip(ax / 32768.0, -1, 1),
            np.clip(ay / 32768.0, -1, 1),
            np.clip(az / 32768.0, -1, 1),
            np.clip(motor_L / 255.0, -1, 1),
            np.clip(motor_R / 255.0, -1, 1),
            np.clip(dir_y, -1, 1),
            np.clip(dir_x, -1, 1),
            np.clip(ramp_speed / 200.0, 0, 1),
        ], dtype=np.float32)
        
        # Inférence TorchScript
        with torch.no_grad():
            state_t = torch.from_numpy(state).unsqueeze(0)
            action = self.scripted_model(state_t).numpy()[0]
        
        # Conversion en valeurs physiques
        self.trim_L = action[0] * self.config.max_trim
        self.trim_R = action[1] * self.config.max_trim
        self.ramp_boost = action[2] * self.config.max_ramp_boost
        
        # Smoothing
        alpha = 0.2
        self.trim_L = (1-alpha) * self.trim_L + alpha * self.trim_L
        self.trim_R = (1-alpha) * self.trim_R + alpha * self.trim_R
        self.ramp_boost = (1-alpha) * self.ramp_boost + alpha * self.ramp_boost
        
        latency = (time.perf_counter() - start) * 1000
        log = f"Pi inference: {latency:.1f}ms | trims:({self.trim_L:+.1f},{self.trim_R:+.1f})"
        
        return float(self.trim_L), float(self.trim_R), float(self.ramp_boost), log
    
    def get_training_stats(self):
        """Retourne les statistiques (compatibilité PC/Pi)"""
        if self.is_raspberry_pi:
            return {
                'episodes': self.episode_count,
                'steps': self.step_count,
                'noise_scale': 0.0,
                'best_reward': 0.0,
                'memory_size': 0,
                'trim_L': self.trim_L,
                'trim_R': self.trim_R,
                'ramp_boost': self.ramp_boost,
                'mode': 'raspberry_pi_inference'
            }
        else:
            stats = self.agent.get_training_stats()
            stats['mode'] = 'pc_training'
            return stats
    
    def get_learned_values(self):
        """Retourne les valeurs apprises"""
        if self.is_raspberry_pi:
            return self.trim_L, self.trim_R, self.ramp_boost
        else:
            return self.agent.get_learned_values()
    
    def reset_episode(self):
        """Reset (no-op sur Pi)"""
        if not self.is_raspberry_pi:
            self.agent.reset_episode()
    
    def stop(self):
        """Arrêt propre"""
        if not self.is_raspberry_pi:
            self.agent.stop()
