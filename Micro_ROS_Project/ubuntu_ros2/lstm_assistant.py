#!/usr/bin/env python3
"""
Advisory LSTM assistant for mission planning.

Goals:
- Run on Raspberry Pi in real time.
- Predict next action + next target.
- Keep classical MissionEngine as the authoritative fallback.
- Record trajectories even when LSTM guidance is disabled.

The module is intentionally safe to import even if PyTorch is missing:
- recording still works
- inference is simply disabled until a model is available
"""

from __future__ import annotations

import json
import os
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Deque, Dict, Iterable, List, Optional, Tuple

import numpy as np

try:  # PyTorch is optional at import time.
    import torch
    import torch.nn as nn

    TORCH_AVAILABLE = True
except Exception:  # pragma: no cover - import guard
    torch = None
    nn = None
    TORCH_AVAILABLE = False


ACTION_VOCAB = [
    "IDLE",
    "SCAN_360",
    "NAVIGATE_TAG",
    "DETECT_CUBE",
    "NAVIGATE_CUBE",
    "OPEN_GRIPPER",
    "APPROACH_CUBE",
    "CLOSE_GRIPPER",
    "NAVIGATE_DROP",
    "RELEASE",
    "BACK_HOME",
    "RECORD",
    "AVOID",
    "STUCK",
    "ERROR",
]

TARGET_VOCAB = [
    "NONE",
    "HOME",
    "PICKUP_BLUE",
    "PICKUP_CYAN",
    "DROP_BLUE",
    "DROP_CYAN",
]

STATE_TO_INDEX = {name: idx for idx, name in enumerate(ACTION_VOCAB)}
TARGET_TO_INDEX = {name: idx for idx, name in enumerate(TARGET_VOCAB)}
INDEX_TO_ACTION = {idx: name for name, idx in STATE_TO_INDEX.items()}
INDEX_TO_TARGET = {idx: name for name, idx in TARGET_TO_INDEX.items()}


@dataclass(frozen=True)
class MissionHint:
    action: str
    target: str
    confidence: float
    source: str
    reason: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "action": self.action,
            "target": self.target,
            "confidence": float(self.confidence),
            "source": self.source,
            "reason": self.reason,
        }


class MissionSequenceModel(nn.Module):
    """Small two-head LSTM: next action + next target."""

    def __init__(self, input_size: int, hidden_size: int = 48, num_layers: int = 1):
        super().__init__()
        self.lstm = nn.LSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
        )
        self.action_head = nn.Linear(hidden_size, len(ACTION_VOCAB))
        self.target_head = nn.Linear(hidden_size, len(TARGET_VOCAB))

    def forward(self, x):
        out, _ = self.lstm(x)
        last = out[:, -1, :]
        return self.action_head(last), self.target_head(last)


class LSTMAssistant:
    """Real-time advisory layer with JSONL logging and optional PyTorch inference."""

    def __init__(
        self,
        *,
        model_path: Optional[str] = None,
        record_path: Optional[str] = None,
        enabled: bool = True,
        recording_enabled: bool = True,
        confidence_threshold: float = 0.75,
        window_size: int = 20,
        infer_period_s: float = 0.12,
    ):
        self.enabled = bool(enabled)
        self.recording_enabled = bool(recording_enabled)
        self.confidence_threshold = float(confidence_threshold)
        self.window_size = max(5, int(window_size))
        self.infer_period_s = max(0.05, float(infer_period_s))

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._new_sample_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._history: Deque[Dict[str, Any]] = deque(maxlen=max(self.window_size * 6, 120))
        self._latest_hint: Optional[MissionHint] = None
        self._last_fallback_reason: str = "model_not_loaded"
        self._last_prediction_time: float = 0.0

        base_dir = Path(__file__).resolve().parent
        self.record_path = Path(record_path) if record_path else base_dir / "mission_traces" / "mission_traces.jsonl"
        self.model_path = Path(model_path) if model_path else base_dir / "mission_traces" / "mission_lstm.pt"
        self.record_path.parent.mkdir(parents=True, exist_ok=True)
        self.model_path.parent.mkdir(parents=True, exist_ok=True)

        self.model = None
        self.model_ready = False
        self.device = None

        self._feature_size = len(self._feature_names())

        if TORCH_AVAILABLE:
            self.device = torch.device("cpu")
            self.model = MissionSequenceModel(self._feature_size)
            self.model.to(self.device)
            self._try_load_model(self.model_path)

        self.start()

    # ------------------------------------------------------------------
    # Public control
    # ------------------------------------------------------------------

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self.enabled = bool(enabled)
            if not self.enabled:
                self._latest_hint = None
                self._last_fallback_reason = "disabled_by_user"

    def set_recording(self, recording_enabled: bool) -> None:
        with self._lock:
            self.recording_enabled = bool(recording_enabled)

    def set_confidence_threshold(self, threshold: float) -> None:
        with self._lock:
            self.confidence_threshold = max(0.0, min(1.0, float(threshold)))

    def set_model_path(self, model_path: str) -> None:
        with self._lock:
            self.model_path = Path(model_path)
            self._try_load_model(self.model_path)

    # ------------------------------------------------------------------
    # Recording / ingestion
    # ------------------------------------------------------------------

    def observe(self, sample: Dict[str, Any]) -> None:
        """Append one mission tick. Recording continues even when disabled."""
        safe_sample = self._sanitize_sample(sample)
        with self._lock:
            self._history.append(safe_sample)
            if self.recording_enabled:
                self._append_jsonl(safe_sample)
        self._new_sample_event.set()

    def get_window(self) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self._history)[-self.window_size :]

    # ------------------------------------------------------------------
    # Prediction / status
    # ------------------------------------------------------------------

    def get_latest_hint(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            return None if self._latest_hint is None else self._latest_hint.to_dict()

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "enabled": self.enabled,
                "recording_enabled": self.recording_enabled,
                "model_ready": self.model_ready,
                "confidence_threshold": self.confidence_threshold,
                "window_size": self.window_size,
                "history_size": len(self._history),
                "last_prediction": None if self._latest_hint is None else self._latest_hint.to_dict(),
                "last_fallback_reason": self._last_fallback_reason,
                "last_prediction_time": self._last_prediction_time,
                "model_path": str(self.model_path),
                "record_path": str(self.record_path),
            }

    def predict_now(self) -> Optional[MissionHint]:
        window = self.get_window()
        return self._predict_from_window(window)

    # ------------------------------------------------------------------
    # Internal worker
    # ------------------------------------------------------------------

    def _worker(self) -> None:
        while not self._stop_event.is_set():
            self._new_sample_event.wait(timeout=self.infer_period_s)
            self._new_sample_event.clear()

            try:
                hint = self.predict_now()
                with self._lock:
                    if hint is not None:
                        self._latest_hint = hint
                        self._last_fallback_reason = ""
                        self._last_prediction_time = time.time()
                    else:
                        self._latest_hint = None
                        if not self.enabled:
                            self._last_fallback_reason = "disabled_by_user"
                        elif not self.model_ready:
                            self._last_fallback_reason = "model_not_loaded"
                        elif len(self._history) < self.window_size:
                            self._last_fallback_reason = "window_incomplete"
                        else:
                            self._last_fallback_reason = "low_confidence"
            except Exception as exc:  # pragma: no cover - safety net
                with self._lock:
                    self._latest_hint = None
                    self._last_fallback_reason = f"predict_error:{type(exc).__name__}"

    # ------------------------------------------------------------------
    # Prediction helpers
    # ------------------------------------------------------------------

    def _predict_from_window(self, window: List[Dict[str, Any]]) -> Optional[MissionHint]:
        if not self.enabled:
            return None
        if not self.model_ready or not TORCH_AVAILABLE or self.model is None:
            return None
        if len(window) < self.window_size:
            return None

        features = np.stack([self._feature_vector(step) for step in window], axis=0).astype(np.float32)
        tensor = torch.from_numpy(features).unsqueeze(0).to(self.device)

        self.model.eval()
        with torch.no_grad():
            action_logits, target_logits = self.model(tensor)
            action_probs = torch.softmax(action_logits, dim=-1)[0]
            target_probs = torch.softmax(target_logits, dim=-1)[0]

            action_idx = int(torch.argmax(action_probs).item())
            target_idx = int(torch.argmax(target_probs).item())
            action_conf = float(action_probs[action_idx].item())
            target_conf = float(target_probs[target_idx].item())
            confidence = min(action_conf, target_conf)

        if confidence < self.confidence_threshold:
            return None

        return MissionHint(
            action=INDEX_TO_ACTION.get(action_idx, "IDLE"),
            target=INDEX_TO_TARGET.get(target_idx, "NONE"),
            confidence=confidence,
            source="model",
            reason="ok",
        )

    def _try_load_model(self, model_path: Path) -> None:
        if not TORCH_AVAILABLE or self.model is None:
            self.model_ready = False
            return
        if not model_path.exists():
            self.model_ready = False
            return

        try:
            payload = torch.load(str(model_path), map_location=self.device)
            if isinstance(payload, dict) and "state_dict" in payload:
                self.model.load_state_dict(payload["state_dict"])
            elif isinstance(payload, dict):
                self.model.load_state_dict(payload)
            else:
                self.model.load_state_dict(payload)
            self.model_ready = True
        except Exception:
            self.model_ready = False

    # ------------------------------------------------------------------
    # Feature engineering
    # ------------------------------------------------------------------

    def _feature_names(self) -> List[str]:
        return [
            "state_idx",
            "running",
            "enabled",
            "recording_enabled",
            "target_color_idx",
            "target_tag_idx",
            "pose_x",
            "pose_y",
            "pose_yaw",
            "yaw_deg",
            "omega_z",
            "us_front",
            "us_back",
            "us_left",
            "us_right",
            "linear_cmd",
            "angular_cmd",
            "cube_dist_cm",
            "cube_pixel_x",
            "color_confidence",
            "tag_confidence",
            "gripper_state",
            "mission_progress",
        ]

    def _sanitize_sample(self, sample: Dict[str, Any]) -> Dict[str, Any]:
        s = dict(sample)
        s.setdefault("timestamp", time.time())
        s.setdefault("state", "IDLE")
        s.setdefault("action", s.get("state", "IDLE"))
        s.setdefault("target", "NONE")
        s.setdefault("target_color", "NONE")
        s.setdefault("target_tag_id", None)
        s.setdefault("running", False)
        s.setdefault("odom", {"x": 0.0, "y": 0.0, "yaw": 0.0})
        s.setdefault("us", [-1.0, -1.0, -1.0, -1.0])
        s.setdefault("yaw_deg", 0.0)
        s.setdefault("omega_z", 0.0)
        s.setdefault("linear_cmd", 0.0)
        s.setdefault("angular_cmd", 0.0)
        s.setdefault("cube_dist_cm", -1.0)
        s.setdefault("cube_pixel_x", -1.0)
        s.setdefault("color_confidence", 0.0)
        s.setdefault("tag_confidence", 0.0)
        s.setdefault("gripper_state", 0.0)
        s.setdefault("mission_progress", 0.0)
        return s

    def _feature_vector(self, sample: Dict[str, Any]) -> np.ndarray:
        s = self._sanitize_sample(sample)

        state_idx = STATE_TO_INDEX.get(str(s.get("state", "IDLE")), 0) / max(1, len(ACTION_VOCAB) - 1)
        running = 1.0 if s.get("running") else 0.0
        enabled = 1.0 if self.enabled else 0.0
        recording = 1.0 if self.recording_enabled else 0.0

        color_name = str(s.get("target_color", "NONE")).upper()
        target_color_idx = TARGET_TO_INDEX.get(f"PICKUP_{color_name}", 0) / max(1, len(TARGET_VOCAB) - 1)
        target_tag_idx = self._normalize_tag_id(s.get("target_tag_id"))

        odom = s.get("odom", {}) or {}
        pose_x = self._norm_float(odom.get("x", 0.0), scale=5.0)
        pose_y = self._norm_float(odom.get("y", 0.0), scale=5.0)
        pose_yaw = self._norm_float(odom.get("yaw", 0.0), scale=180.0)

        yaw_deg = self._norm_float(s.get("yaw_deg", 0.0), scale=180.0)
        omega_z = self._norm_float(s.get("omega_z", 0.0), scale=180.0)

        us = list(s.get("us", [-1.0, -1.0, -1.0, -1.0]))[:4]
        while len(us) < 4:
            us.append(-1.0)
        us_front, us_back, us_left, us_right = [self._norm_distance(v) for v in us]

        linear_cmd = self._norm_float(s.get("linear_cmd", 0.0), scale=0.5)
        angular_cmd = self._norm_float(s.get("angular_cmd", 0.0), scale=1.5)

        cube_dist_cm = self._norm_distance_cm(s.get("cube_dist_cm", -1.0))
        cube_pixel_x = self._norm_pixel_x(s.get("cube_pixel_x", -1.0))

        color_conf = float(np.clip(float(s.get("color_confidence", 0.0)), 0.0, 1.0))
        tag_conf = float(np.clip(float(s.get("tag_confidence", 0.0)), 0.0, 1.0))
        gripper_state = 1.0 if float(s.get("gripper_state", 0.0)) > 0.5 else 0.0
        mission_progress = float(np.clip(float(s.get("mission_progress", 0.0)), 0.0, 100.0)) / 100.0

        vector = np.array(
            [
                state_idx,
                running,
                enabled,
                recording,
                target_color_idx,
                target_tag_idx,
                pose_x,
                pose_y,
                pose_yaw,
                yaw_deg,
                omega_z,
                us_front,
                us_back,
                us_left,
                us_right,
                linear_cmd,
                angular_cmd,
                cube_dist_cm,
                cube_pixel_x,
                color_conf,
                tag_conf,
                gripper_state,
                mission_progress,
            ],
            dtype=np.float32,
        )
        return vector

    @staticmethod
    def _norm_float(value: Any, *, scale: float) -> float:
        try:
            return float(np.clip(float(value) / max(scale, 1e-6), -1.0, 1.0))
        except Exception:
            return 0.0

    @staticmethod
    def _norm_distance(value: Any) -> float:
        try:
            v = float(value)
            if v <= 0.0:
                return 0.0
            return float(np.clip(v / 200.0, 0.0, 1.0))
        except Exception:
            return 0.0

    @staticmethod
    def _norm_distance_cm(value: Any) -> float:
        try:
            v = float(value)
            if v <= 0.0:
                return 0.0
            return float(np.clip(v / 200.0, 0.0, 1.0))
        except Exception:
            return 0.0

    @staticmethod
    def _norm_pixel_x(value: Any) -> float:
        try:
            v = float(value)
            if v < 0.0:
                return 0.0
            return float(np.clip(v / 640.0, 0.0, 1.0))
        except Exception:
            return 0.0

    @staticmethod
    def _normalize_tag_id(value: Any) -> float:
        try:
            if value is None:
                return 0.0
            return float(np.clip(float(value) / 10.0, 0.0, 1.0))
        except Exception:
            return 0.0

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def _append_jsonl(self, sample: Dict[str, Any]) -> None:
        try:
            with self.record_path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(sample, ensure_ascii=False) + "\n")
        except Exception:
            # Recording must never crash the robot loop.
            pass


__all__ = [
    "ACTION_VOCAB",
    "TARGET_VOCAB",
    "MissionHint",
    "LSTMAssistant",
    "MissionSequenceModel",
    "TORCH_AVAILABLE",
]
