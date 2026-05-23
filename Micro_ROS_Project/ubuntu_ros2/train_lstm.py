#!/usr/bin/env python3
"""
Offline trainer for the mission LSTM assistant.

Input:
- JSONL trajectories recorded by MissionEngine / LSTMAssistant

Output:
- PyTorch checkpoint compatible with lstm_assistant.MissionSequenceModel

This script is intentionally simple so it can be run on the PC side and later
exported back to the Raspberry Pi.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import deque
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

import numpy as np

try:
    import torch
    import torch.nn as nn
    from torch.utils.data import DataLoader, Dataset
except Exception as exc:  # pragma: no cover
    raise SystemExit(
        "PyTorch is required for training. Install torch before running train_lstm.py."
    ) from exc

from lstm_assistant import ACTION_VOCAB, TARGET_VOCAB, MissionSequenceModel


STATE_TO_INDEX = {name: idx for idx, name in enumerate(ACTION_VOCAB)}
TARGET_TO_INDEX = {name: idx for idx, name in enumerate(TARGET_VOCAB)}


def _norm_float(value: Any, *, scale: float) -> float:
    try:
        return float(np.clip(float(value) / max(scale, 1e-6), -1.0, 1.0))
    except Exception:
        return 0.0


def _norm_distance(value: Any) -> float:
    try:
        v = float(value)
        if v <= 0.0:
            return 0.0
        return float(np.clip(v / 200.0, 0.0, 1.0))
    except Exception:
        return 0.0


def _norm_pixel_x(value: Any) -> float:
    try:
        v = float(value)
        if v < 0.0:
            return 0.0
        return float(np.clip(v / 640.0, 0.0, 1.0))
    except Exception:
        return 0.0


def _normalize_tag_id(value: Any) -> float:
    try:
        if value is None:
            return 0.0
        return float(np.clip(float(value) / 10.0, 0.0, 1.0))
    except Exception:
        return 0.0


def sample_to_features(sample: Dict[str, Any]) -> np.ndarray:
    state_idx = STATE_TO_INDEX.get(str(sample.get("state", "IDLE")), 0) / max(1, len(ACTION_VOCAB) - 1)
    running = 1.0 if sample.get("running") else 0.0
    enabled = 1.0 if sample.get("lstm_enabled", True) else 0.0
    recording = 1.0 if sample.get("recording_enabled", True) else 0.0

    color_name = str(sample.get("target_color", "NONE")).upper()
    target_color_idx = TARGET_TO_INDEX.get(f"PICKUP_{color_name}", 0) / max(1, len(TARGET_VOCAB) - 1)
    target_tag_idx = _normalize_tag_id(sample.get("target_tag_id"))

    odom = sample.get("odom", {}) or {}
    pose_x = _norm_float(odom.get("x", 0.0), scale=5.0)
    pose_y = _norm_float(odom.get("y", 0.0), scale=5.0)
    pose_yaw = _norm_float(odom.get("yaw", 0.0), scale=180.0)

    yaw_deg = _norm_float(sample.get("yaw", 0.0), scale=180.0)
    omega_z = _norm_float(sample.get("omega_z", 0.0), scale=180.0)

    us = list(sample.get("us", [-1.0, -1.0, -1.0, -1.0]))[:4]
    while len(us) < 4:
        us.append(-1.0)
    us_front, us_back, us_left, us_right = [_norm_distance(v) for v in us]

    linear_cmd = _norm_float(sample.get("linear_cmd", 0.0), scale=0.5)
    angular_cmd = _norm_float(sample.get("angular_cmd", 0.0), scale=1.5)
    cube_dist_cm = _norm_distance(sample.get("cube_dist_cm", -1.0))
    cube_pixel_x = _norm_pixel_x(sample.get("cube_pixel_x", -1.0))
    color_conf = float(np.clip(float(sample.get("color_confidence", 0.0)), 0.0, 1.0))
    tag_conf = float(np.clip(float(sample.get("tag_confidence", 0.0)), 0.0, 1.0))
    gripper_state = 1.0 if float(sample.get("gripper_state", 0.0)) > 0.5 else 0.0
    mission_progress = float(np.clip(float(sample.get("mission_progress", 0.0)), 0.0, 100.0)) / 100.0

    return np.array(
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


def target_from_sample(sample: Dict[str, Any]) -> Tuple[int, int]:
    action = str(sample.get("action", sample.get("state", "IDLE")))
    target = str(sample.get("target", "NONE"))
    return STATE_TO_INDEX.get(action, 0), TARGET_TO_INDEX.get(target, 0)


class MissionSequenceDataset(Dataset):
    def __init__(self, windows: List[np.ndarray], targets: List[Tuple[int, int]]):
        self.windows = windows
        self.targets = targets

    def __len__(self) -> int:
        return len(self.windows)

    def __getitem__(self, idx: int):
        x = torch.from_numpy(self.windows[idx])
        y_action, y_target = self.targets[idx]
        return x, torch.tensor(y_action, dtype=torch.long), torch.tensor(y_target, dtype=torch.long)


def load_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def build_sequences(rows: List[Dict[str, Any]], window_size: int) -> Tuple[List[np.ndarray], List[Tuple[int, int]]]:
    windows: List[np.ndarray] = []
    targets: List[Tuple[int, int]] = []
    if len(rows) <= window_size:
        return windows, targets

    for idx in range(window_size - 1, len(rows) - 1):
        window_rows = rows[idx - window_size + 1 : idx + 1]
        next_row = rows[idx + 1]
        x = np.stack([sample_to_features(row) for row in window_rows], axis=0).astype(np.float32)
        windows.append(x)
        targets.append(target_from_sample(next_row))

    return windows, targets


def train(args: argparse.Namespace) -> None:
    rows = load_jsonl(Path(args.data))
    if len(rows) < args.window_size + 2:
        raise SystemExit(f"Not enough samples in {args.data}. Need at least {args.window_size + 2} rows.")

    windows, targets = build_sequences(rows, args.window_size)
    if not windows:
        raise SystemExit("No training windows could be built from the dataset.")

    split = max(1, int(len(windows) * (1.0 - args.val_ratio)))
    train_windows, val_windows = windows[:split], windows[split:]
    train_targets, val_targets = targets[:split], targets[split:]

    train_ds = MissionSequenceDataset(train_windows, train_targets)
    val_ds = MissionSequenceDataset(val_windows, val_targets) if val_windows else None

    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True, drop_last=False)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False, drop_last=False) if val_ds else None

    model = MissionSequenceModel(input_size=train_windows[0].shape[-1], hidden_size=args.hidden_size, num_layers=args.num_layers)
    device = torch.device("cpu")
    model.to(device)

    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    action_loss_fn = nn.CrossEntropyLoss()
    target_loss_fn = nn.CrossEntropyLoss()

    best_val = math.inf
    for epoch in range(1, args.epochs + 1):
        model.train()
        total_loss = 0.0
        for x, y_action, y_target in train_loader:
            x = x.to(device)
            y_action = y_action.to(device)
            y_target = y_target.to(device)

            opt.zero_grad(set_to_none=True)
            action_logits, target_logits = model(x)
            loss = action_loss_fn(action_logits, y_action) + target_loss_fn(target_logits, y_target)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            opt.step()
            total_loss += float(loss.item()) * x.size(0)

        train_loss = total_loss / max(1, len(train_ds))
        val_loss = train_loss
        if val_loader is not None:
            model.eval()
            val_total = 0.0
            with torch.no_grad():
                for x, y_action, y_target in val_loader:
                    x = x.to(device)
                    y_action = y_action.to(device)
                    y_target = y_target.to(device)
                    action_logits, target_logits = model(x)
                    loss = action_loss_fn(action_logits, y_action) + target_loss_fn(target_logits, y_target)
                    val_total += float(loss.item()) * x.size(0)
            val_loss = val_total / max(1, len(val_ds))

        print(f"Epoch {epoch:03d} | train_loss={train_loss:.4f} | val_loss={val_loss:.4f}")
        if val_loss < best_val:
            best_val = val_loss
            checkpoint = {
                "state_dict": model.state_dict(),
                "hidden_size": args.hidden_size,
                "num_layers": args.num_layers,
                "window_size": args.window_size,
                "input_size": train_windows[0].shape[-1],
                "action_vocab": ACTION_VOCAB,
                "target_vocab": TARGET_VOCAB,
                "val_loss": val_loss,
            }
            torch.save(checkpoint, args.output)

    print(f"Saved best checkpoint to {args.output}")


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Train the mission LSTM assistant from JSONL traces")
    p.add_argument("--data", required=True, help="Path to mission_traces.jsonl")
    p.add_argument("--output", required=True, help="Path to save the checkpoint (.pt)")
    p.add_argument("--window-size", type=int, default=20)
    p.add_argument("--batch-size", type=int, default=64)
    p.add_argument("--epochs", type=int, default=20)
    p.add_argument("--hidden-size", type=int, default=48)
    p.add_argument("--num-layers", type=int, default=1)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--val-ratio", type=float, default=0.2)
    return p


if __name__ == "__main__":
    args = build_arg_parser().parse_args()
    train(args)
