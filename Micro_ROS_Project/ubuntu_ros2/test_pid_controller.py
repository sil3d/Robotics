#!/usr/bin/env python3
"""Unit tests for PIDController in navigation_node.py"""

import math
import sys
import time

sys.path.insert(0, '.')

# Import standalone — pas besoin de rclpy pour PIDController
from navigation_node import PIDController


def test_proportional_only():
    """Kp seul : output = kp * error"""
    pid = PIDController(kp=2.0, ki=0.0, kd=0.0, output_limit=100.0)
    # Premier compute : dt sera ~0 → on injecte manuellement un ts passé
    pid._last_ts -= 0.05
    out = pid.compute(current=0.0, target=1.0)
    assert abs(out - 2.0) < 0.01, f"Expected 2.0, got {out}"
    print("[PASS] proportional only")


def test_output_clamping():
    """Output ne dépasse pas ±output_limit"""
    pid = PIDController(kp=100.0, ki=0.0, kd=0.0, output_limit=1.0)
    pid._last_ts -= 0.05
    out = pid.compute(current=0.0, target=10.0)
    assert out == 1.0, f"Expected 1.0 (clamped), got {out}"
    pid2 = PIDController(kp=100.0, ki=0.0, kd=0.0, output_limit=1.0)
    pid2._last_ts -= 0.05
    out2 = pid2.compute(current=10.0, target=0.0)
    assert out2 == -1.0, f"Expected -1.0 (clamped), got {out2}"
    print("[PASS] output clamping")


def test_reset_clears_state():
    """reset() remet integral et prev_error à zéro"""
    pid = PIDController(kp=1.0, ki=1.0, kd=0.0, output_limit=100.0)
    pid._last_ts -= 0.05
    pid.compute(current=0.0, target=5.0)
    assert pid.integral != 0.0
    pid.reset()
    assert pid.integral == 0.0
    assert pid.prev_error == 0.0
    print("[PASS] reset clears state")


def test_anti_windup_bounded():
    """L'intégrale ne dépasse pas windup_limit = min(5, output_limit/ki)"""
    pid = PIDController(kp=0.0, ki=0.1, kd=0.0, output_limit=0.2)
    # windup_limit = min(5.0, 0.2/0.1) = min(5.0, 2.0) = 2.0
    for _ in range(200):
        pid._last_ts -= 0.05
        pid.compute(current=0.0, target=100.0)
    assert abs(pid.integral) <= 2.01, f"Integral exceeded windup limit: {pid.integral}"
    print("[PASS] anti-windup bounded")


def test_dt_fallback_on_large_gap():
    """Si dt > 0.5s (pause), on utilise dt=0.05 fallback pour éviter pic dérivée"""
    pid = PIDController(kp=0.0, ki=0.0, kd=1.0, output_limit=100.0)
    pid.prev_error = 1.0
    # Simuler un gap de 2s
    pid._last_ts -= 2.0
    out = pid.compute(current=0.0, target=2.0)
    # Sans fallback : derivative = (1.0) / 2.0 = 0.5 → out = 0.5
    # Avec fallback dt=0.05 : derivative = (1.0) / 0.05 = 20 → out clamped
    # On vérifie que le fallback est bien appliqué (dt>0.5 → dt=0.05)
    assert abs(out) == 100.0, f"Expected clamped output with dt fallback, got {out}"
    print("[PASS] dt fallback on large gap")


def test_real_dt_used():
    """Le dt réel (via perf_counter) est utilisé entre deux appels rapprochés"""
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, output_limit=1000.0)
    t0 = time.perf_counter()
    time.sleep(0.1)
    pid.compute(current=0.0, target=1.0)
    dt_approx = time.perf_counter() - t0
    # integral ≈ 1.0 * dt_approx (autour de 0.1s)
    assert 0.08 < pid.integral < 0.15, f"Integral {pid.integral} doesn't match ~0.1s dt"
    print("[PASS] real dt used")


if __name__ == '__main__':
    print("Running PIDController tests...")
    test_proportional_only()
    test_output_clamping()
    test_reset_clears_state()
    test_anti_windup_bounded()
    test_dt_fallback_on_large_gap()
    test_real_dt_used()
    print("\n[SUCCESS] All PIDController tests passed!")
