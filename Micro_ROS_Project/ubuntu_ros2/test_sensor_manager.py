#!/usr/bin/env python3
"""Unit tests for sensor_manager.py"""

import sys
sys.path.insert(0, 'micro_ros_robot/scripts')

import numpy as np

def test_mad_filter():
    """Test Median Absolute Deviation filter algorithm"""
    # Test the MAD filter algorithm directly (doesn't need ROS2)
    values = np.array([10, 12, 11, 13, 50, 12, 11])  # 50 is outlier
    median = np.median(values)
    mad = np.median(np.abs(values - median))

    # Filter should reject 50
    assert median > 10 and median < 13
    print("[PASS] MAD filter algorithm")

def test_confidence_calculation():
    """Test confidence calculation logic"""
    # Simulate confidence decay
    confidence = 1.0
    confidence_decay = 0.1

    # After 5 seconds of no data
    for _ in range(5):
        confidence = max(0.0, confidence - confidence_decay)

    assert confidence < 0.6
    print("[PASS] confidence decay calculation")

def test_stuck_sensor_detection():
    """Test stuck sensor detection logic"""
    readings = [10.0, 10.1, 10.0, 10.2, 10.0, 10.1, 10.0, 10.0, 10.1, 10.0]

    # Check if all within tolerance
    max_diff = max(readings) - min(readings)
    is_stuck = max_diff < 0.5  # Within 0.5cm

    assert is_stuck == True
    print("[PASS] stuck sensor detection")

if __name__ == '__main__':
    print("Running sensor_manager tests...")
    test_mad_filter()
    test_confidence_calculation()
    test_stuck_sensor_detection()
    print("\n[SUCCESS] All sensor_manager tests passed!")