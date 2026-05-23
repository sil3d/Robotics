#!/usr/bin/env python3
"""Unit tests for robot_state.py"""

import sys
import time
sys.path.insert(0, 'micro_ros_robot/scripts')

from robot_state import RobotState, SensorReading

def test_robot_state_init():
    """Test RobotState initialization"""
    rs = RobotState()
    assert rs.pose_x == 0.0
    assert rs.pose_y == 0.0
    assert rs.pose_theta == 0.0
    assert rs.pose_confidence == 0.0
    print("[PASS] RobotState init")

def test_update_pose():
    """Test pose update with LKGS"""
    rs = RobotState()
    rs.update_pose(1.0, 2.0, 0.5, 0.9)
    assert rs.pose_x == 1.0
    assert rs.pose_y == 2.0
    assert rs.pose_theta == 0.5
    assert rs.pose_confidence == 0.9
    print("[PASS] update_pose")

def test_sensor_health_tracking():
    """Test sensor health updates"""
    rs = RobotState()
    rs.update_sensor_health('imu', True, 0.95)
    assert rs.sensors['imu']['current'].is_healthy == True
    assert rs.sensors['imu']['current'].confidence == 0.95
    print("[PASS] sensor health tracking")

def test_lkgs_handoff():
    """Test LKGS handoff when sensor fails"""
    rs = RobotState()
    rs.update_pose(1.0, 2.0, 0.5, 0.9)
    rs.update_sensor_health('imu', True, 0.9)

    # Now simulate sensor failure
    rs.update_sensor_health('imu', False, 0.1)

    # LKGS should still have good value
    lkgs = rs.get_last_known_good_pose()
    assert lkgs is not None
    print("[PASS] LKGS handoff on failure")

def test_gripper_state():
    """Test gripper state tracking"""
    rs = RobotState()
    rs.update_gripper_state(45.0, False)
    assert rs.gripper_state.position == 45.0
    assert rs.gripper_state.has_box == False
    print("[PASS] gripper state")

def test_fault_state():
    """Test fault entry/exit"""
    rs = RobotState()
    rs.enter_fault('IMU_FAILURE', 'IMU timeout detected')
    assert rs.fault_state.in_fault == True
    assert rs.fault_state.fault_type == 'IMU_FAILURE'
    rs.exit_fault()
    assert rs.fault_state.in_fault == False
    print("[PASS] fault state")

def test_overall_confidence():
    """Test confidence calculation"""
    rs = RobotState()
    rs.update_sensor_health('imu', True, 0.9)
    rs.update_sensor_health('camera_apriltag', True, 0.8)
    # Update timestamps to make sensors appear "fresh"
    rs._last_imu_time = time.time()
    rs._last_apriltag_time = time.time()
    # Add ultrasonic readings so they agree (low variance -> +0.3)
    rs.update_sensor_health('ultrasonic_front', True, 1.0, value=25.0)
    rs.update_sensor_health('ultrasonic_back', True, 1.0, value=25.0)
    rs.update_sensor_health('ultrasonic_left', True, 1.0, value=25.0)
    rs.update_sensor_health('ultrasonic_right', True, 1.0, value=25.0)
    conf = rs.get_overall_confidence()
    # AprilTag (0.4) + IMU (0.3) + Ultrasonics (0.3 if agreeing) should give ~1.0
    assert conf > 0.7
    print("[PASS] overall confidence")

if __name__ == '__main__':
    print("Running robot_state tests...")
    test_robot_state_init()
    test_update_pose()
    test_sensor_health_tracking()
    test_lkgs_handoff()
    test_gripper_state()
    test_fault_state()
    test_overall_confidence()
    print("\n[SUCCESS] All robot_state tests passed!")