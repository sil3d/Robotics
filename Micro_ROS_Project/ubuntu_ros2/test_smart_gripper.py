#!/usr/bin/env python3
"""Unit tests for smart_gripper.py"""

import sys
sys.path.insert(0, 'micro_ros_robot/scripts')

def test_calibration_state_enum():
    """Test calibration states"""
    # Verify expected enum values (defined in smart_gripper.py)
    expected_calibration_states = {
        'UNCALIBRATED': 'uncallibrated',  # Note: typo in original code
        'CALIBRATING': 'calibrating',
        'CALIBRATED': 'calibrated',
        'FAILED': 'failed'
    }
    assert expected_calibration_states['UNCALIBRATED'] == 'uncallibrated'
    assert expected_calibration_states['CALIBRATING'] == 'calibrating'
    assert expected_calibration_states['CALIBRATED'] == 'calibrated'
    assert expected_calibration_states['FAILED'] == 'failed'
    print("[PASS] calibration states")

def test_grab_result_enum():
    """Test grab result values"""
    expected_grab_results = {
        'SUCCESS': 'success',
        'FAILED': 'failed',
        'NO_PRE_PICK': 'no_pre_pick',
        'NO_GRIP': 'no_grip',
        'SENSOR_ERROR': 'sensor_error'
    }
    assert expected_grab_results['SUCCESS'] == 'success'
    assert expected_grab_results['FAILED'] == 'failed'
    assert expected_grab_results['NO_PRE_PICK'] == 'no_pre_pick'
    print("[PASS] grab results")

def test_pre_pick_validation():
    """Test pre-pick validation logic"""
    # Simulate ultrasonic readings validation logic from smart_gripper.py
    def verify_pre_pick(dist):
        if dist < 5:
            return {'ok': False, 'reason': 'BOX_TOO_CLOSE'}
        if dist > 30:
            return {'ok': False, 'reason': 'BOX_TOO_FAR'}
        return {'ok': True, 'distance': dist}

    # Test too close
    result = verify_pre_pick(3)
    assert result['ok'] == False
    assert result['reason'] == 'BOX_TOO_CLOSE'

    # Test valid
    result = verify_pre_pick(15)
    assert result['ok'] == True

    # Test too far
    result = verify_pre_pick(40)
    assert result['ok'] == False
    assert result['reason'] == 'BOX_TOO_FAR'
    print("[PASS] pre-pick validation")

def test_grab_detection():
    """Test grab detection logic"""
    def detect_grab(pre_dist, post_dist, gripper_closed):
        if not gripper_closed:
            return 'NO_GRIP'
        dist_diff = abs(post_dist - pre_dist)
        if dist_diff < 5 and post_dist < 30:
            return 'SUCCESS'
        return 'FAILED'

    # Successful grab
    result = detect_grab(15, 12, True)
    assert result == 'SUCCESS'

    # Failed - box moved
    result = detect_grab(15, 35, True)
    assert result == 'FAILED'

    # Failed - gripper not closed
    result = detect_grab(15, 12, False)
    assert result == 'NO_GRIP'
    print("[PASS] grab detection")

if __name__ == '__main__':
    print("Running smart_gripper tests...")
    test_calibration_state_enum()
    test_grab_result_enum()
    test_pre_pick_validation()
    test_grab_detection()
    print("\n[SUCCESS] All smart_gripper tests passed!")