#!/usr/bin/env python3
"""
 ===========================================================================
   ROBOT LAUNCH FILE - Starts all ROS2 nodes
 ===========================================================================

 Usage:
   ros2 launch robot_bringup.launch.py

 Nodes started:
   - camera_node      : AprilTag detection + color detection
   - localization_node: Robot pose estimation from AprilTag
   - navigation_node   : Waypoint following with PID
   - task_manager_node: Mission state machine

 Topics:
   /image_raw          : Camera frames (from camera_node)
   /aruco_detections   : Detected AprilTag poses
   /box_color          : Detected box color (red/green/none)
   /robot_pose         : Estimated robot position
   /cmd_vel            : Velocity commands (navigation -> ESP32)
   /waypoint           : Target waypoint (task_manager -> navigation)
   /gripper_cmd        : Gripper commands (task_manager -> ESP32)
   /imu_data           : IMU data (ESP32)
   /ultrasonic_data    : Ultrasonic distances (ESP32)

 ===========================================================================
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Camera Node - AprilTag detection + color detection
        Node(
            package='micro_ros_robot',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'camera_index': 0,
                'tag_size': 0.10,
            }]
        ),

        # Localization Node - Robot pose estimation
        Node(
            package='micro_ros_robot',
            executable='localization_node',
            name='localization_node',
            output='screen',
            parameters=[{
                'marker_map': {
                    '0': [0.0, 0.0, 0.0],
                    '1': [1.5, 0.0, 0.0],
                    '2': [0.0, 1.5, 0.0],
                    '3': [1.5, 1.5, 0.0],
                },
                'robot_frame': 'map',
            }]
        ),

        # Navigation Node - Waypoint following with PID
        Node(
            package='micro_ros_robot',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[{
                'max_linear_speed': 0.20,
                'max_angular_speed': 1.5,
                'pid_linear_kp': 2.0,
                'pid_linear_ki': 0.1,
                'pid_linear_kd': 0.5,
                'pid_angular_kp': 3.0,
                'pid_angular_ki': 0.2,
                'pid_angular_kd': 1.0,
                'waypoint_threshold': 0.15,
                'angle_threshold': 0.2,
            }]
        ),

        # Task Manager Node - Mission state machine
        Node(
            package='micro_ros_robot',
            executable='task_manager_node',
            name='task_manager_node',
            output='screen',
            parameters=[{
                'home_position': [0.0, 0.0, 0.0],
                'manufacturing_position': [1.5, 0.0, 0.0],
                'storage_a_position': [0.0, 1.5, 0.0],
                'storage_b_position': [1.5, 1.5, 0.0],
            }]
        ),
    ])