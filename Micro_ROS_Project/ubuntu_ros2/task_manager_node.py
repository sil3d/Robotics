#!/usr/bin/env python3
"""
Mission manager ROS2 bridge.

Connects MissionEngine to the Micro-ROS firmware topics:
- /cmd_vel
- /gripper_cmd
- /robot_cfg
- /task_status
- /mission_state
- /mission_log

It receives:
- /imu_data
- /ultrasonic_data
- /odom_data
- /sensor_health
"""

import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Accel, Point, Twist
from std_msgs.msg import String
from std_srvs.srv import Empty

from mission_engine import MissionEngine


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager")

        self.engine = MissionEngine(ros_node=self)
        self.engine.set_velocity_callback(self._send_velocity)
        self.engine.set_gripper_callback(self._send_gripper)
        self.engine.set_config_callback(self._send_config)
        self.engine.set_log_callback(self._log)

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.gripper_pub = self.create_publisher(String, "/gripper_cmd", 10)
        self.cfg_pub = self.create_publisher(String, "/robot_cfg", 10)
        self.task_status_pub = self.create_publisher(String, "/task_status", 10)
        self.mission_state_pub = self.create_publisher(String, "/mission_state", 10)
        self.mission_log_pub = self.create_publisher(String, "/mission_log", 10)

        self.create_subscription(Accel, "/imu_data", self._imu_cb, 10)
        self.create_subscription(String, "/ultrasonic_data", self._us_cb, 10)
        self.create_subscription(Point, "/odom_data", self._odom_cb, 10)
        self.create_subscription(String, "/sensor_health", self._health_cb, 10)
        self.create_subscription(String, "/mission_ctrl", self._mission_ctrl_cb, 10)

        self.create_service(Empty, "/start_task", self._start_cb)
        self.create_service(Empty, "/cancel_task", self._cancel_cb)
        self.create_service(Empty, "/reset_odom", self._reset_odom_cb)

        self._odom = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._us_limit = 1.0

        # C3/M3: buffer IMU brut — màj à chaque callback, flush vers engine à 20 Hz
        self._imu_yaw_raw   = 0.0
        self._imu_omega_raw = 0.0
        self._last_imu_flush = time.perf_counter()
        self._IMU_FLUSH_INTERVAL = 0.05  # 20 Hz

        self.create_timer(0.1, self._publish_state)
        self.get_logger().info("TaskManagerNode ready")

    def _imu_cb(self, msg: Accel):
        # M3: stocker seulement, flush à 20 Hz dans _flush_imu_to_engine
        self._imu_yaw_raw   = float(msg.linear.x)
        self._imu_omega_raw = float(msg.linear.y)
        now = time.perf_counter()
        if now - self._last_imu_flush >= self._IMU_FLUSH_INTERVAL:
            self._last_imu_flush = now
            self._flush_imu_to_engine()

    def _flush_imu_to_engine(self):
        # C3: utiliser yaw SLAM si tracker initialisé, sinon fallback IMU
        if self.engine.tracker.initialized:
            yaw_source = self.engine.yaw_deg  # déjà mis à jour par _update_tracker
        else:
            yaw_source = self._imu_yaw_raw
        self.engine.update_sensors(
            yaw=yaw_source,
            omega_z=self._imu_omega_raw,
            odom=dict(self._odom),
            us=list(self.engine.us),
            us_limit=self._us_limit,
        )

    def _us_cb(self, msg: String):
        # Format JSON: {"us":[d0,d1,d2,d3]}
        # US1=avant droit, US2=avant gauche, US3=arrière gauche, US4=arrière droite
        try:
            data = json.loads(msg.data)
            self.engine.us = [float(v) for v in data["us"]]
        except Exception:
            pass

    def _odom_cb(self, msg: Point):
        self._odom = {"x": msg.x, "y": msg.y, "yaw": msg.z}
        self.engine.odom = dict(self._odom)

    def _health_cb(self, msg: String):
        try:
            health = json.loads(msg.data)
            self._us_limit = float(health.get("usl", 1.0))
        except Exception:
            pass

    def _mission_ctrl_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        if "enabled" in data:
            self.engine.set_lstm_enabled(bool(data["enabled"]))
        if "recording_enabled" in data:
            self.engine.set_lstm_recording(bool(data["recording_enabled"]))
        if "confidence_threshold" in data:
            try:
                self.engine.set_lstm_threshold(float(data["confidence_threshold"]))
            except Exception:
                pass

    def _start_cb(self, request, response):
        self.engine.start()
        return response

    def _cancel_cb(self, request, response):
        self.engine.stop()
        return response

    def _reset_odom_cb(self, request, response):
        self._send_config({"reset_odom": 1})
        return response

    def _send_velocity(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.vel_pub.publish(msg)

    def _send_gripper(self, value):
        msg = String()
        msg.data = str(value)
        self.gripper_pub.publish(msg)

    def _send_config(self, cfg: dict):
        msg = String()
        msg.data = json.dumps(cfg)
        self.cfg_pub.publish(msg)

    def _log(self, text: str):
        self.get_logger().info(f"[MISSION] {text}")
        msg = String()
        msg.data = text
        self.mission_log_pub.publish(msg)

    def _publish_state(self):
        status = self.engine.get_status()
        payload = json.dumps(status)

        msg = String()
        msg.data = payload
        self.task_status_pub.publish(msg)
        self.mission_state_pub.publish(msg)

    def destroy_node(self):
        self.engine.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

