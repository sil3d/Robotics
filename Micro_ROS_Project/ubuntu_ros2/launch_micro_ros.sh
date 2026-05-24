#!/bin/bash
# ============================================================================
#   MICRO-ROS LAUNCH SCRIPT - Ubuntu / ROS 2 Humble
# ============================================================================
# Launches: micro-ROS agent + bridge to Flask web interface
#
# USAGE:
#   chmod +x launch_micro_ros.sh
#   ./launch_micro_ros.sh
#
# REQUIREMENTS:
#   - ROS 2 Humble installed
#   - micro-ros-agent built
#   - Python 3 with flask, rclpy
# ============================================================================

echo "=========================================="
echo "  MICRO-ROS ROBOT LAUNCHER"
echo "=========================================="

# Source ROS 2
echo "[1/5] Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

# Create workspace if needed
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Check if workspace is sourced properly
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ros2 command not found. Is ROS 2 installed correctly?"
    exit 1
fi

echo "[2/5] Starting micro-ROS Agent on Serial (USB)..."
# Launch micro-ros agent in background (Serial mode)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 &
AGENT_PID=$!
echo "Agent PID: $AGENT_PID"
sleep 2

# Check if agent started
if ! kill -0 $AGENT_PID 2>/dev/null; then
    echo "ERROR: micro-ROS agent failed to start"
    exit 1
fi

echo "[3/5] Starting Flask-ROS Bridge..."
# Launch Flask-ROS bridge (UI interface)
cd ~/ros2_ws/src/micro_ros_robot/ubuntu_ros2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 flask_ros_bridge.py &
FLASK_PID=$!
echo "Flask PID: $FLASK_PID"
sleep 2

echo "[4/5] Testing ROS topics..."
ros2 topic list | grep -E "(imu|motor|ultrasonic|cmd_result)" || echo "  (no topics yet - ESP32 not connected)"

echo "[5/5] All services started!"
echo ""
echo "=========================================="
echo "  STATUS"
echo "=========================================="
echo "  micro-ROS Agent : PID $AGENT_PID (Serial /dev/ttyUSB0)"
echo "  Flask-ROS Bridge: PID $FLASK_PID (http://localhost:5000)"
echo ""
echo "  ROS Topics available:"
ros2 topic list 2>/dev/null || echo "  (waiting for ESP32)"
echo ""
echo "  To stop all: kill $AGENT_PID $FLASK_PID"
echo "=========================================="
echo ""
echo "Connect ESP32 via USB to /dev/ttyUSB0 for micro-ROS!"
echo "Open browser at http://localhost:5000 for UI control"

# Wait for ctrl-c
trap "echo 'Stopping...'; kill $AGENT_PID $FLASK_PID 2>/dev/null; exit" INT TERM
wait