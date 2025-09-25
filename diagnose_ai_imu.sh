#!/bin/bash

echo "=== AI IMU System Diagnostic ==="

# Check if processes are running
echo "1. Checking running processes..."
ps aux | grep -E "(px4|mavros|python3.*ai_imu)" | grep -v grep

echo ""
echo "2. Checking ROS topics..."
source /opt/ros/noetic/setup.bash
timeout 3 rostopic list | grep -E "(imu|mavros)" 2>/dev/null || echo "ROS master not accessible"

echo ""
echo "3. Checking PX4 parameter status..."
echo "   To check EKF2_IMU_SRC, run in PX4 console: param show EKF2_IMU_SRC"

echo ""
echo "4. Manual recovery options:"
echo "   A) Set back to raw IMU:  param set EKF2_IMU_SRC 0"
echo "   B) Set to AI IMU:       param set EKF2_IMU_SRC 1"
echo "   C) Restart EKF2:        ekf2 stop && ekf2 start"

echo ""
echo "5. Check tmux panes:"
echo "   tmux list-panes -t px4_sim"
