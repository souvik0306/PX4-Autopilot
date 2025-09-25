#!/bin/bash

echo "🔧 MAVROS Connection Troubleshooting Guide"
echo "========================================="

echo ""
echo "STEP 1: Check current tmux session"
if tmux has-session -t px4_sim 2>/dev/null; then
    echo "✅ tmux session 'px4_sim' is running"
    echo "Panes:"
    tmux list-panes -t px4_sim -F '  Pane #{pane_index}: #{pane_current_command}'
    echo ""
    echo "To connect: tmux attach -t px4_sim"
    echo "To switch panes: Ctrl+B then arrow keys"
else
    echo "❌ No tmux session running"
    echo "Start with: ./launch_px4_souvik.sh"
fi

echo ""
echo "STEP 2: Manual MAVROS test (if tmux is not working)"
echo "Run these commands in separate terminals:"
echo ""
echo "Terminal 1 - PX4 SITL:"
echo "  cd ~/Downloads/gestelt_ws/PX4-Autopilot"
echo "  make px4_sitl gazebo"
echo ""
echo "Terminal 2 - MAVROS (wait for PX4 to start):"
echo "  source /opt/ros/noetic/setup.bash"
echo "  roslaunch mavros px4.launch fcu_url:=udp://:14540@localhost:14580"
echo ""
echo "Terminal 3 - Test IMU data:"
echo "  source /opt/ros/noetic/setup.bash"
echo "  rostopic echo /mavros/imu/data -n 1"

echo ""
echo "STEP 3: Quick poll timeout fix"
echo "If you see poll timeouts in PX4 console, run:"
echo "  param set EKF2_IMU_SRC 0"
echo "  (This switches back to raw IMU data)"

echo ""
echo "STEP 4: Check processes"
ps aux | grep -E "(px4|mavros|roslaunch)" | grep -v grep | while read line; do
    echo "Running: $line"
done

echo ""
echo "🎯 Goal: Get /mavros/imu/data publishing before testing AI system"
