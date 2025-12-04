#!/bin/bash

SESSION="px4_sim"
PX4_ROOT="${PX4_ROOT:-$HOME/gestelt_ws/PX4-Autopilot}"
PX4_LAUNCH_CMD="${PX4_LAUNCH_CMD:-make px4_sitl gazebo}"
IMU_BRIDGE_PORT="${IMU_BRIDGE_PORT:-14565}"  # AI IMU bridge port (avoid conflict with 14560 MAVLink)

tmux new-session -d -s "$SESSION"

# Split horizontally: left (pane 0) for QGC, right (pane 1) for PX4 SITL
tmux split-window -h -p 95 -t "$SESSION"    # Pane 0: 5% width (QGC), Pane 1: 95% width (PX4 SITL)

# Pane 0: QGroundControl (left, small)
tmux send-keys -t "$SESSION":0.0 "cd ~/Downloads && ./QGroundControl.AppImage" C-m

# Pane 1: PX4 SITL with Gazebo (right, large)
tmux send-keys -t "$SESSION":0.1 "cd $PX4_ROOT && $PX4_LAUNCH_CMD" C-m

# Host-side waits, then nudge the pxh prompt before starting the bridge
sleep 8
tmux send-keys -t "$SESSION":0.1 C-m
tmux send-keys -t "$SESSION":0.1 "imu_ai_bridge start -p $IMU_BRIDGE_PORT" C-m

# Pane 2: MAVROS launch (split below PX4 SITL)
tmux split-window -v -t $SESSION:0.1
tmux send-keys -t $SESSION:0.2 "source /opt/ros/noetic/setup.bash && roslaunch mavros px4.launch fcu_url:=udp://:14540@localhost:14580 fcu_protocol:=v2.0" C-m

# Pane 3: EKF2 UDP Telemetry Listener (C++ or Python version, split below MAVROS)
# tmux split-window -v -t $SESSION:0.2
# Use C++ version:
# tmux send-keys -t $SESSION:0.3 "sleep 3; cd $PX4_ROOT/src/modules/imu_ai_bridge && ./listen_ekf2_udp" C-m
# Or use Python version instead:
# tmux send-keys -t $SESSION:0.3 "python3 $PX4_ROOT/src/modules/imu_ai_bridge/listen_ekf2_udp.py" C-m

# Pane 4: AI IMU Client (TCP pipeline)
tmux split-window -v -t $SESSION:0.2
tmux send-keys -t $SESSION:0.3 "python3 ~/gestelt_ws/PX4-Autopilot/src/modules/ekf2/ai_imu_client.py" C-m

# Attach to the session
tmux attach -t $SESSION

# To force kill all related processes, use:
# killall -9 QGroundControl.AppImage px4 roslaunch python3
