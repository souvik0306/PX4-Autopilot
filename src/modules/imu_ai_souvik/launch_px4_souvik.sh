#!/bin/bash

SESSION="px4_sim"

tmux new-session -d -s $SESSION

# Split horizontally: left (pane 0) for QGC, right (pane 1) for PX4 SITL
tmux split-window -h -p 95 -t $SESSION    # Pane 0: 5% width (QGC), Pane 1: 95% width (PX4 SITL)

# Pane 0: QGroundControl (left, small)
tmux send-keys -t $SESSION:0.0 "cd ~/Downloads && ./QGroundControl.AppImage" C-m

# Pane 1: PX4 SITL with Gazebo (right, large)
tmux send-keys -t $SESSION:0.1 "cd ~/Downloads/gestelt_ws/PX4-Autopilot && make px4_sitl gazebo" C-m

# Host-side waits, then send commands into PX4 shell (pane 1)
sleep 8
tmux send-keys -t $SESSION:0.1 "imu_ai_bridge start -p 14561" C-m

# Optionally switch EKF2 to AI source after the stream should be alive.
# Assumes EKF2_IMU_SRC exists in your build. Comment these if you prefer manual switch.
# Note: Avoid auto-switching EKF2 to prevent poll timeouts while EKF2 is stopped.
# After confirming 'listener vehicle_imu_ai 5' shows updates, run manually in pxh>:
#   param set EKF2_IMU_SRC 1; ekf2 stop; ekf2 start; ekf2 status

# Pane 2: MAVROS launch (split below PX4 SITL)
tmux split-window -v -t $SESSION:0.1
tmux send-keys -t $SESSION:0.2 "source /opt/ros/noetic/setup.bash && roslaunch mavros px4.launch fcu_url:=udp://:14540@localhost:14580 fcu_protocol:=v2.0" C-m

# Pane 3: AI IMU Sender (split below MAVROS)
tmux split-window -v -t $SESSION:0.2
tmux send-keys -t $SESSION:0.3 "sleep 15; source /opt/ros/noetic/setup.bash && python3 ~/Downloads/gestelt_ws/PX4-Autopilot/fake_imu.py" C-m

# Attach to the session
tmux attach -t $SESSION

# To force kill all related processes, use:
# killall -9 QGroundControl.AppImage px4 roslaunch python3
