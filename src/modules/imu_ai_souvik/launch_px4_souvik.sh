#!/bin/bash

SESSION="px4_sim"

# Resolve the PX4 workspace location. Prefer an explicitly provided PX4_ROOT,
# otherwise probe a list of common checkouts so the script "just works" on
# both ~/gestelt_ws and ~/Downloads/gestelt_ws layouts.
if [ -z "${PX4_ROOT:-}" ]; then
    for candidate in "$HOME/gestelt_ws/PX4-Autopilot" \
                     "$HOME/Downloads/gestelt_ws/PX4-Autopilot"; do
        if [ -d "$candidate" ]; then
            PX4_ROOT="$candidate"
            break
        fi
    done
fi

if [ -z "${PX4_ROOT:-}" ]; then
    echo "[launch_px4_souvik] ERROR: Unable to locate PX4 checkout. Set PX4_ROOT before running." >&2
    exit 1
fi

PX4_LAUNCH_CMD="${PX4_LAUNCH_CMD:-make px4_sitl gazebo}"
IMU_BRIDGE_PORT="${IMU_BRIDGE_PORT:-14560}"

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

# EKF2 now switches to vehicle_imu_ai automatically once the bridge is publishing.
# Use the pxh console only if you need to force a specific source:
#   param set EKF2_IMU_SRC 1   # raw only
#   param set EKF2_IMU_SRC 2   # AI only
#   param set EKF2_IMU_SRC 0   # return to automatic
#   ekf2 stop; ekf2 start; ekf2 status

# Pane 2: MAVROS launch (split below PX4 SITL)
tmux split-window -v -t $SESSION:0.1
tmux send-keys -t $SESSION:0.2 "source /opt/ros/noetic/setup.bash && roslaunch mavros px4.launch fcu_url:=udp://:14540@localhost:14580 fcu_protocol:=v2.0" C-m

# Pane 3: AI IMU Sender (split below MAVROS)
tmux split-window -v -t $SESSION:0.2
tmux send-keys -t $SESSION:0.3 "sleep 5; source /opt/ros/noetic/setup.bash && python3 \"$PX4_ROOT/fake_imu.py\" _udp_port:=$IMU_BRIDGE_PORT" C-m

# Attach to the session
tmux attach -t $SESSION

# To force kill all related processes, use:
# killall -9 QGroundControl.AppImage px4 roslaunch python3
