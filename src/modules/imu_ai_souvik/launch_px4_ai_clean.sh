#!/bin/bash

# ============================================================================
# PX4 AI IMU Launch Script
# Repository: /home/souvik/Downloads/imu_project/PX4-Autopilot
# ============================================================================

SESSION="px4_ai_sim"
PX4_ROOT="/home/souvik/Downloads/imu_project/PX4-Autopilot"
PX4_LAUNCH_CMD="${PX4_LAUNCH_CMD:-make px4_sitl gazebo}"
IMU_BRIDGE_PORT="${IMU_BRIDGE_PORT:-14565}"  # AI IMU bridge port (avoid conflict with 14560 MAVLink)
QGC_PATH="$HOME/Downloads/QGroundControl.AppImage"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}PX4 AI IMU Simulation Launcher${NC}"
echo -e "${GREEN}========================================${NC}"

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo -e "${RED}Error: tmux is not installed. Install with: sudo apt install tmux${NC}"
    exit 1
fi

# Check if QGroundControl exists
if [ ! -f "$QGC_PATH" ]; then
    echo -e "${YELLOW}Warning: QGroundControl not found at $QGC_PATH${NC}"
    echo -e "${YELLOW}Will skip QGC launch. Download from: https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage${NC}"
    SKIP_QGC=true
else
    # Make sure QGC is executable
    chmod +x "$QGC_PATH"
    SKIP_QGC=false
fi

# Check if PX4 directory exists
if [ ! -d "$PX4_ROOT" ]; then
    echo -e "${RED}Error: PX4 directory not found at $PX4_ROOT${NC}"
    exit 1
fi

# Check if ai_imu_client.py exists
if [ ! -f "$PX4_ROOT/src/modules/ekf2/ai_imu_client.py" ]; then
    echo -e "${YELLOW}Warning: ai_imu_client.py not found at $PX4_ROOT/src/modules/ekf2/ai_imu_client.py${NC}"
fi

# Check if ROS is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Warning: ROS not sourced. Attempting to source /opt/ros/noetic/setup.bash${NC}"
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        source /opt/ros/noetic/setup.bash
    else
        echo -e "${RED}Error: ROS Noetic not found. MAVROS pane will fail.${NC}"
    fi
fi

# Kill existing session if it exists
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo -e "${YELLOW}Killing existing tmux session: $SESSION${NC}"
    tmux kill-session -t "$SESSION"
fi

echo -e "${GREEN}Creating new tmux session: $SESSION${NC}"
echo -e "${GREEN}PX4 Root: $PX4_ROOT${NC}"
echo -e "${GREEN}IMU Bridge Port: $IMU_BRIDGE_PORT${NC}"

# Create new tmux session
tmux new-session -d -s "$SESSION"

# ============================================================================
# PANE LAYOUT
# ============================================================================
# Pane 0: QGroundControl (left, 5% width)
# Pane 1: PX4 SITL + Gazebo (right top, 95% width)
# Pane 2: MAVROS (right middle)
# Pane 3: AI IMU Client (right bottom)
# ============================================================================

# Split horizontally: left (pane 0) for QGC, right (pane 1) for PX4 SITL
if [ "$SKIP_QGC" = false ]; then
    tmux split-window -h -p 95 -t "$SESSION"    # Pane 0: 5% width (QGC), Pane 1: 95% width (PX4 SITL)

    # Pane 0: QGroundControl (left, small)
    echo -e "${GREEN}[Pane 0] Launching QGroundControl...${NC}"
    tmux send-keys -t "$SESSION":0.0 "echo -e '${GREEN}Starting QGroundControl...${NC}'" C-m
    tmux send-keys -t "$SESSION":0.0 "cd ~/Downloads && $QGC_PATH" C-m
else
    # If no QGC, use full width for PX4
    echo -e "${YELLOW}[Pane 0] Skipping QGroundControl${NC}"
fi

# Pane 1: PX4 SITL with Gazebo (right, large)
PANE_PX4=$([ "$SKIP_QGC" = false ] && echo "0.1" || echo "0.0")
echo -e "${GREEN}[Pane 1] Launching PX4 SITL + Gazebo...${NC}"
tmux send-keys -t "$SESSION":$PANE_PX4 "echo -e '${GREEN}Starting PX4 SITL + Gazebo...${NC}'" C-m
tmux send-keys -t "$SESSION":$PANE_PX4 "cd $PX4_ROOT && $PX4_LAUNCH_CMD" C-m

# Wait for PX4 to initialize, then start the IMU AI bridge
echo -e "${YELLOW}Waiting 8 seconds for PX4 to initialize...${NC}"
sleep 8

# Nudge the pxh prompt and start the bridge
tmux send-keys -t "$SESSION":$PANE_PX4 C-m
tmux send-keys -t "$SESSION":$PANE_PX4 "echo -e '${GREEN}Starting imu_ai_bridge on port $IMU_BRIDGE_PORT...${NC}'" C-m
tmux send-keys -t "$SESSION":$PANE_PX4 "imu_ai_bridge start -p $IMU_BRIDGE_PORT" C-m

# Pane 2: MAVROS launch (split below PX4 SITL)
echo -e "${GREEN}[Pane 2] Launching MAVROS...${NC}"
tmux split-window -v -t "$SESSION":$PANE_PX4
PANE_MAVROS=$([ "$SKIP_QGC" = false ] && echo "0.2" || echo "0.1")
tmux send-keys -t "$SESSION":$PANE_MAVROS "echo -e '${GREEN}Starting MAVROS...${NC}'" C-m
tmux send-keys -t "$SESSION":$PANE_MAVROS "source /opt/ros/noetic/setup.bash && roslaunch mavros px4.launch fcu_url:=udp://:14540@localhost:14580 fcu_protocol:=v2.0" C-m

# Pane 3: AI IMU Client (split below MAVROS)
echo -e "${GREEN}[Pane 3] Launching AI IMU Client...${NC}"
tmux split-window -v -t "$SESSION":$PANE_MAVROS
PANE_AI=$([ "$SKIP_QGC" = false ] && echo "0.3" || echo "0.2")
tmux send-keys -t "$SESSION":$PANE_AI "echo -e '${GREEN}Waiting 3 seconds before starting AI IMU Client...${NC}'" C-m
tmux send-keys -t "$SESSION":$PANE_AI "sleep 3" C-m
tmux send-keys -t "$SESSION":$PANE_AI "echo -e '${GREEN}Starting AI IMU Client (TCP pipeline)...${NC}'" C-m
tmux send-keys -t "$SESSION":$PANE_AI "python3 $PX4_ROOT/src/modules/ekf2/ai_imu_client.py" C-m

# Optional: Pane 4 for EKF2 UDP Telemetry Listener (uncomment if needed)
# echo -e "${GREEN}[Pane 4] Launching EKF2 UDP Listener...${NC}"
# tmux split-window -v -t "$SESSION":$PANE_AI
# PANE_UDP=$([ "$SKIP_QGC" = false ] && echo "0.4" || echo "0.3")
# tmux send-keys -t "$SESSION":$PANE_UDP "sleep 5" C-m
# # Use C++ version:
# tmux send-keys -t "$SESSION":$PANE_UDP "cd $PX4_ROOT/src/modules/imu_ai_bridge && ./listen_ekf2_udp" C-m
# # Or use Python version instead:
# # tmux send-keys -t "$SESSION":$PANE_UDP "python3 $PX4_ROOT/src/modules/imu_ai_bridge/listen_ekf2_udp.py" C-m

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}All panes launched successfully!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}Tmux Commands:${NC}"
echo -e "  Ctrl+B then D      - Detach from session"
echo -e "  Ctrl+B then arrow  - Navigate between panes"
echo -e "  Ctrl+B then [      - Scroll mode (Q to exit)"
echo -e "  tmux attach -t $SESSION  - Re-attach to session"
echo ""
echo -e "${YELLOW}To kill everything:${NC}"
echo -e "  tmux kill-session -t $SESSION"
echo -e "  killall -9 QGroundControl.AppImage px4 roslaunch python3"
echo ""

# Attach to the session
tmux attach -t "$SESSION"
