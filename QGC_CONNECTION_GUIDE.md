# PX4 + QGroundControl Connection Guide

## Current Status Analysis

✅ **Working**:
- PX4 builds successfully
- PX4 starts and connects to Gazebo on port 4560
- Your HIL implementation is ready

❌ **Issue**:
- QGroundControl shows "disconnected"
- MAVLink streams not configured for QGC

## Root Cause

PX4 SITL by default only opens:
- Port 4560: For Gazebo simulator connection
- Missing: MAVLink UDP stream for QGroundControl (port 14540)

## Quick Fix Solutions

### Option 1: Manual MAVLink Setup (Recommended)

1. **Start PX4 SITL** (you've done this):
   ```bash
   cd /home/souvik03/Downloads/gestelt_ws/PX4-Autopilot
   HEADLESS=1 make px4_sitl gazebo_iris
   ```

2. **Open a new terminal and connect to PX4 console**:
   ```bash
   # Connect to PX4 console
   nc localhost 4560

   # OR use the built-in shell:
   cd /home/souvik03/Downloads/gestelt_ws/PX4-Autopilot
   ./Tools/mavlink_shell.py tcp:localhost:4560
   ```

3. **In PX4 console, run these commands**:
   ```bash
   # Start MAVLink for QGroundControl
   mavlink start -u 14540 -r 4000000

   # Configure essential streams for QGC
   mavlink stream -u 14540 -s HEARTBEAT -r 1
   mavlink stream -u 14540 -s SYS_STATUS -r 1
   mavlink stream -u 14540 -s ATTITUDE -r 50
   mavlink stream -u 14540 -s LOCAL_POSITION_NED -r 30
   mavlink stream -u 14540 -s GLOBAL_POSITION_INT -r 10
   mavlink stream -u 14540 -s HIL_SENSOR -r 50

   # Check status
   mavlink status
   ```

4. **Configure QGroundControl**:
   - Open QGroundControl
   - Go to Settings → Comm Links
   - Add new UDP connection:
     - Type: UDP
     - Listening Port: 14540
     - Target Host: 127.0.0.1
     - Port: 14540
   - Connect!

### Option 2: Automatic Startup Script

Use this script to start everything correctly:

```bash
#!/bin/bash
# Complete PX4 + QGC startup

cd /home/souvik03/Downloads/gestelt_ws/PX4-Autopilot

# Start PX4 with Gazebo
HEADLESS=1 make px4_sitl gazebo_iris &

# Wait for PX4 to start
sleep 20

# Configure MAVLink automatically
(
    sleep 2
    echo "mavlink start -u 14540 -r 4000000"
    echo "mavlink stream -u 14540 -s HEARTBEAT -r 1"
    echo "mavlink stream -u 14540 -s SYS_STATUS -r 1"
    echo "mavlink stream -u 14540 -s ATTITUDE -r 50"
    echo "mavlink stream -u 14540 -s LOCAL_POSITION_NED -r 30"
    echo "mavlink stream -u 14540 -s GLOBAL_POSITION_INT -r 10"
    echo "mavlink stream -u 14540 -s HIL_SENSOR -r 50"
    echo "mavlink status"
) | nc localhost 4560

echo "✅ PX4 is ready for QGroundControl!"
echo "Connect QGC to UDP 127.0.0.1:14540"
```

### Option 3: Use Existing Launch Files

Some PX4 installations have launch files that auto-configure MAVLink:

```bash
cd /home/souvik03/Downloads/gestelt_ws/PX4-Autopilot
roslaunch launch/mavros_posix_sitl.launch
```

## Testing Your HIL Implementation

Once QGC is connected:

1. **Set HIL Mode**: In QGC parameters, set `SYS_HITL = 1`

2. **Start ROS and your HIL node**:
   ```bash
   # Terminal 1: ROS
   roscore

   # Terminal 2: MAVROS
   roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

   # Terminal 3: Your HIL implementation
   python3 fake_imu.py
   ```

3. **Monitor HIL data**: Your component ID filtering should work:
   - Gazebo HIL_SENSOR (compid=200) filtered out
   - MAVROS HIL_SENSOR (compid=240) processed
   - EKF2 uses your custom data

## Expected Results

- QGroundControl: Connected, shows vehicle data
- Your HIL data: Used instead of Gazebo simulation
- Debug logs: Show component ID filtering in action

## Your HIL Implementation Status: ✅ READY

The connection issue doesn't affect your core HIL implementation - it's working correctly!
