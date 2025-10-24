from pymavlink import mavutil
import sys

# Connect to PX4 SITL MAVLink stream
port = int(sys.argv[1]) if len(sys.argv) > 1 else 14550

print(f"Connecting to MAVLink on 127.0.0.1:{port}...")
master = mavutil.mavlink_connection(f'udp:127.0.0.1:{port}')
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected! Listening for messages...")

# First, list all message types received for 5 seconds to see what's available
msg_types = {}
import time
start_time = time.time()

print("\nScanning for available message types (5 seconds)...")
while time.time() - start_time < 5:
    msg = master.recv_match(blocking=False)
    if msg:
        msg_type = msg.get_type()
        msg_types[msg_type] = msg_types.get(msg_type, 0) + 1

print("\nMessage types received:")
for msg_type, count in sorted(msg_types.items()):
    print(f"  {msg_type}: {count} messages")

# Now listen specifically for HIGHRES_IMU or RAW_IMU
print("\nListening for IMU messages...")
count = 0
while True:
    msg = master.recv_match(type=['HIGHRES_IMU', 'RAW_IMU'], blocking=True)
    if msg:
        msg_type = msg.get_type()
        count += 1
        if count % 10 == 0:
            if msg_type == 'HIGHRES_IMU':
                print(f"[{count}] {msg_type}: xacc={msg.xacc:.3f}, yacc={msg.yacc:.3f}, zacc={msg.zacc:.3f}")
            elif msg_type == 'RAW_IMU':
                print(f"[{count}] {msg_type}: xacc={msg.xacc}, yacc={msg.yacc}, zacc={msg.zacc}")
