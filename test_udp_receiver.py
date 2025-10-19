#!/usr/bin/env python3
"""
UDP Receiver Test - Simulates PX4 imu_ai_bridge for testing
"""

import socket
import struct
import time

def test_udp_receiver(port=14560):
    """Test UDP receiver to verify AI IMU data format"""
    print(f"Starting UDP receiver on port {port}")
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', port))
    sock.settimeout(1.0)  # 1 second timeout
    
    print("Waiting for AI IMU data...")
    
    message_count = 0
    start_time = time.time()
    
    try:
        while True:
            try:
                # Receive data
                data, addr = sock.recvfrom(1024)
                message_count += 1
                
                # Unpack message (little-endian format)
                if len(data) == 48:  # Expected size for vehicle_imu_ai
                    fmt = '<QQII3f3fHHBBB'
                    unpacked = struct.unpack(fmt, data)
                    
                    timestamp, timestamp_sample, accel_id, gyro_id, \
                    da_x, da_y, da_z, dv_x, dv_y, dv_z, \
                    da_dt, dv_dt, clipping, accel_cal, gyro_cal = unpacked
                    
                    current_time = time.time()
                    elapsed = current_time - start_time
                    rate = message_count / elapsed if elapsed > 0 else 0
                    
                    print(f"Msg {message_count:4d}: "
                          f"ts={timestamp_sample:12d} "
                          f"da=[{da_x:7.4f},{da_y:7.4f},{da_z:7.4f}] "
                          f"dv=[{dv_x:7.4f},{dv_y:7.4f},{dv_z:7.4f}] "
                          f"dt={da_dt:4d}μs "
                          f"rate={rate:6.1f}Hz")
                    
                    # Print detailed info every 50 messages
                    if message_count % 50 == 0:
                        print(f"  Device IDs: Accel=0x{accel_id:08X}, Gyro=0x{gyro_id:08X}")
                        print(f"  Clipping: {clipping}, Cal counts: Accel={accel_cal}, Gyro={gyro_cal}")
                        print(f"  Timestamp diff: {timestamp - timestamp_sample}μs")
                        print()
                
                else:
                    print(f"Received {len(data)} bytes (expected 48)")
                    
            except socket.timeout:
                if message_count == 0:
                    print("No data received (timeout)")
                else:
                    print(f"Timeout after {message_count} messages")
                break
                
    except KeyboardInterrupt:
        print(f"\nReceived {message_count} messages")
    
    finally:
        sock.close()
        print("UDP receiver stopped")

if __name__ == "__main__":
    test_udp_receiver()