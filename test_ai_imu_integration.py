#!/usr/bin/env python3
"""
Complete test for AI IMU integration
Tests both UDP communication and data processing
"""

import socket
import struct
import time
import threading
from ai_imu_processor import AIIMUProcessor, IMUSample

class UDPReceiver:
    def __init__(self, port=14560):
        self.port = port
        self.sock = None
        self.running = False
        self.received_count = 0
        self.last_data = None
        
    def start(self):
        """Start UDP receiver in background thread"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', self.port))
        self.sock.settimeout(0.1)
        self.running = True
        
        self.thread = threading.Thread(target=self._receive_loop)
        self.thread.daemon = True
        self.thread.start()
        print(f"[UDP Receiver] Started on port {self.port}")
    
    def _receive_loop(self):
        """Background receive loop"""
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                self.received_count += 1
                
                if len(data) == 55:  # Expected size for vehicle_imu_ai
                    fmt = '<QQII3f3fHHBBB'
                    unpacked = struct.unpack(fmt, data)
                    self.last_data = unpacked
                    
                    timestamp, timestamp_sample, accel_id, gyro_id, \
                    da_x, da_y, da_z, dv_x, dv_y, dv_z, \
                    da_dt, dv_dt, clipping, accel_cal, gyro_cal = unpacked
                    
                    print(f"[UDP Receiver] Msg {self.received_count}: "
                          f"da=[{da_x:6.4f},{da_y:6.4f},{da_z:6.4f}] "
                          f"dv=[{dv_x:6.4f},{dv_y:6.4f},{dv_z:6.4f}] "
                          f"dt={da_dt}μs")
                else:
                    print(f"[UDP Receiver] Unexpected size: {len(data)} bytes")
                    
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[UDP Receiver] Error: {e}")
                break
    
    def stop(self):
        """Stop UDP receiver"""
        self.running = False
        if self.sock:
            self.sock.close()
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1.0)
        print(f"[UDP Receiver] Stopped. Received {self.received_count} messages")

def test_ai_imu_integration():
    """Test complete AI IMU integration"""
    print("AI IMU Integration Test")
    print("=====================")
    
    # Start UDP receiver
    receiver = UDPReceiver()
    receiver.start()
    
    # Create AI processor
    processor = AIIMUProcessor()
    
    try:
        print("\n1. Testing single sample processing...")
        # Create first sample (will be used for initialization)
        sample1 = IMUSample(
            timestamp=time.time(),
            gyro=[0.1, 0.05, 0.02],
            accel=[0.0, 0.0, 9.81]
        )
        processor.process_imu_sample(sample1)
        print("   First sample processed (initialization)")
        
        # Wait a bit for UDP
        time.sleep(0.1)
        
        print("\n2. Testing continuous data flow...")
        # Generate several samples
        for i in range(10):
            sample = IMUSample(
                timestamp=time.time() + i * 0.005,  # 200Hz
                gyro=[0.1 + i*0.01, 0.05 + i*0.005, 0.02],
                accel=[0.0, 0.0, 9.81 + i*0.1]
            )
            processor.process_imu_sample(sample)
            time.sleep(0.01)  # Small delay between samples
        
        print("   Continuous samples processed")
        
        # Wait for UDP processing
        time.sleep(0.5)
        
        print("\n3. Testing realistic motion patterns...")
        # Simulate some realistic motion
        for i in range(20):
            t = i * 0.005
            # Oscillating motion
            gyro = [
                0.1 * math.sin(2 * math.pi * 0.5 * t),  # Roll oscillation
                0.05 * math.cos(2 * math.pi * 0.3 * t),  # Pitch oscillation
                0.02 * math.sin(2 * math.pi * 0.1 * t)   # Yaw oscillation
            ]
            accel = [
                0.0,  # Forward
                0.0,  # Right
                9.81  # Down (gravity)
            ]
            
            sample = IMUSample(
                timestamp=time.time() + t,
                gyro=gyro,
                accel=accel
            )
            processor.process_imu_sample(sample)
            time.sleep(0.005)  # 200Hz
        
        print("   Motion patterns processed")
        
        # Final wait
        time.sleep(0.5)
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    finally:
        # Stop receiver
        receiver.stop()
        
        # Print statistics
        print(f"\nTest Results:")
        print(f"  Messages sent: {processor.msg_count}")
        print(f"  Messages received: {receiver.received_count}")
        print(f"  Samples processed: {processor.stats['samples_processed']}")
        print(f"  Samples dropped: {processor.stats['samples_dropped']}")
        print(f"  Average dt: {processor.stats['avg_dt']*1e3:.2f} ms")
        
        if receiver.last_data:
            print(f"\nLast received data:")
            print(f"  Timestamp: {receiver.last_data[0]}")
            print(f"  Sample time: {receiver.last_data[1]}")
            print(f"  Device IDs: Accel=0x{receiver.last_data[2]:08X}, Gyro=0x{receiver.last_data[3]:08X}")
            print(f"  Delta angle: [{receiver.last_data[4]:.4f}, {receiver.last_data[5]:.4f}, {receiver.last_data[6]:.4f}]")
            print(f"  Delta velocity: [{receiver.last_data[7]:.4f}, {receiver.last_data[8]:.4f}, {receiver.last_data[9]:.4f}]")
            print(f"  Integration time: {receiver.last_data[10]}μs")

if __name__ == "__main__":
    import math
    test_ai_imu_integration()