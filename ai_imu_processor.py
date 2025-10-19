#!/usr/bin/env python3
"""
AI IMU Processor - Standalone version for testing PX4 AI integration
Simulates the complete flow: Raw IMU -> AI Processing -> UDP to PX4
"""

import socket
import struct
import time
import math
import numpy as np
import threading
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class IMUSample:
    """IMU sample data structure"""
    timestamp: float
    gyro: List[float]  # [x, y, z] in rad/s
    accel: List[float]  # [x, y, z] in m/s²

class AIIMUProcessor:
    def __init__(self, udp_host='127.0.0.1', udp_port=14560):
        # UDP settings
        self.udp_host = udp_host
        self.udp_port = udp_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = (self.udp_host, self.udp_port)
        
        # AI processing parameters
        self.alpha = 0.2  # Low-pass filter coefficient
        self.max_dt = 0.02 * 2  # Max integration window (2x nominal 200Hz)
        self.min_dt = 5e-4  # Min integration window (0.5ms)
        
        # Device IDs for AI-processed data
        self.accel_device_id = 0xA14ACC01
        self.gyro_device_id = 0xA14A7701
        
        # State variables
        self.last_stamp = None
        self.filt_gyro = [0.0, 0.0, 0.0]
        self.filt_accel = [0.0, 0.0, 0.0]
        self.msg_count = 0
        self.start_time = time.time()
        
        # Statistics
        self.stats = {
            'samples_processed': 0,
            'samples_dropped': 0,
            'avg_dt': 0.0,
            'last_rate': 0.0
        }
        
        print(f"[AI IMU Processor] UDP target: {self.udp_host}:{self.udp_port}")
        print(f"[AI IMU Processor] Filter alpha: {self.alpha}")
        print(f"[AI IMU Processor] Device IDs - Accel: 0x{self.accel_device_id:08X}, Gyro: 0x{self.gyro_device_id:08X}")
    
    def _enu_to_frd(self, vec: List[float]) -> List[float]:
        """Convert ENU to FRD (Forward, Right, Down) frame"""
        return [vec[1], vec[0], -vec[2]]
    
    def _clamp(self, x: float, lo: float, hi: float) -> float:
        """Clamp value between bounds"""
        return max(lo, min(hi, x))
    
    def process_imu_sample(self, sample: IMUSample):
        """Process a single IMU sample through AI pipeline"""
        current_time = time.time()
        
        # Initialize on first sample
        if self.last_stamp is None:
            self.last_stamp = sample.timestamp
            # Convert to FRD and initialize filters
            gyro_frd = self._enu_to_frd(sample.gyro)
            accel_frd = self._enu_to_frd(sample.accel)
            self.filt_gyro = gyro_frd
            self.filt_accel = accel_frd
            return
        
        # Calculate time delta
        dt = sample.timestamp - self.last_stamp
        
        # Skip invalid or too short intervals
        if dt <= 0.0 or dt < self.min_dt:
            self.stats['samples_dropped'] += 1
            if self.stats['samples_dropped'] % 100 == 0:
                print(f"[AI IMU Processor] Dropped {self.stats['samples_dropped']} samples (dt={dt*1e6:.1f}μs)")
            return
        
        # Update timestamp
        self.last_stamp = sample.timestamp
        dt = self._clamp(dt, self.min_dt, self.max_dt)
        
        # Convert to FRD frame
        gyro_frd = self._enu_to_frd(sample.gyro)
        accel_frd = self._enu_to_frd(sample.accel)
        
        # Apply low-pass filter (simple exponential filter)
        alpha = self.alpha
        inv_alpha = 1.0 - alpha
        
        self.filt_gyro = [
            alpha * gyro_frd[0] + inv_alpha * self.filt_gyro[0],
            alpha * gyro_frd[1] + inv_alpha * self.filt_gyro[1],
            alpha * gyro_frd[2] + inv_alpha * self.filt_gyro[2]
        ]
        
        self.filt_accel = [
            alpha * accel_frd[0] + inv_alpha * self.filt_accel[0],
            alpha * accel_frd[1] + inv_alpha * self.filt_accel[1],
            alpha * accel_frd[2] + inv_alpha * self.filt_accel[2]
        ]
        
        # Integrate to deltas
        delta_angle = [
            self.filt_gyro[0] * dt,  # rad
            self.filt_gyro[1] * dt,
            self.filt_gyro[2] * dt
        ]
        
        delta_velocity = [
            self.filt_accel[0] * dt,  # m/s
            self.filt_accel[1] * dt,
            self.filt_accel[2] * dt
        ]
        
        # Pack into vehicle_imu_ai message format
        self._send_ai_imu_message(sample.timestamp, delta_angle, delta_velocity, dt)
        
        # Update statistics
        self.stats['samples_processed'] += 1
        self.stats['avg_dt'] = (self.stats['avg_dt'] * (self.stats['samples_processed'] - 1) + dt) / self.stats['samples_processed']
        
        # Print status every 200 samples
        if self.stats['samples_processed'] % 200 == 0:
            elapsed = current_time - self.start_time
            rate = self.stats['samples_processed'] / elapsed if elapsed > 0 else 0.0
            self.stats['last_rate'] = rate
            print(f"[AI IMU Processor] Processed: {self.stats['samples_processed']}, "
                  f"Rate: {rate:.1f} Hz, Avg dt: {self.stats['avg_dt']*1e3:.2f} ms, "
                  f"Dropped: {self.stats['samples_dropped']}")
    
    def _send_ai_imu_message(self, timestamp: float, delta_angle: List[float], 
                           delta_velocity: List[float], dt: float):
        """Send AI-processed IMU data to PX4 via UDP"""
        try:
            # Convert timestamps to microseconds
            now_us = int(time.time() * 1e6)
            timestamp_us = int(timestamp * 1e6)
            
            # Convert dt to microseconds and clamp to uint16 range
            delta_angle_dt = max(0, min(0xFFFF, int(round(dt * 1e6))))
            delta_velocity_dt = max(0, min(0xFFFF, int(round(dt * 1e6))))
            
            # Pack message in little-endian format matching vehicle_imu_ai.msg
            fmt = '<QQII3f3fHHBBB'
            payload = struct.pack(
                fmt,
                now_us,                    # timestamp
                timestamp_us,              # timestamp_sample
                self.accel_device_id,      # accel_device_id
                self.gyro_device_id,       # gyro_device_id
                delta_angle[0], delta_angle[1], delta_angle[2],  # delta_angle[3]
                delta_velocity[0], delta_velocity[1], delta_velocity[2],  # delta_velocity[3]
                delta_angle_dt,            # delta_angle_dt
                delta_velocity_dt,         # delta_velocity_dt
                0,                         # delta_velocity_clipping
                0,                         # accel_calibration_count
                0                          # gyro_calibration_count
            )
            
            # Send UDP packet
            self.sock.sendto(payload, self.target)
            self.msg_count += 1
            
        except Exception as e:
            print(f"[AI IMU Processor] UDP send error: {e}")
    
    def generate_test_data(self, duration: float = 60.0, rate: float = 200.0):
        """Generate test IMU data for demonstration"""
        print(f"[AI IMU Processor] Generating test data for {duration}s at {rate}Hz")
        
        dt = 1.0 / rate
        start_time = time.time()
        
        # Add some realistic motion patterns
        t = 0.0
        while t < duration:
            current_time = start_time + t
            
            # Simulate some realistic motion
            # Roll oscillation
            roll_rate = 0.1 * math.sin(2 * math.pi * 0.5 * t)  # 0.5 Hz oscillation
            pitch_rate = 0.05 * math.cos(2 * math.pi * 0.3 * t)  # 0.3 Hz oscillation
            yaw_rate = 0.02 * math.sin(2 * math.pi * 0.1 * t)  # 0.1 Hz oscillation
            
            # Add some noise
            noise_scale = 0.01
            gyro = [
                roll_rate + np.random.normal(0, noise_scale),
                pitch_rate + np.random.normal(0, noise_scale),
                yaw_rate + np.random.normal(0, noise_scale)
            ]
            
            # Simulate acceleration (gravity + some motion)
            accel = [
                0.0 + np.random.normal(0, noise_scale * 10),  # Forward
                0.0 + np.random.normal(0, noise_scale * 10),  # Right
                9.81 + np.random.normal(0, noise_scale * 10)  # Down (gravity)
            ]
            
            # Create sample
            sample = IMUSample(
                timestamp=current_time,
                gyro=gyro,
                accel=accel
            )
            
            # Process through AI pipeline
            self.process_imu_sample(sample)
            
            # Sleep to maintain rate
            time.sleep(dt)
            t += dt
        
        print(f"[AI IMU Processor] Test data generation complete")
        self.print_stats()
    
    def print_stats(self):
        """Print processing statistics"""
        print("\n=== AI IMU Processor Statistics ===")
        print(f"Samples processed: {self.stats['samples_processed']}")
        print(f"Samples dropped: {self.stats['samples_dropped']}")
        print(f"Average dt: {self.stats['avg_dt']*1e3:.2f} ms")
        print(f"Last rate: {self.stats['last_rate']:.1f} Hz")
        print(f"Messages sent: {self.msg_count}")
        print("===================================")

def main():
    """Main function for testing"""
    print("AI IMU Processor - PX4 Integration Test")
    print("======================================")
    
    # Create processor
    processor = AIIMUProcessor()
    
    try:
        # Generate test data
        processor.generate_test_data(duration=30.0, rate=200.0)
        
    except KeyboardInterrupt:
        print("\n[AI IMU Processor] Interrupted by user")
    finally:
        processor.print_stats()
        print("[AI IMU Processor] Shutting down")

if __name__ == "__main__":
    main()