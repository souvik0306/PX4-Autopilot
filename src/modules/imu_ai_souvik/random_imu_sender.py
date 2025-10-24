#!/usr/bin/env python3
"""
Random IMU generator sending delta data over UDP to PX4 imu_ai_bridge.

Generates synthetic IMU data at 200-250 Hz with realistic noise characteristics,
converts to FRD frame, and sends packed structs matching vehicle_imu_ai.msg format.

Struct layout:
    uint64  timestamp
    uint64  timestamp_sample
    uint32  accel_device_id
    uint32  gyro_device_id
    float32[3] delta_angle
    float32[3] delta_velocity
    uint16  delta_angle_dt           # microseconds
    uint16  delta_velocity_dt        # microseconds
    uint8   delta_angle_clipping
    uint8   delta_velocity_clipping
    uint8   accel_calibration_count
    uint8   gyro_calibration_count
"""

import socket
import struct
import numpy as np
import time
import sys

# --- configuration ---
TARGET_RATE = 250  # Hz (midpoint of 200-250 Hz)
TIMING_JITTER = 0.1  # ±10% timing variation


class RandomIMUSender:
    """Generates random IMU data and sends over UDP to PX4."""

    def __init__(self, udp_host='127.0.0.1', udp_port=14560):
        self.udp_host = udp_host
        self.udp_port = udp_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = (self.udp_host, self.udp_port)

        # Device IDs matching fake_imu.py
        self.accel_device_id = 0xA14ACC01
        self.gyro_device_id = 0xA14A7701

        # State for realistic data generation
        self.last_time = time.time()
        self.msg_count = 0
        self.start_time = time.time()

        # Base values with small bias
        self.gyro_bias = np.array([0.001, -0.002, 0.0015])  # rad/s
        self.accel_bias = np.array([0.01, -0.01, 0.005])    # m/s²
        self.gravity = 9.81  # m/s²

        print(f"[RandomIMUSender] Initialized UDP -> {self.udp_host}:{self.udp_port}")
        print(f"[RandomIMUSender] Target rate: {TARGET_RATE} Hz ±{TIMING_JITTER*100:.0f}%")

    def generate_imu_sample(self):
        """Generate realistic synthetic IMU data in FRD frame."""
        # Gyroscope: small random rotation rates ±5 deg/s with bias
        gyro = np.random.randn(3) * np.radians(5.0) + self.gyro_bias

        # Accelerometer: gravity + small vibrations in FRD frame
        # FRD: X forward, Y right, Z down
        # At rest on ground: accel should be [0, 0, -9.81] (gravity pointing down)
        accel = np.array([
            np.random.randn() * 0.5 + self.accel_bias[0],      # forward
            np.random.randn() * 0.5 + self.accel_bias[1],      # right
            -self.gravity + np.random.randn() * 0.5 + self.accel_bias[2]  # down
        ])

        return gyro, accel

    def pack_payload(self, delta_angle, delta_velocity, dt):
        """Pack IMU delta data into struct for UDP."""
        now_us = int(time.time() * 1e6)
        timestamp_us = now_us
        timestamp_sample = now_us

        delta_angle_dt = int(round(dt * 1e6))
        delta_velocity_dt = int(round(dt * 1e6))

        # Saturate to uint16
        delta_angle_dt = max(0, min(0xFFFF, delta_angle_dt))
        delta_velocity_dt = max(0, min(0xFFFF, delta_velocity_dt))

        delta_angle_clipping = 0
        delta_velocity_clipping = 0
        accel_calibration_count = 0
        gyro_calibration_count = 0

        # Format: Q=8, Q=8, I=4, I=4, 3f=12, 3f=12, H=2, H=2, B=1, B=1, B=1, B=1 = 56 bytes
        fmt = '<QQII3f3fHHBBBB'
        return struct.pack(
            fmt,
            timestamp_us,
            timestamp_sample,
            self.accel_device_id,
            self.gyro_device_id,
            delta_angle[0], delta_angle[1], delta_angle[2],
            delta_velocity[0], delta_velocity[1], delta_velocity[2],
            delta_angle_dt,
            delta_velocity_dt,
            delta_angle_clipping,
            delta_velocity_clipping,
            accel_calibration_count,
            gyro_calibration_count,
        )

    def send_sample(self):
        """Generate and send one IMU sample."""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Clamp dt to reasonable range (matches fake_imu.py)
        dt = max(5e-4, min(dt, 0.04))  # 0.5ms to 40ms

        # Generate synthetic IMU data
        gyro, accel = self.generate_imu_sample()

        # Integrate to deltas (matches fake_imu.py)
        delta_angle = gyro * dt
        delta_velocity = accel * dt

        # Pack and send
        payload = self.pack_payload(delta_angle, delta_velocity, dt)

        # Debug: print first few samples to verify values
        if self.msg_count < 3:
            print(f"[RandomIMUSender] Sample {self.msg_count}: dt={dt*1e3:.2f}ms, "
                  f"da=[{delta_angle[0]:.6f},{delta_angle[1]:.6f},{delta_angle[2]:.6f}], "
                  f"dv=[{delta_velocity[0]:.6f},{delta_velocity[1]:.6f},{delta_velocity[2]:.6f}]")

        try:
            self.sock.sendto(payload, self.target)
            self.msg_count += 1

            # Log stats every 200 messages
            if self.msg_count % 200 == 0:
                elapsed = time.time() - self.start_time
                rate = self.msg_count / elapsed if elapsed > 0 else 0.0
                print(f"[RandomIMUSender] sent={self.msg_count}, "
                      f"rate={rate:.1f} Hz, dt={dt*1e3:.2f} ms")
        except Exception as e:
            print(f"[RandomIMUSender] UDP send failed: {e}", file=sys.stderr)

    def run(self):
        """Main loop: generate and send IMU data at target rate."""
        print(f"[RandomIMUSender] Starting transmission loop...")
        print(f"[RandomIMUSender] Press Ctrl+C to stop")

        try:
            while True:
                self.send_sample()

                # Sleep with jitter for realistic timing variation
                base_period = 1.0 / TARGET_RATE
                jitter = np.random.uniform(-TIMING_JITTER, TIMING_JITTER) * base_period
                sleep_time = base_period + jitter

                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print(f"\n[RandomIMUSender] Stopped by user")
            elapsed = time.time() - self.start_time
            avg_rate = self.msg_count / elapsed if elapsed > 0 else 0.0
            print(f"[RandomIMUSender] Final stats: {self.msg_count} messages, "
                  f"avg rate={avg_rate:.1f} Hz, duration={elapsed:.1f}s")


def main():
    """Entry point."""
    # Parse command line args (support both positional and ROS-style)
    udp_host = '127.0.0.1'
    udp_port = 14560

    for arg in sys.argv[1:]:
        if arg.startswith('_udp_host:='):
            udp_host = arg.split(':=', 1)[1]
        elif arg.startswith('_udp_port:='):
            udp_port = int(arg.split(':=', 1)[1])
        elif '.' in arg or arg == 'localhost':  # Looks like a hostname
            udp_host = arg
        elif arg.isdigit():  # Looks like a port number
            udp_port = int(arg)

    print("=" * 70)
    print("[RandomIMUSender] Synthetic IMU Generator")
    print(f"[RandomIMUSender] Target: {udp_host}:{udp_port}")
    print("=" * 70)

    sender = RandomIMUSender(udp_host, udp_port)
    sender.run()


if __name__ == '__main__':
    main()
