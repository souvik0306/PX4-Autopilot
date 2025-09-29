#!/usr/bin/env python3
"""
UDP delta sender for PX4 imu_ai_bridge.

Subscribes to /mavros/imu/data, applies simple low-pass denoising,
computes integrated deltas and sends a packed struct compatible with
vehicle_imu_ai over UDP to the PX4 bridge.

Struct layout (little-endian):
  uint64 timestamp
  uint64 timestamp_sample
  uint32 accel_device_id
  uint32 gyro_device_id
  float32[3] delta_angle
  float32[3] delta_velocity
  float32 delta_angle_dt
  float32 delta_velocity_dt
  uint8  delta_angle_clipping
  uint8  delta_velocity_clipping
  uint8  accel_calibration_count
  uint8  gyro_calibration_count
"""

import socket
import struct
import math
import rospy
from sensor_msgs.msg import Imu


class AIDeltaSender:
    def __init__(self):
        # Params
        self.udp_host = rospy.get_param('~udp_host', '127.0.0.1')
        self.udp_port = int(rospy.get_param('~udp_port', 14560))
        self.alpha = float(rospy.get_param('~alpha', 0.2))  # LPF coefficient [0..1]
        self.max_dt = float(rospy.get_param('~max_dt', 0.02 * 2))  # cap dt to 2x nominal 200 Hz
        self.min_dt = float(rospy.get_param('~min_dt', 1e-4))

        # Optional static biases (remove if already compensated upstream)
        self.gyro_bias = rospy.get_param('~gyro_bias', [0.0, 0.0, 0.0])
        self.accel_bias = rospy.get_param('~accel_bias', [0.0, 0.0, 0.0])

        # UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = (self.udp_host, self.udp_port)

        # State
        self.last_stamp = None
        self.filt_gyro = [0.0, 0.0, 0.0]
        self.filt_accel = [0.0, 0.0, 0.0]
        self.msg_count = 0
        self.start_time = rospy.Time.now().to_sec()

        rospy.Subscriber('/mavros/imu/data', Imu, self.on_imu)
        rospy.loginfo(f"[ai_delta_sender] UDP -> {self.udp_host}:{self.udp_port}, alpha={self.alpha}")

    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def on_imu(self, imu: Imu):
        # Time handling
        stamp = imu.header.stamp.to_sec() if imu.header.stamp and imu.header.stamp.to_sec() > 0 else rospy.Time.now().to_sec()
        if self.last_stamp is None:
            self.last_stamp = stamp
            # Initialize filters with first sample
            self.filt_gyro = [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]
            self.filt_accel = [imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z]
            return

        dt = stamp - self.last_stamp
        self.last_stamp = stamp
        dt = self._clamp(dt, self.min_dt, self.max_dt)

        # Raw values with bias removal
        g = [imu.angular_velocity.x - self.gyro_bias[0],
             imu.angular_velocity.y - self.gyro_bias[1],
             imu.angular_velocity.z - self.gyro_bias[2]]
        a = [imu.linear_acceleration.x - self.accel_bias[0],
             imu.linear_acceleration.y - self.accel_bias[1],
             imu.linear_acceleration.z - self.accel_bias[2]]

        # First-order low-pass filter
        alp = self.alpha
        inv = 1.0 - alp
        self.filt_gyro = [alp * g[0] + inv * self.filt_gyro[0],
                          alp * g[1] + inv * self.filt_gyro[1],
                          alp * g[2] + inv * self.filt_gyro[2]]
        self.filt_accel = [alp * a[0] + inv * self.filt_accel[0],
                           alp * a[1] + inv * self.filt_accel[1],
                           alp * a[2] + inv * self.filt_accel[2]]

        # Integrate to deltas
        da = [self.filt_gyro[0] * dt, self.filt_gyro[1] * dt, self.filt_gyro[2] * dt]  # rad
        dv = [self.filt_accel[0] * dt, self.filt_accel[1] * dt, self.filt_accel[2] * dt]  # m/s

        # Pack struct
        timestamp_us = int(rospy.Time.now().to_sec() * 1e6)
        timestamp_sample = 0  # let the PX4 bridge fill with hrt time at publish
        accel_device_id = 0
        gyro_device_id = 0
        delta_angle_dt = float(dt)
        delta_velocity_dt = float(dt)
        delta_angle_clipping = 0
        delta_velocity_clipping = 0
        accel_calibration_count = 0
        gyro_calibration_count = 0

        fmt = '<QQII3f3f2fBBBB'
        payload = struct.pack(
            fmt,
            timestamp_us,
            timestamp_sample,
            accel_device_id,
            gyro_device_id,
            da[0], da[1], da[2],
            dv[0], dv[1], dv[2],
            delta_angle_dt,
            delta_velocity_dt,
            delta_angle_clipping,
            delta_velocity_clipping,
            accel_calibration_count,
            gyro_calibration_count,
        )

        try:
            self.sock.sendto(payload, self.target)
            self.msg_count += 1
            if self.msg_count % 200 == 0:
                elapsed = rospy.Time.now().to_sec() - self.start_time
                rate = self.msg_count / elapsed if elapsed > 0 else 0.0
                rospy.loginfo(f"[ai_delta_sender] sent={self.msg_count}, rate={rate:.1f} Hz, dt={dt*1e3:.2f} ms")
        except Exception as e:
            rospy.logwarn(f"[ai_delta_sender] UDP send failed: {e}")

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('ai_delta_sender')
    node = AIDeltaSender()
    node.spin()
