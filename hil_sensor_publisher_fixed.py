#!/usr/bin/env python3

"""
HIL Sensor Publisher for PX4

This node reads IMU data from MAVROS, processes it (adds bias for testing),
and publishes it as HIL sensor data that PX4 can use for state estimation.

The processed data includes a small bias to make it distinguishable from
the original Gazebo simulation data, allowing us to verify that PX4 is
using our custom HIL sensor data instead of the default simulation data.
"""

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from mavros_msgs.msg import HilSensor


class HILSensorPublisher:
    def __init__(self):
        rospy.init_node('hil_sensor_publisher', anonymous=True)

        # Publisher for HIL sensor data
        self.hil_pub = rospy.Publisher('/mavros/hil/sensor', HilSensor, queue_size=10)

        # Subscriber to IMU data
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)

        # Add distinguishable bias to sensor data
        self.accel_bias = np.array([0.1, 0.1, 0.1])  # m/s² bias
        self.gyro_bias = np.array([0.05, 0.05, 0.05])  # rad/s bias

        rospy.loginfo("HIL Sensor Publisher started - processing IMU data and publishing as HIL sensors")
        rospy.loginfo("Adding bias to make processed data distinguishable: accel_bias=[0.1, 0.1, 0.1], gyro_bias=[0.05, 0.05, 0.05]")

    def imu_callback(self, msg):
        """Process IMU data and publish as HIL sensor data"""

        # Create HIL sensor message
        hil_msg = HilSensor()

        # Header with timestamp
        hil_msg.header.stamp = rospy.Time.now()
        hil_msg.header.frame_id = "hil_sensor"

        # Linear acceleration (with bias added to make it distinguishable)
        accel_raw = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        accel_processed = accel_raw + self.accel_bias

        hil_msg.acc.x = accel_processed[0]
        hil_msg.acc.y = accel_processed[1]
        hil_msg.acc.z = accel_processed[2]

        # Angular velocity (with bias added)
        gyro_raw = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        gyro_processed = gyro_raw + self.gyro_bias

        hil_msg.gyro.x = gyro_processed[0]
        hil_msg.gyro.y = gyro_processed[1]
        hil_msg.gyro.z = gyro_processed[2]

        # Magnetometer (dummy values)
        hil_msg.mag.x = 0.2
        hil_msg.mag.y = 0.0
        hil_msg.mag.z = -0.4

        # Pressure/temperature sensors (dummy values)
        hil_msg.abs_pressure = 1013.25  # mbar
        hil_msg.diff_pressure = 0.0
        hil_msg.pressure_alt = 0.0      # meters
        hil_msg.temperature = 25.0      # celsius

        # Fields updated mask (indicate which sensors have new data)
        # Bit mask for accelerometer + gyroscope + magnetometer
        hil_msg.fields_updated = 0x1FF  # All sensors updated

        # Publish the message
        self.hil_pub.publish(hil_msg)

        # Log every 20th message to avoid spam
        if hasattr(self, 'msg_count'):
            self.msg_count += 1
        else:
            self.msg_count = 1

        if self.msg_count % 20 == 0:
            rospy.loginfo(f"HIL Sensor #{self.msg_count}: acc=[{accel_processed[0]:.3f}, {accel_processed[1]:.3f}, {accel_processed[2]:.3f}] "
                         f"gyro=[{gyro_processed[0]:.3f}, {gyro_processed[1]:.3f}, {gyro_processed[2]:.3f}]")


if __name__ == '__main__':
    try:
        publisher = HILSensorPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("HIL Sensor Publisher stopped")
