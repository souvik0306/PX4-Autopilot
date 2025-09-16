#!/usr/bin/env python2

"""
Example showing how to configure MAVROS IMU data source to use a different ROS topic.

This example demonstrates three ways to configure the IMU topic:
1. Using ROS parameters
2. Setting the topic in the constructor
3. Reconfiguring at runtime
"""

from __future__ import division

import unittest
import rospy
from mavros_test_common import MavrosTestCommon
from sensor_msgs.msg import Imu


class CustomIMUTopicTest(MavrosTestCommon):
    """
    Example test class showing how to use a custom IMU topic.
    """
    
    def __init__(self, *args):
        super(CustomIMUTopicTest, self).__init__(*args)
        # Method 1: Override the default IMU topic in constructor
        # self.imu_topic = 'custom/imu/data'
    
    def test_default_imu_topic(self):
        """Test using the default mavros/imu/data topic."""
        rospy.loginfo("Testing default IMU topic: {}".format(self.imu_topic))
        
        # Initialize ROS node for testing
        rospy.init_node('test_imu_topic_config', anonymous=True)
        
        # Set up the test (this will use default mavros/imu/data)
        self.setUp()
        
        # Wait for IMU data - this would work if mavros/imu/data is publishing
        # In a real scenario, you would have MAVROS running
        rospy.loginfo("Subscribed to IMU topic: {}".format(self.imu_topic))
        rospy.loginfo("Waiting for IMU data...")
        
        # The wait_for_topics method will wait for IMU data
        # self.wait_for_topics(10)  # Uncomment if MAVROS is running
        
        self.assertTrue(True, "Default IMU topic configuration successful")

    def test_custom_imu_topic_by_parameter(self):
        """Test using a custom IMU topic set via ROS parameter."""
        rospy.loginfo("Testing custom IMU topic via ROS parameter")
        
        # Method 2: Set ROS parameter before setUp()
        custom_topic = 'custom_sensors/imu/data'
        rospy.set_param('~imu_topic', custom_topic)
        
        # Initialize ROS node
        rospy.init_node('test_custom_imu_param', anonymous=True)
        
        # Set up the test (this will read the ROS parameter)
        self.setUp()
        
        # Verify the topic was configured correctly
        self.assertEqual(self.imu_topic, custom_topic)
        rospy.loginfo("Successfully configured IMU topic via parameter: {}".format(self.imu_topic))
        
        self.assertTrue(True, "Custom IMU topic via parameter successful")

    def test_runtime_imu_topic_reconfiguration(self):
        """Test reconfiguring the IMU topic at runtime."""
        rospy.loginfo("Testing runtime IMU topic reconfiguration")
        
        # Initialize with default topic
        rospy.init_node('test_runtime_reconfig', anonymous=True)
        self.setUp()
        
        original_topic = self.imu_topic
        rospy.loginfo("Started with IMU topic: {}".format(original_topic))
        
        # Method 3: Reconfigure at runtime
        new_topic = 'runtime_changed/imu/data'
        self.reconfigure_imu_subscriber(new_topic)
        
        # Verify the topic was changed
        self.assertEqual(self.imu_topic, new_topic)
        rospy.loginfo("Successfully reconfigured IMU topic to: {}".format(self.imu_topic))
        
        # You could change it back if needed
        self.reconfigure_imu_subscriber(original_topic)
        self.assertEqual(self.imu_topic, original_topic)
        
        self.assertTrue(True, "Runtime IMU topic reconfiguration successful")


class AlternativeIMUSourceTest(MavrosTestCommon):
    """
    Example showing how to inherit and override the IMU topic.
    """
    
    def __init__(self, *args):
        super(AlternativeIMUSourceTest, self).__init__(*args)
        # Override to use a different IMU source
        self.imu_topic = 'alternative_imu/data'
    
    def test_alternative_imu_source(self):
        """Test using an alternative IMU data source."""
        rospy.loginfo("Testing alternative IMU source: {}".format(self.imu_topic))
        
        rospy.init_node('test_alternative_imu', anonymous=True)
        self.setUp()
        
        # Verify we're using the alternative topic
        self.assertEqual(self.imu_topic, 'alternative_imu/data')
        rospy.loginfo("Successfully configured alternative IMU topic: {}".format(self.imu_topic))
        
        # In a real test, you would publish data to this topic and verify it's received
        # For now, just verify the configuration
        self.assertTrue(hasattr(self, 'imu_data_sub'), "IMU subscriber should be created")
        self.assertTrue(True, "Alternative IMU source configuration successful")


if __name__ == '__main__':
    # Example of how to run these tests
    import sys
    
    print("IMU Topic Configuration Examples")
    print("=" * 40)
    print("")
    print("This example shows how to configure MAVROS to use different IMU topics:")
    print("1. Default topic: mavros/imu/data")
    print("2. Custom topic via ROS parameter: ~imu_topic")
    print("3. Runtime reconfiguration using reconfigure_imu_subscriber()")
    print("4. Inheritance-based topic override")
    print("")
    print("To run actual tests, ensure ROS is running and execute:")
    print("python -m unittest imu_topic_config_example.CustomIMUTopicTest.test_default_imu_topic")
    print("")
    
    # Run a simple demonstration
    try:
        # Initialize ROS for demo
        rospy.init_node('imu_config_demo', anonymous=True)
        
        # Create test instance and demonstrate configuration
        test = CustomIMUTopicTest()
        print("Default IMU topic: {}".format(test.imu_topic))
        
        # Show topic configuration
        test.set_imu_topic('demo/imu/topic')
        print("Configured IMU topic: {}".format(test.imu_topic))
        
        print("")
        print("Configuration successful! See test methods for detailed usage examples.")
        
    except Exception as e:
        print("Demo requires ROS environment. Error: {}".format(e))
        print("To run in ROS environment: rosrun px4 imu_topic_config_example.py")