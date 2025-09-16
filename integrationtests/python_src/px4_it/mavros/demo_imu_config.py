#!/usr/bin/env python3

"""
Demonstration script showing IMU topic configuration capabilities.
This script shows the implementation without requiring ROS dependencies.
"""

class MockRospy:
    """Mock rospy for demonstration purposes."""
    @staticmethod
    def loginfo(msg):
        print(f"[INFO] {msg}")
    
    @staticmethod
    def has_param(param):
        return False
    
    @staticmethod
    def get_param(param):
        return None

class MockSubscriber:
    """Mock ROS subscriber for demonstration."""
    def __init__(self, topic, msg_type, callback):
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback
        print(f"[SUBSCRIBER] Created subscriber for topic: {topic}")
    
    def unregister(self):
        print(f"[SUBSCRIBER] Unregistered subscriber for topic: {self.topic}")

# Mock the imports for demonstration
import sys
import types
rospy_mock = types.ModuleType('rospy')
rospy_mock.loginfo = MockRospy.loginfo
rospy_mock.has_param = MockRospy.has_param
rospy_mock.get_param = MockRospy.get_param
rospy_mock.Subscriber = MockSubscriber
sys.modules['rospy'] = rospy_mock

# Mock sensor_msgs
sensor_msgs_mock = types.ModuleType('sensor_msgs')
sensor_msgs_mock.msg = types.ModuleType('msg')
sensor_msgs_mock.msg.Imu = object
sys.modules['sensor_msgs'] = sensor_msgs_mock
sys.modules['sensor_msgs.msg'] = sensor_msgs_mock.msg

class DemoMavrosTestCommon:
    """Simplified version of MavrosTestCommon for demonstration."""
    
    def __init__(self):
        # Default IMU topic - can be overridden by subclasses
        self.imu_topic = 'mavros/imu/data'
        print(f"[INIT] Default IMU topic: {self.imu_topic}")

    def setUp(self):
        """Set up test with configurable IMU topic."""
        # Configure IMU topic from ROS parameter if available
        if rospy_mock.has_param('~imu_topic'):
            self.imu_topic = rospy_mock.get_param('~imu_topic')
            rospy_mock.loginfo("Using custom IMU topic: {}".format(self.imu_topic))
        else:
            rospy_mock.loginfo("Using default IMU topic: {}".format(self.imu_topic))

        # Create IMU subscriber
        self.imu_data_sub = rospy_mock.Subscriber(self.imu_topic,
                                                 sensor_msgs_mock.msg.Imu,
                                                 self.imu_data_callback)

    def set_imu_topic(self, topic_name):
        """Configure the IMU topic source."""
        self.imu_topic = topic_name
        rospy_mock.loginfo("IMU topic configured to: {}".format(self.imu_topic))

    def reconfigure_imu_subscriber(self, new_topic):
        """Reconfigure the IMU subscriber to use a different topic at runtime."""
        # Unregister the current subscriber
        if hasattr(self, 'imu_data_sub'):
            self.imu_data_sub.unregister()
            rospy_mock.loginfo("Unregistered IMU subscriber from: {}".format(self.imu_topic))
        
        # Update the topic and create new subscriber
        self.imu_topic = new_topic
        self.imu_data_sub = rospy_mock.Subscriber(self.imu_topic,
                                                 sensor_msgs_mock.msg.Imu,
                                                 self.imu_data_callback)
        rospy_mock.loginfo("Reconfigured IMU subscriber to: {}".format(self.imu_topic))

    def imu_data_callback(self, data):
        """Callback for IMU data."""
        print(f"[CALLBACK] Received IMU data on topic: {self.imu_topic}")


def demonstrate_imu_topic_configuration():
    """Demonstrate all IMU topic configuration methods."""
    
    print("=" * 60)
    print("MAVROS IMU Topic Configuration Demonstration")
    print("=" * 60)
    print()
    
    # Method 1: Default configuration
    print("1. DEFAULT CONFIGURATION:")
    print("-" * 30)
    test1 = DemoMavrosTestCommon()
    test1.setUp()
    print()
    
    # Method 2: Programmatic configuration
    print("2. PROGRAMMATIC CONFIGURATION:")
    print("-" * 30)
    test2 = DemoMavrosTestCommon()
    test2.set_imu_topic('custom_sensors/imu/data')
    test2.setUp()
    print()
    
    # Method 3: Runtime reconfiguration
    print("3. RUNTIME RECONFIGURATION:")
    print("-" * 30)
    test3 = DemoMavrosTestCommon()
    test3.setUp()
    print("  Switching to backup IMU...")
    test3.reconfigure_imu_subscriber('backup_imu/data')
    print("  Switching to high-precision IMU...")
    test3.reconfigure_imu_subscriber('precision_imu/data')
    print()
    
    # Method 4: Inheritance-based configuration
    print("4. INHERITANCE-BASED CONFIGURATION:")
    print("-" * 30)
    
    class CustomIMUTest(DemoMavrosTestCommon):
        def __init__(self):
            super().__init__()
            self.imu_topic = 'alternative_sensors/imu/data'
    
    test4 = CustomIMUTest()
    test4.setUp()
    print()
    
    print("=" * 60)
    print("CONFIGURATION SUMMARY")
    print("=" * 60)
    print()
    print("Available configuration methods:")
    print("• Default: Uses 'mavros/imu/data'")
    print("• ROS Parameter: Set '~imu_topic' parameter")
    print("• Programmatic: Call set_imu_topic() before setUp()")
    print("• Runtime: Call reconfigure_imu_subscriber() anytime")
    print("• Inheritance: Override imu_topic in constructor")
    print()
    print("Use cases:")
    print("• Testing with simulated IMU data")
    print("• Switching between hardware/software IMU")
    print("• Multi-IMU system validation")
    print("• Backup IMU failover testing")
    print()
    print("All changes maintain backward compatibility!")
    print("Existing tests continue to work without modification.")


if __name__ == '__main__':
    demonstrate_imu_topic_configuration()