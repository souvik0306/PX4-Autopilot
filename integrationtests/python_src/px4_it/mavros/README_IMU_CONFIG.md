# MAVROS IMU Topic Configuration

This guide explains how to configure the MAVROS IMU data source to use different ROS topics.

## Overview

The `MavrosTestCommon` class has been enhanced to support configurable IMU topic sources. By default, it subscribes to `mavros/imu/data`, but you can now configure it to use any ROS topic that publishes `sensor_msgs/Imu` messages.

## How IMU Data is Processed

The MAVROS IMU data flow works as follows:

1. **Topic Subscription**: The test framework subscribes to an IMU topic (default: `mavros/imu/data`)
2. **Data Callback**: When IMU data is received, it's stored in `self.imu_data`
3. **Data Usage**: Tests can access IMU data fields like:
   - `self.imu_data.angular_velocity.x/y/z` - Angular velocity
   - `self.imu_data.orientation.x/y/z/w` - Orientation quaternion
   - `self.imu_data.linear_acceleration.x/y/z` - Linear acceleration

## Configuration Methods

### Method 1: ROS Parameter Configuration

Set the IMU topic using a ROS parameter:

```bash
# Set parameter before running tests
rosparam set /test_node/imu_topic "custom_sensors/imu/data"

# Or in a launch file
<param name="~imu_topic" value="custom_sensors/imu/data" />
```

### Method 2: Programmatic Configuration

Configure the topic in your test code:

```python
class MyTest(MavrosTestCommon):
    def test_with_custom_imu(self):
        # Set topic before setUp()
        self.set_imu_topic('custom_sensors/imu/data')
        self.setUp()
        # Test proceeds with custom topic
```

### Method 3: Runtime Reconfiguration

Change the IMU topic while tests are running:

```python
class MyTest(MavrosTestCommon):
    def test_multiple_imu_sources(self):
        self.setUp()  # Uses default topic
        
        # Switch to different IMU source
        self.reconfigure_imu_subscriber('backup_imu/data')
        
        # Test with backup IMU
        # ... test logic ...
        
        # Switch back if needed
        self.reconfigure_imu_subscriber('mavros/imu/data')
```

### Method 4: Inheritance-based Configuration

Override the default topic in a subclass:

```python
class CustomIMUTest(MavrosTestCommon):
    def __init__(self, *args):
        super(CustomIMUTest, self).__init__(*args)
        self.imu_topic = 'alternative_sensors/imu/data'
```

## Use Cases

### 1. Testing with Simulated IMU Data

```python
# Publish simulated IMU data to a custom topic
# Configure tests to use the simulated data
self.set_imu_topic('simulation/imu/data')
```

### 2. Switching Between Hardware and Software IMU

```python
# Start with hardware IMU
self.setUp()  # Uses mavros/imu/data

# Switch to software/backup IMU during test
self.reconfigure_imu_subscriber('software_imu/data')
```

### 3. Multi-IMU Systems

```python
# Test with primary IMU
self.set_imu_topic('imu_primary/data')
self.setUp()
# ... run tests ...

# Test with secondary IMU
self.reconfigure_imu_subscriber('imu_secondary/data')
# ... run tests ...
```

## Example Usage

See `imu_topic_config_example.py` for complete working examples of all configuration methods.

## Topic Requirements

The alternative IMU topic must:
- Publish `sensor_msgs/Imu` messages
- Have the same coordinate frame as expected by your tests
- Provide data at a reasonable frequency for test timing requirements

## Testing Your Configuration

To verify your IMU topic configuration:

1. Check the topic is publishing: `rostopic hz your_imu_topic`
2. Verify message format: `rostopic echo your_imu_topic`
3. Run a simple test to ensure data reception:

```python
def test_imu_data_reception(self):
    self.set_imu_topic('your_custom/imu/topic')
    self.setUp()
    self.wait_for_topics(10)  # Wait up to 10 seconds
    # If this passes, your topic is working
```

## Backwards Compatibility

All existing tests continue to work without modification. The default behavior remains unchanged - tests will subscribe to `mavros/imu/data` unless explicitly configured otherwise.