# PX4 AI IMU Integration Guide

This guide explains how to enable PX4 to send IMU data to an AI model, process it, and pass it back to EKF2.

## 🎯 Overview

The system allows PX4 to switch between raw IMU data and AI-processed IMU data using the `EKF2_IMU_SRC` parameter:

- `EKF2_IMU_SRC = 0`: Use raw `vehicle_imu` data (default)
- `EKF2_IMU_SRC = 1`: Use AI-processed `vehicle_imu_ai` data

## 🔧 System Architecture

```
Raw IMU Sensors → vehicle_imu → MAVROS → /mavros/imu/data
                                                      ↓
                                              Your AI Model
                                                      ↓
                                              UDP (port 14560)
                                                      ↓
                                              imu_ai_bridge
                                                      ↓
                                              vehicle_imu_ai
                                                      ↓
                                              EKF2 (when EKF2_IMU_SRC=1)
```

## 📁 Files Created

1. **`ai_imu_processor.py`** - Standalone AI IMU processor for testing
2. **`test_ai_imu_integration.py`** - Complete integration test
3. **`setup_ai_imu.sh`** - PX4 setup script
4. **`test_udp_receiver.py`** - UDP receiver for testing

## 🚀 Quick Start

### 1. Test the System (Standalone)

```bash
# Test the AI processing and UDP communication
python3 test_ai_imu_integration.py
```

This will:
- Start a UDP receiver on port 14560
- Generate test IMU data with realistic motion patterns
- Process data through AI pipeline (low-pass filtering)
- Send processed data via UDP
- Display statistics and validation

### 2. Set Up PX4 Integration

```bash
# Run the setup script
./setup_ai_imu.sh
```

This will:
- Build PX4 SITL if needed
- Create configuration files
- Set up launch scripts
- Create validation tools

### 3. Launch Complete System

```bash
# Launch all components
./launch_ai_test.sh
```

This opens three terminals:
- **Terminal 1**: PX4 SITL
- **Terminal 2**: IMU AI Bridge
- **Terminal 3**: AI IMU Processor

## 🔄 Switching Between Modes

### Enable AI Mode
```bash
# In PX4 shell
param set EKF2_IMU_SRC 1
ekf2 stop
ekf2 start
```

### Return to Raw Mode
```bash
# In PX4 shell
param set EKF2_IMU_SRC 0
ekf2 stop
ekf2 start
```

## 📊 Validation Commands

```bash
# Check if AI data is flowing
listener vehicle_imu_ai

# Compare with raw data
listener vehicle_imu

# Check EKF2 status
ekf2 status

# Monitor topic rates
uorb top
```

## 🔍 Data Format

The `vehicle_imu_ai` message contains:

```cpp
uint64 timestamp          // PX4 system time
uint64 timestamp_sample   // Sample end time
uint32 accel_device_id    // Unique device ID
uint32 gyro_device_id     // Unique device ID
float32[3] delta_angle    // Integrated gyro (rad)
float32[3] delta_velocity // Integrated accel (m/s)
uint16 delta_angle_dt     // Integration time (μs)
uint16 delta_velocity_dt  // Integration time (μs)
uint8 delta_velocity_clipping // Clipping flags
uint8 accel_calibration_count
uint8 gyro_calibration_count
```

**Total size: 55 bytes**

## ⚙️ Configuration Parameters

### AI Processor Settings
- **UDP Port**: 14560 (configurable)
- **Filter Alpha**: 0.2 (low-pass filter coefficient)
- **Sample Rate**: 200Hz (5ms intervals)
- **Min dt**: 0.5ms (minimum integration window)
- **Max dt**: 40ms (maximum integration window)

### Device IDs
- **Accel Device ID**: 0xA14ACC01
- **Gyro Device ID**: 0xA14A7701

## 🎛️ Customizing Your AI Model

### 1. Modify the AI Processing

Edit `ai_imu_processor.py` and customize the `process_imu_sample()` method:

```python
def process_imu_sample(self, sample: IMUSample):
    # Your AI processing here
    # Input: Raw IMU data (gyro, accel)
    # Output: Processed deltas (delta_angle, delta_velocity)
    
    # Example: Apply your AI model
    processed_gyro = your_ai_model.process_gyro(sample.gyro)
    processed_accel = your_ai_model.process_accel(sample.accel)
    
    # Convert to deltas and send
    # ... rest of the method
```

### 2. Integration with ROS/MAVROS

If you have ROS available, modify the processor to subscribe to MAVROS topics:

```python
import rospy
from sensor_msgs.msg import Imu

def __init__(self):
    # ... existing code ...
    rospy.Subscriber('/mavros/imu/data', Imu, self.on_imu)

def on_imu(self, imu_msg):
    # Convert ROS Imu message to IMUSample
    sample = IMUSample(
        timestamp=imu_msg.header.stamp.to_sec(),
        gyro=[imu_msg.angular_velocity.x, 
              imu_msg.angular_velocity.y, 
              imu_msg.angular_velocity.z],
        accel=[imu_msg.linear_acceleration.x,
               imu_msg.linear_acceleration.y,
               imu_msg.linear_acceleration.z]
    )
    self.process_imu_sample(sample)
```

## ⏱️ Timing Considerations

### 1. Integration Windows
- Your AI must output **delta values**, not raw rates
- `delta_angle` = integrated angular velocity over `delta_angle_dt`
- `delta_velocity` = integrated acceleration over `delta_velocity_dt`

### 2. Timestamp Management
- `timestamp_sample` must mark the **end** of the integration window
- Use PX4 time reference for consistency
- Ensure monotonic timestamps

### 3. Latency Handling
- AI processing adds latency
- Consider tuning `EKF2_IMU_DLY` parameter if needed
- Monitor EKF2 status for time offset warnings

## 🐛 Troubleshooting

### Common Issues

1. **No data flowing**
   - Check if `imu_ai_bridge` is running
   - Verify UDP port 14560 is not blocked
   - Check `listener vehicle_imu_ai`

2. **EKF2 not using AI data**
   - Verify `EKF2_IMU_SRC = 1`
   - Restart EKF2 after parameter change
   - Check `ekf2 status`

3. **Timing issues**
   - Monitor integration windows (`dt` values)
   - Check for dropped samples
   - Verify sample rate consistency

4. **Data format errors**
   - Verify message size (55 bytes)
   - Check struct packing format
   - Validate device IDs

### Debug Commands

```bash
# Check running processes
ps aux | grep -E "(ekf2|imu_ai_bridge|ai_imu_processor)"

# Monitor UDP traffic
sudo netstat -ulnp | grep 14560

# Check PX4 parameters
param show EKF2_IMU_SRC

# Monitor topic rates
uorb top | grep vehicle_imu
```

## 📈 Performance Monitoring

The system provides statistics:
- Samples processed
- Samples dropped
- Average integration time
- Message send rate
- UDP receive rate

Monitor these to ensure optimal performance.

## 🔒 Safety Considerations

1. **Fallback Mechanism**: Always keep raw IMU data available
2. **Parameter Validation**: Verify AI data quality before switching
3. **Timing Validation**: Ensure consistent sample rates
4. **Error Handling**: Implement proper error detection and recovery

## 📝 Next Steps

1. **Test with Real Data**: Use actual MAVROS IMU data
2. **Implement Your AI Model**: Replace the simple filter with your AI
3. **Tune Parameters**: Optimize for your specific use case
4. **Add Monitoring**: Implement health checks and alerts
5. **Performance Testing**: Test under various flight conditions

## 🤝 Support

If you encounter issues:
1. Check the troubleshooting section
2. Run the integration test: `python3 test_ai_imu_integration.py`
3. Verify PX4 setup: `./validate_ai_imu.sh`
4. Check logs and error messages

The system is designed to be robust and easy to debug. Most issues are related to timing or configuration rather than the core integration.