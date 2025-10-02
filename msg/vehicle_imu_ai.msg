# AI-processed IMU readings in SI units.

uint64 timestamp                # Time since system start (microseconds)
uint64 timestamp_sample         # Sample timestamp (microseconds)

uint32 accel_device_id          # Unique accelerometer device ID (persistent across power cycles)
uint32 gyro_device_id           # Unique gyroscope device ID (persistent across power cycles)

float32[3] delta_angle          # Delta angle [rad] about FRD body frame XYZ over integration period
float32[3] delta_velocity       # Delta velocity [m/s] in FRD body frame XYZ over integration period

uint16 delta_angle_dt           # Delta angle integration period (microseconds)
uint16 delta_velocity_dt        # Delta velocity integration period (microseconds)

# Accelerometer clipping bitfield
uint8 CLIPPING_X = 1
uint8 CLIPPING_Y = 2
uint8 CLIPPING_Z = 4

uint8 delta_angle_clipping
uint8 delta_velocity_clipping   # Bitfield: accelerometer clipping per axis during integration

uint8 accel_calibration_count   # Increments when accelerometer calibration changes
uint8 gyro_calibration_count    # Increments when gyroscope calibration changes
