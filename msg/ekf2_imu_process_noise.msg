# EKF2 IMU process noise variances used during covariance prediction.

uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # IMU sample timestamp (microseconds)

float32[3] gyro_var_uorb        # [gyro_var_uorb(0), gyro_var_uorb(1), gyro_var_uorb(2)] ((rad/s)**2)
float32[3] accel_var_uorb        # [accel_var_uorb(0), accel_var_uorb(1), accel_var_uorb(2)] ((m/s^2)**2)
