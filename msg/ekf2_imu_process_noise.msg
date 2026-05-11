# EKF2 IMU process noise variances used during covariance prediction.

uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # IMU sample timestamp (microseconds)

float32[3] delta_ang_var        # [daxVar, dayVar, dazVar] (rad^2)
float32[3] delta_vel_var        # [dvxVar, dvyVar, dvzVar] ((m/s)^2)
