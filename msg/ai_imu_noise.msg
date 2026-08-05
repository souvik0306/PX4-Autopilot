# AI-predicted IMU noise received via MAVLink.
# Published as a uORB topic so any PX4 module can subscribe and use
# the noise values (e.g. EKF2 tuning, logging, custom estimators).
#
# Axes follow the FRD (Forward-Right-Down) body frame convention,
# consistent with all other PX4 IMU topics.

uint64 timestamp                # time since system start (microseconds)

float32[3] ai_acc_noise          # AI-predicted accelerometer noise XYZ (m/s^2)^2  [0]=X [1]=Y [2]=Z
float32[3] ai_gyro_noise         # AI-predicted gyroscope noise XYZ    (rad/s)^2   [0]=X [1]=Y [2]=Z
