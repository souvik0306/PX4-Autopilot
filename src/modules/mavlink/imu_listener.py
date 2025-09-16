#!/usr/bin/env python3
import rospy
import numpy as np
import onnxruntime as ort
import pickle
import os
from sensor_msgs.msg import Imu
import sys

try:
    from mavros_msgs.msg import HilSensor
except ImportError:  # pragma: no cover - dependency optional for tests
    HilSensor = None

# --- configuration ---
SEQLEN   = 50        # Number of IMU samples per inference window
INTERVAL = 9          # Interval between inference windows
OVERLAP  = INTERVAL + 1  # Number of samples kept between windows for overlap

DEFAULT_IMU_TOPIC = "/mavros/imu/data"
DEFAULT_CORRECTED_TOPIC = "/corrected_imu"
DEFAULT_HIL_TOPIC = "/mavros/hil_sensor/imu"
DEFAULT_ONNX_PATH = "/path/to/model.onnx"
DEFAULT_RESULTS_PATH = "/tmp/imu_results.pkl"

# HIL_SENSOR field bitmasks from mavlink_receiver.h
HIL_SENSOR_ACCEL = 0b111
HIL_SENSOR_GYRO  = 0b111000
HIL_SENSOR_FIELDS = HIL_SENSOR_ACCEL | HIL_SENSOR_GYRO

class IMUBuffer:
    """Buffer for storing IMU data and managing inference windows."""
    def __init__(self, seqlen, overlap):
        self.max_size = seqlen * 2
        self.seqlen = seqlen
        self.overlap = overlap
        self.time_buf = np.zeros(self.max_size, dtype=np.float32)
        self.acc_buf  = np.zeros((self.max_size, 3), dtype=np.float32)
        self.gyro_buf = np.zeros((self.max_size, 3), dtype=np.float32)
        self.buf_idx = 0

    def add(self, msg: Imu):
        self.time_buf[self.buf_idx] = msg.header.stamp.to_sec()
        self.acc_buf[self.buf_idx]  = [msg.linear_acceleration.x,
                                       msg.linear_acceleration.y,
                                       msg.linear_acceleration.z]
        self.gyro_buf[self.buf_idx] = [msg.angular_velocity.x,
                                       msg.angular_velocity.y,
                                       msg.angular_velocity.z]
        self.buf_idx += 1

    def ready(self):
        """Check if enough samples are collected for inference."""
        return self.buf_idx >= self.seqlen

    def get_window(self):
        """Get the current window of buffered IMU data."""
        return (self.time_buf[:self.buf_idx],
                self.acc_buf[:self.buf_idx],
                self.gyro_buf[:self.buf_idx])

    def slide_window(self):
        """Keep only the last overlap samples after inference."""
        # Keep last overlap samples
        self.time_buf[:self.overlap] = self.time_buf[self.buf_idx - self.overlap : self.buf_idx]
        self.acc_buf[:self.overlap]  = self.acc_buf[self.buf_idx - self.overlap : self.buf_idx]
        self.gyro_buf[:self.overlap] = self.gyro_buf[self.buf_idx - self.overlap : self.buf_idx]
        self.buf_idx = self.overlap

class CorrectedIMUPublisher:
    """Publishes corrected IMU messages and forwards them as HIL_SENSOR."""

    def __init__(self, topic_name="/corrected_imu", hil_topic="/mavros/hil_sensor/imu"):
        self.pub = rospy.Publisher(topic_name, Imu, queue_size=100)
        self.hil_pub = rospy.Publisher(hil_topic, HilSensor, queue_size=10) if HilSensor else None

    def publish(self, original_msg: Imu, corrected_acc, corrected_gyro):
        """Publish corrected IMU data while preserving original header."""
        imu_msg = Imu()

        # Propagate original timestamp and frame
        imu_msg.header.stamp = original_msg.header.stamp
        imu_msg.header.frame_id = original_msg.header.frame_id

        # Corrected acceleration
        imu_msg.linear_acceleration.x = float(corrected_acc[0])
        imu_msg.linear_acceleration.y = float(corrected_acc[1])
        imu_msg.linear_acceleration.z = float(corrected_acc[2])

        # Corrected gyroscope
        imu_msg.angular_velocity.x = float(corrected_gyro[0])
        imu_msg.angular_velocity.y = float(corrected_gyro[1])
        imu_msg.angular_velocity.z = float(corrected_gyro[2])

        # Orientation not provided
        imu_msg.orientation_covariance[0] = -1

        # Example covariance values for corrected data
        imu_msg.angular_velocity_covariance = [0.001, 0.0, 0.0,
                                               0.0, 0.001, 0.0,
                                               0.0, 0.0, 0.001]
        imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0,
                                                   0.0, 0.01, 0.0,
                                                   0.0, 0.0, 0.01]

        self.pub.publish(imu_msg)

        if self.hil_pub is not None:
            hil_msg = HilSensor()
            hil_msg.header = imu_msg.header
            hil_msg.xacc = imu_msg.linear_acceleration.x
            hil_msg.yacc = imu_msg.linear_acceleration.y
            hil_msg.zacc = imu_msg.linear_acceleration.z
            hil_msg.xgyro = imu_msg.angular_velocity.x
            hil_msg.ygyro = imu_msg.angular_velocity.y
            hil_msg.zgyro = imu_msg.angular_velocity.z
            hil_msg.fields_updated = HIL_SENSOR_FIELDS
            self.hil_pub.publish(hil_msg)

class IMUInferenceNode:
    """Main node for IMU inference and publishing corrected data."""

    def __init__(self):
        self.buffer = IMUBuffer(SEQLEN, OVERLAP)
        self.results = []
        self.correction_counter = 0
        self.onnx_model = None
        self.corrected_imu_pub = None
        self.onnx_path = ""
        self.pickle_path = ""
        self.last_msg = None

    def check_files(self):
        if not os.path.isfile(self.onnx_path):
            rospy.logerr(f"ONNX model file not found: {self.onnx_path}")
            return False
        return True

    def load_model(self):
        """Load the ONNX model for inference."""
        try:
            session_options = ort.SessionOptions()
            session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            session_options.intra_op_num_threads = os.cpu_count()
            session_options.execution_mode = ort.ExecutionMode.ORT_PARALLEL
            self.onnx_model = ort.InferenceSession(
                self.onnx_path, sess_options=session_options, providers=["CPUExecutionProvider"]
            )
            rospy.loginfo(f"[READY] Model loaded at ROS time: {rospy.Time.now().to_sec():.2f}")
        except Exception as e:
            rospy.logerr(f"Failed to load ONNX model: {e}")
            rospy.signal_shutdown("Fatal error: Model loading failed.")

    def save_results(self):
        """Save inference results to a pickle file."""
        try:
            os.makedirs(os.path.dirname(self.pickle_path), exist_ok=True)
            with open(self.pickle_path, "wb") as f:
                pickle.dump(self.results, f, protocol=pickle.HIGHEST_PROTOCOL)
        except Exception as e:
            rospy.logerr(f"Failed to save results: {e}")

    def run_inference(self):
        try:
            time, acc, gyro = self.buffer.get_window()
            dt = np.diff(time)[..., None]
            acc = acc[:-1]
            gyro = gyro[:-1]
            acc_b  = acc[None, ...]
            gyro_b = gyro[None, ...]
            corr_acc, corr_gyro = self.onnx_model.run(None, {"acc": acc_b, "gyro": gyro_b})

            start = OVERLAP - 1
            corrected_acc  = acc_b[:, start:, :]  + corr_acc
            corrected_gyro = gyro_b[:, start:, :] + corr_gyro
            dt_trim        = dt[start:, :]

            self.correction_counter += 1
            rospy.loginfo(f"[Correction #{self.correction_counter}]")
            rospy.loginfo(f"Corrected accel: {corrected_acc[0, -1]}")
            rospy.loginfo(f"Corrected gyro:  {corrected_gyro[0, -1]}")
            rospy.loginfo("-----------------------")

            # Publish the last corrected IMU message
            if self.last_msg is not None:
                self.corrected_imu_pub.publish(
                    self.last_msg, corrected_acc[0, -1], corrected_gyro[0, -1]
                )

            self.results.append({
                "correction_acc":  corr_acc[0],
                "correction_gyro": corr_gyro[0],
                "corrected_acc":   corrected_acc[0],
                "corrected_gyro":  corrected_gyro[0],
                "dt":              dt_trim,
            })

            self.save_results()
        except Exception as e:
            rospy.logerr(f"Inference error: {e}")

    def imu_callback(self, msg: Imu):
        self.buffer.add(msg)
        self.last_msg = msg
        if self.buffer.ready():
            self.run_inference()
            self.buffer.slide_window()

    def start(self):
        rospy.init_node("imu_inference_node")
        rospy.loginfo(f"[INIT] Node started at ROS time: {rospy.Time.now().to_sec():.2f}")
        rospy.loginfo(f"[INFO] Python version: {sys.version}")

        self.onnx_path = rospy.get_param(
            "~onnx_path",
            rospy.get_param("~onnx_model_path", DEFAULT_ONNX_PATH),
        )
        self.pickle_path = rospy.get_param("~results_path", DEFAULT_RESULTS_PATH)
        imu_topic = rospy.get_param("~imu_topic", DEFAULT_IMU_TOPIC)
        corrected_topic = rospy.get_param("~corrected_topic", DEFAULT_CORRECTED_TOPIC)
        hil_topic = rospy.get_param("~hil_topic", DEFAULT_HIL_TOPIC)
        self.corrected_imu_pub = CorrectedIMUPublisher(corrected_topic, hil_topic)

        if not self.check_files():
            rospy.signal_shutdown("Required files missing.")
            return

        self.load_model()
        rospy.Subscriber(imu_topic, Imu, self.imu_callback, queue_size=1000)
        rospy.spin()

if __name__ == "__main__":
    node = IMUInferenceNode()
    node.start()
