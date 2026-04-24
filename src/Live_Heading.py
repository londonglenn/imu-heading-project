import json
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion

from Calibration import apply_calibration  # optional calibration [6]
from Heading_estimate import HeadingEstimator  # heading filter [2]


def _safe_unit(v: np.ndarray, eps: float = 1e-12):
    """
    Safely normalize a vector.
    Returns:
        - v / ||v|| if finite and norm > eps
        - None otherwise
    """
    if v is None:
        return None
    if not np.all(np.isfinite(v)):
        return None
    n = float(np.linalg.norm(v))
    if n < eps:
        return None
    return v / n


class RosHeadingNode(Node):
    """
    ROS2 node that fuses IMU (gyro + accel) and magnetometer
    into a heading estimate using the HeadingEstimator.

    Expectations:
        - Gyro data (from Imu.msg) is in deg/s (sensor frame).
        - Accel and mag are in sensor frame.
        - Sensor orientation (from your description):
            - -Z_sensor points forward
            - +X_sensor points to the right
            - +Y_sensor points down
        - Desired body frame is NED:
            - X (North)  = forward
            - Y (East)   = right
            - Z (Down)   = down

        This node:
            - Converts sensor-frame IMU/MAG to NED body frame.
            - Converts gyro from deg/s to rad/s.
            - Optionally applies magnetometer calibration if
              magnetometer_calibration_vectors.json is available.
            - Publishes heading in degrees and the full orientation
              quaternion.
    """

    def __init__(self):
        super().__init__("ros_heading_node")

        # --- Parameters: topics and smoothing ---
        self.imu_topic = self.declare_parameter("imu_topic", "/imu/data_raw").value
        self.mag_topic = self.declare_parameter("mag_topic", "/mag/data_raw").value

        self.heading_topic = self.declare_parameter("heading_topic", "/heading_deg").value
        self.quat_topic = self.declare_parameter("quat_topic", "/heading_quat").value

        smooth_factor = float(self.declare_parameter("smooth_factor", 0.95).value)
        self.estimator = HeadingEstimator(smooth_factor=smooth_factor)

        # Latest NED body-frame vectors (gyro_rad, acc, mag)
        self.latest_gyro = None   # rad/s
        self.latest_acc = None    # m/s^2 (unit depends on IMU)
        self.latest_mag = None    # magnetic field units

        # Optional magnetometer calibration
        self.bias = None
        self.C_total = None
        self._load_calibration_optional()

        # QoS for IMU / MAG subscriptions
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        # Subscriptions
        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.on_imu, qos)
        self.sub_mag = self.create_subscription(MagneticField, self.mag_topic, self.on_mag, qos)

        # Publishers
        self.pub_heading = self.create_publisher(Float32, self.heading_topic, 10)
        self.pub_quat = self.create_publisher(Quaternion, self.quat_topic, 10)

        # Timer to compute heading periodically
        self.timer = self.create_timer(0.2, self.compute_heading)

        if self.bias is not None and self.C_total is not None:
            self.get_logger().info(
                f"Heading node running with MAG calibration | "
                f"IMU={self.imu_topic} MAG={self.mag_topic}"
            )
        else:
            self.get_logger().warn(
                f"Heading node running WITHOUT MAG calibration "
                f"(optional calibration file not found or invalid) | "
                f"IMU={self.imu_topic} MAG={self.mag_topic}"
            )

    # -------------------------------------------------------------------------
    # Optional calibration loader
    # -------------------------------------------------------------------------
    def _load_calibration_optional(self):
        """
        Attempt to load magnetometer calibration parameters from
        magnetometer_calibration_vectors.json.

        If the file is missing or invalid, calibration is disabled
        but the node still runs.
        """
        try:
            with open("magnetometer_calibration_vectors.json", "r") as f:
                params = json.load(f)
            bias = np.array(params["hard_iron_bias"], dtype=float)
            C_total = np.array(params["overall_calibration_matrix"], dtype=float)

            if bias.shape == (3,) and C_total.shape == (3, 3):
                self.bias = bias
                self.C_total = C_total
                self.get_logger().info("Loaded magnetometer calibration from JSON.")
            else:
                self.get_logger().warn(
                    "Calibration JSON has unexpected shapes; "
                    "disabling magnetometer calibration."
                )
                self.bias = None
                self.C_total = None
        except (FileNotFoundError, json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(
                f"Could not load magnetometer calibration JSON "
                f"(optional): {e}"
            )
            self.bias = None
            self.C_total = None

    # -------------------------------------------------------------------------
    # Sensor-frame -> NED body-frame mapping
    # -------------------------------------------------------------------------
    @staticmethod
    def sensor_to_ned(v: np.ndarray) -> np.ndarray:
        """
        Map sensor-frame vector to NED body frame.

        Sensor frame (given):
            -Z_sensor : forward
            +X_sensor : right
            +Y_sensor : down

        NED body frame:
            X_N : forward (North)
            Y_E : right   (East)
            Z_D : down    (Down)

        Mapping:
            X_N = -Z_sensor
            Y_E =  X_sensor
            Z_D =  Y_sensor
        """
        x_s, y_s, z_s = v
        x_n = -z_s  # forward
        y_e = x_s   # right
        z_d = y_s   # down
        return np.array([x_n, y_e, z_d], dtype=float)

    # -------------------------------------------------------------------------
    # IMU callback
    # -------------------------------------------------------------------------
    def on_imu(self, msg: Imu):
        """
        Handle incoming IMU messages.

        - Extract gyro (deg/s) and accel (sensor frame).
        - Convert to NED body frame.
        - Convert gyro to rad/s.
        """
        # Sensor-frame gyro (deg/s)
        gyro_sensor = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ], dtype=float)

        # Sensor-frame accel
        acc_sensor = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ], dtype=float)

        # Sensor -> NED body
        gyro_body_deg = self.sensor_to_ned(gyro_sensor)
        acc_body = self.sensor_to_ned(acc_sensor)

        # Convert gyro from deg/s to rad/s
        gyro_body_rad = np.deg2rad(gyro_body_deg)

        self.latest_gyro = gyro_body_rad
        self.latest_acc = acc_body

    # -------------------------------------------------------------------------
    # MAG callback
    # -------------------------------------------------------------------------
    def on_mag(self, msg: MagneticField):
        """
        Handle incoming magnetic field messages.

        - Extract mag (sensor frame).
        - Convert to NED body frame.
        - Optionally apply calibration if available.
        """
        mag_sensor = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z,
        ], dtype=float)

        # Sensor -> NED body
        mag_body = self.sensor_to_ned(mag_sensor)

        # Optional calibration
        if self.bias is not None and self.C_total is not None:
            mag_body = apply_calibration(mag_body, self.bias, self.C_total)

        self.latest_mag = mag_body

    # -------------------------------------------------------------------------
    # Main heading computation
    # -------------------------------------------------------------------------
    def compute_heading(self):
        """
        Periodically called to compute and publish heading and quaternion.

        - Requires that latest_gyro, latest_acc, and latest_mag are set.
        - Normalizes accel and mag to unit vectors before fusion.
        - Calls HeadingEstimator.update(gyro_rad, acc_unit, mag_unit).
        """
        if (
            self.latest_gyro is None
            or self.latest_acc is None
            or self.latest_mag is None
        ):
            # Not enough data yet
            return

        # Normalize accel and mag; skip if invalid/zero
        acc_u = _safe_unit(self.latest_acc)
        mag_u = _safe_unit(self.latest_mag)

        if acc_u is None or mag_u is None:
            return

        # HeadingEstimator is assumed to take gyro in rad/s
        heading_deg = self.estimator.update(self.latest_gyro, acc_u, mag_u)

        # Publish heading (deg)
        hmsg = Float32()
        hmsg.data = float(heading_deg)
        self.pub_heading.publish(hmsg)

        # Publish orientation quaternion
        q = self.estimator.q
        if q is not None and q.shape == (4,):
            qmsg = Quaternion()
            qmsg.w = float(q[0])
            qmsg.x = float(q[1])
            qmsg.y = float(q[2])
            qmsg.z = float(q[3])
            self.pub_quat.publish(qmsg)


def main():
    rclpy.init()
    node = RosHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()