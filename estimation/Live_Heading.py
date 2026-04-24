import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion

# --- Calibration disabled for now ---
# from Calibration import calibrate_magnetometer, apply_calibration

from Heading_estimate import HeadingEstimator


def _safe_unit(v: np.ndarray, eps: float = 1e-12):
    if v is None:
        return None
    if not np.all(np.isfinite(v)):
        return None
    n = float(np.linalg.norm(v))
    if n < eps:
        return None
    return v / n


class RosHeadingNode(Node):
    def __init__(self):
        super().__init__("ros_heading_node")

        self.imu_topic = self.declare_parameter("imu_topic", "/imu/data_raw").value
        self.mag_topic = self.declare_parameter("mag_topic", "/mag/data_raw").value

        self.heading_topic = self.declare_parameter("heading_topic", "/heading_deg").value
        self.quat_topic = self.declare_parameter("quat_topic", "/heading_quat").value

        smooth_factor = float(self.declare_parameter("smooth_factor", 0.95).value)
        self.estimator = HeadingEstimator(smooth_factor=smooth_factor)

        # --- Calibration disabled ---
        # self.bias = None
        # self.C_total = None
        # self._load_calibration()

        self.latest_gyro = None
        self.latest_acc = None
        self.latest_mag = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.on_imu, qos)
        self.sub_mag = self.create_subscription(MagneticField, self.mag_topic, self.on_mag, qos)

        self.pub_heading = self.create_publisher(Float32, self.heading_topic, 10)
        self.pub_quat = self.create_publisher(Quaternion, self.quat_topic, 10)

        self.timer = self.create_timer(0.2, self.compute_heading)

        self.get_logger().info(
            f"Running WITHOUT magnetometer calibration | "
            f"IMU={self.imu_topic} MAG={self.mag_topic}"
        )

    # --- Entire calibration loader removed ---
    # def _load_calibration(self):
    #     pass

    def on_imu(self, msg: Imu):
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Match your Test_heading axis mapping
        self.latest_gyro = np.array([gz, gx, gy], dtype=float)
        self.latest_acc = np.array([az, ax, ay], dtype=float)

    def on_mag(self, msg: MagneticField):
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        mz = msg.magnetic_field.z

        mag = np.array([mz, mx, my], dtype=float)  

        # Match Test_heading axis mapping
        # mag = mag[[2, 0, 1]]

        # --- Calibration disabled ---
        # if self.bias is not None and self.C_total is not None:
        #     mag = apply_calibration(mag, self.bias, self.C_total)

        self.latest_mag = mag

    def compute_heading(self):
        #if self.latest_gyro is None or self.latest_acc is None or self.latest_mag is None:
        #    return

        #acc_u = _safe_unit(self.latest_acc)
        #mag_u = _safe_unit(self.latest_mag)

        #if acc_u is None or mag_u is None:
        #    return

        heading_deg = self.estimator.update(self.latest_gyro, self.latest_acc, self.latest_mag)

        # Publish heading
        hmsg = Float32()
        hmsg.data = float(heading_deg)
        self.pub_heading.publish(hmsg)

        # Publish quaternion
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