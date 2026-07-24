#!/usr/bin/env python3
"""
imu_combiner_node.py
Объединяет /camera/accel/sample и /camera/gyro/sample
в единый /camera/imu (sensor_msgs/msg/Imu).
Gyro ~200 Hz задаёт ритм публикации; accel кешируется.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu

GYRO_FREQ = 200  # Hz — ритм публикации


class ImuCombiner(Node):
    def __init__(self):
        super().__init__('imu_combiner')

        # ---------- параметры ----------
        self.declare_parameter('accel_topic', '/camera/accel/sample')
        self.declare_parameter('gyro_topic',  '/camera/gyro/sample')
        self.declare_parameter('imu_topic',   '/camera/imu')
        self.declare_parameter('frame_id',    'camera_imu_optical_frame')

        accel_topic = self.get_parameter('accel_topic').value
        gyro_topic  = self.get_parameter('gyro_topic').value
        imu_topic   = self.get_parameter('imu_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        # ---------- QoS — лучший sensor profile ----------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------- кеш последнего accel ----------
        self._accel: Imu | None = None

        # ---------- подписки ----------
        self.create_subscription(Imu, accel_topic, self._accel_cb, sensor_qos)
        self.create_subscription(Imu, gyro_topic,  self._gyro_cb,  sensor_qos)

        # ---------- публикатор ----------
        self._pub = self.create_publisher(Imu, imu_topic, sensor_qos)

        self.get_logger().info(
            f'ImuCombiner: {accel_topic} + {gyro_topic} → {imu_topic}'
        )

    # ------------------------------------------------------------------
    def _accel_cb(self, msg: Imu) -> None:
        self._accel = msg  # просто кешируем

    def _gyro_cb(self, msg: Imu) -> None:
        if self._accel is None:
            return  # ждём первый accel

        out = Imu()
        out.header.stamp    = msg.header.stamp   # timestamp gyro (выше частота)
        out.header.frame_id = 'camera_link'

        # angular velocity — из gyro
        out.angular_velocity       = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        # linear acceleration — из последнего accel
        out.linear_acceleration       = self._accel.linear_acceleration
        out.linear_acceleration_covariance = self._accel.linear_acceleration_covariance

        # orientation неизвестна — стандартное соглашение ROS: covariance[0] = -1
        out.orientation_covariance[0] = -1.0

        self._pub.publish(out)


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ImuCombiner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()