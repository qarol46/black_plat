import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np
import struct
import math


class PointSelector(Node):
    def __init__(self):
        super().__init__('point_selector')

        # -------------------- Параметры --------------------
        self.front_distance = 2.0
        self.back_distance = 2.0
        self.side_distance = 0.3
        self.max_height = 3.0

        self.num_bins = 60
        self.num_points_per_bin = 1

        # Квантование (как в исходнике) — можно выключить, если не нужно
        self.quantize_step = 0.05

        # -------------------- Топики --------------------
        # Входы
        self.odom_topic = '/diff_drive_controller/odom'
        self.cloud_topic = '/lio_sam/mapping/cloud_registered'  # уже в map

        # Выходы
        self.green_topic = '/visualized_pointcloud_around_robot'
        self.red_topic = '/projected_pointcloud_xz_contour'
        self.robot_topic = '/projected_pointcloud_xz_contour_robot'

        # -------------------- Подписки --------------------
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_callback, 10)

        # -------------------- Публикаторы --------------------
        self.green_publisher = self.create_publisher(PointCloud2, self.green_topic, 10)
        self.red_publisher = self.create_publisher(PointCloud2, self.red_topic, 10)
        self.robot_contour_publisher = self.create_publisher(PointCloud2, self.robot_topic, 10)

        # -------------------- Состояние --------------------
        self.points_map = np.empty((0, 3), dtype=np.float32)  # точки в map
        self.last_cloud_stamp = None

        self.published_once_green = False
        self.published_once_red = False
        self.published_once_blue = False

    # ============================================================
    # Вход: облако точек (уже в map)
    # ============================================================
    def cloud_callback(self, msg: PointCloud2):
        pts = []

        for x, y, z in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            if self.quantize_step and self.quantize_step > 0.0:
                x = round(x / self.quantize_step) * self.quantize_step
                y = round(y / self.quantize_step) * self.quantize_step
                z = round(z / self.quantize_step) * self.quantize_step
            pts.append([x, y, z])

        if len(pts) == 0:
            self.points_map = np.empty((0, 3), dtype=np.float32)
            return

        arr = np.array(pts, dtype=np.float32)
        # Удаление дублей (как было в исходнике)
        self.points_map = np.unique(arr, axis=0)
        self.last_cloud_stamp = msg.header.stamp

    # ============================================================
    # Вход: одометрия
    # ============================================================
    def odom_callback(self, msg: Odometry):
        if self.points_map.size == 0:
            return

        # Положение робота (в той же системе, что и cloud_registered: map)
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        robot_z = msg.pose.pose.position.z

        # yaw робота
        yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

        # 1) фильтруем точки вокруг робота (область в робото-координатах, точки остаются в map)
        filtered_points_map = self.filter_points_around_robot(robot_x, robot_y, robot_z, yaw)

        # 2) публикуем зелёные (в map)
        self.publish_green_pointcloud(filtered_points_map)

        # 3) извлекаем контур (в map и в robot frame)
        contour_map, contour_robot = self.extract_top_points_in_bins(filtered_points_map, robot_x, robot_y, yaw)

        # 4) публикуем красный (в map) и синий (в base_link)
        self.publish_red_pointcloud(contour_map)
        self.publish_robot_contour_pointcloud(contour_robot)

    # ============================================================
    # Фильтрация точек вокруг робота (вход/выход: map coords)
    # ============================================================
    def filter_points_around_robot(self, robot_x, robot_y, robot_z, yaw):
        points = self.points_map

        # Фильтрация по высоте
        points = points[points[:, 2] <= robot_z + self.max_height]
        if points.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        # В робото-координаты (для маски прямоугольника)
        dx = points[:, 0] - robot_x
        dy = points[:, 1] - robot_y

        cos_y = np.cos(-yaw)
        sin_y = np.sin(-yaw)

        x_r = dx * cos_y - dy * sin_y
        y_r = dx * sin_y + dy * cos_y

        mask = (
            (-self.back_distance <= x_r) &
            (x_r <= self.front_distance) &
            (-self.side_distance <= y_r) &
            (y_r <= self.side_distance)
        )

        return points[mask]

    # ============================================================
    # Контур: проекция XZ (относительно робота), бины по X, max Z
    # Возврат: (map_points, robot_points) — robot_points в base_link
    # ============================================================
    def extract_top_points_in_bins(self, points_map, robot_x, robot_y, yaw):
        if points_map.size == 0:
            return (np.empty((0, 3), dtype=np.float32),
                    np.empty((0, 3), dtype=np.float32))

        # Проекция в робото-координаты на XZ (y=0)
        projected_robot = self.project_points_xz_relative_to_robot(points_map, robot_x, robot_y, yaw)
        if projected_robot.size == 0:
            return (np.empty((0, 3), dtype=np.float32),
                    np.empty((0, 3), dtype=np.float32))

        min_x = np.min(projected_robot[:, 0])
        max_x = np.max(projected_robot[:, 0])

        # если вдруг все x одинаковые — не падаем
        if abs(max_x - min_x) < 1e-9:
            bins = np.array([min_x, max_x + 1e-6], dtype=np.float32)
            num_bins = 1
        else:
            bins = np.linspace(min_x, max_x, self.num_bins + 1, dtype=np.float32)
            num_bins = self.num_bins

        contour_map_points = []
        contour_robot_points = []

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        for i in range(num_bins):
            bin_mask = (projected_robot[:, 0] >= bins[i]) & (projected_robot[:, 0] < bins[i + 1])
            bin_points = projected_robot[bin_mask]

            if len(bin_points) == 0:
                continue

            # берём точки с максимальным Z
            top_pts = bin_points[np.argsort(bin_points[:, 2])][-self.num_points_per_bin:]

            # в map (для красной публикации)
            for xr, yr, z in top_pts:
                # yr у нас всегда 0.0, но оставляем формулу общей
                map_x = robot_x + xr * cos_yaw - yr * sin_yaw
                map_y = robot_y + xr * sin_yaw + yr * cos_yaw
                contour_map_points.append([map_x, map_y, z])

            # в base_link (для синей публикации)
            contour_robot_points.extend(top_pts.tolist())

        return (np.array(contour_map_points, dtype=np.float32),
                np.array(contour_robot_points, dtype=np.float32))

    def project_points_xz_relative_to_robot(self, points_map, robot_x, robot_y, yaw):
        # map -> base_link (вокруг Z), затем y=0
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        dx = points_map[:, 0] - robot_x
        dy = points_map[:, 1] - robot_y

        rel_x = dx * cos_yaw + dy * sin_yaw
        rel_z = points_map[:, 2]

        rel_y = np.zeros_like(rel_x)
        return np.stack([rel_x, rel_y, rel_z], axis=1).astype(np.float32)

    # ============================================================
    # Публикация облаков
    # ============================================================
    def publish_green_pointcloud(self, points):
        self._publish_colored(points, frame_id='map', rgb_color=(0, 255, 0), publisher=self.green_publisher)
        if (not self.published_once_green) and points.size != 0:
            self.get_logger().info(f'Published {len(points)} green points (map).')
            self.published_once_green = True

    def publish_red_pointcloud(self, points):
        self._publish_colored(points, frame_id='map', rgb_color=(255, 0, 0), publisher=self.red_publisher)
        if (not self.published_once_red) and points.size != 0:
            self.get_logger().info(f'Published {len(points)} red contour points (map).')
            self.published_once_red = True

    def publish_robot_contour_pointcloud(self, points):
        # ВАЖНО: это точки в системе робота (base_link)
        self._publish_colored(points, frame_id='base_link', rgb_color=(0, 0, 255), publisher=self.robot_contour_publisher)
        if (not self.published_once_blue) and points.size != 0:
            self.get_logger().info(f'Published {len(points)} blue contour points (base_link).')
            self.published_once_blue = True

    def _publish_colored(self, points, frame_id, rgb_color, publisher):
        if points.size == 0:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        r, g, b = rgb_color
        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]

        colored_points = [[float(x), float(y), float(z), rgb] for x, y, z in points]

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, colored_points)
        publisher.publish(cloud_msg)

    # ============================================================
    # yaw из кватерниона
    # ============================================================
    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = PointSelector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
