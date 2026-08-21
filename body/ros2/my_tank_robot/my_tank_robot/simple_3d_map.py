#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class MapBuilderNode(Node):
    def __init__(self):
        super().__init__('map_builder_node')

        # Параметры
        self.declare_parameter('record_frequency', 1.0)  # Гц (частота записи в файл)
        self.declare_parameter('file_name', 'points3.txt')
        self.declare_parameter('clear_after_save', False)

        self.record_frequency = self.get_parameter('record_frequency').value
        self.file_name = self.get_parameter('file_name').value
        self.clear_after_save = self.get_parameter('clear_after_save').value

        # Подписка на одометрию
        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )

        # Подписка на лидар
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.lidar_callback,
            10
        )

        # Текущая поза робота (x, y, theta), упрощённо.
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0  # угол вокруг Z

        # Список для накопленных *уже перенесённых* в /odom координат + смещение
        self.accumulated_points = []

        # Таймер записи в файл (периодический вызов save_points)
        if self.record_frequency > 0.0:
            period = 1.0 / self.record_frequency
            self.timer = self.create_timer(period, self.save_points)
        else:
            self.timer = None
            self.get_logger().warn("record_frequency <= 0. Отключена периодическая запись.")

        # Смещение, которое нужно добавить к каждой точке (x, y, z)
        self.offset_x = 0.154
        self.offset_y = -0.001
        self.offset_z = 0.251

        self.get_logger().info(
            f'Запущен {self.get_name()}.\n'
            f'Частота записи: {self.record_frequency} Гц. Файл: {self.file_name}\n'
            'Подписываемся на /velodyne_points (PointCloud2) и /odom (Odometry).\n'
            f'Смещение каждой точки: ({self.offset_x}, {self.offset_y}, {self.offset_z})'
        )

    def odom_callback(self, odom_msg: Odometry):
        """
        Сохраняем текущую позу (x, y, yaw) робота из одометрии.
        Упрощённо берём углы Эйлера из quaternions, используем только yaw (Z).
        """
        pose = odom_msg.pose.pose
        # Извлекаем x, y
        self.robot_x = pose.position.x
        self.robot_y = pose.position.y

        # Для yaw возьмём quaternion → yaw
        q = pose.orientation
        self.robot_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def lidar_callback(self, cloud_msg: PointCloud2):
        """
        При поступлении лидарных данных:
        1) Берём (lx, ly, lz) в системе лидара (base_link, предположительно),
        2) Переносим в /odom (учитывая позицию и ориентацию робота),
        3) Применяем дополнительное смещение (offset_x, offset_y, offset_z),
        4) Сохраняем результат в self.accumulated_points.
        """
        for (lx, ly, lz) in pc2.read_points(
            cloud_msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):
            # Упрощённое преобразование в /odom: 2D-поворот и сдвиг
            gx, gy = transform_to_odom_2d(lx, ly, self.robot_x, self.robot_y, self.robot_yaw)
            gz = lz  # высоту берём как есть

            # Применяем дополнительное смещение
            shifted_x = gx + self.offset_x
            shifted_y = gy + self.offset_y
            shifted_z = gz + self.offset_z

            self.accumulated_points.append((shifted_x, shifted_y, shifted_z))

    def save_points(self):
        """
        Периодический колбэк для записи накопленных точек в текстовый файл (append).
        """
        if not self.accumulated_points:
            self.get_logger().debug("Нет новых точек для записи.")
            return

        try:
            with open(self.file_name, 'a') as f:
                for (x, y, z) in self.accumulated_points:
                    f.write(f'{x:.4f} {y:.4f} {z:.4f}\n')

            self.get_logger().info(
                f'Сохранены {len(self.accumulated_points)} точек в {self.file_name}'
            )

            if self.clear_after_save:
                self.accumulated_points.clear()
        except Exception as ex:
            self.get_logger().error(f"Ошибка при записи в файл: {ex}")

def euler_from_quaternion(qx, qy, qz, qw):
    """
    Простейшая функция для извлечения yaw (Z-угол) из кватерниона.
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def transform_to_odom_2d(lx, ly, rx, ry, r_yaw):
    """
    Упрощённая 2D-преобразование:
    (lx, ly) — точка в локальной системе (лидара),
    (rx, ry) — положение робота в /odom,
    r_yaw — ориентация робота вокруг оси Z.
    Возвращаем (gx, gy) — координаты в /odom.
    """
    cos_yaw = math.cos(r_yaw)
    sin_yaw = math.sin(r_yaw)
    gx = rx + (lx * cos_yaw - ly * sin_yaw)
    gy = ry + (lx * sin_yaw + ly * cos_yaw)
    return gx, gy

def main(args=None):
    rclpy.init(args=args)
    node = MapBuilderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Остановка по Ctrl+C')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
