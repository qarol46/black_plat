#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

# Типы сообщений
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class LidarRecordAndConstantPublishNode(Node):
    def __init__(self):
        super().__init__('lidar_record_and_constant_publish_node')

        # Параметры диапазонов фильтрации
        self.x_min, self.x_max = 0.0, 2.0
        self.y_min, self.y_max = -0.50, 0.50
        self.z_min, self.z_max = -2.0, 2.0

        # Список для накопления точек (после фильтра)
        self.accumulated_points = []

        # Флаг, показывающий, принимаем ли мы ещё данные
        self.recording_active = True

        # 1) Подписка на лидар
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.lidar_callback,
            10
        )

        # 2) Публикатор для отфильтрованного облака
        self.filtered_pub = self.create_publisher(
            PointCloud2,
            '/projected_pointcloud_xz_contour_robot',
            10
        )

        # 3) Таймер на 2 секунды — по истечении останавливаем запись
        self.declare_parameter('record_seconds', 2.0)  # Можно переопределять через Launch
        self.record_duration = self.get_parameter('record_seconds').value
        self.stop_timer = self.create_timer(self.record_duration, self.stop_recording)

        # Пока не остановились, новый таймер для постоянной публикации не создаём
        self.publish_timer = None

        self.get_logger().info(
            f'Нода запущена. Записываем данные из /velodyne_points {self.record_duration} c.\n'
            f'Фильтр: x ∈ [{self.x_min}, {self.x_max}], '
            f'y ∈ [{self.y_min}, {self.y_max}], z ∈ [{self.z_min}, {self.z_max}].'
        )

    def lidar_callback(self, cloud_msg):
        """
        Колбэк для входящего облака. Если recording_active=True, фильтруем и накапливаем точки.
        Когда запись остановлена, игнорируем данные.
        """
        if not self.recording_active:
            # Игнорируем новые пакеты
            return

        # Преобразуем PointCloud2 → генератор точек
        gen_points = point_cloud2.read_points(
            cloud_msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        )

        # Фильтрация по заданным диапазонам
        for (x, y, z) in gen_points:
            if (self.x_min <= x <= self.x_max and
                self.y_min <= y <= self.y_max and
                self.z_min <= z <= self.z_max):
                self.accumulated_points.append((x, y, z))

    def stop_recording(self):
        """
        Вызывается таймером один раз по истечении self.record_duration.
        Прекращаем запись, отписываемся от лидара, запускаем постоянную публикацию накопленных точек.
        """
        # Останавливаем таймер, чтобы не вызывался повторно
        self.stop_timer.cancel()

        # Ставим флаг, что новые данные не принимаем
        self.recording_active = False

        # Отписываемся от топика лидара, если не планируем дальше его слушать
        self.destroy_subscription(self.lidar_sub)

        self.get_logger().info(
            f'Сбор данных остановлен. Накоплено {len(self.accumulated_points)} точек.'
        )

        # 4) Запускаем второй таймер, который будет ПОВТОРНО публиковать накопленное облако
        #    Например, каждую секунду
        self.publish_timer = self.create_timer(1.0, self.publish_final_points)

    def publish_final_points(self):
        """
        Периодически (например, каждую секунду) публикует одно и то же накопленное облако.
        """
        if not self.accumulated_points:
            self.get_logger().warn('Нет точек для публикации (accumulated_points пуст).')
            return

        header = PointCloud2().header
        header.frame_id = 'map'  # Или любая нужная вам рамка
        header.stamp = self.get_clock().now().to_msg()

        cloud_msg = point_cloud2.create_cloud_xyz32(header, self.accumulated_points)
        self.filtered_pub.publish(cloud_msg)

        # Можно вывести debug-лог: self.get_logger().debug('Публикуем накопленное облако...')


def main(args=None):
    rclpy.init(args=args)
    node = LidarRecordAndConstantPublishNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Остановка по Ctrl+C')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
