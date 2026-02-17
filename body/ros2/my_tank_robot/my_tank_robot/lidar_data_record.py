#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# для записи точек необходимо сначала запустить ноду лидара
# ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py


class VelodynePointsRecorder(Node):
    def __init__(self):
        super().__init__('velodyne_points_recorder')

        # Подписываемся на топик с облаком точек
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.pointcloud_callback,
            10
        )

        # Множество для хранения уникальных точек.
        # (Если хотите сохранять абсолютно все, включая дубликаты, используйте list.)
        self.points = set()

        # Время записи (в секундах)
        self.record_duration = 0.10

        # Флаг, показывающий, идёт ли ещё запись
        self.recording_active = True

        # Таймер, по истечении которого остановим запись
        self.timer_stop = self.create_timer(self.record_duration, self.stop_recording)

        self.get_logger().info(
            f'Запуск записи из /velodyne_points на {self.record_duration:.2f} секунды.\n'
            'Результат будет сохранён в points2.txt'
        )

        # Смещение, которое нужно добавить к каждой точке (x, y, z)
        self.offset_x = 0.154
        self.offset_y = -0.001
        self.offset_z = 0.251

    def pointcloud_callback(self, msg):
        """Колбэк: считываем и накапливаем точки, пока запись активна."""
        if not self.recording_active:
            return

        # Читаем (x, y, z) из сообщения PointCloud2
        for (x, y, z) in pc2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):
            # Добавляем смещение
            shifted_x = x + self.offset_x
            shifted_y = y + self.offset_y
            shifted_z = z + self.offset_z

            # Добавляем в set (уникальные точки)
            self.points.add((shifted_x, shifted_y, shifted_z))

    def stop_recording(self):
        """
        Вызывается таймером (через self.record_duration секунд).
        Останавливаем запись, сохраняем данные в points2.txt, завершаем работу.
        """
        # Останавливаем таймер, чтобы не вызывался повторно
        self.timer_stop.cancel()

        # Выключаем приём новых сообщений
        self.recording_active = False
        self.destroy_subscription(self.subscription)

        # Пишем итог в файл
        self.save_to_txt('points2.txt')

        self.get_logger().info(
            f'Запись остановлена. Всего собрано {len(self.points)} уникальных точек.\n'
            'Завершаем работу...'
        )

        # Завершаем работу
        rclpy.shutdown()

    def save_to_txt(self, filename):
        """Сохраняет накопленные точки (x, y, z) в текстовый файл."""
        with open(filename, 'w') as txt_file:
            for (x, y, z) in self.points:
                txt_file.write(f'{x:.5f} {y:.5f} {z:.5f}\n')

        self.get_logger().info(f'Данные записаны в {filename}')

def main(args=None):
    rclpy.init(args=args)
    node = VelodynePointsRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Остановка по Ctrl+C')
        rclpy.shutdown()

if __name__ == '__main__':
    main()
