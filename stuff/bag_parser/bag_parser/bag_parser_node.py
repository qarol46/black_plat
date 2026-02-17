#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import csv
import os
from datetime import datetime
from math import atan2
from collections import defaultdict

from nav_msgs.msg import Odometry
from std_msgs.msg import String

class MultiOdomParserNode(Node):

    def __init__(self):
        super().__init__('bag_parser_node')
        
        # Параметры ноды
        self.declare_parameter('odom_topics', ['/aft_mapped_to_init'])
        self.declare_parameter('output_directory', 'output_csv')
        self.declare_parameter('update_rate', 2.0)  # Hz
        self.declare_parameter('use_best_effort', True)  # Новый параметр для QoS
        
        # Получаем значения параметров
        odom_topics = self.get_parameter('odom_topics').get_parameter_value().string_array_value
        output_directory = self.get_parameter('output_directory').get_parameter_value().string_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        use_best_effort = self.get_parameter('use_best_effort').get_parameter_value().bool_value
        
        # Настраиваем QoS профиль
        if use_best_effort:
            # BEST_EFFORT - для совместимости с большинством bag-файлов
            qos_profile = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE
            )
        else:
            # RELIABLE - по умолчанию в ROS2
            qos_profile = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE
            )
        
        # Создаем директорию для выходных файлов
        self.output_dir = output_directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Словари для хранения данных по каждому топику
        self.data_buffers = defaultdict(lambda: {
            'x': None,
            'y': None, 
            'z': None,  # Добавлена координата Z
            'yaw': None,
            'linear_x': None,
            'angular_z': None,
            'last_update': None
        })
        
        self.csv_files = {}
        self.csv_writers = {}
        
        # Создаем CSV файлы и подписки для каждого топика
        for topic in odom_topics:
            self.setup_topic(topic, qos_profile)
        
        # Таймер для записи данных в CSV
        timer_period = 1.0 / update_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f"Multi Odom Parser started")
        self.get_logger().info(f"Monitoring topics: {odom_topics}")
        self.get_logger().info(f"Output directory: {self.output_dir}")
        self.get_logger().info(f"QoS profile: {'BEST_EFFORT' if use_best_effort else 'RELIABLE'}")

    def setup_topic(self, topic, qos_profile):
        """Создает CSV файл и подписку для указанного топика"""
        try:
            # Создаем безопасное имя файла из названия топика
            filename = topic.replace('/', '_').lstrip('_')
            if not filename:
                filename = 'odom'
            filepath = os.path.join(self.output_dir, f"{filename}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
            
            # Создаем CSV файл и записываем заголовок
            csv_file = open(filepath, 'w', newline='')
            writer = csv.writer(csv_file)
            # Обновлен заголовок с добавлением координаты z
            writer.writerow(['timestamp', 'x', 'y', 'z', 'yaw', 'linear_x', 'angular_z', 'frame_id', 'child_frame_id'])
            
            self.csv_files[topic] = csv_file
            self.csv_writers[topic] = writer
            
            # Подписываемся на топик с указанным QoS профилем
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, t=topic: self.odom_callback(msg, t),
                qos_profile=qos_profile)  # Передаем QoS профиль
            
            self.get_logger().info(f"Setup complete for topic: {topic} -> {filepath}")
            
        except Exception as e:
            self.get_logger().error(f"Error setting up topic {topic}: {str(e)}")

    def odom_callback(self, msg, topic):
        """Обработчик сообщений Odometry для конкретного топика"""
        try:
            # Извлекаем данные из Odometry сообщения
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z  # Добавлено извлечение координаты Z
            
            # Извлекаем yaw из кватерниона
            orientation = msg.pose.pose.orientation
            t3 = +2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
            t4 = +1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
            yaw = atan2(t3, t4)
            
            # Линейная и угловая скорость
            linear_x = msg.twist.twist.linear.x
            angular_z = msg.twist.twist.angular.z
            
            # Обновляем буфер данных с добавлением координаты z
            self.data_buffers[topic].update({
                'x': x,
                'y': y,
                'z': z,  # Добавлено сохранение координаты z
                'yaw': yaw,
                'linear_x': linear_x,
                'angular_z': angular_z,
                'frame_id': msg.header.frame_id,
                'child_frame_id': msg.child_frame_id,
                'last_update': self.get_clock().now()
            })
            
        except Exception as e:
            self.get_logger().error(f"Error in odom_callback for {topic}: {str(e)}")

    def timer_callback(self):
        """Периодическая запись данных в CSV файлы"""
        current_time = self.get_clock().now()
        
        for topic, buffer in self.data_buffers.items():
            if topic not in self.csv_writers:
                continue
                
            # Записываем данные только если они были обновлены
            if buffer['last_update'] is not None:
                try:
                    timestamp = current_time.nanoseconds / 1e9
                    
                    # Подготавливаем строку для записи с добавлением координаты z
                    row = [
                        timestamp,
                        buffer['x'] if buffer['x'] is not None else '-',
                        buffer['y'] if buffer['y'] is not None else '-',
                        buffer['z'] if buffer['z'] is not None else '-',  # Добавлена координата z
                        buffer['yaw'] if buffer['yaw'] is not None else '-',
                        buffer['linear_x'] if buffer['linear_x'] is not None else '-',
                        buffer['angular_z'] if buffer['angular_z'] is not None else '-',
                        buffer['frame_id'] if buffer['frame_id'] else '-',
                        buffer['child_frame_id'] if buffer['child_frame_id'] else '-'
                    ]
                    
                    # Записываем в CSV
                    self.csv_writers[topic].writerow(row)
                    self.csv_files[topic].flush()
                    
                except Exception as e:
                    self.get_logger().error(f"Error writing data for {topic}: {str(e)}")

    def destroy_node(self):
        """Корректное завершение работы"""
        self.get_logger().info("Shutting down Multi Odom Parser...")
        
        # Закрываем все CSV файлы
        for topic, file in self.csv_files.items():
            try:
                file.close()
                self.get_logger().info(f"Closed file for topic: {topic}")
            except Exception as e:
                self.get_logger().error(f"Error closing file for {topic}: {str(e)}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MultiOdomParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
