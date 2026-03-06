#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
from datetime import datetime
from math import atan2
from collections import defaultdict
import tf2_ros
import geometry_msgs.msg
from tf2_ros import TransformException

class TFtoCSVNode(Node):

    def __init__(self):
        super().__init__('tf_to_csv_node')
        
        # Параметры ноды
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'map')
        self.declare_parameter('output_directory', 'output_csv')
        self.declare_parameter('update_rate', 2.0)  # Hz
        self.declare_parameter('include_velocity', False)  # Вычислять ли скорости
        
        # Получаем значения параметров
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        output_directory = self.get_parameter('output_directory').get_parameter_value().string_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.include_velocity = self.get_parameter('include_velocity').get_parameter_value().bool_value
        
        # Создаем директорию для выходных файлов
        self.output_dir = output_directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Создаем CSV файл
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"tf_{self.source_frame}_to_{self.target_frame}_{timestamp}.csv"
        filepath = os.path.join(self.output_dir, filename)
        
        # Создаем CSV файл и записываем заголовок
        self.csv_file = open(filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Формируем заголовок в зависимости от того, нужны ли скорости
        header = ['timestamp', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']
        if self.include_velocity:
            header.extend(['vx', 'vy', 'vz', 'vroll', 'vpitch', 'vyaw'])
        
        self.csv_writer.writerow(header)
        
        # Инициализация TF2 буфера и слушателя
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Для вычисления скорости (если нужно)
        if self.include_velocity:
            self.last_transform = None
            self.last_time = None
        
        # Таймер для периодического получения трансформации
        timer_period = 1.0 / update_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f"TF to CSV Node started")
        self.get_logger().info(f"Monitoring transform: {self.source_frame} -> {self.target_frame}")
        self.get_logger().info(f"Output file: {filepath}")
        self.get_logger().info(f"Include velocity: {self.include_velocity}")

    def quaternion_to_euler(self, q):
        """Преобразование кватерниона в углы Эйлера (roll, pitch, yaw)"""
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = copysign(3.14159 / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = atan2(sinp, cosr_cosp)  # более стабильный расчет pitch

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def calculate_velocity(self, current_transform, current_time):
        """Вычисление линейной и угловой скорости на основе предыдущего измерения"""
        if self.last_transform is None or self.last_time is None:
            return [0.0] * 6
        
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return [0.0] * 6
        
        # Линейная скорость
        vx = (current_transform.transform.translation.x - self.last_transform.transform.translation.x) / dt
        vy = (current_transform.transform.translation.y - self.last_transform.transform.translation.y) / dt
        vz = (current_transform.transform.translation.z - self.last_transform.transform.translation.z) / dt
        
        # Угловая скорость (упрощенно - разница углов)
        current_roll, current_pitch, current_yaw = self.quaternion_to_euler(current_transform.transform.rotation)
        last_roll, last_pitch, last_yaw = self.quaternion_to_euler(self.last_transform.transform.rotation)
        
        # Учитываем переход через 2π
        vroll = (current_roll - last_roll) / dt
        vpitch = (current_pitch - last_pitch) / dt
        vyaw = (current_yaw - last_yaw) / dt
        
        return [vx, vy, vz, vroll, vpitch, vyaw]

    def timer_callback(self):
        """Периодическое получение трансформации и запись в CSV"""
        try:
            # Получаем трансформацию из source_frame в target_frame
            # lookup_transform(target_frame, source_frame, time) вернет transform от source_frame к target_frame
            transform_stamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time()
            )
            
            current_time = self.get_clock().now()
            timestamp = current_time.nanoseconds / 1e9
            
            # Извлекаем данные
            x = transform_stamped.transform.translation.x
            y = transform_stamped.transform.translation.y
            z = transform_stamped.transform.translation.z
            
            # Преобразуем кватернион в углы Эйлера
            roll, pitch, yaw = self.quaternion_to_euler(transform_stamped.transform.rotation)
            
            # Формируем строку для записи
            row = [timestamp, x, y, z, roll, pitch, yaw]
            
            # Добавляем скорости, если нужно
            if self.include_velocity:
                velocity = self.calculate_velocity(transform_stamped, current_time)
                row.extend(velocity)
                
                # Сохраняем текущую трансформацию для следующего вычисления скорости
                self.last_transform = transform_stamped
                self.last_time = current_time
            
            # Записываем в CSV
            self.csv_writer.writerow(row)
            self.csv_file.flush()
            
            # Логирование для отладки (можно закомментировать)
            self.get_logger().debug(f"Recorded transform: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
            
        except TransformException as e:
            self.get_logger().warn(f"Could not get transform from {self.source_frame} to {self.target_frame}: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {str(e)}")

    def destroy_node(self):
        """Корректное завершение работы"""
        self.get_logger().info("Shutting down TF to CSV Node...")
        
        # Закрываем CSV файл
        try:
            self.csv_file.close()
            self.get_logger().info("Closed CSV file")
        except Exception as e:
            self.get_logger().error(f"Error closing CSV file: {str(e)}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TFtoCSVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()