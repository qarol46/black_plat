#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import subprocess
import math
import time
import os
import numpy as np
from scipy.spatial.transform import Rotation

class SimpleGroundTruth(Node):
    def __init__(self):
        super().__init__('simple_gz_ground_truth')
        
        # Параметры
        self.declare_parameter('model_name', 'go1')
        self.model_name = self.get_parameter('model_name').value
        
        self.publisher_ = self.create_publisher(Odometry, '/gazebo/ground_truth', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        # Флаги и переменные для отслеживания начального положения
        self.initial_pose = None
        self.initial_orientation = None
        self.has_initial_pose = False
        self.initial_yaw = 0.0
        
        # Приращения от начальной точки
        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_z = 0.0
        self.delta_yaw = 0.0
        self.delta_roll = 0.0
        self.delta_pitch = 0.0
        
        self.get_logger().info(f'Gazebo ground truth for model: {self.model_name}')
        self.get_logger().info('Starting from zero position. Tracking relative motion...')
    
    def get_current_pose(self):
        """Получаем текущую позу робота из Gazebo"""
        # Метод 1: Попробуем через gz model --pose
        pose_data = self.try_method1()
        
        # Метод 2: Если не сработал, попробуем другой формат
        if not pose_data:
            pose_data = self.try_method2()
            
        # Метод 3: Через gz topic
        if not pose_data:
            pose_data = self.try_method3()
            
        return pose_data
    
    def try_method1(self):
        """Попробуем оригинальный метод"""
        try:
            cmd = ['gz', 'model', '-p', '-m', self.model_name]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=1.0)
            
            if result.returncode == 0 and result.stdout.strip():
                parts = result.stdout.strip().split()
                if len(parts) >= 6:
                    return {
                        'x': float(parts[0]),
                        'y': float(parts[1]),
                        'z': float(parts[2]),
                        'roll': float(parts[3]),
                        'pitch': float(parts[4]),
                        'yaw': float(parts[5])
                    }
        except:
            pass
        return None
    
    def try_method2(self):
        """Альтернативный формат команды"""
        try:
            cmd = ['gz', 'model', '--model-name', self.model_name, '--pose']
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=1.0)
            
            if result.returncode == 0 and result.stdout.strip():
                output = result.stdout.strip()
                if 'Model pose:' in output:
                    import re
                    nums = re.findall(r"[-+]?\d*\.\d+|\d+", output)
                    if len(nums) >= 6:
                        return {
                            'x': float(nums[0]),
                            'y': float(nums[1]),
                            'z': float(nums[2]),
                            'roll': float(nums[3]),
                            'pitch': float(nums[4]),
                            'yaw': float(nums[5])
                        }
        except:
            pass
        return None
    
    def try_method3(self):
        """Получаем позу через топик"""
        try:
            cmd = ['gz', 'topic', '-e', '-n', '1', '/gazebo/default/pose/info']
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=1.0)
            
            if result.returncode == 0 and result.stdout.strip():
                import yaml
                data = yaml.safe_load(result.stdout)
                
                if 'pose' in data:
                    for pose_item in data['pose']:
                        if 'name' in pose_item and self.model_name in pose_item['name']:
                            if 'position' in pose_item and 'orientation' in pose_item:
                                pos = pose_item['position']
                                orient = pose_item['orientation']
                                
                                # Конвертируем кватернион в RPY
                                try:
                                    r = Rotation.from_quat([
                                        orient.get('x', 0),
                                        orient.get('y', 0), 
                                        orient.get('z', 0),
                                        orient.get('w', 1)
                                    ])
                                    roll, pitch, yaw = r.as_euler('xyz')
                                    
                                    return {
                                        'x': pos.get('x', 0),
                                        'y': pos.get('y', 0),
                                        'z': pos.get('z', 0),
                                        'roll': float(roll),
                                        'pitch': float(pitch),
                                        'yaw': float(yaw)
                                    }
                                except:
                                    # Если не получилось с scipy, используем простой расчет для yaw
                                    w = orient.get('w', 1)
                                    z = orient.get('z', 0)
                                    yaw = 2 * math.atan2(z, w) if w != 0 else 0
                                    return {
                                        'x': pos.get('x', 0),
                                        'y': pos.get('y', 0),
                                        'z': pos.get('z', 0),
                                        'roll': 0.0,
                                        'pitch': 0.0,
                                        'yaw': float(yaw)
                                    }
        except Exception as e:
            self.get_logger().debug(f'Method 3 failed: {e}')
        
        return None
    
    def calculate_relative_pose(self, current_pose):
        """Вычисляем относительную позицию от начального положения"""
        if not self.has_initial_pose:
            # Сохраняем начальное положение
            self.initial_pose = {
                'x': current_pose['x'],
                'y': current_pose['y'], 
                'z': current_pose['z']
            }
            self.initial_orientation = {
                'roll': current_pose['roll'],
                'pitch': current_pose['pitch'],
                'yaw': current_pose['yaw']
            }
            self.initial_yaw = current_pose['yaw']
            self.has_initial_pose = True
            self.get_logger().info(f'Initial pose captured: {self.initial_pose}')
            self.get_logger().info(f'Initial orientation: {self.initial_orientation}')
            
            # Возвращаем нули для первого измерения
            return {
                'x': 0.0, 'y': 0.0, 'z': 0.0,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
            }
        
        # Вычисляем приращения
        # Для позиции: просто вычитаем начальные координаты
        self.delta_x = current_pose['x'] - self.initial_pose['x']
        self.delta_y = current_pose['y'] - self.initial_pose['y']
        self.delta_z = current_pose['z'] - self.initial_pose['z']
        
        # Для ориентации: вычисляем разницу углов
        # Важно: учитываем периодичность углов (например, переход через 180/-180 градусов)
        self.delta_roll = self.normalize_angle(current_pose['roll'] - self.initial_orientation['roll'])
        self.delta_pitch = self.normalize_angle(current_pose['pitch'] - self.initial_orientation['pitch'])
        self.delta_yaw = self.normalize_angle(current_pose['yaw'] - self.initial_orientation['yaw'])
        
        return {
            'x': self.delta_x,
            'y': self.delta_y, 
            'z': self.delta_z,
            'roll': self.delta_roll,
            'pitch': self.delta_pitch,
            'yaw': self.delta_yaw
        }
    
    def normalize_angle(self, angle):
        """Нормализует угол в диапазон [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def timer_callback(self):
        """Основной таймер"""
        current_pose = self.get_current_pose()
        
        if current_pose:
            # Получаем относительную позицию
            relative_pose = self.calculate_relative_pose(current_pose)
            
            # Создаем и публикуем Odometry сообщение
            odom = self.create_odometry_message(relative_pose)
            self.publisher_.publish(odom)
            
            # Логируем первую публикацию
            if not self.has_initial_pose:
                self.get_logger().info('Published first ground truth message (all zeros)')
        else:
            self.get_logger().debug('Could not get model pose', throttle_duration_sec=2.0)
    
    def create_odometry_message(self, pose_data):
        """Создает Odometry сообщение из данных позы"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'lidar_link'
        
        # Позиция (уже относительная)
        odom.pose.pose.position.x = pose_data['x']
        odom.pose.pose.position.y = pose_data['y']
        odom.pose.pose.position.z = pose_data['z']
        
        # Ориентация RPY -> кватернион
        roll = pose_data['roll']
        pitch = pose_data['pitch']
        yaw = pose_data['yaw']
        
        # Преобразуем RPY в кватернион
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        odom.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        odom.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        odom.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        odom.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        # Ковариация (небольшая неопределенность)
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = 0.0001  # x
        odom.pose.covariance[7] = 0.0001  # y
        odom.pose.covariance[14] = 0.0001 # z
        odom.pose.covariance[21] = 0.001  # rotation around X
        odom.pose.covariance[28] = 0.001  # rotation around Y
        odom.pose.covariance[35] = 0.001  # rotation around Z
        
        # Скорости (нули, так как мы не вычисляем скорость)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0
        
        odom.twist.covariance = [0.0] * 36
        
        return odom

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGroundTruth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
