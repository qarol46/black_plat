import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv
import os
import math

def quaternion_to_euler_angle(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = (math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = (math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = (math.atan2(t3, t4))

    return X, Y, Z
 
# ros2 bag play imu_data/imu_data_0.db3 --rate 100 --read-ahead-queue-size 10000

class ImuRecorder(Node):
    def __init__(self):
        super().__init__('imu_recorder_node')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        
        self.start_time = None
        self.fieldnames = [
            'time_data',
            'linear_acceleration_x',
            'linear_acceleration_y',
            'linear_acceleration_z',
            'angular_velocity_x',
            'angular_velocity_y',
            'angular_velocity_z',
            'orientation_x',
            'orientation_y',
            'orientation_z'  
        ]

        # Создаем директорию, если она не существует
        dir_path = './imu_data_2'
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        
        # Открываем файл ОДИН раз при инициализации
        self.file_path = os.path.join(dir_path, 'imu_data.csv')
        self.f = open(self.file_path, mode='w', newline='')
        
        # Создаем объект записи
        self.csv_writer = csv.DictWriter(self.f, fieldnames=self.fieldnames)
        self.csv_writer.writeheader()
        
        self.get_logger().info(f'Файл открыт для записи: {self.file_path}')
        self.get_logger().info('Ожидание данных...')

    def imu_callback(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        # Вычисляем время в секундах от начала работы
        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9 * 100.0
        
        # Записываем данные в уже открытый файл
        try:
            self.csv_writer.writerow({
                self.fieldnames[0]: elapsed_time,
                self.fieldnames[1]: msg.linear_acceleration.x,
                self.fieldnames[2]: msg.linear_acceleration.y,
                self.fieldnames[3]: msg.linear_acceleration.z,
                self.fieldnames[4]: msg.angular_velocity.x,
                self.fieldnames[5]: msg.angular_velocity.y,
                self.fieldnames[6]: msg.angular_velocity.z,
                self.fieldnames[7]: quaternion_to_euler_angle(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)[0],
                self.fieldnames[8]: quaternion_to_euler_angle(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)[1],
                self.fieldnames[9]: quaternion_to_euler_angle(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)[2]
            })
            # Опционально: сбрасываем буфер на диск, чтобы данные не пропали при сбое
            # self.f.flush() 
            self.get_logger().info(f'Запись: t={elapsed_time:.2f}')
        except Exception as e:
            self.get_logger().error(f'Ошибка записи: {e}')

    def destroy_node(self):
        # Закрываем файл перед уничтожением узла
        self.get_logger().info('Закрытие файла...')
        self.f.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImuRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Остановка по сигналу пользователя...')
    finally:
        # Важно вызвать деструктор, чтобы файл закрылся корректно
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()