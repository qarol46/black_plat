import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import ByteMultiArray
import serial
import struct

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('pwm_teleop_node')

        # Параметры (можно переопределять через --ros-args -p ...)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('deadzone', 0.10)
        self.declare_parameter('koef_pwm', 40.0)
        self.declare_parameter('mode_char', 'v')  # один ASCII символ

        self.joy_dz = float(self.get_parameter('deadzone').value)
        self.koef_pwm = float(self.get_parameter('koef_pwm').value)
        self.mode = str(self.get_parameter('mode_char').value)
        if not self.mode or len(self.mode) != 1:
            self.get_logger().warn("mode_char должен быть одним символом; установлено 'v'")
            self.mode = 'v'

        port = str(self.get_parameter('port').value)
        baud = int(self.get_parameter('baud').value)

        # Подписка на /joy
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Публикация байтов (для отладки/логгирования)
        self.bytes_array_pub = self.create_publisher(ByteMultiArray, 'bytes_array_topic', 10)

        # Подключение Serial
        try:
            self.serial_port = serial.Serial(port, baudrate=baud, timeout=0.02)
            self.get_logger().info(f'Opened serial: {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed open serial {port}: {e}')
            self.serial_port = None

        self.wheels_cmd = [0.0, 0.0, 0.0, 0.0]  # [FL, FR, BR, BL]

    def joy_callback(self, msg: Joy):
        # Оси (проверь индексы под свой геймпад)
        RX, RY, LX, LY = -msg.axes[3], msg.axes[4], msg.axes[0], msg.axes[1]

        # Dead zone
        if abs(RX) < self.joy_dz: RX = 0.0
        if abs(RY) < self.joy_dz: RY = 0.0
        if abs(LX) < self.joy_dz: LX = 0.0
        if abs(LY) < self.joy_dz: LY = 0.0

        # Простейшая схема распределения на 4 колеса
        self.wheels_cmd[3] = LY + LX + RY - RX
        self.wheels_cmd[0] = LY - LX + RY + RX
        self.wheels_cmd[2] = LY - LX + RY - RX
        self.wheels_cmd[1] = LY + LX + RY + RX
        # Масштабирование
        for i in range(4):
            self.wheels_cmd[i] *= self.koef_pwm

        # Пакет: 1 char + 4 double (little-endian, native alignment off)
        try:
            buffer_bytes = struct.pack('=cdddd', self.mode.encode('ascii'),
                                       self.wheels_cmd[0], self.wheels_cmd[1],
                                       self.wheels_cmd[2], self.wheels_cmd[3])
        except Exception as e:
            self.get_logger().error(f'Pack error: {e}')
            return

        # Публикация в ROS (ByteMultiArray.data — это список bytes)
        msg_out = ByteMultiArray()
        # ИСПРАВЛЕНИЕ: создаем список байтовых объектов
        msg_out.data = [bytes([b]) for b in buffer_bytes]
        self.bytes_array_pub.publish(msg_out)

        # Отправка в Serial
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(buffer_bytes)
                # Логируем отправленные данные для отладки
                self.get_logger().info(f'Sent: {[b for b in buffer_bytes]}', throttle_duration_sec=1.0)
            except Exception as e:
                self.get_logger().error(f'Serial write error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = JoySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()