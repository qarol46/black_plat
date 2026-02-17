#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64  # Используем только Float64 для устранения предупреждения
import termios
import tty
import sys
import select

class KeyboardFlipperControl(Node):
    def __init__(self):
        super().__init__('flipper_control')
        
        # Параметры управления
        self.declare_parameter('up_key', 'q')
        self.declare_parameter('down_key', 'a')
        self.declare_parameter('step_size', 0.5)  # шаг изменения положения
        self.declare_parameter('publish_rate', 20.0)
        
        # Получение параметров
        self.up_key = self.get_parameter('up_key').value
        self.down_key = self.get_parameter('down_key').value
        self.step = self.get_parameter('step_size').value
        
        # Текущее состояние
        self.current_pos = 0.0
        self.up_pressed = False
        self.down_pressed = False
        
        # Публикатор (используем только Float64)
        self.pub = self.create_publisher(Float64, '/fliper_position_controller/commands', 10)
        
        # Таймер для плавного изменения
        self.timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value,
            self.update_position
        )
        
        # Настройка терминала
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info("Keyboard flipper control ready!")
        self.get_logger().info(f"Press and hold '{self.up_key}' to move up, '{self.down_key}' to move down")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def update_position(self):
        key = self.get_key()
        if key == self.up_key:
            self.up_pressed = True
            self.down_pressed = False
        elif key == self.down_key:
            self.down_pressed = True
            self.up_pressed = False
        elif key == '\x03':  # Ctrl+C
            raise KeyboardInterrupt
        
        # Изменение положения при зажатых кнопках
        if self.up_pressed:
            self.current_pos += self.step
        elif self.down_pressed:
            self.current_pos -= self.step
        
        # Публикация
        msg = Float64()
        msg.data = self.current_pos
        self.pub.publish(msg)

    def __del__(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardFlipperControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()