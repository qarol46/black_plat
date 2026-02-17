#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64MultiArray


class CommandsRepublisher(Node):
    def __init__(self):
        super().__init__('commands_republisher')

        # Параметр для настройки частоты публикации (Гц)
        self.declare_parameter('publish_frequency', 0.5)
        pub_freq = self.get_parameter('publish_frequency').value
        if pub_freq <= 0.0:
            pub_freq = 10.0
            self.get_logger().warn("publish_frequency <= 0. Устанавливаем 10 Гц по умолчанию.")

        # Подписка на входной топик
        self.subscriber = self.create_subscription(
            Float64MultiArray,
            '/geom_position_controller/commands_before',
            self.commands_callback,
            10
        )

        # Паблишер в выходной топик
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/geom_position_controller/commands',
            10
        )

        # Таймер для гарантированной публикации ровно в pub_freq Гц
        self.timer = self.create_timer(1.0 / pub_freq, self.timer_callback)

        # Храним последнее валидное сообщение (изначально None)
        self.last_valid_cmd = None
        # Храним последнее полученное сообщение (может быть None)
        self.latest_cmd = None

        self.get_logger().info(
            f"Запущена нода {self.get_name()}. "
            f"Перепубликация команд на /geom_position_controller/commands с частотой {pub_freq} Гц."
        )

    def commands_callback(self, msg: Float64MultiArray):
        """
        Колбэк при приходе нового сообщения с /geom_position_controller/commands_before.
        Сохраняем последнее сообщение.
        """
        self.latest_cmd = msg
        if msg is not None:
            self.last_valid_cmd = msg

    def timer_callback(self):
        """
        Вызывается раз в 1/pub_freq секунд.
        Публикуем последнее валидное сообщение (если оно есть).
        """
        if self.last_valid_cmd is not None:
            self.publisher.publish(self.last_valid_cmd)
            # Конвертируем радианы в градусы для вывода
            front_angle_rad = self.last_valid_cmd.data[0]  # Первый элемент - угол переднего флиппера
            front_angle_deg = math.degrees(front_angle_rad)
            
            # Добавляем пометку, если текущее сообщение None
            status_note = " [LATEST VALID]" if self.latest_cmd is None else ""
            self.get_logger().info(f"Send front flipper => {front_angle_deg:.1f} deg ({front_angle_rad:.2f} rad){status_note}")
        else:
            self.get_logger().debug("Нет валидных сообщений для публикации")


def main(args=None):
    rclpy.init(args=args)
    node = CommandsRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Прерывание по сигналу пользователя")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()