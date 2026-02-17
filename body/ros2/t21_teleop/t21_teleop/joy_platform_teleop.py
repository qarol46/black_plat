#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class JoyFullTeleop(Node):
    def __init__(self):
        super().__init__('joy_platform_teleop')

        # ───────── параметры осей / кнопок ─────────
        self.declare_parameter('axis_lin',        1)    # левый стик Y
        self.declare_parameter('axis_ang',        0)    # левый стик X

        self.declare_parameter('scale_lin',       0.3)  # м/с
        self.declare_parameter('scale_ang',       0.5)  # рад/с
        self.declare_parameter('deadzone',        0.05)

        p = self.get_parameter
        self.axis_lin       = p('axis_lin').value
        self.axis_ang       = p('axis_ang').value

        self.scale_lin      = p('scale_lin').value
        self.scale_ang      = p('scale_ang').value
        self.deadzone       = p('deadzone').value

        # ───────── паблишеры ─────────
        self.pub_cmd = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        # ───────── подписка на джой ─────────
        self.create_subscription(Joy, '/joy', self.cb_joy, 10)

    # ───────────────── callback ─────────────────
    def cb_joy(self, joy: Joy):
        # оси движения
        v = joy.axes[self.axis_lin] if self.axis_lin < len(joy.axes) else 0.0
        w = joy.axes[self.axis_ang] if self.axis_ang < len(joy.axes) else 0.0
        v = 0.0 if abs(v) < self.deadzone else v
        w = 0.0 if abs(w) < self.deadzone else w

        # ───── publish ─────
        twist = Twist()
        twist.linear.x  = float(v * self.scale_lin)
        twist.angular.z = float(w * self.scale_ang)
        self.pub_cmd.publish(twist)


def main():
    rclpy.init()
    rclpy.spin(JoyFullTeleop())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
