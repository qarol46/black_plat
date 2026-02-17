#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class JoyFlipperTeleop(Node):
    def __init__(self):
        super().__init__('joy_flipper_teleop')

        # ── параметры ───────────────────────────────────────────────
        self.declare_parameter('axis_index',   1)     # вертикаль левого стика Xbox
        self.declare_parameter('scale',        1.0)   # макс-угол (рад)
        self.declare_parameter('deadzone',     0.05)  # мёртвая зона
        self.declare_parameter('command_size', 1)     # длина массива в /commands
        self.declare_parameter('joint_index',  0)     # позиция флиппера в массиве
        # ────────────────────────────────────────────────────────────

        self.axis       = self.get_parameter('axis_index').get_parameter_value().integer_value
        self.scale      = self.get_parameter('scale').get_parameter_value().double_value
        self.deadzone   = self.get_parameter('deadzone').get_parameter_value().double_value
        self.cmd_size   = self.get_parameter('command_size').get_parameter_value().integer_value
        self.joint_idx  = self.get_parameter('joint_index').get_parameter_value().integer_value

        self.pub = self.create_publisher(Float64MultiArray,
                                         '/geom_position_controller/commands', 10)
        self.create_subscription(Joy, '/joy', self.cb, 10)

    def cb(self, joy: Joy):
        if self.axis >= len(joy.axes):
            return
        val = joy.axes[self.axis]
        if abs(val) < self.deadzone:
            val = 0.0
        val *= self.scale

        msg = Float64MultiArray()
        msg.data = [0.0] * self.cmd_size
        if 0 <= self.joint_idx < self.cmd_size:
            msg.data[self.joint_idx] = val
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(JoyFlipperTeleop())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
