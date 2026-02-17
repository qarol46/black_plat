#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class JoyFullTeleop(Node):
    def __init__(self):
        super().__init__('joy_full_teleop')

        self.declare_parameter('axis_lin', 1)      # left stick Y
        self.declare_parameter('axis_ang', 0)      # left stick X
        self.declare_parameter('btn_flip_up', 5)   # RB
        self.declare_parameter('btn_flip_down', 4) # LB

        self.declare_parameter('scale_lin', 1.5)
        self.declare_parameter('scale_ang', 1.0)
        self.declare_parameter('flip_step_deg', 5.0)
        self.declare_parameter('deadzone', 0.05)

        # телепоп-рамки (можно оставить как доп. защиту)
        self.declare_parameter('flip_min_deg', 80.0)
        self.declare_parameter('flip_max_deg', 300.0)

        p = self.get_parameter
        self.axis_lin = int(p('axis_lin').value)
        self.axis_ang = int(p('axis_ang').value)
        self.btn_up   = int(p('btn_flip_up').value)
        self.btn_down = int(p('btn_flip_down').value)

        self.scale_lin = float(p('scale_lin').value)
        self.scale_ang = float(p('scale_ang').value)
        self.flip_step = math.radians(float(p('flip_step_deg').value))
        self.deadzone  = float(p('deadzone').value)

        self.flip_min = math.radians(float(p('flip_min_deg').value))
        self.flip_max = math.radians(float(p('flip_max_deg').value))

        # --- состояние флиппера ---
        self.flip_target = None         # пока не знаем
        self.geom_name = 'geom_joint'
        self.geom_index = None          # индекс в joint_states, когда найдём

        # паблишеры
        self.pub_cmd = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.pub_flip = self.create_publisher(Float64MultiArray, '/geom_position_controller/commands', 10)

        # подписки
        self.create_subscription(Joy, '/joy', self.cb_joy, 10)
        self.create_subscription(JointState, '/joint_states', self.cb_js, 10)

    def cb_js(self, msg: JointState):
        # найти индекс geom_joint один раз
        if self.geom_index is None:
            try:
                self.geom_index = msg.name.index(self.geom_name)
                self.get_logger().info(f'Found {self.geom_name} in joint_states at index {self.geom_index}')
            except ValueError:
                return

        if self.geom_index < len(msg.position):
            cur = float(msg.position[self.geom_index])
            # инициализация цели текущим положением
            if self.flip_target is None:
                self.flip_target = max(min(cur, self.flip_max), self.flip_min)
                self.get_logger().info(f'Init flip_target = {math.degrees(self.flip_target):.1f} deg')

    def cb_joy(self, joy: Joy):
        # движение
        v = joy.axes[self.axis_lin] if self.axis_lin < len(joy.axes) else 0.0
        w = joy.axes[self.axis_ang] if self.axis_ang < len(joy.axes) else 0.0
        v = 0.0 if abs(v) < self.deadzone else v
        w = 0.0 if abs(w) < self.deadzone else w

        twist = Twist()
        twist.linear.x  = float(v * self.scale_lin)
        twist.angular.z = float(w * self.scale_ang)
        self.pub_cmd.publish(twist)

        # флипперы: пока не знаем текущее положение — НЕ публикуем
        if self.flip_target is None:
            return

        up = (self.btn_up < len(joy.buttons) and joy.buttons[self.btn_up] == 1)
        down = (self.btn_down < len(joy.buttons) and joy.buttons[self.btn_down] == 1)

        changed = False
        if up:
            self.flip_target += self.flip_step
            changed = True
        elif down:
            self.flip_target -= self.flip_step
            changed = True

        if changed:
            self.flip_target = max(min(self.flip_target, self.flip_max), self.flip_min)
            self.pub_flip.publish(Float64MultiArray(data=[self.flip_target]))


def main():
    rclpy.init()
    rclpy.spin(JoyFullTeleop())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
