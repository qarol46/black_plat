#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import array

class JoyFullTeleop(Node):
    def __init__(self):
        super().__init__('joy_platform_teleop')

        # ───────── параметры осей / кнопок ─────────
        self.declare_parameter('axis_lin', 1)      # левый стик Y
        self.declare_parameter('axis_ang', 0)      # левый стик X
        self.declare_parameter('btn_forward', 12)  # кнопка вперед
        self.declare_parameter('btn_backward', 13) # кнопка назад
        self.declare_parameter('btn_left', 14)     # кнопка влево
        self.declare_parameter('btn_right', 15)    # кнопка вправо
        self.declare_parameter('btn_emergency', 0) # кнопка экстренного торможения (X)

<<<<<<< HEAD:src/t21_teleop/t21_teleop/joy_platform_teleop.py
        self.declare_parameter('scale_lin', 0.5)   # м/с
        self.declare_parameter('scale_ang', 1.0)   # рад/с
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('move_distance', 1.0) # расстояние для движения (м)
        self.declare_parameter('turn_angle', 90.0)  # угол для поворота (градусы)
=======
        self.declare_parameter('scale_lin',       0.3)  # м/с
        self.declare_parameter('scale_ang',       1.0)  # рад/с
        self.declare_parameter('deadzone',        0.05)
>>>>>>> work:body/ros2/t21_teleop/t21_teleop/joy_platform_teleop.py

        p = self.get_parameter
        self.axis_lin = p('axis_lin').value
        self.axis_ang = p('axis_ang').value
        self.btn_forward = p('btn_forward').value
        self.btn_backward = p('btn_backward').value
        self.btn_left = p('btn_left').value
        self.btn_right = p('btn_right').value
        self.btn_emergency = p('btn_emergency').value

        self.scale_lin = p('scale_lin').value
        self.scale_ang = p('scale_ang').value
        self.deadzone = p('deadzone').value
        self.move_distance = p('move_distance').value
        self.turn_angle = math.radians(p('turn_angle').value)

        # ───────── паблишеры ─────────
        self.pub_cmd = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        # ───────── подписка на джой и одометрию ─────────
        self.create_subscription(Joy, '/joy', self.cb_joy, 10)
        self.create_subscription(Odometry, 'diff_drive_controller/odom', self.cb_odom, 10)
        
        # Для хранения предыдущего состояния кнопок
        self.prev_buttons = array.array('B', [0] * 16)
        self.current_odom = None

        # Состояние motion emulator
        self.motion_active = False
        self.motion_type = None
        self.target_distance = 0.0
        self.target_angle = 0.0
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        # Параметры motion emulator
        self.linear_speed = 0.1
        self.angular_speed = 0.5
        self.position_tolerance = 0.01
        self.angle_tolerance = 0.02

        # Флаг экстренного торможения
        self.emergency_stop = False

    def cb_odom(self, msg):
        self.current_odom = msg

    def cb_joy(self, joy: Joy):
        current_buttons = array.array('B', joy.buttons)
        
        # Проверка кнопки экстренного торможения
        if len(current_buttons) > self.btn_emergency and current_buttons[self.btn_emergency] == 1:
            self.emergency_stop = True
            self.motion_active = False  # Прерываем любое активное движение
            twist = Twist()  # Нулевая команда
            self.pub_cmd.publish(twist)
            self.get_logger().warn("EMERGENCY STOP ACTIVATED!", once=True)
            return
        
        # Сбрасываем флаг торможения, если кнопка отпущена
        if self.emergency_stop and len(current_buttons) > self.btn_emergency and current_buttons[self.btn_emergency] == 0:
            self.emergency_stop = False
            self.get_logger().info("Emergency stop released")

        # Если активно торможение - игнорируем все другие команды
        if self.emergency_stop:
            return

        # Обработка аналоговых стиков (только если нет активного движения)
        if not self.motion_active:
            v = joy.axes[self.axis_lin] if self.axis_lin < len(joy.axes) else 0.0
            w = joy.axes[self.axis_ang] if self.axis_ang < len(joy.axes) else 0.0
            v = 0.0 if abs(v) < self.deadzone else v
            w = 0.0 if abs(w) < self.deadzone else w

            twist = Twist()
            twist.linear.x = float(v * self.scale_lin)
            twist.angular.z = float(w * self.scale_ang)
            self.pub_cmd.publish(twist)

        # Проверяем нажатия кнопок
        for i in range(min(len(current_buttons), len(self.prev_buttons))):
            if current_buttons[i] == 1 and self.prev_buttons[i] == 0:
                self.handle_button_press(i)
        
        self.prev_buttons = array.array('B', current_buttons)

        # Обработка активного движения
        if self.motion_active and self.current_odom is not None:
            self.handle_motion()

    def handle_button_press(self, button_id):
        if button_id == self.btn_forward:
            self.start_motion('forward', self.move_distance)
        elif button_id == self.btn_backward:
            self.start_motion('forward', -self.move_distance)
        elif button_id == self.btn_left:
            self.start_motion('turn', self.turn_angle)
        elif button_id == self.btn_right:
            self.start_motion('turn', -self.turn_angle)

    def start_motion(self, motion_type, value):
        if self.current_odom is None:
            self.get_logger().warn("No odometry data received yet")
            return

        self.motion_active = True
        self.motion_type = motion_type
        
        self.initial_x = self.current_odom.pose.pose.position.x
        self.initial_y = self.current_odom.pose.pose.position.y
        
        q = self.current_odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.initial_yaw = math.atan2(siny_cosp, cosy_cosp)

        if motion_type == 'forward':
            self.target_distance = value
            self.get_logger().info(f"Starting forward motion: {value} meters")
        else:
            self.target_angle = value
            self.get_logger().info(f"Starting turn: {math.degrees(value)} degrees")

    def handle_motion(self):
        if self.emergency_stop:
            return

        twist = Twist()
        current_x = self.current_odom.pose.pose.position.x
        current_y = self.current_odom.pose.pose.position.y
        
        q = self.current_odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.motion_type == 'forward':
            dx = current_x - self.initial_x
            dy = current_y - self.initial_y
            distance = math.hypot(dx, dy)
            
            if abs(distance) < abs(self.target_distance) - self.position_tolerance:
                direction = 1.0 if self.target_distance > 0 else -1.0
                twist.linear.x = direction * self.linear_speed
            else:
                self.motion_active = False
                self.get_logger().info("Forward motion completed")
        
        elif self.motion_type == 'turn':
            angle_diff = self.normalize_angle(current_yaw - self.initial_yaw - self.target_angle)
            
            if abs(angle_diff) > self.angle_tolerance:
                direction = 1.0 if angle_diff < 0 else -1.0
                twist.angular.z = direction * self.angular_speed
            else:
                self.motion_active = False
                self.get_logger().info("Turn completed")

        self.pub_cmd.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main():
    rclpy.init()
    rclpy.spin(JoyFullTeleop())
    rclpy.shutdown()

if __name__ == '__main__':
    main()