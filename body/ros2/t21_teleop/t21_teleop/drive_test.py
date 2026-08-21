#!/usr/bin/env python3
# coding: utf-8
"""
drive_test.py  ─  «1-метровый» тест одометрии.

Параметры командной строки (все необязательные):
    V_lin   [м/с]        — линейная скорость (по умолчанию 0.2)
    T_sec   [с]          — время движения    (по умолчанию 5.0)
    odom_topic           — топик одометрии   (def: /diff_drive_controller/odom)
    cmd_topic            — топик команд      (def: /diff_drive_controller/cmd_vel_unstamped)

Примеры:
    ros2 run t21_teleop drive_test
    ros2 run t21_teleop drive_test 0.25 4 /odom /cmd_vel
"""
import sys, math, rclpy
from rclpy.node     import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg      import Odometry
from tf_transformations import euler_from_quaternion


class DriveTest(Node):
    def __init__(self, v_lin, duration, odom_topic, cmd_topic):
        super().__init__('drive_test')

        # pubs / subs
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.sub     = self.create_subscription(
            Odometry, odom_topic, self.cb_odom, 10)

        # params
        self.v_lin = v_lin
        self.duration = duration

        # state
        self.start_pose = None
        self.end_pose   = None
        self.started    = False
        self.deadline   = self.get_clock().now() + rclpy.duration.Duration(seconds=1)

        # 20 Гц таймер
        self.timer = self.create_timer(0.05, self.cb_timer)

    # ─────────────── callbacks ───────────────
    def cb_odom(self, msg: Odometry):
        # поза XY, yaw
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        pose = (p.x, p.y, yaw)

        if self.start_pose is None:
            self.start_pose = pose
            self.started = True
            self.t0 = self.get_clock().now()
        self.end_pose = pose

    def cb_timer(self):
        if not self.started:
            # ждём первый /odom не дольше 1 с
            if self.get_clock().now() > self.deadline:
                self.get_logger().error("Нет данных /odom — проверьте имя топика")
                self.finish()
            return

        elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        # публикуем скорость
        twist = Twist()
        if elapsed < self.duration:
            twist.linear.x = self.v_lin
        self.cmd_pub.publish(twist)

        # время вышло
        if elapsed >= self.duration:
            self.report()
            self.finish()

    # ─────────────── helpers ───────────────
    def report(self):
        dx = self.end_pose[0] - self.start_pose[0]
        dy = self.end_pose[1] - self.start_pose[1]
        dist = math.hypot(dx, dy)

        self.get_logger().info(
            f"\n=== DRIVE-TEST DONE ===\n"
            f"Команда:  V = {self.v_lin:.3f} м/с   T = {self.duration:.2f} c\n"
            f"Одометрия: {dist:.3f} м\n"
            f"Измерьте рулеткой фактическую дистанцию.\n"
            f"k_dist  =  real_dist  /  {dist:.3f}\n"
            f"R_new   =  R_old  /  k_dist\n")

    def finish(self):
        # стоп-кадр: нулевая команда, отменяем таймер
        self.cmd_pub.publish(Twist())
        self.timer.cancel()
        # корректно завершаем узел и всё rclpy
        self.destroy_node()
        rclpy.shutdown()


# ─────────────────────────── main ───────────────────────────
def main():
    rclpy.init()

    # CLI-аргументы
    v   = float(sys.argv[1]) if len(sys.argv) > 1 else 0.2
    t   = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    odom_topic = sys.argv[3] if len(sys.argv) > 3 else '/diff_drive_controller/odom'
    cmd_topic  = sys.argv[4] if len(sys.argv) > 4 else '/diff_drive_controller/cmd_vel_unstamped'

    node = DriveTest(v, t, odom_topic, cmd_topic)
    rclpy.spin(node)          # блокирующий цикл – завершится в finish()


if __name__ == '__main__':
    main()
