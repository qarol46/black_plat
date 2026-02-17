import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TrackController(Node):
    def __init__(self):
        super().__init__('track_controller')

        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.front_left_angle = 0.0
        self.front_right_angle = 0.0
        self.rear_left_angle = 0.0
        self.rear_right_angle = 0.0
        self.direction = 1

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'front_left_flipper_j', 'front_right_flipper_j', 
            'rear_left_flipper_j', 'rear_right_flipper_j'
        ]

        point = JointTrajectoryPoint()
        point.positions = [
            self.front_left_angle, self.front_right_angle,
            self.rear_left_angle, self.rear_right_angle
        ]
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points = [point]

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing joint trajectory: {msg.joint_names} positions: {point.positions}')

        # Изменяем угол для плавного движения
        self.front_left_angle += self.direction * 0.01
        self.front_right_angle -= self.direction * 0.01
        self.rear_left_angle += self.direction * 0.01
        self.rear_right_angle -= self.direction * 0.01

        if self.front_left_angle > 0.3 or self.front_left_angle < -0.3:
            self.direction *= -1

def main(args=None):
    rclpy.init(args=args)
    node = TrackController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
