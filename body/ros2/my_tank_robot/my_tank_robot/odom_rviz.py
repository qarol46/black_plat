from math import sin, cos
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class OdomToTFPublisher(Node):

    def __init__(self):
        super().__init__('odom_to_tf_publisher')

        qos_profile = QoSProfile(depth=10)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        # Use sim time if needed, проверяем наличие параметра
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.use_sim_time = self.get_parameter('use_sim_time').value
        if self.use_sim_time:
            self.get_logger().info("Using simulation time")

        # Подписываемся на топик /odom
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.handle_odom_msg,
            qos_profile)

    def handle_odom_msg(self, msg):
        # Создаём сообщение TransformStamped для передачи преобразования
        odom_trans = TransformStamped()
        odom_trans.header.stamp = msg.header.stamp  # Используем временную метку из сообщения /odom
        odom_trans.header.frame_id = 'map'  # Рамка родителя
        odom_trans.child_frame_id = 'chassis_link'  # Дочерняя рамка

        # Переносим данные о положении из сообщения /odom
        odom_trans.transform.translation.x = msg.pose.pose.position.x
        odom_trans.transform.translation.y = msg.pose.pose.position.y
        odom_trans.transform.translation.z = msg.pose.pose.position.z
        odom_trans.transform.rotation = msg.pose.pose.orientation

        # Отправляем преобразование
        self.broadcaster.sendTransform(odom_trans)

def main():
    rclpy.init()
    node = OdomToTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
