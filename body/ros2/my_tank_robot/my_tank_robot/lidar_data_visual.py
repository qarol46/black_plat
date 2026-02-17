import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_publisher')
        self.publisher = self.create_publisher(PointCloud2, '/visualized_pointcloud', 10)
        self.timer = self.create_timer(1.0, self.publish_pointcloud)  # Публикуем точки каждую секунду

        # Загрузка и предобработка точек из файла
        self.points = self.load_points('points.txt')
        self.published_once = False

    def load_points(self, filename):
        points = []
        try:
            with open(filename, 'r') as f:
                for line in f:
                    x, y, z = map(float, line.split())
                    # Округляем координаты до ближайшего 0.05
                    x = round(x / 0.05) * 0.05
                    y = round(y / 0.05) * 0.05
                    z = round(z / 0.05) * 0.05
                    points.append([x, y, z])
        except FileNotFoundError:
            self.get_logger().error(f"File {filename} not found.")
            return []

        # Удаление дубликатов на основе округленных значений (оставляем только уникальные точки)
        unique_points = {tuple(point): point for point in points}.values()
        return list(unique_points)

    def publish_pointcloud(self):
        if not self.points:
            self.get_logger().info("No points available for publishing.")
            return

        # Создаем заголовок сообщения
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # Убедитесь, что frame_id соответствует используемой карте
        #header.frame_id = 'odom'  # Убедитесь, что frame_id соответствует используемой карте



        # Создаем сообщение PointCloud2
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        cloud_msg = pc2.create_cloud(header, fields, self.points)

        # Публикуем PointCloud2 сообщение
        self.publisher.publish(cloud_msg)
        if not self.published_once:
            self.get_logger().info(f'Published {len(self.points)} points.')
            self.published_once = True


def main(args=None):
    rclpy.init(args=args)
    pointcloud_publisher = PointCloudPublisher()

    try:
        rclpy.spin(pointcloud_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        pointcloud_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
