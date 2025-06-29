import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2
import struct

class MockCameraPublisher(Node):
    def __init__(self):
        super().__init__('mock_camera_publisher')
        self.rgb_pub = self.create_publisher(Image, 'camera_color_image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera_depth_image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()

    def timer_callback(self):
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(rgb, (200, 150), (400, 350), (0, 0, 255), -1)  # red box = obstacle
        depth = np.full((480, 640), 2.0, dtype=np.float32)
        self.rgb_pub.publish(self.bridge.cv2_to_imgmsg(rgb, 'rgb8'))
        self.depth_pub.publish(self.bridge.cv2_to_imgmsg(depth, '32FC1'))

class MockLidarPublisher(Node):
    def __init__(self):
        super().__init__('mock_lidar_publisher')
        self.pub = self.create_publisher(PointCloud2, 'lidar_points', 10)
        self.timer = self.create_timer(0.2, self.publish_mock)

    def publish_mock(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "lidar"

        # Create 2D point cloud of (x, y) obstacle points in front of robot
        points = []
        for angle in np.linspace(-np.pi / 4, np.pi / 4, 30):
            dist = np.random.uniform(1.0, 4.0)
            x = dist * np.cos(angle)
            y = dist * np.sin(angle)
            z = 0.0
            points.append([x, y, z])

        data = b''.join([struct.pack('fff', *pt) for pt in points])

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.data = data

        self.pub.publish(msg)

def main():
    rclpy.init()
    camera = MockCameraPublisher()
    lidar = MockLidarPublisher()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(camera)
    executor.add_node(lidar)
    executor.spin()
    camera.destroy_node()
    lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()