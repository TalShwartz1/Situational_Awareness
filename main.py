import rclpy, torch, struct
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import numpy as np

class FusionMapper(Node):
    def __init__(self):
        super().__init__('fusion_mapper')
        self.rgb = self.depth = self.lidar = None
        self.bridge = CvBridge()
        self.model = torch.hub.load('pytorch/vision', 'deeplabv3_resnet50', pretrained=True).eval().cuda()
        self.create_subscription(Image, 'camera_color_image_raw', lambda m: setattr(self, 'rgb', self.bridge.imgmsg_to_cv2(m, 'rgb8')), 10)
        self.create_subscription(Image, 'camera_depth_image_raw', lambda m: setattr(self, 'depth', self.bridge.imgmsg_to_cv2(m, '32FC1')), 10)
        self.create_subscription(PointCloud2, 'lidar_points', lambda m: setattr(self, 'lidar', m), 10)
        self.pub = self.create_publisher(OccupancyGrid, 'map', 10)
        self.create_timer(0.2, self.update)

    def update(self):
        grid = np.full((100, 100), -1, np.int8)
        if self.rgb is not None and self.depth is not None:
            img = torch.from_numpy(self.rgb).permute(2,0,1).unsqueeze(0).float().cuda() / 255
            with torch.no_grad(): mask = self.model(img)['out'].argmax(1).squeeze().cpu().numpy()
            for y in range(0, 480, 10):
                for x in range(0, 640, 10):
                    d = self.depth[y, x]
                    if 1.0 < d < 5.0:
                        gx, gy = int(x / 640 * 100), int(y / 480 * 100)
                        if 0 <= gx < 100 and 0 <= gy < 100:
                            grid[gy, gx] = 100 if mask[y, x] else 0
        if self.lidar:
            for i in range(0, len(self.lidar.data), self.lidar.point_step * 5):
                try:
                    x, y = struct.unpack_from('ff', self.lidar.data, i)
                    if 1.0 < np.hypot(x, y) < 5.0:
                        gx, gy = int(x*10+50), int(y*10+50)
                        if 0 <= gx < 100 and 0 <= gy < 100:
                            grid[gy, gx] = 100
                            for r in np.linspace(0, 1, 5):
                                fx, fy = int(x*r*10+50), int(y*r*10+50)
                                if 0 <= fx < 100 and 0 <= fy < 100 and grid[fy, fx] == -1: grid[fy, fx] = 0
                except:
                    pass

        msg = OccupancyGrid()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), "map"
        msg.info.resolution, msg.info.width, msg.info.height = 0.1, 100, 100
        msg.info.origin.position.x = msg.info.origin.position.y = -5.0
        msg.data = grid.flatten().tolist()
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(FusionMapper())
    rclpy.shutdown()

if __name__ == '__main__':
    main()