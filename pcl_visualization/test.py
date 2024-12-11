import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/pointcloud_topic',
            self.point_cloud_callback,
            10
        )     
        self.points = []

    def point_cloud_callback(self, msg):
        # PointCloud2 메시지를 NumPy 배열로 변환
        vertices = self.pointcloud2_to_numpy(msg)
        vertices = np.transpose(vertices)
        print(vertices[2])

    def pointcloud2_to_numpy(self, cloud_msg):
        # cloud_msg의 data 배열을 numpy 배열로 변환
        dtype_list = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
        cloud_arr = np.frombuffer(cloud_msg.data, dtype=dtype_list)
        return cloud_arr





def main(args=None):
    rclpy.init(args=args)
    pointcloud_subscriber = PointCloudSubscriber()
    rclpy.spin(pointcloud_subscriber)
    pointcloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
