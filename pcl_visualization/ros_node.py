
# my_ros_node.py
import rclpy
import numpy as np
from rclpy.node import Node
from lift_srv.srv import LiftCommand
from sensor_msgs.msg import PointCloud2

class ROSNode(Node):
    def __init__(self):
        super().__init__('my_ros_node')

        self.lift_node = rclpy.create_node('lift_cmd')
        self.lift_client = self.lift_node.create_client(LiftCommand, 'lift_command')


        self.pointcloud_sub = self.create_subscription(PointCloud2, '/pointcloud_topic', self.point_cloud_callback, 10)
        self.points = []

        # while not self.lift_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('서비스를 기다리는 중...')

    def point_cloud_callback(self, msg):
        point_step = msg.point_step
        data = np.frombuffer(msg.data, dtype=np.float32)

        # 포인트 수 계산
        num_points = len(data) // (point_step // 4)
        self.points = []
        for i in range(num_points):
            x = data[i * point_step // 4]  # x 좌표
            y = data[i * point_step // 4 + 1]  # y 좌표
            z = data[i * point_step // 4 + 2]  # z 좌표
            self.points.append((x, y, z))


    def lift_service_request(self, command, value):
        request = LiftCommand.Request()
        request.command = command
        request.value = value
        self.lift_client.call_async(request)

    def send_lift_command(self, command, value):
        self.lift_service_request(command, value)

    

    def spin(self):
        rclpy.spin(self)

def start_ros_node():
    rclpy.init()
    node = ROSNode()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()
