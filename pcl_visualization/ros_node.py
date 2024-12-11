# my_ros_node.py
import rclpy
import numpy as np
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2

class ROSNode(Node):
    def __init__(self):
        super().__init__('my_ros_node')

        # Trigger 서비스 클라이언트 생성
        self.lift_client = self.create_client(Trigger, 'lift_command')

        self.pointcloud_sub = self.create_subscription(PointCloud2, '/pointcloud_topic', self.point_cloud_callback, 10)
        self.points = []

        # 서비스가 준비될 때까지 대기
        # while not self.lift_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('서비스를 기다리는 중...')

    def point_cloud_callback(self, msg):
        point_step = msg.point_step
        data = np.frombuffer(msg.data, dtype=np.float32)

        # 포인트 수 계산
        num_points = len(data) // (point_step // 4)
        self.points = []
        for i in range(num_points):
            y = -data[i * point_step // 4]  # x 좌표
            x = data[i * point_step // 4 + 1]  # y 좌표
            z = data[i * point_step // 4 + 2]  # z 좌표
            self.points.append((x, y, z))

    def rotate_points(self, roll, pitch, yaw):
        # 라디안으로 변환
        roll = np.radians(roll)
        pitch = np.radians(pitch)
        yaw = np.radians(yaw)

        # 회전 행렬 정의
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])

        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])

        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])

        # 전체 회전 행렬
        R = R_z @ R_y @ R_x

        # 각 점 회전
        rotated_points = []
        for point in self.points:
            rotated_point = R @ np.array(point)
            rotated_points.append(rotated_point)

        self.points = rotated_points  # 회전된 점으로 업데이트

    def send_lift_command(self):
        request = Trigger.Request()  # Trigger 요청 생성

        # 서비스 요청 보내기
        future = self.lift_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'리프트 명령이 성공적으로 전송되었습니다: {future.result().message}')
        else:
            self.get_logger().error('리프트 명령 호출 실패')

    def spin(self):
        rclpy.spin(self)

def start_ros_node():
    rclpy.init()
    node = ROSNode()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()
