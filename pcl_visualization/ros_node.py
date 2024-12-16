import rclpy
import numpy as np
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
import tf_transformations

class ROSNode(Node):
    def __init__(self):
        super().__init__('my_ros_node')

        # Trigger 서비스 클라이언트 생성
        self.lift_client = self.create_client(Trigger, 'lift_command')

        self.pointcloud_sub = self.create_subscription(PointCloud2, '/pointcloud_topic', self.point_cloud_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom3', self.odom_callback, 10)
        self.robotpose_sub = self.create_subscription(String, '/current_pose', self.pose_callback, 10)
        self.points = []
        

        # 서비스가 준비될 때까지 대기
        # while not self.lift_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('서비스를 기다리는 중...')
        self.robot_pos = [0, 0, 0, 0, 0, 0]     # [x, y, z, roll, pitch, yaw]
        self.arm_pos = [0, 0, 0, 0, 0, 0]       # [x, y, z, roll, pitch, yaw]

    def pose_callback(self, msg):
        x, y, z, r, p, y = eval(msg.data.split(";")[0])
        self.arm_pos = [x, y, z, r, p, y]

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        x, y, z = pos.x, pos.y, pos.z
        r, p, y = tf_transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        self.robot_pos = [x, y, z, r, p, y]

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

    def calculate_end_effector_position(self):
        # 로봇의 현재 위치
        robot_x, robot_y, robot_z, robot_r, robot_p, robot_y = self.robot_pos
        # 로봇팔 원점의 상대 위치 (여기서는 예시로 설정)
        arm_x, arm_y, arm_z, arm_r, arm_p, arm_y = self.arm_pos  # [x, y, z, roll, pitch, yaw]

        # 로봇팔 원점의 절대 좌표 계산
        arm_origin_x = robot_x + arm_x
        arm_origin_y = robot_y + arm_y
        arm_origin_z = robot_z + arm_z

        # 로봇팔의 상대 위치
        end_offset = [0.3, 0.0, 0.5]  # [x, y, z, roll, pitch, yaw]

        # 로봇팔 끝점의 절대 좌표 계산
        end_effector_x = arm_origin_x + end_offset[0]
        end_effector_y = arm_origin_y + end_offset[1]
        end_effector_z = arm_origin_z + end_offset[2]

        # 자세 (orientation) 계산
        end_effector_r = robot_r + arm_r + end_offset[3]
        end_effector_p = robot_p + arm_p + end_offset[4]
        end_effector_y = robot_y + arm_y + end_offset[5]

        return (end_effector_x, end_effector_y, end_effector_z, end_effector_r, end_effector_p, end_effector_y)

    def rotate_points(self, points, roll, pitch, yaw):
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
        for point in points:
            rotated_point = R @ np.array(point)
            rotated_points.append(rotated_point)

        return rotated_points  # 회전된 점으로 업데이트

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

if __name__ == '__main__':
    start_ros_node()
