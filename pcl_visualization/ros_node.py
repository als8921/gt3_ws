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

        self.pointcloud_sub = self.create_subscription(PointCloud2, '/pointcloud_topic', self.point_cloud_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom3', self.odom_callback, 10)
        self.robotpose_sub = self.create_subscription(String, '/current_pose', self.pose_callback, 10)
        self.points = []
        self.robot_pos = [0, 0, 0, 0, 0, 0]     # [x, y, z, roll, pitch, yaw]
        self.arm_pos = [0, 0, 0, 0, 0, 0]       # [x, y, z, roll, pitch, yaw]

    def pose_callback(self, msg):
        x, y, z, r, p, yaw = eval(msg.data.split(";")[0])
        self.arm_pos = [x / 1000, y / 1000, z / 1000, 0, 0, 0]
        print(r, p, yaw)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        x, y, z = pos.x, pos.y, pos.z
        r, p, yaw = tf_transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        self.robot_pos = [x, y, z, r, p, yaw]

    def point_cloud_callback(self, msg):
        point_step = msg.point_step
        data = np.frombuffer(msg.data, dtype=np.float32)

        # 포인트 수 계산
        num_points = len(data) // (point_step // 4)
        points = []
        for i in range(num_points):
            # 포인트 데이터를 가져옵니다.
            x = data[i * point_step // 4]
            y = data[i * point_step // 4 + 1]
            z = data[i * point_step // 4 + 2]
            points.append((x, y, z))

        # 방금 계산한 끝점 위치를 구하고 포인트 클라우드를 변환합니다.
        end_effector_position = self.calculate_end_effector_position()
        self.points = self.transform_points(points, end_effector_position)

    def calculate_end_effector_position(self):
        # 로봇의 현재 위치
        robot_x, robot_y, robot_z, robot_r, robot_p, robot_yaw = self.robot_pos
        # 로봇팔 원점의 상대 위치 (여기서는 예시로 설정)
        arm_offset = self.arm_pos  # [x, y, z, roll, pitch, yaw]

        # 로봇팔 원점의 절대 좌표 계산
        arm_origin_x = robot_x + arm_offset[0]
        arm_origin_y = robot_y + arm_offset[1]
        arm_origin_z = robot_z + arm_offset[2]

        # 로봇팔 끝점의 상대 위치 (여기서는 예시로 설정)
        end_offset = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]

        # 로봇팔 끝점의 절대 좌표 계산
        end_effector_x = arm_origin_x + end_offset[0]
        end_effector_y = arm_origin_y + end_offset[1]
        end_effector_z = arm_origin_z + end_offset[2]

        # 자세 (orientation) 계산
        end_effector_r = robot_r + arm_offset[3] + end_offset[3]
        end_effector_p = robot_p + arm_offset[4] + end_offset[4]
        end_effector_yaw = robot_yaw + arm_offset[5] + end_offset[5]

        return (end_effector_x, end_effector_y, end_effector_z, end_effector_r, end_effector_p, end_effector_yaw)

    def transform_points(self, points, end_effector_position):
        # 끝점 위치와 자세를 분리
        end_x, end_y, end_z, end_r, end_p, end_yaw = end_effector_position

        # 회전 행렬을 계산
        rotation_matrix = self.get_rotation_matrix(end_r, end_p, end_yaw)

        # 변환된 포인트 리스트
        transformed_points = []

        for point in points:
            # 포인트에 끝점의 위치를 더하고 회전
            rotated_point = rotation_matrix @ np.array(point)
            transformed_point = rotated_point + np.array([end_x, end_y, end_z])
            transformed_points.append(transformed_point)

        return transformed_points

    def get_rotation_matrix(self, roll, pitch, yaw):

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
        return R_z @ R_y @ R_x

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
