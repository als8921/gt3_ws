import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry

import tf2_ros
import tf_transformations

class ROSNode(Node):
    def __init__(self):
        super().__init__('my_ros_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)



        ##### PUBLISH
        self.mobile_move_pub = self.create_publisher(Bool, '/mobile/move_flag', 10)
        self.front_info_pub = self.create_publisher(Float32MultiArray, '/front_scan/info', 10)
        self.rear_info_pub = self.create_publisher(Float32MultiArray, '/rear_scan/info', 10)

        ##### SUBSCRIBE
        self.mobile_arrival_sub = self.create_subscription(Bool, '/mobile/arrival_flag', self.mobile_arrived_callback, 10)
        self.pointcloud_sub = self.create_subscription(PointCloud2, '/pointcloud_topic', self.point_cloud_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom3', self.odom_callback, 10)
        self.rear_scan_sub = self.create_subscription(LaserScan, '/rear_scan', self.rear_scan_callback, 10)
        self.front_scan_sub = self.create_subscription(LaserScan, '/front_scan', self.front_scan_callback, 10)

        self.points = []
        self.rgb_array = []
        self.odom_pos = [0, 0, 0, 0, 0, 0]          # [x, y, z, roll, pitch, yaw]
        self.end_effector_pos = [0, 0, 0, 0, 0, 0]  # [x, y, z, roll, pitch, yaw]

        self.isArrived = False

        # LaserScan 데이터 저장
        self.rear_scan = None
        self.front_scan = None

    def front_scan_info_pub(self, msg):
        self.front_info_pub.publish(Float32MultiArray(data = msg))

    def rear_scan_info_pub(self, msg):
        self.rear_info_pub.publish(Float32MultiArray(data = msg))

    def mobile_arrived_callback(self, msg):
        self.isArrived = msg.data

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        x, y, z = pos.x, pos.y, pos.z
        r, p, yaw = tf_transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        self.odom_pos = [x, y, z, r, p, yaw]

    def point_cloud_callback(self, msg):
        point_step = msg.point_step
        data = np.frombuffer(msg.data, dtype=np.float32)

        # 포인트 수 계산
        num_points = len(data) // (point_step // 4)
        points = []
        rgb_array = []
        for i in range(num_points):
            x = data[i * point_step // 4]
            y = data[i * point_step // 4 + 1]
            z = data[i * point_step // 4 + 2]
            rgb = data[i * point_step // 4 + 3]
            points.append((x, y, z))
            rgb_array.append(rgb)

        self.points = points
        self.rgb_array = rgb_array
        self.end_effector_pos = self.calculate_end_effector_position()

    def rear_scan_callback(self, msg):
        self.rear_scan = msg

    def front_scan_callback(self, msg):
        self.front_scan = msg

    def calculate_end_effector_position(self):
        try:
            target_frame = "tcp_camera_link"
            source_frame = "base_link"
            transform = self.tf_buffer.lookup_transform(source_frame, target_frame, rclpy.time.Time())
            
            x, y, z = transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z
            r, p, yaw = tf_transformations.euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])

        except:
            print(f"{source_frame} 에서 {target_frame} 까지의 TF를 불러올 수 없음.")
            x, y, z, r, p, yaw = 0, 0, 0, 0, 0, 0

        # 로봇의 현재 위치
        robot_x, robot_y, robot_z, robot_r, robot_p, robot_yaw = self.odom_pos

        # 로봇팔 원점의 절대 좌표 계산
        arm_origin_x = robot_x + x
        arm_origin_y = robot_y + y
        arm_origin_z = robot_z + z

        # 로봇팔 끝점의 상대 위치 (여기서는 예시로 설정)
        end_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]

        # 로봇팔 끝점의 절대 좌표 계산
        end_effector_x = arm_origin_x + end_offset[0]
        end_effector_y = arm_origin_y + end_offset[1]
        end_effector_z = arm_origin_z + end_offset[2]

        # 자세 (orientation) 계산
        end_effector_r = robot_r + r + end_offset[3]
        end_effector_p = robot_p + p + end_offset[4]
        end_effector_yaw = robot_yaw + yaw + end_offset[5]

        return (end_effector_x, end_effector_y, end_effector_z, end_effector_r, end_effector_p, end_effector_yaw)

    def spin(self):
        rclpy.spin(self)

if __name__ == '__main__':
    rclpy.init()
    node = ROSNode()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()
