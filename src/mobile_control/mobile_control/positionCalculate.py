import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from yhs_can_interfaces.msg import CtrlCmd
import tf_transformations
import math

def normalize_angle(angle):
    return (angle + 180) % 360 - 180

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.create_subscription(Odometry, '/odom', self.listener_callback, 10)
        self.publisher = self.create_publisher(CtrlCmd, 'ctrl_cmd', 10)
        
        # 목표 위치 및 자세 설정
        self.target_position = (0, 0)   # 목표 위치 (X, Y)
        self.target_yaw = 0             # 목표 자세 (Degree)

        # 최대 속도 및 각속도 설정
        self.max_linear_speed = 0.1  # 최대 선속도 (m/s)
        self.max_angular_speed = 30  # 최대 각속도 (deg/s)

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30Hz

        self.current_position = (0.0, 0.0)
        self.current_yaw = 0.0

    def listener_callback(self, msg):
        # 현재 위치와 자세 업데이트
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        self.current_yaw = math.degrees(self.current_yaw)

    def timer_callback(self):
        # 목표 위치와 현재 위치의 차이 계산
        target_x, target_y = self.target_position
        current_x, current_y = self.current_position

        # 목표 방향 계산
        angle_to_target = math.degrees(math.atan2(target_y - current_y, target_x - current_x))
        distance_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

        # 선속도 및 각속도 계산
        if distance_to_target > 0.1:  # 목표 위치에 도달하지 않았을 때
            # 목표 방향과 현재 방향의 차이 계산
            angle_diff = normalize_angle(angle_to_target - self.current_yaw)
            
            # 각속도 계산
            angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angle_diff))
            
            # 각도가 30도 이상 차이나면 회전만 하도록 설정
            if abs(angle_diff) > 30:  # 30도 이상 차이나면
                linear_speed = 0.0  # 전진 속도 0
            else:
                linear_speed = min(distance_to_target, self.max_linear_speed)

        else:
            # 목표 위치에 도달한 경우
            linear_speed = 0.0
            
            # 목표 자세로 정렬하기
            angle_diff = normalize_angle(self.target_yaw - self.current_yaw)
            print(self.target_yaw, self.current_yaw)
            if abs(angle_diff) > 1:  # 1도 이상 차이나면 회전
                angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angle_diff))
            else:
                angular_speed = 0.0  # 각도가 정렬되면 각속도 0


        # CtrlCmd 메시지 생성
        ctrl_cmd = CtrlCmd()
        ctrl_cmd.ctrl_cmd_gear = 6  # 4T4D 기어 설정
        ctrl_cmd.ctrl_cmd_x_linear = float(linear_speed)
        ctrl_cmd.ctrl_cmd_z_angular = float(angular_speed)  # °/s로 변환
        ctrl_cmd.ctrl_cmd_y_linear = 0.0  # Y축 속도는 0으로 설정

        self.get_logger().info(f'POS : [{current_x},{current_y}, {self.current_yaw:.2f} deg], Linear Speed: {linear_speed:.2f}[m/s], Angular Speed: {angular_speed:.2f}[deg/s]')
        # 메시지 발행
        self.publisher.publish(ctrl_cmd)

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
