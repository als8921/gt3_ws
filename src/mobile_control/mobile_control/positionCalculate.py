import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from yhs_can_interfaces.msg import CtrlCmd
from std_msgs.msg import Float32MultiArray  # Float32MultiArray 메시지 추가
import tf_transformations
import math
import time

def normalize_angle(angle):
    return (angle + 180) % 360 - 180

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.create_subscription(Odometry, '/odom', self.listener_callback, 10)
        self.create_subscription(Float32MultiArray, '/target', self.target_callback, 10)  # 목표 위치 및 자세 구독
        self.publisher = self.create_publisher(CtrlCmd, 'ctrl_cmd', 10)

        # 초기 목표 위치 및 자세 설정
        self.target_position = (0.0, 0.0)   # 목표 위치 (X, Y)
        self.target_theta = 0.0              # 목표 자세 (Degree)

        # 최대 속도 및 각속도 설정
        self.maxlinear_speed = 0.1    # 최대 선속도 (m/s)
        self.maxangular_speed = 30     # 최대 각속도 (deg/s)

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30Hz

        self.current_position = (0.0, 0.0)
        self.current_theta = 0.0

        # 상태 변수 추가
        self.state = 'idle'  # 초기 상태를 idle로 설정

        # 이동 거리 및 시작 위치 변수
        self.target_distance = 0.0
        self.start_position = (0.0, 0.0)

    def listener_callback(self, msg):
        # 현재 위치와 자세 업데이트
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_theta = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        self.current_theta = math.degrees(self.current_theta)

    def target_callback(self, msg):
        # 목표 위치 및 자세 업데이트
        if len(msg.data) >= 3:
            self.target_position = (msg.data[0], msg.data[1])  # 목표 위치 (X, Y)
            self.target_theta = msg.data[2]                     # 목표 자세 (Degree)
            self.state = 'rotate_to_target'                     # 상태 변경
            self.get_logger().info(f'Target received: Position: {self.target_position}, Theta: {self.target_theta}')

    def LimitSpeed(self, speed, maxSpeed=0):
        return float(max(-maxSpeed, min(speed, maxSpeed)))

    def timer_callback(self):
        if self.state == 'rotate_to_target':
            self.Rotate(normalize_angle(math.degrees(math.atan2(self.target_position[1] - self.current_position[1],
                                                                  self.target_position[0] - self.current_position[0]))))
            if abs(normalize_angle(math.degrees(math.atan2(self.target_position[1] - self.current_position[1],
                                                               self.target_position[0] - self.current_position[0]))) - self.current_theta) <= 0.1:
                self.state = 'move_forward'
                self.start_position = self.current_position  # 시작 위치 저장
                self.target_distance = math.sqrt((self.target_position[0] - self.start_position[0]) ** 2 +
                                                  (self.target_position[1] - self.start_position[1]) ** 2)

                self.PublishCtrlCmd()  # 최종적으로 속도 0으로 설정
                time.sleep(1)

        elif self.state == 'move_forward':
            current_distance = math.sqrt((self.current_position[0] - self.start_position[0]) ** 2 +
                                         (self.current_position[1] - self.start_position[1]) ** 2)
            _error = self.target_distance - current_distance
            
            if abs(_error) <= 0.01:  # 목표 거리 도달 시
                self.PublishCtrlCmd()  # 최종적으로 속도 0으로 설정
                self.get_logger().info(f'MoveForward Finish {self.target_distance:.2f}[m]')
                self.state = 'rotate_to_final_theta'
                time.sleep(1)
            else:
                _linear_speed = self.PControl(_error, Kp=1)  # 선속도 계산
                self.PublishCtrlCmd(linear_speed=_linear_speed)  # 속도 명령 발행

        elif self.state == 'rotate_to_final_theta':
            self.Rotate(normalize_angle(self.target_theta))
            if abs(normalize_angle(self.target_theta - self.current_theta)) <= 0.1:
                self.PublishCtrlCmd()  # 최종적으로 속도 0으로 설정
                self.state = 'finished'
                self.get_logger().info('목표 위치 도달.')
                time.sleep(1)

    def Rotate(self, _desired_theta):
        _error = normalize_angle(_desired_theta - self.current_theta)
        _angular_speed = self.PControl(_error, Kp=1)
        self.PublishCtrlCmd(angular_speed=_angular_speed)

    def PControl(self, error, Kp=1):
        return Kp * error

    def PublishCtrlCmd(self, linear_speed=0, angular_speed=0):
        ctrl_cmd = CtrlCmd()
        ctrl_cmd.ctrl_cmd_gear = 6  # 4T4D 기어 설정
        ctrl_cmd.ctrl_cmd_x_linear = self.LimitSpeed(linear_speed, maxSpeed=self.maxlinear_speed)
        ctrl_cmd.ctrl_cmd_y_linear = 0.0
        ctrl_cmd.ctrl_cmd_z_angular = self.LimitSpeed(angular_speed, maxSpeed=self.maxangular_speed)
        self.get_logger().info(f'Linear Speed: {linear_speed:.2f}[m/s], Angular Speed: {ctrl_cmd.ctrl_cmd_z_angular:.2f}[deg/s]')
        self.publisher.publish(ctrl_cmd)

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
