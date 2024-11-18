import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from yhs_can_interfaces.msg import CtrlCmd
import tf_transformations
import math
import time

def normalize_angle(angle):
    return (angle + 180) % 360 - 180

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.create_subscription(Odometry, '/odom', self.listener_callback, 10)
        self.publisher = self.create_publisher(CtrlCmd, 'ctrl_cmd', 10)
        
        # 목표 위치 및 자세 설정
        self.target_position = (1, 1)   # 목표 위치 (X, Y)
        self.target_theta = 0           # 목표 자세 (Degree)

        # 최대 속도 및 각속도 설정
        self.maxlinear_speed = 0.2     # 최대 선속도 (m/s)
        self.maxangular_speed = 30     # 최대 각속도 (deg/s)

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30Hz

        self.current_position = (0.0, 0.0)
        self.current_theta = 0.0

    def listener_callback(self, msg):
        # 현재 위치와 자세 업데이트
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_theta = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        self.current_theta = math.degrees(self.current_theta)
    
    def LimitSpeed(self, speed, minSpeed = 0, maxSpeed = 0):
        return float(max(minSpeed, min(speed, maxSpeed)))

        

    def ControlTask(self, x_d, y_d, theta_d):
        """
        X, Y, Theta 명령을 받았을 때 지령을 생성하기
        1. 목표 좌표를 바라보기
        2. 목표 좌표까지 이동
        3. 목표 각도로 회전
        """
        self.Rotate(normalize_angle(math.degrees(math.atan2(y_d - self.current_position[1], x_d - self.current_position[0]))))
        self.MoveForward(math.sqrt((self.current_position[0] - x_d)**2 + (self.current_position[1] - y_d)**2))
        self.Rotate(normalize_angle(theta_d))


    def Rotate(self, _desired_theta):
        _error = _desired_theta - self.current_theta
        print(_error)
        while(abs(_error) > 0.1):
            _angular_speed = self.PControl(_error, Kp = 1)
            self.PublishCtrlCmd(angular_speed = _angular_speed)
            time.sleep(1.0 / 30.0)
        self.PublishCtrlCmd()
        # time.sleep(1)
        self.get_logger().info(f'Rotate Finish {_desired_theta}[deg]')
            
    def MoveForward(self, _distance):
        startX, startY = self.current_position
        _error = _distance - math.sqrt((self.current_position[0] - startX)**2 + (self.current_position[1] - startY)**2)
        while(abs(_error) > 0.01):
            _linear_speed = self.PControl(_error, Kp = 1)
            self.PublishCtrlCmd(linear_speed = _linear_speed)
            time.sleep(1.0 / 30.0)
        self.PublishCtrlCmd()
        # time.sleep(1)
        self.get_logger().info(f'MoveForward Finish {_distance}[m]')

    def PControl(self, error, Kp = 1):
        return Kp * error

    def PublishCtrlCmd(self, linear_speed = 0, angular_speed = 0):
        ctrl_cmd = CtrlCmd()
        ctrl_cmd.ctrl_cmd_gear = 6  # 4T4D 기어 설정
        ctrl_cmd.ctrl_cmd_x_linear = self.LimitSpeed(linear_speed, maxSpeed = self.maxlinear_speed)
        ctrl_cmd.ctrl_cmd_z_angular = self.LimitSpeed(angular_speed, maxSpeed = self.maxangular_speed)
        # self.get_logger().info(f'POS : [{self.current_position}, {self.current_theta:.2f} deg], Linear Speed: {linear_speed:.2f}[m/s], Angular Speed: {angular_speed:.2f}[deg/s]')
        self.publisher.publish(ctrl_cmd)


    def timer_callback(self):
        self.ControlTask(1, 1, 90)
        self.ControlTask(0, 0, 0)




def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
