import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from yhs_can_interfaces.msg import CtrlCmd, SteeringCtrlCmd
from std_msgs.msg import Float32MultiArray, Bool, String
from rclpy.qos import qos_profile_sensor_data
import tf_transformations
import math
import time

from .submodules.utils import *
from .submodules.parameters import *
from .submodules.enums import *
    
class ControlNode(Node):
    def __init__(self):
        super().__init__('mobile_control')
        self.create_subscription(Odometry, '/odom3', self.odom_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/target', self.command_callback, 10)  # 목표 위치 및 자세 구독
        self.ctrl_cmd_pub = self.create_publisher(CtrlCmd, 'ctrl_cmd', 10)
        self.steering_cmd_pub = self.create_publisher(SteeringCtrlCmd, 'steering_ctrl_cmd', 10)
        self.pub_arrival_flag = self.create_publisher(String, '/unity/cmd', 10)
        self.mobile_move_pub = self.create_publisher(Bool, '/mobile/move_flag', 10)

        self.timer = self.create_timer(1.0 / Hz, self.timer_callback)

        ### Position 객체 ###
        self.Pos = Position()       # Robot의 Odom 위치
        self.StartPos = Position()  # 전진 명령을 시작할 때의 위치
        self.CmdPos = Command()    # 목표 위치

        self.state = State.Stop
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.target_distance = 0.0
        self.init_odom_state = False

        self.lateral_direction = 0
        self.init_rotate_angle = None
        self.scan_rotate_start_time = None
        self.lateral_start_time = None

    def reset(self):
        self.state = State.Stop
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.target_distance = 0.0
        self.init_odom_state = False

        self.lateral_direction = 0
        self.init_rotate_angle = None
        self.scan_rotate_start_time = None
        self.lateral_start_time = None

    def odom_callback(self, msg):
        # 현재 위치와 자세 업데이트
        self.Pos.x = msg.pose.pose.position.x
        self.Pos.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.Pos.theta = math.degrees(yaw)

        self.init_odom_state = True

    def command_callback(self, msg):
        # 목표 위치 및 자세 업데이트
        if len(msg.data) >= 5:
            self.CmdPos = Command()
            self.CmdPos.gearSetting = msg.data[0]
            self.CmdPos.x = msg.data[1]
            self.CmdPos.y = msg.data[2]
            self.CmdPos.theta = msg.data[3]
            self.CmdPos.height = msg.data[4]
            self.CmdPos.paintMode = True if msg.data[5] == 1 else False
            self.get_logger().info('-------------------------- Target received --------------------------')
            self.get_logger().info(f'Position: {[self.CmdPos.x, self.CmdPos.y]}, Theta: {self.CmdPos.theta}')
            if(self.CmdPos.gearSetting == Gear.Rotate):
                self.state = State.ScanRotate
                self.CmdPos.theta = self.Pos.theta
                self.CmdPos.scanMode = True

            elif(self.CmdPos.gearSetting == Gear.Disable):
                self.state = State.Stop
                self.get_logger().info('-------------------------- Disabled --------------------------')
                self.get_logger().info('Mobile_Disable')
                
            else:
                if(math.sqrt((self.CmdPos.x - self.Pos.x) ** 2  + (self.CmdPos.y - self.Pos.y) ** 2) < 0.15):
                    self.state = State.FinalRotate
                else:
                    self.state = State.InitialRotate

    def timer_callback(self):
        if(self.init_odom_state == True):
            self.control()

    def control(self):
        if self.state == State.Stop:
            self.PublishCtrlCmd()
            self.reset()

        elif self.state == State.ScanRotate:
            if self.scan_rotate_start_time is None:
                self.get_logger().info('-------------------------- RotateScan Start --------------------------')
                self.scan_rotate_start_time = self.get_clock().now().nanoseconds
                
            elapsed_time = (self.get_clock().now().nanoseconds - self.scan_rotate_start_time) / 1e9  # 경과 시간 계산
            # self.get_logger().info("RotateScan : {elapsed_time}/30.0[s]")
            if elapsed_time >= 30:  # 30초가 경과했는지 확인
                self.get_logger().info('-------------------------- RotateScan Finish --------------------------')
                self.scan_rotate_start_time = None
                self.state = State.FinalRotate  # 상태를 FinalRotate으로 변경
                
            else:
                self.PublishCtrlCmd(gear=Gear.Differential, angular_speed=ScanRotateSpeed)  # 각속도를 12도로 설정
                
        elif self.state == State.InitialRotate:
            if(self.CmdPos.gearSetting == Gear.Differential):
                self.Rotate(RelativeAngle(self.Pos, self.CmdPos))
                if abs(RelativeAngle(self.Pos, self.CmdPos) - self.Pos.theta) <= ThetaErrorBoundary:
                    self.state = State.MoveForward
                    self.StartPos.x = self.Pos.x
                    self.StartPos.y = self.Pos.y
                    self.target_distance = RelativeDistance(self.CmdPos, self.StartPos)
                    self.current_linear_speed = 0
                    self.current_angular_speed = 0
                    self.get_logger().info('-------------------------- InitialRotate Finish --------------------------')
                    self.get_logger().info(f'각도 : {self.Pos.theta} / {RelativeAngle(self.Pos, self.CmdPos)}')

                    self.PublishCtrlCmd()  # 최종적으로 속도 0으로 설정
                    time.sleep(1)
                    
            elif(self.CmdPos.gearSetting == Gear.Lateral):

                if(self.lateral_direction == 0):
                    angle_to_target = RelativeAngle(self.Pos, self.CmdPos)
                    _angle_error = NormalizeAngle(angle_to_target - self.Pos.theta)

                    self.lateral_direction = 1 if _angle_error >= 0 else -1  # 1 일때 +y 방향, -1 일때 -y 방향

                if(self.init_rotate_angle == None):
                    self.init_rotate_angle = NormalizeAngle(RelativeAngle(self.Pos, self.CmdPos) - (90 * self.lateral_direction))


                self.Rotate(self.init_rotate_angle)
                if abs(NormalizeAngle(self.init_rotate_angle - self.Pos.theta)) <= ThetaErrorBoundary:
                    self.state = State.MoveLateral
                    self.StartPos.x = self.Pos.x
                    self.StartPos.y = self.Pos.y
                    self.target_distance = RelativeDistance(self.CmdPos, self.StartPos)
                    self.current_linear_speed = 0
                    self.current_angular_speed = 0
                    self.get_logger().info('-------------------------- InitialRotate Finish --------------------------')
                    self.get_logger().info(f'각도 : {self.Pos.theta} / {self.init_rotate_angle}[deg]')
                    self.init_rotate_angle = None
                    self.PublishCtrlCmd()  # 최종적으로 속도 0으로 설정
                    time.sleep(1)



        elif self.state == State.MoveForward:
            current_distance = RelativeDistance(self.Pos, self.StartPos)
            _error = self.target_distance - current_distance
            
            if _error <= 0.01:  # 목표 거리 도달 시
                self.current_linear_speed = 0
                self.current_angular_speed = 0
                self.PublishCtrlCmd()  # 최종적으로 속도 0으로 설정
                self.get_logger().info('-------------------------- MoveForward Finish --------------------------')
                self.get_logger().info(f'거리 : {current_distance} / {self.target_distance}[m]')
                self.state = State.FinalRotate
                time.sleep(1)
            else:
                # 목표 선속도 계산 (최대 선속도로 제한)
                _target_linear_speed = LinearXSpeedLimit(self.PControl(_error, Kp = LinearKp))
                
                if(_target_linear_speed > 0 and self.current_linear_speed < _target_linear_speed):
                    self.current_linear_speed += 0.002  # 속도를 천천히 증가
                elif(_target_linear_speed < 0 and self.current_linear_speed > _target_linear_speed):
                    self.current_linear_speed -= 0.002  # 속도를 천천히 감소
                else:
                    self.current_linear_speed = _target_linear_speed

                # 선속도를 목표 속도로 설정
                self.PublishCtrlCmd(gear = Gear.Differential, linear_speed = LinearXSpeedLimit(self.current_linear_speed))

        elif self.state == State.MoveLateral:
            current_distance = RelativeDistance(self.Pos, self.StartPos)
            _error = self.target_distance - current_distance
            
            if _error <= 0.01:  # 목표 거리 도달 시
                self.current_linear_speed = 0
                self.current_angular_speed = 0
                self.PublishCtrlCmd()  # 최종적으로 속도 0으로 설정
                self.get_logger().info('-------------------------- MoveLateral Finish --------------------------')
                self.get_logger().info(f'거리 : {current_distance} / {self.target_distance}[m]')
                self.state = State.FinalRotate
                self.lateral_direction = 0
                self.lateral_start_time = None
                time.sleep(1)
            else:

                if self.lateral_start_time is None:
                    self.lateral_start_time = self.get_clock().now().nanoseconds
                    
                elapsed_time = (self.get_clock().now().nanoseconds - self.lateral_start_time) / 1e9

                if elapsed_time < 1:
                    self.PublishCtrlCmd(gear = Gear.Lateral, linear_speed = 0)
                    
                else:
                    # 목표 선속도 계산 (최대 선속도로 제한)
                    _target_linear_speed = self.lateral_direction * LinearYSpeedLimit(self.PControl(_error, Kp = LinearKp))
                    
                    # 선속도 점진적 증가 로직
                    if self.current_linear_speed < _target_linear_speed:
                        self.current_linear_speed += 0.01  # 속도를 천천히 증가
                    elif self.current_linear_speed > _target_linear_speed:
                        self.current_linear_speed -= 0.01  # 속도를 천천히 감소
                    else:
                        self.current_linear_speed = _target_linear_speed
                    # 선속도를 목표 속도로 설정
                    self.PublishCtrlCmd(gear = Gear.Lateral, linear_speed = LinearYSpeedLimit(self.current_linear_speed))


        elif self.state == State.FinalRotate:
            self.Rotate(NormalizeAngle(self.CmdPos.theta))
            if abs(NormalizeAngle(self.CmdPos.theta - self.Pos.theta)) <= ThetaErrorBoundary:
                self.PublishCtrlCmd()  # 최종적으로 속도 0으로 설정
                self.state = State.Stop
                self.get_logger().info('-------------------------- Target Arrived --------------------------')
                self.get_logger().info(f'목표 위치 : {self.CmdPos.x, self.CmdPos.y}, {self.CmdPos.theta}[deg]')
                self.get_logger().info(f'현재 위치 : {self.Pos.x, self.Pos.y}, {self.Pos.theta}[deg]')
                self.get_logger().info(f'error : {RelativeDistance(self.CmdPos, self.Pos)}[m], {self.CmdPos.theta - self.Pos.theta}[deg]')
                if(self.CmdPos.paintMode):
                    self.pub_arrival_flag.publish(String(data = 'mobile_arrived;' + str(self.CmdPos.height)))
                else:
                    self.mobile_move_pub.publish(Bool(data = True))
                self.CmdPos = Command()
                time.sleep(1)

    def Rotate(self, _desired_theta):
        _error = NormalizeAngle(_desired_theta - self.Pos.theta)
        _angular_speed = self.PControl(_error, Kp = AngularKp)

        _target_angular_speed = self.PControl(_error, Kp = AngularKp)

        if(_target_angular_speed > 0 and self.current_angular_speed < _target_angular_speed):
            self.current_angular_speed += 0.07  # 속도를 천천히 증가
        elif(_target_angular_speed < 0 and self.current_angular_speed > _target_angular_speed):
            self.current_angular_speed -= 0.07  # 속도를 천천히 감소
        else:
            self.current_angular_speed = _target_angular_speed


        if(self.CmdPos.scanMode):
            _angular_speed = AngularSpeedLimit(_angular_speed, maxSpeed = ScanRotateSpeed)
        else:
            _angular_speed = self.current_angular_speed

        self.PublishCtrlCmd(gear = Gear.Differential, angular_speed = _angular_speed)

    def PControl(self, error, Kp=1):
        return Kp * error

    def PublishCtrlCmd(self, gear = Gear.Disable, linear_speed = 0, angular_speed=0):
        if(gear == Gear.Disable):
            ctrl_cmd = CtrlCmd()
            ctrl_cmd.ctrl_cmd_gear = Gear.Disable

            steering_cmd = SteeringCtrlCmd()
            steering_cmd.ctrl_cmd_gear = Gear.Disable

            self.ctrl_cmd_pub.publish(ctrl_cmd)
            self.steering_cmd_pub.publish(steering_cmd)

        elif(gear == Gear.Differential):
            ctrl_cmd = CtrlCmd()
            ctrl_cmd.ctrl_cmd_gear = gear
            ctrl_cmd.ctrl_cmd_x_linear = -LinearXSpeedLimit(linear_speed)
            ctrl_cmd.ctrl_cmd_z_angular = AngularSpeedLimit(angular_speed)
            self.ctrl_cmd_pub.publish(ctrl_cmd)

        elif(gear == Gear.Lateral):
            steering_cmd = SteeringCtrlCmd()
            steering_cmd.ctrl_cmd_gear = 7
            steering_cmd.steering_ctrl_cmd_steering = float(90)
            steering_cmd.steering_ctrl_cmd_velocity = -LinearYSpeedLimit(linear_speed)
            self.steering_cmd_pub.publish(steering_cmd)

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
