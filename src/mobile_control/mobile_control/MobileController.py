import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from yhs_can_interfaces.msg import CtrlCmd
from std_msgs.msg import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data
import tf_transformations
import math
import time
from enum import Enum

### Parameters ###
Hz = 30
LinearKp = 0.73
AngularKp = 1.5

MinXLinearSpeed = 0.0       # [m/s]    후진기능을 넣을 시 음수로 전환
MaxXLinearSpeed = 0.5       # [m/s]
MaxYLinearSpeed = 0.3       # [m/s]
MaxAngularSpeed = 30        # [deg/s]

ThetaErrorBoundary = 1      # 각도 명령 허용 오차
####################

class State(Enum):
    Stop = "Stop"
    InitialRotate = 0
    MoveForward = 1
    MoveLateral = 2
    FinalRotate = 3

class Gear:
    Disable = 0
    Parking = 1
    Neutral = 2
    Differential = 6
    Lateral = 8

class Position:
    def __init__(self):
        self.x = 0.0        # [m]
        self.y = 0.0        # [m]
        self.theta = 0.0    # [deg]

class Command(Position):
    def __init__(self):
        super().__init__()
        self.gearSetting = Gear.Differential

def NormalizeAngle(angle):
    return (angle + 180) % 360 - 180

def AngularSpeedLimit(speed, maxSpeed = MaxAngularSpeed):
    return float(max(-maxSpeed, min(speed, maxSpeed)))

def LinearXSpeedLimit(speed, minSpeed = MinXLinearSpeed, maxSpeed = MaxXLinearSpeed):
    return float(max(minSpeed, min(speed, maxSpeed)))

def LinearYSpeedLimit(speed, minSpeed = -MaxYLinearSpeed, maxSpeed = MaxYLinearSpeed):
    return float(max(minSpeed, min(speed, maxSpeed)))

def RelativeDistance(a : Position, b : Position):
    return math.sqrt((b.x - a.x)**2 + (b.y - a.y)**2)

def RelativeAngle(a : Position, b : Position):
    return NormalizeAngle(math.degrees(math.atan2(b.y - a.y, b.x - a.x)))
    
class ControlNode(Node):
    def __init__(self):
        super().__init__('mobile_control')
        self.create_subscription(Odometry, '/odom3', self.odom_cb, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/target', self.command_cb, 10)  # 목표 위치 및 자세 구독
        self.pub_command = self.create_publisher(CtrlCmd, 'ctrl_cmd', 10)

        self.timer = self.create_timer(1.0 / Hz, self.timer_callback)

        ### Position 객체 ###
        self.Pos = Position()       # Robot의 Odom 위치
        self.StartPos = Position()  # 전진 명령을 시작할 때의 위치
        self.CmdPos = Command()    # 목표 위치

        self.current_linear_speed = 0.0
        self.target_distance = 0.0
        self.state = State.Stop


    def odom_cb(self, msg):
        # 현재 위치와 자세 업데이트
        self.Pos.x = msg.pose.pose.position.x
        self.Pos.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.Pos.theta = math.degrees(yaw)

    def command_cb(self, msg):
        # 목표 위치 및 자세 업데이트
        if len(msg.data) >= 4:
            self.CmdPos.gearSetting = msg.data[0]
            self.CmdPos.x = msg.data[1]
            self.CmdPos.y = msg.data[2]
            self.CmdPos.theta = msg.data[3]

            self.get_logger().info(f'Target received: Position: {[self.CmdPos.x, self.CmdPos.y]}, Theta: {self.CmdPos.theta}')
            if(math.sqrt((self.CmdPos.x - self.Pos.x) ** 2  + (self.CmdPos.y - self.Pos.y) ** 2) < 0.15):
                self.state = State.FinalRotate
            else:
                self.state = State.InitialRotate

    def timer_callback(self):
        if self.state == State.Stop:
            self.PublishCtrlCmd(0, 0)  # 최종적으로 속도 0으로 설정
        elif self.state == State.InitialRotate:
            if(self.CmdPos.gearSetting == Gear.Differential):
                self.Rotate(RelativeAngle(self.Pos, self.CmdPos))
                if abs(RelativeAngle(self.Pos, self.CmdPos) - self.Pos.theta) <= ThetaErrorBoundary:
                    self.state = State.MoveForward
                    self.StartPos.x = self.Pos.x
                    self.StartPos.y = self.Pos.y
                    self.target_distance = RelativeDistance(self.CmdPos, self.StartPos)
                    self.current_linear_speed = 0
                    self.get_logger().info(f'InitialRotate Finish, MoveForward{self.target_distance}[m]')

                    self.PublishCtrlCmd()  # 최종적으로 속도 0으로 설정

                    # time.sleep(0.5)
                    
            elif(self.CmdPos.gearSetting == Gear.Lateral):
                self.Rotate(NormalizeAngle(self.CmdPos.theta))
                if abs(NormalizeAngle(self.CmdPos.theta - self.Pos.theta)) <= ThetaErrorBoundary:
                    self.state = State.MoveLateral
                    self.StartPos.x = self.Pos.x
                    self.StartPos.y = self.Pos.y
                    self.target_distance = RelativeDistance(self.CmdPos, self.StartPos)
                    self.current_linear_speed = 0
                    self.get_logger().info(f'InitialRotate Finish, MoveLateral{self.target_distance}[m]')
                    self.PublishCtrlCmd()  # 최종적으로 속도 0으로 설정
                    # time.sleep(0.5)



        elif self.state == State.MoveForward:
            current_distance = RelativeDistance(self.Pos, self.StartPos)
            _error = self.target_distance - current_distance
            
            if _error <= 0.01:  # 목표 거리 도달 시
                self.current_linear_speed = 0
                self.PublishCtrlCmd()  # 최종적으로 속도 0으로 설정
                self.get_logger().info(f'MoveForward Finish {self.target_distance:.2f}[m]')
                self.state = State.FinalRotate
                # time.sleep(0.5)
            else:
                # 목표 선속도 계산 (최대 선속도로 제한)
                _target_linear_speed = LinearXSpeedLimit(self.PControl(_error, Kp = LinearKp))
                
                # 선속도 점진적 증가 로직
                if self.current_linear_speed < _target_linear_speed:
                    self.current_linear_speed += 0.01  # 속도를 천천히 증가
                elif self.current_linear_speed > _target_linear_speed:
                    self.current_linear_speed -= 0.01  # 속도를 천천히 감소
                else:
                    self.current_linear_speed = _target_linear_speed

                # 선속도를 목표 속도로 설정
                self.PublishCtrlCmd(gear = Gear.Differential, linear_speed = LinearXSpeedLimit(self.current_linear_speed))

        elif self.state == State.MoveLateral:
            current_distance = RelativeDistance(self.Pos, self.StartPos)
            _error = self.target_distance - current_distance
            
            if _error <= 0.01:  # 목표 거리 도달 시
                self.current_linear_speed = 0
                self.PublishCtrlCmd()  # 최종적으로 속도 0으로 설정
                self.get_logger().info(f'MoveLateral Finish {self.target_distance:.2f}[m]')
                self.state = State.FinalRotate
                # time.sleep(0.5)
            else:
                # 목표 선속도 계산 (최대 선속도로 제한)
                _target_linear_speed = LinearYSpeedLimit(self.PControl(_error, Kp = LinearKp))
                
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
                self.get_logger().info('목표 위치 도달.')
                self.get_logger().info(f'목표 위치 : {self.CmdPos.x, self.CmdPos.y, self.CmdPos.theta}')
                self.get_logger().info(f'현재 위치 : {self.Pos.x, self.Pos.y, self.Pos.theta}')
                self.get_logger().info(f'error_Distance : {RelativeDistance(self.CmdPos, self.Pos)}[m]')
                self.get_logger().info(f'error_Theta : {self.CmdPos.theta - self.Pos.theta}[deg]')
                self.get_logger().info(f'FinalRotate Finish')
                # time.sleep(0.5)

    def Rotate(self, _desired_theta):
        _error = NormalizeAngle(_desired_theta - self.Pos.theta)
        _angular_speed = self.PControl(_error, Kp = AngularKp)
        self.PublishCtrlCmd(gear = Gear.Differential, angular_speed = _angular_speed)

    def PControl(self, error, Kp=1):
        return Kp * error

    def PublishCtrlCmd(self, gear = Gear.Disable, linear_speed = 0, angular_speed=0):
        ctrl_cmd = CtrlCmd()
        ctrl_cmd.ctrl_cmd_gear = gear

        if(gear == Gear.Differential):
            ctrl_cmd.ctrl_cmd_x_linear = -LinearXSpeedLimit(linear_speed)
            ctrl_cmd.ctrl_cmd_z_angular = AngularSpeedLimit(angular_speed)
        elif(gear == Gear.Lateral):
            ctrl_cmd.ctrl_cmd_y_linear = LinearYSpeedLimit(linear_speed)
        # self.get_logger().info(f'{(self.Pos.x, self.Pos.y)}[m], {self.Pos.theta:.2f}[deg]')
        # self.get_logger().info(f'{linear_speed:.2f}[m/s], {ctrl_cmd.ctrl_cmd_z_angular:.2f}[deg/s]')
        self.pub_command.publish(ctrl_cmd)

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
