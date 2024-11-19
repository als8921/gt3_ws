import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yhs_can_interfaces.msg import CtrlCmd

class CtrlCmdToTwistNode(Node):
    def __init__(self):
        super().__init__('ctrl_cmd_to_twist_node')
        
        # /cmd_vel 토픽으로 Twist 메시지를 퍼블리시하기 위한 퍼블리셔 생성
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # CtrlCmd 메시지를 구독하기 위한 서브스크라이버 생성
        self.create_subscription(CtrlCmd, 'ctrl_cmd', self.ctrl_cmd_callback, 10)  # CtrlCmd 메시지를 받는 토픽 이름으로 변경하세요.

    def ctrl_cmd_callback(self, ctrl_cmd):
        # Twist 메시지 생성
        twist = Twist()
        
        # CtrlCmd에서 값을 가져와서 Twist에 설정
        twist.linear.x = -ctrl_cmd.ctrl_cmd_x_linear
        twist.linear.y = ctrl_cmd.ctrl_cmd_y_linear  # Y축 속도는 0으로 설정
        twist.angular.z = ctrl_cmd.ctrl_cmd_z_angular * 3.141592 / 180.0  # 각속도 설정
        
        self.get_logger().info(f'Publishing: Linear Speed: {twist.linear.x:.2f}[m/s], Angular Speed: {ctrl_cmd.ctrl_cmd_z_angular:.2f}[deg/s]')
        # /cmd_vel 토픽에 Twist 메시지 퍼블리시
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CtrlCmdToTwistNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
