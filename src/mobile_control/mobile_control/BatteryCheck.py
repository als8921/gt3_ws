import rclpy
from rclpy.node import Node
from yhs_can_interfaces.msg import ChassisInfoFb
from std_msgs.msg import Int32
import time

class BmsPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.subscriber = self.create_subscription(ChassisInfoFb, '/chassis_info_fb', self.callback, 10)
        self.publisher = self.create_publisher(Int32, '/mobile/battery', 10)

    def callback(self, msg):
        bms_flag_value = int(msg.bms_flag_fb.bms_flag_fb_soc)
        self.publisher.publish(Int32(data=bms_flag_value))
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)

    bms_publisher = BmsPublisher()
    rclpy.spin(bms_publisher)

    bms_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
