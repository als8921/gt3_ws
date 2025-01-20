import rclpy
from rclpy.node import Node
from yhs_can_interfaces.msg import ChassisInfoFb
from std_msgs.msg import Int32
import time

class BmsPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.subscriber = self.create_subscription(ChassisInfoFb, '/chassis_info_fb', self.callback, 10)
        self.battery_pub = self.create_publisher(Int32, '/mobile/battery', 10)
        self.temperature_pub = self.create_publisher(Int32, '/mobile/temperature', 10)

    def callback(self, msg):
        battery_value = int(msg.bms_flag_fb.bms_flag_fb_soc)
        battery_temperature = int(msg.bms_flag_fb.bms_flag_fb_hight_temperature)
        self.battery_pub.publish(Int32(data=battery_value))
        self.temperature_pub.publish(Int32(data=battery_temperature))

def main(args=None):
    rclpy.init(args=args)

    bms_publisher = BmsPublisher()
    rclpy.spin(bms_publisher)

    bms_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
