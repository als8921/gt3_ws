# my_ros_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyROSNode(Node):
    def __init__(self):
        super().__init__('my_ros_node')
        self.publisher_ = self.create_publisher(String, 'topic_name', 10)
        self.subscription = self.create_subscription(String, 'topic_name', self.listener_callback, 10)

    def listener_callback(self, msg):
        print(f"Received: {msg.data}")

    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

    def spin(self):
        rclpy.spin(self)

def start_ros_node():
    rclpy.init()
    node = MyROSNode()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()
