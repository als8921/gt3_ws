# my_ros_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lift_srv.srv import LiftCommand

class ROSNode(Node):
    def __init__(self):
        super().__init__('my_ros_node')
        self.publisher_ = self.create_publisher(String, 'topic_name', 10)
        self.subscription = self.create_subscription(String, 'topic_name', self.listener_callback, 10)

        self.lift_node = rclpy.create_node('lift_cmd')
        self.lift_client = self.lift_node.create_client(LiftCommand, 'lift_command')

        # while not self.lift_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('서비스를 기다리는 중...')

    def listener_callback(self, msg):
        print(f"Received: {msg.data}")

    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

    def lift_service_request(self, command, value):
        request = LiftCommand.Request()
        request.command = command
        request.value = value
        self.lift_client.call_async(request)

    def send_lift_command(self, command, value):
        self.lift_service_request(command, value)

    

    def spin(self):
        rclpy.spin(self)

def start_ros_node():
    rclpy.init()
    node = ROSNode()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()
