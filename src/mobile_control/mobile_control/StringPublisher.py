import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import tkinter as tk

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.bool_publisher = self.create_publisher(Bool, 'trigger_topic', 10)
        self.string_publisher = self.create_publisher(String, 'input_string_topic', 10)

        # UI 설정
        self.root = tk.Tk()
        self.root.title("ROS2 Publisher")
        self.root.geometry("1200x500")  # 창 크기 설정


        # String 전송 입력란
        self.string_entry = tk.Entry(self.root, width=30, font=('Arial', 14))
        self.string_entry.pack(pady=10)

        self.string_button = tk.Button(self.root, text="Send String", command=self.send_string, width=20, height=2)
        self.string_button.pack(pady=10)

        # Bool 전송 버튼
        self.bool_button = tk.Button(self.root, text="Send True", command=self.send_true, width=20, height=2)
        self.bool_button.pack(pady=10)


        self.reset_button = tk.Button(self.root, text="Reset", command=self.reset, width=20, height=2)
        self.reset_button.pack(pady=10)

    def reset(self):
        msg = Bool()
        msg.data = False
        self.bool_publisher.publish(msg)
        self.get_logger().info('Published: False')

    def send_true(self):
        msg = Bool()
        msg.data = True
        self.bool_publisher.publish(msg)
        self.get_logger().info('Published: True')


    def send_string(self):
        msg = String()
        msg.data = self.string_entry.get()
        self.string_publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    publisher_node = PublisherNode()
    try:
        publisher_node.run()
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
