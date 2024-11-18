import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
from tkinter import messagebox

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, '/target', 10)

    def publish_target(self, x, y, theta):
        msg = Float32MultiArray()
        msg.data = [x, y, theta]  # 목표 위치 (X, Y) 및 자세 (Theta)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published target: {msg.data}')

class TargetControlUI:
    def __init__(self):
        self.publisher = TargetPublisher()
        self.root = tk.Tk()
        self.root.title("Target Publisher")

        # 입력 칸 추가
        tk.Label(self.root, text="Target X:").grid(row=0, column=0)
        tk.Label(self.root, text="Target Y:").grid(row=1, column=0)
        tk.Label(self.root, text="Target Theta:").grid(row=2, column=0)

        self.target_x = tk.Entry(self.root)
        self.target_y = tk.Entry(self.root)
        self.target_theta = tk.Entry(self.root)

        self.target_x.grid(row=0, column=1)
        self.target_y.grid(row=1, column=1)
        self.target_theta.grid(row=2, column=1)

        # 버튼 생성
        self.publish_button = tk.Button(self.root, text="Publish Target", command=self.publish_target)
        self.publish_button.grid(row=3, columnspan=2)

        # 종료 버튼
        self.quit_button = tk.Button(self.root, text="Quit", command=self.quit)
        self.quit_button.grid(row=4, columnspan=2)

    def get_value(self, entry):
        try:
            return float(entry.get())
        except ValueError:
            messagebox.showerror("Input Error", "유효한 숫자를 입력하세요.")
            return None

    def publish_target(self):
        x = self.get_value(self.target_x)
        y = self.get_value(self.target_y)
        theta = self.get_value(self.target_theta)

        if x is not None and y is not None and theta is not None:
            self.publisher.publish_target(x, y, theta)

    def quit(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    ui = TargetControlUI()

    # 메인 루프에서 ROS2 spin 실행
    try:
        while rclpy.ok():
            ui.run()
            rclpy.spin_once(ui.publisher)
    finally:
        ui.quit()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
