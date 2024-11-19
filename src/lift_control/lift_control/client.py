import rclpy
from rclpy.node import Node
from lift_srv.srv import LiftCommand
import tkinter as tk
from tkinter import messagebox

class CommandClient(Node):
    def __init__(self):
        super().__init__('command_client')
        self.client = self.create_client(LiftCommand, 'lift_command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스를 기다리는 중...')

    def send_request(self, command, value):
        request = LiftCommand.Request()
        request.command = command
        request.value = value
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class LiftControlUI:
    def __init__(self):
        self.client = CommandClient()
        self.root = tk.Tk()
        self.root.title("Lift Control")

        # 입력 칸 추가
        self.value_label = tk.Label(self.root, text="Value:")
        self.value_label.pack(pady=5)
        self.value_entry = tk.Entry(self.root)
        self.value_entry.pack(pady=5)

        # 버튼 생성
        self.move_button = tk.Button(self.root, text="MOVE", command=self.move_lift)
        self.move_button.pack(pady=10)

        self.stop_button = tk.Button(self.root, text="STOP", command=self.stop_lift)
        self.stop_button.pack(pady=10)

        self.reset_button = tk.Button(self.root, text="RESET", command=self.reset_lift)
        self.reset_button.pack(pady=10)

        self.homing_button = tk.Button(self.root, text="HOMING", command=self.homing_lift)
        self.homing_button.pack(pady=10)

        self.setrpm_button = tk.Button(self.root, text="SET RPM", command=self.set_rpm)
        self.setrpm_button.pack(pady=10)

        self.setcurrent_button = tk.Button(self.root, text="SET CURRENT", command=self.set_current)
        self.setcurrent_button.pack(pady=10)

    def get_value(self):
        try:
            return float(self.value_entry.get())
        except ValueError:
            messagebox.showerror("Input Error", "유효한 숫자를 입력하세요.")
            return None

    def move_lift(self):
        value = self.get_value()
        if value is not None:
            response = self.client.send_request("MOVE", value)
            # self.show_response(response)

    def stop_lift(self):
        response = self.client.send_request("STOP", 0.0)
        # self.show_response(response)

    def reset_lift(self):
        response = self.client.send_request("RESET", 0.0)
        # self.show_response(response)

    def homing_lift(self):
        response = self.client.send_request("HOMING", 0.0)
        # self.show_response(response)

    def set_rpm(self):
        value = self.get_value()
        if value is not None:
            response = self.client.send_request("SETRPM", value)
            # self.show_response(response)

    def set_current(self):
        value = self.get_value()
        if value is not None:
            response = self.client.send_request("SETCURRENT", value)
            # self.show_response(response)

    def show_response(self, response):
        if response is not None:
            messagebox.showinfo("Response", f"응답 상태: {response.status}")
        else:
            messagebox.showerror("Error", "결과를 받을 수 없습니다.")

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    ui = LiftControlUI()
    ui.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
