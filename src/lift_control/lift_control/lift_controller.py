import time
import threading
import serial
import serial.rs485
from .submodules.MD400 import MD400
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from lift_srv.srv import LiftCommand

SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 19200

class RS485Communication:
    def __init__(self):
        self.ser = serial.rs485.RS485(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)
        self.ser.rs485_mode = serial.rs485.RS485Settings()
        self.command_queue = deque()
        self.lift_position = None
        self.position_publisher = None

    def read_data(self):
        if self.ser.readable():
            response = bytearray()
            response.extend(self.ser.read())
            if response and response[0] == md400.TMID:
                response.extend(self.ser.read(4))
                additional_read = response[-1] + 1
                response.extend(self.ser.read(additional_read))
                if response[3] == 197:
                    self.lift_position = md400.bytes_to_pos(response)
                else:
                    self.lift_position = None
                
                if self.lift_position is not None:
                    self.position_publisher.publish(Float32(data=float(self.lift_position)))

    def send_data(self, data):
        try:
            self.ser.write(data)
        except Exception as e:
            self.get_logger().info(f"Error sending data: {e}")

    def run(self):
        while True:
            if self.command_queue:
                packet = self.command_queue.popleft()
                self.send_data(packet)
                self.read_data()
            else:
                packet = md400.get_pos()
                self.send_data(packet)
                self.read_data()
                time.sleep(0.05)

    def close(self):
        self.ser.close()

class LiftServiceServer(Node):
    def __init__(self, rs485_comm):
        super().__init__('lift_service_server')
        self.rs485_comm = rs485_comm
        self.srv = self.create_service(LiftCommand, 'lift_command', self.service_callback)
        self.get_logger().info('lift Service init**')

    def service_callback(self, request, response):
        self.get_logger().info(f'{request.command}, {request.value}')
        response.status = False
        if request.command == "MOVE":
            self.get_logger().info(f'Setting position to: {request.value}')
            if(request.value >= 0):
                self.rs485_comm.command_queue.append(md400.set_direction_ccw())
            else:
                self.rs485_comm.command_queue.append(md400.set_direction_cw())
            self.rs485_comm.command_queue.append(md400.set_pos(abs(request.value)))
            response.status = True

        elif request.command == "STOP":
            self.rs485_comm.command_queue = deque()
            self.rs485_comm.command_queue.append(md400.stop())
            self.get_logger().info('LIFT STOP')
            response.status = True

        elif request.command == "RESET":
            self.rs485_comm.command_queue.append(md400.set_alarm_reset())
            self.get_logger().info('LIFT RESET')
            response.status = True

        elif request.command == "HOMING":
            self.rs485_comm.command_queue.append(md400.homing())
            self.get_logger().info('LIFT HOMING')
            response.status = True

        elif request.command == "SETRPM":
            self.rs485_comm.command_queue.append(md400.set_rpm(request.value))
            self.get_logger().info(f'SET RPM to {request.value}[rpm]')
            response.status = True

        elif request.command == "SETCURRENT":
            self.rs485_comm.command_queue.append(md400.set_current_limit(request.value))
            self.get_logger().info(f'SET CURRENT to {request.value}[A]')
            response.status = True
        return response

def main(args=None):
    rclpy.init(args=args)
    global md400
    md400 = MD400(rmid=0xb7, tmid=0xb8, id=0x01)

    rs485_comm = RS485Communication()                   # 패킷을 주고 받는 Serial 통신 Class
    lift_service_server = LiftServiceServer(rs485_comm) # Ros Service 통신을 위한 Class

    rs485_comm.position_publisher = lift_service_server.create_publisher(Float32, '/gt3/lift_position', 10)

    # RS485 통신을 위한 스레드 시작
    rs485_thread = threading.Thread(target=rs485_comm.run, daemon=True)
    rs485_thread.start()

    try:
        rclpy.spin(lift_service_server)
    finally:
        rs485_comm.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
