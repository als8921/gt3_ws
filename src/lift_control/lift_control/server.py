import rclpy
import time
from rclpy.node import Node
from lift_srv.srv import LiftCommand

class LiftServiceServer(Node):
    def __init__(self):
        super().__init__('lift_service_server')
        self.srv = self.create_service(LiftCommand, 'lift_command', self.handle_request)

    def handle_request(self, request, response):
        self.get_logger().info(f'명령: {request.command}, 값: {request.value}')
        
        # 명령에 따른 처리
        if request.command == "MOVE":
            # 예시로 단순히 상태를 설정
            response.status = True
        else:
            response.status = False
        return response

def main(args=None):
    rclpy.init(args=args)
    lift_service_server = LiftServiceServer()
    rclpy.spin(lift_service_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
