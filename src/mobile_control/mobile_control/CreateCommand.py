import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, String
from collections import deque
import math

class Gear:
    Disable = 0
    Parking = 1
    Neutral = 2
    Differential = 6
    Lateral = 8

class AnglePublisher(Node):
    def __init__(self):
        super().__init__('angle_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/target', 10)
        self.bool_subscriber = self.create_subscription(Bool, 'trigger_topic', self.bool_callback, 10)
        self.string_subscriber = self.create_subscription(String, 'input_string_topic', self.string_callback, 10)
        self.queue = deque()  # 결과를 저장할 큐

    def string_callback(self, msg: String):
        # 수신한 문자열을 파싱하여 큐에 추가
        try:
            x1, y1, x2, y2 = map(float, msg.data.split(';'))
            angle = calculate_perpendicular_intersection(x1, y1, x2, y2)
            self.queue.append((Gear.Differential, x1, y1, angle))
            self.queue.append((Gear.Lateral, x2, y2, angle))
            self.get_logger().info(f'Parsed and added to queue: {x1}, {y1}, {angle}; {x2}, {y2}, {angle}')
        except ValueError:
            self.get_logger().error('Invalid input format. Expected format: "x1;y1;x2;y2"')

    def bool_callback(self, msg: Bool):
        if msg.data:  # True가 들어온 경우
            if self.queue:
                # 큐에서 하나 꺼내서 퍼블리시
                gear, x, y, angle = self.queue.popleft()
                msg = Float32MultiArray()
                msg.data = [float(gear), x, y, angle]
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: {msg.data}')

def NormalizeAngle(angle):
    return (angle + 180) % 360 - 180

def RelativeAngle(a, b):
    return NormalizeAngle(math.degrees(math.atan2(b.y - a.y, b.x - a.x)))

def calculate_perpendicular_intersection(x1, y1, x2, y2):
    interPosition = Position()

    if x2 - x1 == 0:  # 수직선
        interPosition.x = x1
        interPosition.y = 0
    elif y1 - y2 == 0:  # 수평선
        interPosition.x = 0
        interPosition.y = y1
    else:
        m = (y2 - y1) / (x2 - x1)  # 기울기
        n = y1 - m * x1  # y절편
        interPosition.x = n / ((-1 / m) - m)  # 교차점 x
        interPosition.y = (-1 / m) * interPosition.x  # 교차점 y

    return RelativeAngle(Position(), interPosition)

class Position:
    def __init__(self, x=0.0, y=0.0):
        self.x = x  # [m]
        self.y = y  # [m]

def main(args=None):
    rclpy.init(args=args)
    angle_publisher = AnglePublisher()
    rclpy.spin(angle_publisher)
    angle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
