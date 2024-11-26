import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, String
from collections import deque
import numpy as np

D_horizontal = 0.5  # [m] 작업 위치 수평 거리
D_vertical = 1.0    # [m] 작업 위치 수직 거리   
D_task = 1.0        # [m] 작업 사이의 거리

class Gear:
    Disable = 0
    Parking = 1
    Neutral = 2
    Differential = 6
    Lateral = 8

class CommandPositionPublisher(Node):
    def __init__(self):
        super().__init__('command_position_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/target', 10)
        self.bool_subscriber = self.create_subscription(Bool, 'trigger_topic', self.bool_callback, 10)
        self.string_subscriber = self.create_subscription(String, 'unity/cmd', self.string_callback, 10)
        self.queue = deque()  # 결과를 저장할 큐


    def CreateCommandPositionQueue(self, x1, y1, x2, y2, _D_horizontal, _D_vertical, _D_task):
        # (x2 - x1, y2 - y1) 벡터 정의 및 정규화
        move_vector = np.array([x2 - x1, y2 - y1], dtype=float)
        move_vector_mag = np.linalg.norm(move_vector)
        move_vector_norm = move_vector / move_vector_mag
        
        # 외적 방향 벡터 계산
        z_unit_vector = np.array([0, 0, 1], dtype=float)
        direction_vector = np.cross(move_vector, z_unit_vector)[:2]
        direction_vector = direction_vector / np.linalg.norm(direction_vector)
        
        # 새로운 위치 계산
        new_pos = np.array([x1, y1], dtype=float) + move_vector_norm * _D_horizontal + direction_vector * _D_vertical
        theta_degrees = np.degrees(np.arctan2(-direction_vector[1], -direction_vector[0]))

        position_queue = deque()
        position_queue.append((Gear.Differential, new_pos[0], new_pos[1], theta_degrees))

        for i in range(1, int((move_vector_mag - 2 * _D_horizontal) // _D_task) + 1):
            current_position = new_pos + move_vector_norm * (_D_task * i)
            position_queue.append((Gear.Lateral, current_position[0], current_position[1], theta_degrees))

        # 마지막 위치 추가 (모듈로 연산으로 인한 위치)
        if (move_vector_mag - 2 * _D_horizontal) % _D_task != 0:
            last_position = new_pos + move_vector_norm * (move_vector_mag - 2 * _D_horizontal)
            position_queue.append((Gear.Lateral, last_position[0], last_position[1], theta_degrees))
        
        return position_queue
            
    def string_callback(self, msg):
        try:
            # [1, (-2.46, 0.00, -1.52), (-1.73, 0.00, -0.89), (-1.73, 0.52, -0.89)],[2, (-2.46, 0.00, -1.52), (-1.73, 0.00, -0.89), (-1.73, 0.52, -0.89)] 형태
            data_list = msg.data.split("],[")
            data_list[0] = data_list[0][1:]
            data_list[-1] = data_list[-1][:-1]

            for item in data_list:
                item = "[" + item + "]"
                idx, startpos, endpos, height = eval(item)
                index = int(idx)
                h = float(height[1])

                x1, y1, x2, y2 = float(startpos[0]), float(startpos[2]), float(endpos[0]), float(endpos[2])
                self.get_logger().info(f"{index} : {x1}, {y1}, {x2}, {y2}")
                self.queue = self.CreateCommandPositionQueue(x1, y1, x2, y2, D_horizontal, D_vertical, D_task)
                
        except ValueError:
            self.get_logger().error('Invalid input format. Expected format: "x1,y1,x2,y2"')

    def bool_callback(self, msg: Bool):
        if msg.data:  # True가 들어온 경우
            if self.queue:
                # 큐에서 하나 꺼내서 퍼블리시
                gear, x, y, angle = self.queue.popleft()
                msg = Float32MultiArray()
                msg.data = [float(gear), x, y, angle]
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: {msg.data}')
        else:
            self.queue = deque()


def main(args=None):
    rclpy.init(args=args)
    angle_publisher = CommandPositionPublisher()
    rclpy.spin(angle_publisher)
    angle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
