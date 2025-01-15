import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

class LaserScanVisualizer(Node):
    def __init__(self):
        super().__init__('laser_scan_visualizer')
        
        # 라이다 데이터 구독
        self.rear_scan_subscriber = self.create_subscription(
            LaserScan,
            '/rear_scan',
            self.rear_scan_callback,
            10
        )
        self.front_scan_subscriber = self.create_subscription(
            LaserScan,
            '/front_scan',
            self.front_scan_callback,
            10
        )
        
        self.rear_scan = None
        self.front_scan = None

    def rear_scan_callback(self, msg):
        self.rear_scan = msg
        self.visualize_scans()

    def front_scan_callback(self, msg):
        self.front_scan = msg
        self.visualize_scans()

    def laser_scan_to_cartesian(self, scan, position, angle_offset):
        if scan is None:
            return None, None
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        x = scan.ranges * np.cos(angles + angle_offset) + position[0]
        y = scan.ranges * np.sin(angles + angle_offset) + position[1]
        return x, y

    def visualize_scans(self):
        if self.rear_scan is None or self.front_scan is None:
            return
        
        # 첫 번째 LaserScan 데이터 시각화 (후방)
        position1 = (0.15, 0.15)  # 후방 라이다 위치 (x, y)
        angle_offset1 = np.radians(45)   # 후방 라이다의 방향 (180도)
        x1, y1 = self.laser_scan_to_cartesian(self.rear_scan, position1, angle_offset1)
        
        # 두 번째 LaserScan 데이터 시각화 (전방)
        position2 = (-0.15, -0.15)  # 전방 라이다 위치 (x, y)
        angle_offset2 = np.radians(-135)        # 전방 라이다의 방향 (0도)
        x2, y2 = self.laser_scan_to_cartesian(self.front_scan, position2, angle_offset2)
        
        plt.figure(figsize=(8, 8))
        plt.title("Combined LaserScan Visualization")
        plt.plot(x1, y1, 'b.', label='Rear LaserScan')
        plt.plot(x2, y2, 'r.', label='Front LaserScan')
        plt.xlim(-5, 5)  # x축 범위 조정
        plt.ylim(-5, 5)  # y축 범위 조정
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.axhline(0, color='black', linewidth=0.5, ls='--')
        plt.axvline(0, color='black', linewidth=0.5, ls='--')
        plt.grid()
        plt.legend()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    visualizer = LaserScanVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
