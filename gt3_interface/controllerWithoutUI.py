import rclpy
import threading
import time
import numpy as np
from ros_node import ROSNode
from std_msgs.msg import Float32MultiArray

import LaserScan.laserscan_transformation as laserscan_transformation
import LaserScan.laserscan_clustering as laserscan_clustering

class Controller:
    def __init__(self):
        self.ros_node = None
        self.ros_thread = threading.Thread(target=self.run_ros_node)
        self.ros_thread.start()
        

        while self.ros_node is None:
            time.sleep(0.1)

    def update(self):
        if(self.ros_node.front_scan and self.ros_node.rear_scan):
            front_data, rear_data = laserscan_transformation.laser_scan_to_xy(self.ros_node.front_scan, self.ros_node.rear_scan, 0.3, 0.25)

            front_cluster = laserscan_clustering.clustering(*front_data, 0.1, 4)
            rear_cluster = laserscan_clustering.clustering(*rear_data, 0.1, 4)

            for idx, [x_list, y_list] in enumerate(front_cluster):
                if idx == 0:
                    avg_x = np.round(np.mean(x_list), 3)
                    avg_y = np.round(np.mean(y_list), 3)

                    distances = np.square(x_list) + np.square(y_list)
                    min_distance = np.round(np.sqrt(np.min(distances)), 3)
                    min_index = np.argmin(distances)

                    print("FRONT LASERSCAN")
                    print("평균 지점 : ", avg_x, avg_y)
                    print("가장 가까운 거리 : ", min_distance)
                    print("가장 가까운 점 : ", x_list[min_index], y_list[min_index])

                    self.ros_node.front_scan_info_pub([avg_x, avg_y, min_distance])

            for idx, [x_list, y_list] in enumerate(rear_cluster):
                if idx == 0:
                    avg_x = np.round(np.mean(x_list), 3)
                    avg_y = np.round(np.mean(y_list), 3)

                    distances = np.square(x_list) + np.square(y_list)
                    min_distance = np.round(np.sqrt(np.min(distances)), 3)
                    min_index = np.argmin(distances)

                    print("REAR LASERSCAN")
                    print("평균 지점 : ", avg_x, avg_y)
                    print("가장 가까운 거리 : ", min_distance)
                    print("가장 가까운 점 : ", x_list[min_index], y_list[min_index])

                    self.ros_node.rear_scan_info_pub([avg_x, avg_y, min_distance])

    def run_ros_node(self):
        rclpy.init()
        self.ros_node = ROSNode()
        rclpy.spin(self.ros_node)

if __name__ == '__main__':
    controller = Controller()
    try:
        while rclpy.ok():
            controller.update()
    except KeyboardInterrupt:
        print("프로그램 종료 중")
    finally:
        rclpy.shutdown()
        print("프로그램 종료 완료")
