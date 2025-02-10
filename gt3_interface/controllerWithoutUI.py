import rclpy
import threading
import time
import numpy as np
from ros_node import ROSNode

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
                    avg_x = np.mean(x_list)
                    avg_y = np.mean(y_list)

                    distances = np.square(x_list) + np.square(y_list)
                    min_index = np.argmin(distances)
                    print("FRONT LASERSCAN")
                    print("평균 지점 : ", avg_x, avg_y)
                    print("가장 가까운 거리 : ", np.sqrt(np.min(distances)))
                    print("가장 가까운 점 : ", x_list[min_index], y_list[min_index])

            for idx, [x_list, y_list] in enumerate(rear_cluster):
                if idx == 0:
                    avg_x = np.mean(x_list)
                    avg_y = np.mean(y_list)

                    distances = np.square(x_list) + np.square(y_list)
                    min_index = np.argmin(distances)
                    print("REAR LASERSCAN")
                    print("평균 지점 : ", avg_x, avg_y)
                    print("가장 가까운 거리 : ", np.sqrt(np.min(distances)))
                    print("가장 가까운 점 : ", x_list[min_index], y_list[min_index])

    def run_ros_node(self):
        rclpy.init()
        self.ros_node = ROSNode()
        rclpy.spin(self.ros_node)

if __name__ == '__main__':
    controller = Controller()
    try:
        while rclpy.ok():
            loop_start_time = time.time()  # 루프 시작 시간 기록
            controller.update()
            loop_end_time = time.time()  # 루프 종료 시간 기록
            loop_duration = loop_end_time - loop_start_time  # 루프 시간 계산
            print(f"루프 실행 시간: {loop_duration:.10f} 초")
    except KeyboardInterrupt:
        print("프로그램 종료 중")
    finally:
        rclpy.shutdown()
        print("프로그램 종료 완료")