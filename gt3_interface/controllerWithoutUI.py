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
                    if(idx == 0):
                        pass


    def run_ros_node(self):
        rclpy.init()
        self.ros_node = ROSNode()
        rclpy.spin(self.ros_node)

if __name__ == '__main__':
    controller = Controller()
    while rclpy.ok():
        controller.update()
    rclpy.shutdown()