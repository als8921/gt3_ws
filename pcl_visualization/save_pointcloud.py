import rclpy
import numpy as np
import os
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/pointcloud_topic',
            self.point_cloud_callback,
            10
        )
        
        # Create a directory to store point cloud data if it doesn't exist
        self.output_dir = 'pcl_visualization/pointcloud_data'
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Counter to create unique filenames
        self.file_counter = 0
        
        # Initialize points list
        self.points = []

    def point_cloud_callback(self, msg):
        point_step = msg.point_step
        data = np.frombuffer(msg.data, dtype=np.float32)

        # 포인트 수 계산
        num_points = len(data) // (point_step // 4)
        self.points = []
        for i in range(num_points):
            y = -data[i * point_step // 4]  # x 좌표
            x = data[i * point_step // 4 + 1]  # y 좌표
            z = data[i * point_step // 4 + 2]  # z 좌표
            self.points.append((x, y, z))
        
        # Save points to a text file
        self.save_points_to_file()

    def pointcloud2_to_numpy(self, cloud_msg):
        # Convert cloud_msg data array to numpy array
        dtype_list = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
        cloud_arr = np.frombuffer(cloud_msg.data, dtype=dtype_list)
        return cloud_arr

    def save_points_to_file(self):
        # Create a unique filename
        filename = os.path.join(self.output_dir, f'pointcloud_data_{self.file_counter}.txt')
        
        # Write points to the file
        with open(filename, 'w') as f:
            for point in self.points:
                f.write(f"{point[0]} {point[1]} {point[2]}\n")
        
        # Increment file counter
        self.file_counter += 1
        
        # Optional: Clear points list after saving to avoid memory buildup
        self.points.clear()

def main(args=None):
    rclpy.init(args=args)
    pointcloud_subscriber = PointCloudSubscriber()
    
    try:
        rclpy.spin(pointcloud_subscriber)
    except KeyboardInterrupt:
        print('Stopping pointcloud subscriber...')
    finally:
        pointcloud_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()