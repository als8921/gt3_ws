import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
import numpy as np
import os
import glob
import asyncio
import aiofiles
from sensor_msgs.msg import PointCloud2, PointField
from pypcd4 import PointCloud

class PCDFileHandler(Node):
    def __init__(self, directory):
        super().__init__('pcd_file_handler')
        self.directory = directory
        self.subscription = self.create_subscription(String, '/unity/cmd', self.listener_callback, 10)
        self.publisher = self.create_publisher(String, '/unity/cmd', 10)
        self.map_publisher = self.create_publisher(PointCloud2, '/registered_topic', 1)
        self.scan_started = False  # scan_start 메시지 발행 여부 플래그
    
    def map_publish(self, pc_array):
        pc_data = np.array(pc_array, dtype=np.float32).flatten()

        pc2_msg = PointCloud2()
        pc2_msg.header = Header()
        pc2_msg.header.stamp = self.get_clock().now().to_msg()

        pc2_msg.header.frame_id = 'fastlio2_link'

        pc2_msg.height = 1
        pc2_msg.width = len(pc_data) // 4
        pc2_msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        pc2_msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        pc2_msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
        pc2_msg.fields.append(PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1))

        pc2_msg.point_step = 16
        pc2_msg.row_step = pc2_msg.point_step * pc2_msg.width
        pc2_msg.is_dense = True
        pc2_msg.is_bigendian = False
        pc2_msg.data = pc_data.tobytes()

        self.map_publisher.publish(pc2_msg)

    def listener_callback(self, msg):
        """메시지를 수신하면 파일 삭제 및 새로운 파일 확인."""
        if msg.data == "scan":
            self.scan_started = False
            asyncio.run(self.delete_pcd_files())  # 비동기로 파일 삭제
            asyncio.run(self.check_for_new_files(31))  # 삭제 후 새로운 파일 확인

    async def delete_pcd_files(self):
        """지정된 디렉토리의 모든 .pcd 파일을 비동기로 삭제."""
        tasks = []
        for file in glob.glob(os.path.join(self.directory, '*.pcd')):
            tasks.append(self.async_remove(file))

        await asyncio.gather(*tasks)

    async def async_remove(self, file):
        """비동기로 파일을 삭제합니다."""
        await aiofiles.os.remove(file)
        self.get_logger().info(f'Deleted: {file}')

    def extract_numbers_from_filenames(self, filenames):
        """파일 이름에서 숫자를 추출하여 리스트로 반환."""
        numbers = []
        for filename in filenames:
            name_without_extension = os.path.splitext(os.path.basename(filename))[0]
            parts = name_without_extension.split('_')
            if parts:
                last_part = parts[-1]
                if last_part.isdigit():
                    numbers.append(int(last_part))
        return sorted(numbers)

    async def check_for_new_files(self, scan_time):
        """새로운 파일이 생성될 때까지 대기하고, 이후 파일을 확인."""
        new_files = None
        
        # 새로운 파일이 생성될 때까지 대기
        while not new_files:
            new_files = glob.glob(os.path.join(self.directory, '*.pcd'))
            await asyncio.sleep(0.5)  # 비동기 대기

        # 파일이 존재하면 scan_start 메시지 발행
        if new_files and not self.scan_started:
            self.publisher.publish(String(data="scan_start"))
            self.get_logger().info("Published: scan_start")
            self.scan_started = True

        # 지정된 시간 동안 대기
        await asyncio.sleep(scan_time)

        # 확인된 파일 이름에서 숫자 추출
        collected_files = glob.glob(os.path.join(self.directory, '*.pcd'))
        numbers = self.extract_numbers_from_filenames(collected_files)
        numbers = numbers[1:]

        for number in numbers:
            file_name = f'/home/gt3-3/fastlio/src/FAST_LIO/PCD/scans_{number}.pcd'
            self.get_logger().info(file_name)
            pc = PointCloud.from_path(file_name)

            arrar = pc.numpy(("x", "y", "z", "intensity"))
            length = len(arrar)
            for i in range(length // 10000):
                self.map_publish(arrar[i * 10000:i * 10000 + 10000])
                await asyncio.sleep(0.02)  # 비동기 대기

def main(args=None):
    """ROS 2 노드를 초기화하고 실행."""
    rclpy.init(args=args)
    directory = '/home/gt3-3/fastlio/src/FAST_LIO/PCD'  # 여기에 디렉토리 경로를 입력하세요
    pcd_file_handler = PCDFileHandler(directory)
    rclpy.spin(pcd_file_handler)
    pcd_file_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
