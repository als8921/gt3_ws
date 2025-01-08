import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import time
import glob

class PCDFileHandler(Node):
    def __init__(self, directory):
        super().__init__('pcd_file_handler')
        self.directory = directory
        self.subscription = self.create_subscription(String, '/unity/cmd', self.listener_callback, 10)
        self.publisher = self.create_publisher(String, '/unity/cmd', 10)
        self.scan_started = False  # scan_start 메시지 발행 여부 플래그

    def listener_callback(self, msg):
        """메시지를 수신하면 파일 삭제 및 새로운 파일 확인."""
        if msg.data == "scan":
            self.delete_pcd_files()
            self.check_for_new_files(5)

    def delete_pcd_files(self):
        """지정된 디렉토리의 모든 .pcd 파일을 삭제."""
        for file in glob.glob(os.path.join(self.directory, '*.pcd')):
            os.remove(file)
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

    def check_for_new_files(self, scan_time):
        """새로운 파일이 생성될 때까지 대기하고, 이후 파일을 확인."""
        new_files = None
        
        # 새로운 파일이 생성될 때까지 대기
        while not new_files:
            new_files = glob.glob(os.path.join(self.directory, '*.pcd'))
            time.sleep(0.5)  # 0.5초 대기

        # 파일이 존재하면 scan_start 메시지 발행
        if new_files and not self.scan_started:
            self.publisher.publish(String(data="scan_start"))
            self.get_logger().info("Published: scan_start")
            self.scan_started = True

        # 지정된 시간 동안 대기
        time.sleep(scan_time)

        # 확인된 파일 이름에서 숫자 추출
        collected_files = glob.glob(os.path.join(self.directory, '*.pcd'))
        numbers = self.extract_numbers_from_filenames(collected_files)
        self.get_logger().info(f'Extracted numbers: {numbers}')

def main(args=None):
    """ROS 2 노드를 초기화하고 실행."""
    rclpy.init(args=args)
    directory = '/home/lmc/gt3_ws/pcd'  # 여기에 디렉토리 경로를 입력하세요
    pcd_file_handler = PCDFileHandler(directory)
    rclpy.spin(pcd_file_handler)
    pcd_file_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
