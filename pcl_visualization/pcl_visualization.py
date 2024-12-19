import sys
import threading
import numpy as np
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, 
                             QHBoxLayout, QLabel, QLineEdit, QFileDialog)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
from ros_node import ROSNode
import rclpy
from PyQt5.QtCore import Qt
import pcl_clustering  

class QtController(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()

        # ROS 2 노드를 별도의 스레드에서 실행
        self.ros_thread = threading.Thread(target=self.run_ros_node)
        self.ros_thread.start()

        # 초기 점 리스트
        self.points = []
        self.rotated_points = []

        # 기본 축 범위 설정
        self.default_range = 5
        self.current_range = self.default_range
        
        # 축 이동 오프셋 초기화
        self.x_offset = 0
        self.y_offset = 0
        self.z_offset = 0

    def init_ui(self):
        self.setWindowTitle("ROS Control Panel")
        self.setGeometry(100, 100, 3000, 1800)

        # 중앙 위젯과 레이아웃 설정
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        layout = QHBoxLayout(central_widget)  # 수평 레이아웃 사용

        # Matplotlib Figure 설정
        self.figure = plt.figure()
        self.figure.add_subplot(111, projection='3d')
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setFixedSize(1800, 1800)  # 캔버스 크기 설정
        self.canvas.draw()
        layout.addWidget(self.canvas)  # 왼쪽에 Matplotlib 캔버스 추가

        # 버튼 레이아웃 설정
        button_layout = QVBoxLayout()

        # 기존 버튼들
        self.add_button = QPushButton("점 추가")
        self.reset_button = QPushButton("리셋")
        self.rotate_button = QPushButton("회전 적용")
        
        # 새로운 포인트 클라우드 파일 로드 버튼 추가
        self.load_pointcloud_button = QPushButton("포인트 클라우드 파일 로드")
        self.load_pointcloud_button.clicked.connect(self.load_pointcloud_file)

        # 포인트 저장 버튼 추가
        self.save_pointcloud_button = QPushButton("포인트 저장")
        self.save_pointcloud_button.clicked.connect(self.save_pointcloud_file)

        # 종료 버튼 추가
        self.exit_button = QPushButton("종료")
        self.exit_button.clicked.connect(self.close_application)

        # 회전 입력 필드 추가
        self.roll_input = QLineEdit(self)
        self.roll_input.setPlaceholderText("Roll (degrees)")
        self.pitch_input = QLineEdit(self)
        self.pitch_input.setPlaceholderText("Pitch (degrees)")
        self.yaw_input = QLineEdit(self)
        self.yaw_input.setPlaceholderText("Yaw (degrees)")

        self.add_button.clicked.connect(self.get_pointcloud)
        self.reset_button.clicked.connect(self.reset_plot)
        self.rotate_button.clicked.connect(self.apply_rotation)

        # 버튼 레이아웃에 버튼 추가
        button_layout.addWidget(self.add_button)
        button_layout.addWidget(self.reset_button)
        button_layout.addWidget(self.load_pointcloud_button)  # 새 버튼 추가
        button_layout.addWidget(self.save_pointcloud_button)  # 포인트 저장 버튼 추가
        button_layout.addWidget(self.roll_input)
        button_layout.addWidget(self.pitch_input)
        button_layout.addWidget(self.yaw_input)
        button_layout.addWidget(self.rotate_button)
        button_layout.addWidget(self.exit_button)  # 종료 버튼 추가

        layout.addLayout(button_layout)  # 오른쪽에 버튼 레이아웃 추가

    def load_pointcloud_file(self):
        """포인트 클라우드 텍스트 파일을 선택하고 로드합니다."""
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self, "포인트 클라우드 파일 선택", "", "Text Files (*.txt);;All Files (*)", options=options)
        if file_name:
            with open(file_name, 'r') as f:
                new_points = [
                    [float(coord) for coord in line.strip().split()] 
                    for line in f
                ]
            
            self.points.extend(new_points)
            self.plot_points()
            print(f"Successfully loaded {len(new_points)} points from {file_name}")

    def save_pointcloud_file(self):
        """현재 포인트를 텍스트 파일로 저장합니다."""
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getSaveFileName(self, "Save Point Cloud", "", "Text Files (*.txt);;All Files (*)", options=options)
        if file_name:
            np.savetxt(file_name, self.points, fmt='%f', delimiter=' ')
            print(f"Successfully saved {len(self.points)} points to {file_name}")

    def close_application(self):
        """애플리케이션 종료를 처리합니다."""
        self.close()
        self.ros_node.destroy_node()
        rclpy.shutdown()

    def get_pointcloud(self):
        self.points.extend(self.ros_node.points)
        print(len(self.points))
        self.plot_points()

    def reset_plot(self):
        # 점 리스트 초기화
        self.points = []
        self.x_offset = 0
        self.y_offset = 0
        self.z_offset = 0
        self.current_range = self.default_range  # 스케일 초기화
        self.plot_points()

    def plot_points(self, rotate=False):
        # 기존 플롯 초기화
        self.figure.clear()
        ax = self.figure.add_subplot(111, projection='3d')
        closest = []
        remaining = []
        image = np.array([])
        if rotate:
            if self.rotated_points:
                closest, remaining, image = pcl_clustering.cluster_pointcloud(self.rotated_points)
        else:
            if self.points:
                closest, remaining, image = pcl_clustering.cluster_pointcloud(self.points)

        if closest:     
            closest = np.transpose(closest)
            ax.scatter(closest[0], closest[1], closest[2], c='r', marker='o')

        if remaining:     
            remaining = np.transpose(remaining)
            ax.scatter(remaining[0], remaining[1], remaining[2], c='grey', marker='o')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # x, y, z 축의 범위를 동일하게 설정
        all_points = np.transpose(self.points)
        if all_points.size > 0:
            max_range = np.array([all_points[0].max() - all_points[0].min(),
                                all_points[1].max() - all_points[1].min(),
                                all_points[2].max() - all_points[2].min()]).max() / 2.0

            mid_x = (all_points[0].max() + all_points[0].min()) * 0.5
            mid_y = (all_points[1].max() + all_points[1].min()) * 0.5
            mid_z = (all_points[2].max() + all_points[2].min()) * 0.5

            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)

        # 업데이트된 플롯을 표시
        self.canvas.draw()

    def apply_rotation(self):
        # Roll, Pitch, Yaw 값을 가져와서 회전 적용
        try:
            roll = float(self.roll_input.text())
            pitch = float(self.pitch_input.text())
            yaw = float(self.yaw_input.text())
            self.rotated_points = self.ros_node.rotate_points(self.points, roll, pitch, yaw)
            self.plot_points(rotate=True)  # 회전 후 점 다시 플로팅
        except ValueError:
            print("유효한 숫자를 입력하세요.")

    def run_ros_node(self):
        rclpy.init()
        self.ros_node = ROSNode()
        rclpy.spin(self.ros_node)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = QtController()
    window.show()
    sys.exit(app.exec_())
