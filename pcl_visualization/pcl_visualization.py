import sys
import threading
import numpy as np
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QHBoxLayout
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D  # 3D 플롯을 사용하기 위한 임포트
from ros_node import ROSNode
import rclpy

class QtController(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()

        # ROS 2 노드를 별도의 스레드에서 실행
        self.ros_thread = threading.Thread(target=self.run_ros_node)
        self.ros_thread.start()

        # 초기 점 리스트
        self.points = []

    def init_ui(self):
        self.setWindowTitle("ROS Control Panel")
        self.setGeometry(100, 100, 1920, 1080)

        # 중앙 위젯과 레이아웃 설정
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        layout = QHBoxLayout(central_widget)  # 수평 레이아웃 사용

        # Matplotlib Figure 설정
        self.figure = plt.figure()
        self.figure.add_subplot(111, projection='3d')
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setFixedSize(1080, 1080)  # 캔버스 크기 설정
        self.canvas.draw()
        layout.addWidget(self.canvas)  # 왼쪽에 Matplotlib 캔버스 추가

        # 버튼 레이아웃 설정
        button_layout = QVBoxLayout()

        # 버튼 추가
        self.add_button = QPushButton("점 추가")
        self.reset_button = QPushButton("리셋")
        self.add_button.clicked.connect(self.add_random_point)
        self.reset_button.clicked.connect(self.reset_plot)

        button_layout.addWidget(self.add_button)
        button_layout.addWidget(self.reset_button)

        layout.addLayout(button_layout)  # 오른쪽에 버튼 레이아웃 추가

    def add_random_point(self):
        # 랜덤 점 1개 추가
        self.points.extend(self.ros_node.points)
        print(len(self.points))
        self.plot_points()

    def reset_plot(self):
        # 점 리스트 초기화
        self.points = []
        self.plot_points()

    def plot_points(self):
        # 기존 플롯 초기화
        self.figure.clear()
        ax = self.figure.add_subplot(111, projection='3d')

        if self.points:
            points_array = np.transpose(self.points)
            ax.scatter(points_array[0], points_array[1], points_array[2], c='r', marker='o')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # 업데이트된 플롯을 표시
        self.canvas.draw()

    def run_ros_node(self):
        rclpy.init()
        self.ros_node = ROSNode()
        rclpy.spin(self.ros_node)

    def closeEvent(self, event):
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = QtController()
    window.show()
    sys.exit(app.exec_())
