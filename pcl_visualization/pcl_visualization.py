import sys
import threading
import numpy as np
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QFileDialog)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
from ros_node import ROSNode
import rclpy
from PyQt5 import uic, QtCore
import pcl_clustering

QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps)

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

        # Matplotlib Figure 및 Canvas 설정
        self.figure3D = plt.figure()
        self.canvas3D = FigureCanvas(self.figure3D)
        self.layout3D = QVBoxLayout(self.frame)  # 'frame'은 .ui 파일에서 정의된 Frame의 객체 이름
        self.layout3D.addWidget(self.canvas3D)  # Canvas를 Frame에 추가

        # 2D 플롯의 Figure 및 Canvas 초기화
        self.figure2D = plt.figure()  # 추가된 초기화
        self.canvas2D = FigureCanvas(self.figure2D)
        self.layout2D = QVBoxLayout(self.frame_2)  # 'frame_2'은 .ui 파일에서 정의된 Frame의 객체 이름
        self.layout2D.addWidget(self.canvas2D)  # Canvas를 Frame에 추가

        self.plot_points()

    def init_ui(self):
        uic.loadUi('pcl.ui', self)
        self.lineEdit0.setText('Roll')
        self.lineEdit1.setText('Pitch')
        self.lineEdit2.setText('Yaw')

        self.loadButton.clicked.connect(self.get_pointcloud)
        self.loadTxtButton.clicked.connect(self.load_pointcloud_file)
        self.resetButton.clicked.connect(self.reset_plot)
        self.rotateButton.clicked.connect(self.apply_rotation)
        self.quitButton.clicked.connect(self.close_application)
        self.saveButton.clicked.connect(self.save_pointcloud_file)

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
        # 3D 플롯 초기화
        self.figure3D.clear()
        ax3D = self.figure3D.add_subplot(111, projection='3d')
        
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
            ax3D.scatter(closest[0], closest[1], closest[2], c='r', marker='o', s=2)

        if remaining:     
            remaining = np.transpose(remaining)
            ax3D.scatter(remaining[0], remaining[1], remaining[2], c='grey', marker='o', s=2)

        ax3D.set_xlabel('X')
        ax3D.set_ylabel('Y')
        ax3D.set_zlabel('Z')

        # 2D 플롯 초기화
        self.figure2D.clear()
        ax2D = self.figure2D.add_subplot(111)
        ax2D.axis([0, 43, 0, 23])
        ax2D.invert_yaxis()
        
        if image.size > 0:
            # 이미지를 2D 플롯에 표시
            ax2D.imshow(image, cmap='gray')  # 필요한 경우 cmap을 변경하세요.
            ax2D.axis([0, 43, 0, 23])

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
        self.canvas3D.draw()
        self.canvas2D.draw()

    def apply_rotation(self):
        # Roll, Pitch, Yaw 값을 가져와서 회전 적용
        try:
            roll = float(self.lineEdit0.text())
            pitch = float(self.lineEdit1.text())
            yaw = float(self.lineEdit2.text())
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
