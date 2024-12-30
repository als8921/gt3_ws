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
import pcl_normal_vector

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
        
        # 축 이동 오프셋 초기화
        self.x_offset = 0
        self.y_offset = 0
        self.z_offset = 0

        # Matplotlib Figure 및 Canvas 설정
        self.figure3D = plt.figure()
        self.canvas3D = FigureCanvas(self.figure3D)
        self.layout3D = QVBoxLayout(self.frame)
        self.layout3D.addWidget(self.canvas3D)

        # 2D 플롯의 Figure 및 Canvas 초기화
        self.figure2D = plt.figure()
        self.canvas2D = FigureCanvas(self.figure2D)
        self.layout2D = QVBoxLayout(self.frame_2)
        self.layout2D.addWidget(self.canvas2D)

        self.plot_points()

    def init_ui(self):
        uic.loadUi('pcl.ui', self)
        self.eps_lineEdit.setText('0.2')

        self.loadButton.clicked.connect(self.btn_load_pointcloud_ros)
        self.loadTxtButton.clicked.connect(self.btn_load_pointcloud_file)
        self.saveButton.clicked.connect(self.btn_save_pointcloud_file)
        self.resetButton.clicked.connect(self.btn_reset_plot)
        self.quitButton.clicked.connect(self.btn_quit)

    def btn_load_pointcloud_ros(self): # ROS Pointcloud2 데이터 불러오기
        self.points.extend(self.ros_node.points)
        print("point 개수 : ", len(self.points))
        self.plot_points()

    def btn_load_pointcloud_file(self): # Pointcloud txt 파일 불러오기
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

    def btn_save_pointcloud_file(self): # 현재 Pointcloud txt 파일로 저장
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getSaveFileName(self, "Save Point Cloud", "", "Text Files (*.txt);;All Files (*)", options=options)
        if file_name:
            np.savetxt(file_name, self.points, fmt='%f', delimiter=' ')
            print(f"Successfully saved {len(self.points)} points to {file_name}")

    def btn_reset_plot(self): # 현재 Pointcloud 초기화
        # 점 리스트 초기화
        self.points = []
        self.x_offset = 0
        self.y_offset = 0
        self.z_offset = 0
        self.plot_points()

    def btn_quit(self): # 프로그램 종료
        self.close()
        self.ros_node.destroy_node()
        rclpy.shutdown()

    def plot_points(self):
        self.figure3D.clear()
        self.figure2D.clear()

        ax3D = self.figure3D.add_subplot(111, projection='3d')
        ax2D = self.figure2D.add_subplot(111)

        ax3D.set_xlabel('X')
        ax3D.set_ylabel('Y')
        ax3D.set_zlabel('Z')
        ax2D.axis([0, 43, 0, 23])
        ax2D.invert_yaxis()
        
        closest = []
        remaining = []
        cluster_idx = []
        image = np.array([])
        clust = np.array([])
        
        if self.points:
            closest, remaining, image, cluster_idx, clust = pcl_clustering.cluster_pointcloud(self.points, float(self.eps_lineEdit.text()))

        # 클러스터링 된 부분 시각화
        if closest:     
            closest = np.transpose(closest)
            ax3D.scatter(closest[0], closest[1], closest[2], c='g', marker='o', s=2)
            x_min, x_max = closest[0].min(), closest[0].max()
            y_min, y_max = closest[1].min(), closest[1].max()
            z_min, z_max = closest[2].min(), closest[2].max()

            # 초록색 박스 시각화
            ax3D.bar3d(x_min, y_min, z_min, 
                    x_max - x_min, y_max - y_min, z_max - z_min, 
                    color='green', alpha=0.2)

        # 클러스터링 나머지 부분 시각화
        if remaining:     
            remaining = np.transpose(remaining)
            ax3D.scatter(remaining[0], remaining[1], remaining[2], c='grey', marker='o', s=2)

        # 클러스터링 된 데이터에서의 법선 벡터 구하기
        if(clust.size > 0):
            normals = pcl_normal_vector.get_normal_vectors(clust)
            
            for y in range(1, clust.shape[0] - 1):
                for x in range(1, clust.shape[1] - 1):
                    # 법선 벡터 시작점
                    start = clust[y, x]
                    # 법선 벡터 끝점
                    length = 0.1
                    end = start[0] + normals[y, x, 0], start[1] + normals[y, x, 1], start[2] + normals[y, x, 2]
                    ax3D.quiver(start[0], start[1], start[2],
                            normals[y, x, 0], normals[y, x, 1], normals[y, x, 2],
                            length=length,color='r')



        
        # 클러스터링 된 물체의 위치를 Image로 시각화
        if(image.size > 0):
            ax2D.imshow(image, cmap='gray')
            ax2D.axis([0, 43, 0, 23])
            ax2D.invert_yaxis()

        # ax3D x, y, z 축의 범위를 동일하게 설정
        all_points = np.transpose(self.points)
        if all_points.size > 0:
            max_range = np.array([all_points[0].max() - all_points[0].min(),
                                all_points[1].max() - all_points[1].min(),
                                all_points[2].max() - all_points[2].min()]).max() / 2.0

            mid_x = (all_points[0].max() + all_points[0].min()) * 0.5
            mid_y = (all_points[1].max() + all_points[1].min()) * 0.5
            mid_z = (all_points[2].max() + all_points[2].min()) * 0.5

            ax3D.set_xlim(mid_x - max_range, mid_x + max_range)
            ax3D.set_ylim(mid_y - max_range, mid_y + max_range)
            ax3D.set_zlim(mid_z - max_range, mid_z + max_range)



        self.canvas3D.draw()
        self.canvas2D.draw()

    def run_ros_node(self):
        rclpy.init()
        self.ros_node = ROSNode()
        rclpy.spin(self.ros_node)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = QtController()
    window.show()
    sys.exit(app.exec_())
