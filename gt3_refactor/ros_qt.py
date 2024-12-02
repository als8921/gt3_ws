# my_qt_app.py
import sys
import threading
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QTextEdit
from std_msgs.msg import String
from ros_publisher import MyROSNode
import rclpy

class MyQtApp(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        
        # ROS 2 노드를 별도의 스레드에서 실행
        self.ros_thread = threading.Thread(target=self.run_ros_node)
        self.ros_thread.start()

    def init_ui(self):
        self.setWindowTitle('ROS 2 with PyQt')
        self.layout = QVBoxLayout()
        
        self.text_edit = QTextEdit(self)
        self.layout.addWidget(self.text_edit)
        
        self.button = QPushButton('Publish Message', self)
        self.button.clicked.connect(self.publish_message)
        self.layout.addWidget(self.button)
        
        self.setLayout(self.layout)

    def run_ros_node(self):
        rclpy.init()
        self.node = MyROSNode()
        rclpy.spin(self.node)

    def publish_message(self):
        message = self.text_edit.toPlainText()
        self.node.publish_message(message)

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        self.ros_thread.join()  # ROS 스레드 종료 대기
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyQtApp()
    window.show()
    sys.exit(app.exec_())
