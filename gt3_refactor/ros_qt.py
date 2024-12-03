import sys
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QTextEdit
from PyQt5 import uic, QtCore
from std_msgs.msg import String
from ros_publisher import ROSNode
import rclpy
QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps)

class QtController(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()
        
        
        # ROS 2 노드를 별도의 스레드에서 실행
        self.ros_thread = threading.Thread(target=self.run_ros_node)
        self.ros_thread.start()

    def init_ui(self):
        # .ui 파일 로드
        uic.loadUi('viewer.ui', self)
        self.lcdNumber.display('0')
        self.lineEdit.setText('0.0')
        self.lineEdit_2.setText('0.0')
        self.lineEdit_3.setText('0.0')
        self.lineEdit_4.setText('0.0')
        self.lineEdit_5.setText('0.0')
        self.lineEdit_6.setText('0.0')
        self.lineEdit_7.setText('0.0')
        self.lineEdit_8.setText('0.0')
        self.lineEdit_9.setText('0.0')
        self.lineEdit_10.setText('0.0')
        self.lineEdit_11.setText('0.0')
        self.lineEdit_12.setText('0.0')
        self.lineEdit_13.setText('0.9')     # distance
        self.lineEdit_16.setText('10')      # RPM
        self.lineEdit_15.setText('55')      # steer angle
        self.lineEdit_14.setText('90')      # rotate angle
        self.lineEdit_17.setText('50')
        self.lineEdit_18.setText('5')
        self.lineEdit_19.setText('800')
        self.lineEdit_20.setText('500')
        self.lineEdit_22.setText('3')
        self.lineEdit_23.setText('2')
        self.lineEdit_24.setText('0.7')
        self.lineEdit_25.setText('0.0')

        self.radioButton_2.setChecked(True)
        self.checkBox_4.setChecked(True)
        self.checkBox_16.setChecked(False)

        # self.pushButton.clicked.connect(self.go_forward)
        # self.pushButton_9.clicked.connect(self.go_backward)
        # self.pushButton_3.clicked.connect(self.left_forward)
        # self.pushButton_4.clicked.connect(self.right_forward)
        # self.pushButton_12.clicked.connect(self.left_backward)
        # self.pushButton_13.clicked.connect(self.right_backward)
        self.pushButton_2.clicked.connect(self.quit)
        # self.pushButton_5.clicked.connect(self.drive)
        # self.pushButton_43.clicked.connect(self.drive2)
        # self.pushButton_44.clicked.connect(self.drive3)
        # self.pushButton_6.clicked.connect(self.fastlio_map)
        # self.pushButton_7.clicked.connect(self.paint)
        # #self.pushButton_8.clicked.connect(self.steer_minus)
        # #self.pushButton_11.clicked.connect(self.steer_plus)
        # self.pushButton_15.clicked.connect(self.IK_pose)
        # self.pushButton_23.clicked.connect(self.FK_pose)
        # self.pushButton_16.clicked.connect(self.reconnect)
        # self.pushButton_17.clicked.connect(self.robot_home)
        # self.pushButton_14.clicked.connect(self.auto)
        # self.pushButton_10.clicked.connect(self.stop)
        # self.pushButton_20.clicked.connect(self.topview)
        # self.pushButton_19.clicked.connect(self.right_cam)
        # self.pushButton_18.clicked.connect(self.arm_tilt)
        # self.pushButton_21.clicked.connect(self.arm_cam_left)
        # self.pushButton_22.clicked.connect(self.floorview)
        # self.pushButton_24.clicked.connect(self.emergency)
        # self.pushButton_25.clicked.connect(self.leftwall)
        # self.pushButton_26.clicked.connect(self.leftcorner)
        # self.pushButton_27.clicked.connect(self.frontwall)
        # self.pushButton_28.clicked.connect(self.setting)
        # self.pushButton_29.clicked.connect(self.frontview)
        # self.pushButton_42.clicked.connect(self.leftview)
        # self.pushButton_30.clicked.connect(self.refresh)
        # self.pushButton_31.clicked.connect(self.clear)
        # self.pushButton_32.clicked.connect(self.play)
        # self.pushButton_33.clicked.connect(self.set_current)
        # self.pushButton_34.clicked.connect(self.turn90_left)
        # self.pushButton_35.clicked.connect(self.ceilingview)
        # self.pushButton_36.clicked.connect(self.cover_up)
        # self.pushButton_37.clicked.connect(self.cover_down)
        # self.pushButton_38.clicked.connect(self.cover_stop)
        self.pushButton_39.clicked.connect(self.lift_up)
        self.pushButton_40.clicked.connect(self.lift_down)
        self.pushButton_41.clicked.connect(self.lift_stop)
        # self.pushButton_45.clicked.connect(self.adjust_left)
        # self.pushButton_46.clicked.connect(self.adjust_right)
        # self.pushButton_47.clicked.connect(self.delete_task)
        # self.pushButton_48.clicked.connect(self.add_trajectory)
        # self.pushButton_49.clicked.connect(self.execute_task)
        # self.pushButton_50.clicked.connect(self.trajectory_summary)
        # self.pushButton_51.clicked.connect(self.m_home)
        # self.pushButton_52.clicked.connect(self.autoscan)
        # self.pushButton_53.clicked.connect(self.tracking)
        # self.pushButton_54.clicked.connect(self.rail_left)
        # self.pushButton_55.clicked.connect(self.rail_center)
        # self.pushButton_56.clicked.connect(self.rail_right)

        # self.radioButton.clicked.connect(self.plane)
        # self.radioButton_2.clicked.connect(self.curved)
        # self.radioButton_3.clicked.connect(self.front)
        # self.radioButton_4.clicked.connect(self.rear)
        # self.radioButton_5.clicked.connect(self.tower)
        # self.radioButton_6.clicked.connect(self.arm)

        # self.checkBox.clicked.connect(self.side)
        # self.checkBox_2.clicked.connect(self.real)
        # self.checkBox_3.clicked.connect(self.pull)
        # self.checkBox_4.clicked.connect(self.continuous)
        # self.checkBox_6.clicked.connect(self.waypoint_0)
        # self.checkBox_7.clicked.connect(self.waypoint_1)
        # self.checkBox_8.clicked.connect(self.waypoint_2)
        # self.checkBox_9.clicked.connect(self.waypoint_3)
        # self.checkBox_10.clicked.connect(self.waypoint_4)
        # self.checkBox_11.clicked.connect(self.waypoint_5)
        # self.checkBox_12.clicked.connect(self.waypoint_6)
        # self.checkBox_13.clicked.connect(self.waypoint_7)
        # self.checkBox_14.clicked.connect(self.waypoint_8)
        # self.checkBox_15.clicked.connect(self.waypoint_9)
        # self.checkBox_16.clicked.connect(self.wall)
        # self.checkBox_17.clicked.connect(self.addtask)
        # self.checkBox_18.clicked.connect(self.pitch_error)


    def lift_up(self):
        self.ros_node.send_lift_command("MOVE", float(self.lineEdit_25.text()))

    def lift_down(self):
        self.ros_node.send_lift_command("HOMING", 0.0)

    def lift_stop(self):
        self.ros_node.send_lift_command("STOP", 0.0)















    def run_ros_node(self):
        rclpy.init()
        self.ros_node = ROSNode()
        rclpy.spin(self.ros_node)


    def quit(self):
        self.ros_node.destroy_node()
        rclpy.shutdown()
        sys.exit()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = QtController()
    window.show()
    sys.exit(app.exec_())
