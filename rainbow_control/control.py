import sys
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QIcon, QFont
from PyQt5.QtWidgets import (QMainWindow, QApplication, QWidget, QPushButton, QToolTip, QGridLayout, QLabel, QLineEdit, QSlider,
                             QVBoxLayout, QHBoxLayout, QTextEdit, QComboBox)
from PyQt5 import uic, QtCore
import os
QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps)

from cobot import *

class ToCBThread(QThread):
    finished = pyqtSignal()  # 작업 완료 신호

    def __init__(self, ip_address):
        super().__init__()
        self.ip_address = ip_address

    def run(self):
        ToCB(self.ip_address)
        print(f"IP Address: {self.ip_address}")

class QtController(QMainWindow):
    def __init__(self):
        super().__init__()

        # Status Variable
        self.CurrentMode = None
        self.IsConnected = False

        # Buttons
        self.Connect_Button : QPushButton
        self.getPosition_Button : QPushButton
        self.Quit_Button : QPushButton

        # ComboBox
        self.robotReal : QComboBox

        # TextEdit
        self.position_text : QTextEdit

        self.ip : QTextEdit
        self.move_x : QTextEdit
        self.move_x_2 : QTextEdit
        self.move_x_3 : QTextEdit
        self.move_x_4 : QTextEdit
        self.move_x_5 : QTextEdit
        self.move_x_6 : QTextEdit
        
        self.move_y : QTextEdit
        self.move_y_2 : QTextEdit
        self.move_y_3 : QTextEdit
        self.move_y_4 : QTextEdit
        self.move_y_5 : QTextEdit
        self.move_y_6 : QTextEdit

        self.move_z : QTextEdit
        self.move_z_2 : QTextEdit
        self.move_z_3 : QTextEdit
        self.move_z_4 : QTextEdit
        self.move_z_5 : QTextEdit
        self.move_z_6 : QTextEdit
        
        self.move_rx : QTextEdit
        self.move_rx_2 : QTextEdit
        self.move_rx_3 : QTextEdit
        self.move_rx_4 : QTextEdit
        self.move_rx_5 : QTextEdit
        self.move_rx_6 : QTextEdit
        
        self.move_ry : QTextEdit
        self.move_ry_2 : QTextEdit
        self.move_ry_3 : QTextEdit
        self.move_ry_4 : QTextEdit
        self.move_ry_5 : QTextEdit
        self.move_ry_6 : QTextEdit

        self.move_rz : QTextEdit
        self.move_rz_2 : QTextEdit
        self.move_rz_3 : QTextEdit
        self.move_rz_4 : QTextEdit
        self.move_rz_5 : QTextEdit
        self.move_rz_6 : QTextEdit

        # Label
        self.status_label : QLabel

        self.init_ui()

    def init_ui(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        ui_file = os.path.join(current_dir, 'control.ui')

        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.datareset)
        self.timer.start()

        uic.loadUi(ui_file, self)
        self.ip.setText('192.168.1.7')
        self.Connect_Button.clicked.connect(self.start_thread)
        self.getPosition_Button.clicked.connect(self.getCurrentPosition)
        self.status_label.setStyleSheet("background-color: green")
        self.Quit_Button.clicked.connect(self.quit)

    def datareset(self):
        # print("IsRobotReal() : ", IsRobotReal())
        # print("IsIdle() : ", IsIdle())
        # print("IsCommandSockConnect() : ", IsCommandSockConnect())
        # print("IsDataSockConnect() : ", IsDataSockConnect())

        if(IsCommandSockConnect() & IsDataSockConnect()):
            self.IsConnected = True
            self.status_label.setStyleSheet("background-color: green")
            self.status_label.setText("connected")
            self.Connect_Button.setText("DisConnect")
        else:
            self.IsConnected = False
            self.status_label.setStyleSheet("background-color: red")
            self.status_label.setText("disconnected")
            self.Connect_Button.setText("Connect")


        if(self.IsConnected):
            if(self.robotReal.currentText() == "Idle"):
                if(self.CurrentMode != PG_MODE.SIMULATION):
                    self.CurrentMode = PG_MODE.SIMULATION
                    SetProgramMode(PG_MODE.SIMULATION)
                    print("MODE : ", self.CurrentMode)

            elif(self.robotReal.currentText() == "REAL"):
                if(self.CurrentMode != PG_MODE.REAL):
                    self.CurrentMode = PG_MODE.REAL
                    SetProgramMode(PG_MODE.REAL)
                    print("MODE : ", self.CurrentMode)

                
    def getCurrentPosition(self):
        joint, point = GetCurreJP()
        self.position_text.setText(str(joint) + "\n" + str(point))

        self.setPointText(point, self.move_x, self.move_y, self.move_z, self.move_rx, self.move_ry, self.move_rz)
        self.setJointText(joint, self.move_x_2, self.move_y_2, self.move_z_2, self.move_rx_2, self.move_ry_2, self.move_rz_2)
        self.setPointText(point, self.move_x_3, self.move_y_3, self.move_z_3, self.move_rx_3, self.move_ry_3, self.move_rz_3)
        self.setJointText(joint, self.move_x_4, self.move_y_4, self.move_z_4, self.move_rx_4, self.move_ry_4, self.move_rz_4)
        self.setPointText(point, self.move_x_5, self.move_y_5, self.move_z_5, self.move_rx_5, self.move_ry_5, self.move_rz_5)
        self.setPointText(point, self.move_x_6, self.move_y_6, self.move_z_6, self.move_rx_6, self.move_ry_6, self.move_rz_6)

    def setJointText(self, joint : Joint, j0 : QTextEdit, j1 : QTextEdit, j2 : QTextEdit, j3 : QTextEdit, j4 : QTextEdit, j5 : QTextEdit):
        j0.setText(str(round(joint.j0, 2)))
        j1.setText(str(round(joint.j1, 2)))
        j2.setText(str(round(joint.j2, 2)))
        j3.setText(str(round(joint.j3, 2)))
        j4.setText(str(round(joint.j4, 2)))
        j5.setText(str(round(joint.j5, 2)))


    def setPointText(self, point : Point, x : QTextEdit, y : QTextEdit, z : QTextEdit, rx : QTextEdit, ry : QTextEdit, rz : QTextEdit):
        x.setText(str(round(point.x, 2)))
        y.setText(str(round(point.y, 2)))
        z.setText(str(round(point.z, 2)))
        rx.setText(str(round(point.rx, 2)))
        ry.setText(str(round(point.ry, 2)))
        rz.setText(str(round(point.rz, 2)))

    def start_thread(self):
        ip_address = self.ip.toPlainText()
        self.thread = ToCBThread(ip_address)
        self.thread.start()

    def quit(self):
        QApplication.quit()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = QtController()
    window.show()
    sys.exit(app.exec_())
