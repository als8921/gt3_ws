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
        self.Connect_Button = QPushButton()
        self.robotReal = QComboBox()
        self.CurrentMode = None
        self.IsConnected = False
        self.init_ui()

    def init_ui(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        ui_file = os.path.join(current_dir, 'control.ui')

        self.timer = QTimer(self)
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.datareset)
        self.timer.start()

        uic.loadUi(ui_file, self)
        self.ip.setText('192.168.1.7')
        self.Connect_Button.clicked.connect(self.start_thread)
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
                    print(self.CurrentMode)

            elif(self.robotReal.currentText() == "REAL"):
                if(self.CurrentMode != PG_MODE.REAL):
                    self.CurrentMode = PG_MODE.REAL
                    SetProgramMode(PG_MODE.REAL)
                    print(self.CurrentMode)
                


        


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
