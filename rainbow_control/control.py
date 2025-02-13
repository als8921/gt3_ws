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
import testdata

class ToCBThread(QThread):
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

        self.moveL_Button : QPushButton
        self.moveJ_Button : QPushButton
        self.moveJL_Button : QPushButton
        self.moveJB_Button : QPushButton
        self.moveJB_Add_Button : QPushButton
        self.moveJB_Clear_Button : QPushButton
        self.movePB_Button : QPushButton
        self.movePB_Clear_Button : QPushButton
        self.movePB_Add_Button : QPushButton
        self.moveITPL_Button : QPushButton
        self.moveITPL_Clear_Button : QPushButton
        self.moveITPL_Add_Button : QPushButton

        # ComboBox
        self.robotReal : QComboBox
        self.movePB_OPTION : QComboBox
        self.movePB_RTYPE : QComboBox
        self.moveITPL_RTYPE : QComboBox

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

        self.moveL_speed : QTextEdit
        self.moveJ_speed : QTextEdit
        self.moveJL_speed : QTextEdit
        self.moveJB_speed : QTextEdit
        self.movePB_speed : QTextEdit
        self.moveITPL_speed : QTextEdit
        
        self.moveL_acc : QTextEdit
        self.moveJ_acc : QTextEdit
        self.moveJL_acc : QTextEdit
        self.moveJB_acc : QTextEdit
        self.movePB_acc : QTextEdit
        self.moveITPL_acc : QTextEdit

        self.movePB_quantity : QTextEdit

        # Label
        self.status_label : QLabel

        self.init_ui()
        
        self.MOVE_L_POS = (self.move_x, self.move_y, self.move_z, self.move_rx, self.move_ry, self.move_rz)
        self.MOVE_J_POS = (self.move_x_2, self.move_y_2, self.move_z_2, self.move_rx_2, self.move_ry_2, self.move_rz_2)
        self.MOVE_JL_POS = (self.move_x_3, self.move_y_3, self.move_z_3, self.move_rx_3, self.move_ry_3, self.move_rz_3)
        self.MOVE_JB_POS = (self.move_x_4, self.move_y_4, self.move_z_4, self.move_rx_4, self.move_ry_4, self.move_rz_4)
        self.MOVE_PB_POS = (self.move_x_5, self.move_y_5, self.move_z_5, self.move_rx_5, self.move_ry_5, self.move_rz_5)
        self.MOVE_ITPL_POS = (self.move_x_6, self.move_y_6, self.move_z_6, self.move_rx_6, self.move_ry_6, self.move_rz_6)

    def init_ui(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        ui_file = os.path.join(current_dir, 'control.ui')

        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.dataReset)
        self.timer.start()

        uic.loadUi(ui_file, self)
        self.ip.setText('192.168.1.7')
        
        self.status_label.setStyleSheet("background-color: green")

        # Button
        self.Quit_Button.clicked.connect(self.quit)
        self.Connect_Button.clicked.connect(self.start_thread)
        self.getPosition_Button.clicked.connect(self.getCurrentPosition)
        self.moveL_Button.clicked.connect(self.MOVE_L)
        self.moveJ_Button.clicked.connect(self.MOVE_J)
        self.moveJL_Button.clicked.connect(self.MOVE_JL)
        self.moveJB_Button.clicked.connect(self.MOVE_JB)
        self.moveJB_Clear_Button.clicked.connect(self.CLEAR_JB)
        self.moveJB_Add_Button.clicked.connect(self.ADD_JB)


        self.movePB_Button.clicked.connect(self.MOVE_PB)
        self.movePB_Clear_Button.clicked.connect(self.CLEAR_PB)
        self.movePB_Add_Button.clicked.connect(self.ADD_PB)

        self.moveITPL_Button.clicked.connect(self.MOVE_ITPL)
        self.moveITPL_Clear_Button.clicked.connect(self.CLEAR_ITPL)
        self.moveITPL_Add_Button.clicked.connect(self.ADD_ITPL)

    def MOVE_L(self):
        pos = self.getPointFromText(self.MOVE_L_POS)
        try:
            speed = float(self.moveL_speed.toPlainText())
            acc = float(self.moveL_acc.toPlainText())
        except:
            print("speed, acc 데이터 오류")
            return
        MoveL(pos, speed, acc)

    def MOVE_J(self):
        pos = self.getJointFromText(self.MOVE_J_POS)
        try:
            speed = float(self.moveJ_speed.toPlainText())
            acc = float(self.moveJ_acc.toPlainText())
        except:
            print("speed, acc 데이터 오류")
            return
        MoveJ(pos, speed, acc)
    
    def MOVE_JL(self):
        pos = self.getPointFromText(self.MOVE_JL_POS)
        try:
            speed = float(self.moveJL_speed.toPlainText())
            acc = float(self.moveJL_acc.toPlainText())
        except:
            print("speed, acc 데이터 오류")
            return
        MoveJL(pos, speed, acc)

    ############# MOVE_JB #############
    def MOVE_JB(self):
        try:
            speed = float(self.moveJB_speed.toPlainText())
            acc = float(self.moveJB_acc.toPlainText())
        except:
            print("speed, acc 데이터 오류")
            return
        MoveJB_Run(speed, acc)
    
    def ADD_JB(self):
        pos = self.getJointFromText(self.MOVE_JB_POS)
        # MoveJB_Add(pos)

        ## TEST
        for joint in testdata.joint_list:
            MoveJB_Add(*joint)

    def CLEAR_JB(self):
        MoveJB_Clear()

    ############# MOVE_PB #############
    def MOVE_PB(self):
        try:
            acc = float(self.movePB_acc.toPlainText())
        except:
            print("acc 데이터 오류")
            return
    
        rtype = BLEND_RTYPE.INTENDED
        if(self.movePB_RTYPE.currentText() == "INTENDED"):
            rtype = BLEND_RTYPE.INTENDED
        elif(self.movePB_RTYPE.currentText() == "CONSTANT"):
            rtype = BLEND_RTYPE.CONSTANT
        MovePB_Run(acc, rtype)


    def ADD_PB(self):
        pos = self.getPointFromText(self.MOVE_PB_POS)
        try:
            speed = float(self.movePB_speed.toPlainText())
            quantity = float(self.movePB_quantity.toPlainText())
        except:
            print("speed, quantity 데이터 오류")
            return
        
        option = BLEND_OPTION.RATIO
        if(self.movePB_OPTION.currentText() == "RATIO"):
            option = BLEND_OPTION.RATIO
        elif(self.movePB_OPTION.currentText() == "DISTANCE"):
            option = BLEND_OPTION.DISTANCE

        ### REAL
        # MovePB_Add(pos, speed, option, quantity)

        ### TEST
        for point in testdata.point_list:
            MovePB_Add(*point, speed, option, quantity)

    def CLEAR_PB(self):
        MovePB_Clear()
        
    ############# MOVE_ITPL #############
    def MOVE_ITPL(self):
        try:
            acc = float(self.moveITPL_acc.toPlainText())
        except:
            print("acc 데이터 오류")
            return
        
        rtype = ITPL_RTYPE.INTENDED
        if(self.moveITPL_RTYPE.currentText() == "INTENDED"):
            rtype = ITPL_RTYPE.INTENDED
        elif(self.moveITPL_RTYPE.currentText() == "CONSTANT"):
            rtype = ITPL_RTYPE.CONSTANT
        elif(self.moveITPL_RTYPE.currentText() == "SMOOTH"):
            rtype = ITPL_RTYPE.SMOOTH
        elif(self.moveITPL_RTYPE.currentText() == "CA_INTENDED"):
            rtype = ITPL_RTYPE.CA_INTENDED
        elif(self.moveITPL_RTYPE.currentText() == "CA_CONSTANT"):
            rtype = ITPL_RTYPE.CA_CONSTANT
        elif(self.moveITPL_RTYPE.currentText() == "CA_SMOOTH"):
            rtype = ITPL_RTYPE.CA_SMOOTH
        elif(self.moveITPL_RTYPE.currentText() == "RESERVED1"):
            rtype = ITPL_RTYPE.RESERVED1
        elif(self.moveITPL_RTYPE.currentText() == "RESERVED2"):
            rtype = ITPL_RTYPE.RESERVED2
        elif(self.moveITPL_RTYPE.currentText() == "RESERVED3"):
            rtype = ITPL_RTYPE.RESERVED3

        MoveITPL_Run(acc, rtype)


    def ADD_ITPL(self):
        pos = self.getPointFromText(self.MOVE_ITPL_POS)
        try:
            speed = float(self.moveITPL_speed.toPlainText())
        except:
            print("speed 데이터 오류")
            return
        # MoveITPL_Add(pos, speed)

        ### TEST
        for point in testdata.point_list:
            MoveITPL_Add(*point, speed)


    def CLEAR_ITPL(self):
        MoveITPL_Clear()


    def dataReset(self):
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
        # self.position_text.setText(str(joint) + "\n" + str(point))

        joint_text = ", ".join([str(joint.j0), str(joint.j1), str(joint.j2), str(joint.j3), str(joint.j4), str(joint.j5)])
        point_text = ", ".join([str(point.x), str(point.y), str(point.z), str(point.rx), str(point.ry), str(point.rz)])
        self.position_text.setText(joint_text + "\n" + point_text)

        self.setPointToText(point, self.MOVE_L_POS)
        self.setJointToText(joint, self.MOVE_J_POS)
        self.setPointToText(point, self.MOVE_JL_POS)
        self.setJointToText(joint, self.MOVE_JB_POS)
        self.setPointToText(point, self.MOVE_PB_POS)
        self.setPointToText(point, self.MOVE_ITPL_POS)

    def setJointToText(self, joint : Joint, move_pos : list):
        move_pos[0].setText(str(round(joint.j0, 2)))
        move_pos[1].setText(str(round(joint.j1, 2)))
        move_pos[2].setText(str(round(joint.j2, 2)))
        move_pos[3].setText(str(round(joint.j3, 2)))
        move_pos[4].setText(str(round(joint.j4, 2)))
        move_pos[5].setText(str(round(joint.j5, 2)))


    def setPointToText(self, point : Point, move_pos : list):
        move_pos[0].setText(str(round(point.x, 2)))
        move_pos[1].setText(str(round(point.y, 2)))
        move_pos[2].setText(str(round(point.z, 2)))
        move_pos[3].setText(str(round(point.rx, 2)))
        move_pos[4].setText(str(round(point.ry, 2)))
        move_pos[5].setText(str(round(point.rz, 2)))


    def getPointFromText(self, move_pos : list):
        point = Point()
        try:
            point.x = float(move_pos[0].toPlainText())
            point.y = float(move_pos[1].toPlainText())
            point.z = float(move_pos[2].toPlainText())
            point.rx = float(move_pos[3].toPlainText())
            point.ry = float(move_pos[4].toPlainText())
            point.rz = float(move_pos[5].toPlainText())

        except:
            print("데이터 형태가 잘못되었습니다.")
        return point
    
    def getJointFromText(self, move_pos : list):
        joint = Joint()
        try:
            joint.j0 = float(move_pos[0].toPlainText())
            joint.j1 = float(move_pos[1].toPlainText())
            joint.j2 = float(move_pos[2].toPlainText())
            joint.j3 = float(move_pos[3].toPlainText())
            joint.j4 = float(move_pos[4].toPlainText())
            joint.j5 = float(move_pos[5].toPlainText())

        except:
            print("데이터 형태가 잘못되었습니다.")
        return joint

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
