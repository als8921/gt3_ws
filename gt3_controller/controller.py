import pickle as plk
import os
import numpy as np
import sys
import rclpy
import threading
import pc_process
import cv2
import time
import math

from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
import std_msgs.msg as std_msgs
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, PointField, LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tcp_to_ros2 import Socket
from message_filters import ApproximateTimeSynchronizer, Subscriber

from multiprocessing import Process, Pipe, Queue
from mb_process import Mb_process
from PyQt5.QtWidgets import *
from PyQt5 import uic, QtGui
from PyQt5.QtCore import Qt, QPoint, QRect, QSize
from PyQt5.QtGui import QStandardItemModel
from PyQt5.QtGui import QStandardItem

#from flask import Flask, jsonify, abort

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

form_class = uic.loadUiType("viewer.ui")[0]

a_pipe, b_pipe = Pipe(duplex=True)     # imu - mb_process
a_pc, b_pc = Pipe(duplex=True)         # mb_server - mb_process
a_image, b_image = Pipe()
a_vertices, b_vertices = Pipe()
a_robot, b_robot = Pipe()

t0 = Queue()      # mb_server - mb_process  <command>
t1 = Queue()      # sub_loop - mb_process   <imu data>
t2 = Queue()      # sub_loop - mb_process   <robot data>
t3 = Queue()
t5 = Queue()      # sub_loop - window       <current pose>
t6 = Queue()      # robot state

class cmd_Subscriber(Node):

    def __init__(self, t0, t1, t2, t3, t5, t6, a_pc, a_pipe, a_image, a_vertices, a_robot):
        super().__init__('gtm_rcv')
        
        self.sub_gtm = self.create_subscription(std_msgs.String, '/robot_to_mobile', self.robot_Callback, 10)
        self.sub_image = self.create_subscription(Image, '/detectImage', self.image_Callback, qos_profile_sensor_data)    #image
        self.sub_pointcloud = self.create_subscription(PointCloud2, '/pointcloud_topic', self.pc_Callback, qos_profile_sensor_data)
        #self.sub_rear_lidar = self.create_subscription(LaserScan, '/rear_lidar/scan', self.rear_scan_callback, qos_profile_sensor_data)
        self.sub_front_lidar = self.create_subscription(LaserScan, '/front_lidar/scan', self.front_scan_callback, qos_profile_sensor_data)
        #self.sub_imu = self.create_subscription(std_msgs.String, '/imu/data', self.imu_Callback, qos_profile_sensor_data)
        self.sub_odom = self.create_subscription(std_msgs.String, '/fastlio_odom', self.odom_Callback, qos_profile_sensor_data)
        self.sub_robotis = self.create_subscription(std_msgs.String, '/robotis_joint', self.robotis_Callback, 10)
        self.sub_unity = self.create_subscription(std_msgs.String, '/unity/cmd', self.unity_Callback, 10)
        self.sub_current = self.create_subscription(std_msgs.String, '/current_pose', self.current_Callback,
                                                    qos_profile_sensor_data)

        self.tf_broadcaster = TransformBroadcaster(self, 10)

        #self.lidar_sub = Subscriber(self, LaserScan, '/front_lidar/scan')
        #self.odom_sub = Subscriber(self, Odometry, '/odom')

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.target_frame = self.declare_parameter('camera', 'tcp_camera_link').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.control_node = rclpy.create_node('pose2')
        self.odom_node = rclpy.create_node('pose2')
        
        self.pose_pub = self.control_node.create_publisher(std_msgs.String, '/robot_pose', 10)
        self.scan_pub = self.control_node.create_publisher(LaserScan, '/scan', 10)
        #self.odom_pub = self.odom_node.create_publisher(Odometry, '/odom3', qos_profile_sensor_data)

        self.msg = std_msgs.String()
        self.scan_msg = LaserScan()
        self.odom_msg = Odometry()

        self.t0 = t0
        self.t1 = t1
        self.t2 = t2
        self.t3 = t3
        self.t5 = t5
        self.t6 = t6
        self.pipe = a_pc
        self.a_pipe = a_pipe
        self.a_image = a_image
        self.a_vertices = a_vertices
        self.a_robot = a_robot
        self.bridge = CvBridge()
        self.emergency = 0
        self.rear_lidar_data = 0
        self.front_lidar_data = 0
        self.pose = '0;0;0'
        self.transform = [0,0,0,0,0,0]
        self.vertices = 0
        self.servoJ_mode = False

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_z = 0.0
        self.odom_r = 0.0
        self.odom_p = 0.0
        self.odom_y = 0.0
        self.odom_w = 0.0
        self.odom_t_x = 0.0
        self.odom_t_y = 0.0
        self.odom_t_z = 0.0

        self.lidar_timestamp = None

    
    def timer_callback(self):
        from_frame_rel = self.target_frame
        to_frame_rel = 'base_link'
        try:

            t_odm = TransformStamped()
            t_odm.header.stamp = self.get_clock().now().to_msg()
            t_odm.header.frame_id = 'odom'
            t_odm.child_frame_id = 'base_footprint'

            t_odm.transform.translation.x = self.odom_x
            t_odm.transform.translation.y = self.odom_y
            t_odm.transform.translation.z = self.odom_z

            t_odm.transform.rotation.x = self.odom_r
            t_odm.transform.rotation.y = self.odom_p
            t_odm.transform.rotation.z = self.odom_y
            t_odm.transform.rotation.w = self.odom_w

            self.tf_broadcaster.sendTransform(t_odm)

            t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
            trans_x = round(t.transform.translation.x, 2)
            trans_y = round(t.transform.translation.y, 2)
            trans_z = round(t.transform.translation.z, 2)

            # print("t.transform.translation.x = ", round(t.transform.translation.x, 2))
            # print("t.transform.translation.y = ", round(t.transform.translation.y, 2))
            # print("t.transform.translation.z = ", round(t.transform.translation.z, 2))

            roll, pitch, yaw = self.euler_from_quaternion(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w)
            rot_x = round(roll, 2)
            # if rot_x < -3:
            #    rot_y = round(pitch+math.pi/12, 2)
            # else:
            rot_y = round(pitch - math.radians(14.0), 2)

            rot_z = round(yaw, 2)

            # print("roll", round(math.degrees(roll), 2))
            # print("pitch",round(math.degrees(pitch), 2))
            # print("yaw",round(math.degrees(yaw), 2))
            self.transform = [trans_y * (-1), trans_x, trans_z, rot_x, rot_y, rot_z]
            
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        return

    def euler_from_quaternion(self, x, y, z, w):    #chunk = read(handle, remaining)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
            
        t2 = math.sqrt(1 +2.0 * (w * y - z * x))
        t2_1 = math.sqrt(1- 2.0 * (w * y - z * x))    
        pitch_y = -(2 * math.atan2(t2, t2_1)-math.pi/2)
        #t2 = +1.0 if t2 > +1.0 else t2
        #t2 = -1.0 if t2 < -1.0 else t2
        #pitch_y = math.asin(t2)
            
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
            
        return roll_x, pitch_y, yaw_z # in radians
    """
    def rear_scan_callback(self, msg):
        self.lidar_timestamp = msg.header.stamp
        print(self.lidar_timestamp)
        #self.rear_lidar_data = msg.ranges
        #self.rear_lidar_data.append(msg.angle_min)
        #self.rear_lidar_data.append(msg.angle_increment)
    """
    def front_scan_callback(self, msg):
        #self.lidar_timestamp = msg.header.stamp

        self.scan_msg = msg
        self.scan_msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_msg.header.frame_id = 'f_scan'
        self.scan_pub.publish(self.scan_msg)


    def odom_Callback(self, msg):
        self.odom_data = msg.data
        q = self.odom_data.split(';')
        
        try:
            roll, pitch, yaw = self.euler_from_quaternion(float(q[3]), float(q[4]), float(q[5]), float(q[6]))
            #q[1] = float(q[1]) + math.sin(yaw) * 0.25
            #q[0] = float(q[0]) + math.cos(yaw) * 0.25 - 0.25
            self.odom_x = float(q[0])
            self.odom_y = float(q[1])
            self.odom_z = float(q[2])
            self.odom_r = float(q[3])
            self.odom_p = float(q[4])
            self.odom_y = float(q[5])
            self.odom_w = float(q[6])
            """
            t_odm = TransformStamped()
            t_odm.header.stamp = self.get_clock().now().to_msg()
            t_odm.header.frame_id = 'odom'
            t_odm.child_frame_id = "base_link"

            t_odm.transform.translation.x = float(q[0])
            t_odm.transform.translation.y = float(q[1])
            t_odm.transform.translation.z = float(q[2])

            t_odm.transform.rotation.x = float(q[3])
            t_odm.transform.rotation.y = float(q[4])
            t_odm.transform.rotation.z = float(q[5])
            t_odm.transform.rotation.w = float(q[6])

            self.tf_broadcaster.sendTransform(t_odm)
            """
            #print('before :' , q[0], q[1])

            #print('after :', q[0], q[1])

            imu_data = str(q[0]) + ';' + str(q[1]) + ';' + q[2] + ';' + str(math.degrees(roll)) + ';' + str(math.degrees(pitch)) + ';' + str(math.degrees(yaw))

            if self.t1.qsize() != 0:
                get = self.t1.get()
                if get == 'imu':
                    #self.a_pipe.send(self.rear_lidar_data)
                    #self.a_pipe.send(self.front_lidar_data)
                    self.a_pipe.send(imu_data)
        except Exception as e:
            print('*****WARNING*****')
            print(e)

    def robotis_Callback(self, msg):
        joint = eval(msg.data)
        #print(joint[0])
        if self.servoJ_mode:
            j0 = str(round(joint[0]+163, 2))
            j1 = str(round(-joint[1], 2))
            j2 = str(round(-joint[2]-90, 2))
            j3 = str(round(-joint[3]+80, 2))
            if abs(joint[4]) < 0.01:
                j4 = str(round(joint[4]*5000-71.32, 2))
            else:
                j4 = '-71.32'
            self.msg.data = 'servo;J;[' + j0 + ',' + j1 + ',' + j2 + ',' + j3 + ',' + j4 +  ',88.07, ' + str(0.005) + ',' + str(0.025) + ',' + str(0.05) + ',' + str(0.005) + ']'
            self.pose_pub.publish(self.msg)
        if self.t5.qsize() != 0:
            get = self.t5.get()
            if get == 'ServoJ':
                if self.servoJ_mode:
                    self.servoJ_mode = False
                    print('Servo Mode OFF!!!')
                else:
                    self.servoJ_mode = True
                    print('Servo Mode ON!!!')

    def unity_Callback(self, msg):
        unity_cmd = msg.data.split(';')
        print('Order From UNITY : ', unity_cmd)
        cmd = unity_cmd[0]
        try:
            if len(unity_cmd) == 5:
                distance = unity_cmd[1]
                rpm = unity_cmd[2]
                r_a = unity_cmd[3]
                s_a = unity_cmd[4]

                if cmd == 'go_forward':
                    self.t0.put(cmd)
                    self.pipe.send(distance)
                    self.pipe.send(rpm)
                elif cmd == 'go_backward':
                    self.t0.put(cmd)
                    self.pipe.send(distance)
                    self.pipe.send(rpm)
                elif cmd == 'left_forward':
                    self.t0.put(cmd)
                    self.pipe.send(r_a)
                    self.pipe.send(s_a)
                    self.pipe.send(rpm)
                elif cmd == 'right_forward':
                    self.t0.put(cmd)
                    self.pipe.send(r_a)
                    self.pipe.send(s_a)
                    self.pipe.send(rpm)
                elif cmd == 'left_backward':
                    self.t0.put(cmd)
                    self.pipe.send(r_a)
                    self.pipe.send(s_a)
                    self.pipe.send(rpm)
                elif cmd == 'right_backward':
                    self.t0.put(cmd)
                    self.pipe.send(r_a)
                    self.pipe.send(s_a)
                    self.pipe.send(rpm)

            elif cmd == 'stop' or cmd == 'pause' or cmd == 'resume':
                self.t0.put(cmd)

            elif cmd == 'frontview' or cmd == 'floorview' or cmd == 'leftview' or cmd == 'ceilingview':
                self.t3.put(cmd)

            elif cmd == 'fastlio_map':
                self.t3.put(cmd)
                self.t3.put(unity_cmd[1].strip())

            elif cmd == 'cancel':
                self.t3.put(cmd)

        except:
            print('Wrong Order!!!')


    """
    def imu_Callback(self, msg):
        self.imu_data = msg.data
        #print(self.imu_data)
        if self.t1.qsize() != 0: 
            get = self.t1.get()
            if get == 'imu':
                
                self.a_pipe.send(self.rear_lidar_data)
                self.a_pipe.send(self.front_lidar_data)
                self.a_pipe.send(self.imu_data)
    """

    def robot_Callback(self, msg):
        robot_cmd = msg.data
        if robot_cmd == 'detected':
            self.t2.put('detected')

    def image_Callback(self, data):
        #self.image_data = msg
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        #cv_save = cv2.imwrite('output.jpg',cv_image)
        print('img')
        self.a_image.send(cv_image)

    def pc_Callback(self, msg):
        try:
            #pc2_msg = msg #PointCloud2()
            msg.header = std_msgs.Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'tcp_camera_link'

            self.get_logger().info('Received a PointCloud2 message')

            vertices = self.pointcloud2_to_numpy(msg).tolist()
            #position2 = vertices[-1]
            #del vertices[-1]
            #position1 = vertices[-1]
            #del vertices[-1]
            #print(position1, position2)

            self.vertices = np.array(vertices).reshape(24, 43, -1)  #(24, 43, -1)    #(72, 128, -1)
        except Exception as e:
            print(e)


    def pointcloud2_to_numpy(self, cloud_msg):

        # cloud_msg의 data 배열을 numpy 배열로 변환
        dtype_list = [('x', np.float32), ('y', np.float32), ('z', np.float32)]  # 수정 가능
        cloud_arr = np.frombuffer(cloud_msg.data, dtype=dtype_list)
        #cloud_arr = np.reshape(cloud_arr, (int(len(cloud_msg.data) / cloud_msg.point_step), len(dtype_list)))
        return cloud_arr

    def current_Callback(self, msg):

        self.pose = msg.data
        #print(self.pose)
        if self.t6.qsize() != 0:
            get = self.t6.get()
            #print(get)
            if get == 'state':
                data = self.pose.split(';')[2]
                self.a_robot.send(data)
            elif get == 'vertices':
                self.a_vertices.send(self.vertices)
                self.a_vertices.send(self.pose)
                self.a_vertices.send(self.transform)

        robot = msg.data.split(';')
        tcp = robot[0].split(',')

        t_tcp = TransformStamped()
        t_tcp.header.stamp = self.get_clock().now().to_msg()
        t_tcp.header.frame_id = "robotarm_link"
        t_tcp.child_frame_id = "tcp_link"

        t_tcp.transform.translation.x = (float(tcp[0].replace('(', '', 1))+25) / 1000
        t_tcp.transform.translation.y = float(tcp[1]) / 1000
        t_tcp.transform.translation.z = (float(tcp[2])-20) / 1000

        roll = math.radians(float(tcp[3]))
        pitch = math.radians(float(tcp[4]))
        yaw = math.radians(math.trunc(float(tcp[5].replace(')', '', 1))))

        #print('yaw = ', yaw)
        #print('pitch = ', pitch)
        #print('roll = ', roll)
        q = self.quaternion_from_euler(roll, pitch, yaw)
        # q = self.quaternion_from_euler(float(tcp[5].replace(')','',1)), float(tcp[4]), float(tcp[3]))
        # q = self.quaternion_from_euler(float(tcp[3]), float(tcp[4]), float(tcp[5].replace(')','',1)))
        t_tcp.transform.rotation.x = q[0]
        t_tcp.transform.rotation.y = q[1]
        t_tcp.transform.rotation.z = q[2]
        t_tcp.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t_tcp)

        t_camera = TransformStamped()
        t_camera.header.stamp = self.get_clock().now().to_msg()
        t_camera.header.frame_id = "tcp_link"
        t_camera.child_frame_id = "tcp_camera_link"

        t_camera.transform.translation.x = 0.03
        t_camera.transform.translation.y = -0.3
        t_camera.transform.translation.z = -0.01

        q2 = self.quaternion_from_euler(math.pi / 2, 0.0, -math.pi / 2)
        t_camera.transform.rotation.x = q2[0]
        t_camera.transform.rotation.y = q2[1]
        t_camera.transform.rotation.z = q2[2]
        t_camera.transform.rotation.w = q2[3]

        self.tf_broadcaster.sendTransform(t_camera)

        #self.timer_callback()

    def quaternion_from_euler(self, ai, aj, ak):
        (yaw, pitch, roll) = (ak, aj, ai)
        q = np.empty((4,))
        q[0] = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        q[1] = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
            yaw / 2)
        q[2] = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
            yaw / 2)
        q[3] = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        return q


class WindowClass(QMainWindow, form_class):
    def __init__(self, t0, t1, t3, t5, t6, a_pc, b_image, b_vertices, b_robot):
        super().__init__()
        self.setupUi(self)
        self.rubberBand = QRubberBand(QRubberBand.Rectangle, self)
        self.origin = QPoint()

        self.model = QStandardItemModel()
        self.setMouseTracking(True)

        #self.setGeometry(1300, 200, 550, 82
                             # q : D455
        self.t0 = t0                     # mb_process
        self.t1 = t1                     # t : pc_process

        self.t3 = t3                  # 3d process
        self.t4 = Queue()                  # 2d
        self.t5 = t5

        self.point_adjust = 0
        self.lcdNumber.display('0')

        self.pipe = a_pc
        self.b_image = b_image
        self.b_vertices = b_vertices

        self.add_task = False
        self.auto_scan = False

        self.mouse_tracking = []
        
        self.image = pc_process.ShowImg(self.b_image, self.label_5, self.t4)

        self.map = pc_process.PlotCanvas('map', self.pipe, self.frame, 13, 8, 1, 1, None, self.t0, self.t3, None, None)
        self.points = pc_process.PlotCanvas('vertices', self.b_vertices,self.frame_2, 11, 8, 1, 1, None, self.t0, self.t3, t6, b_robot)

        s = threading.Thread(target=self.map.Lidar)
        s.start()
        p = threading.Thread(target=self.image._2D)
        p.start()
        v = threading.Thread(target=self.points._3D)
        v.start()


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

        self.pushButton.clicked.connect(self.go_forward)
        self.pushButton_9.clicked.connect(self.go_backward)
        self.pushButton_3.clicked.connect(self.left_forward)
        self.pushButton_4.clicked.connect(self.right_forward)
        self.pushButton_12.clicked.connect(self.left_backward)
        self.pushButton_13.clicked.connect(self.right_backward)
        self.pushButton_2.clicked.connect(self.quit)
        self.pushButton_5.clicked.connect(self.drive)
        self.pushButton_43.clicked.connect(self.drive2)
        self.pushButton_44.clicked.connect(self.drive3)
        self.pushButton_6.clicked.connect(self.fastlio_map)
        self.pushButton_7.clicked.connect(self.paint)
        #self.pushButton_8.clicked.connect(self.steer_minus)
        #self.pushButton_11.clicked.connect(self.steer_plus)
        self.pushButton_15.clicked.connect(self.IK_pose)
        self.pushButton_23.clicked.connect(self.FK_pose)
        self.pushButton_16.clicked.connect(self.reconnect)
        self.pushButton_17.clicked.connect(self.robot_home)
        self.pushButton_14.clicked.connect(self.auto)
        self.pushButton_10.clicked.connect(self.stop)
        self.pushButton_20.clicked.connect(self.topview)
        self.pushButton_19.clicked.connect(self.right_cam)
        self.pushButton_18.clicked.connect(self.arm_tilt)
        self.pushButton_21.clicked.connect(self.arm_cam_left)
        self.pushButton_22.clicked.connect(self.floorview)
        self.pushButton_24.clicked.connect(self.emergency)
        self.pushButton_25.clicked.connect(self.leftwall)
        self.pushButton_26.clicked.connect(self.leftcorner)
        self.pushButton_27.clicked.connect(self.frontwall)
        self.pushButton_28.clicked.connect(self.setting)
        self.pushButton_29.clicked.connect(self.frontview)
        self.pushButton_42.clicked.connect(self.leftview)
        self.pushButton_30.clicked.connect(self.refresh)
        self.pushButton_31.clicked.connect(self.clear)
        self.pushButton_32.clicked.connect(self.play)
        self.pushButton_33.clicked.connect(self.set_current)
        self.pushButton_34.clicked.connect(self.turn90_left)
        self.pushButton_35.clicked.connect(self.ceilingview)
        self.pushButton_36.clicked.connect(self.cover_up)
        self.pushButton_37.clicked.connect(self.cover_down)
        self.pushButton_38.clicked.connect(self.cover_stop)
        self.pushButton_39.clicked.connect(self.lift_up)
        self.pushButton_40.clicked.connect(self.lift_down)
        self.pushButton_41.clicked.connect(self.lift_stop)
        self.pushButton_45.clicked.connect(self.adjust_left)
        self.pushButton_46.clicked.connect(self.adjust_right)
        self.pushButton_47.clicked.connect(self.delete_task)
        self.pushButton_48.clicked.connect(self.add_trajectory)
        self.pushButton_49.clicked.connect(self.execute_task)
        self.pushButton_50.clicked.connect(self.trajectory_summary)
        self.pushButton_51.clicked.connect(self.m_home)
        self.pushButton_52.clicked.connect(self.autoscan)
        self.pushButton_53.clicked.connect(self.tracking)
        self.pushButton_54.clicked.connect(self.rail_left)
        self.pushButton_55.clicked.connect(self.rail_center)
        self.pushButton_56.clicked.connect(self.rail_right)

        self.radioButton.clicked.connect(self.plane)
        self.radioButton_2.clicked.connect(self.curved)
        self.radioButton_3.clicked.connect(self.front)
        self.radioButton_4.clicked.connect(self.rear)
        self.radioButton_5.clicked.connect(self.tower)
        self.radioButton_6.clicked.connect(self.arm)

        self.checkBox.clicked.connect(self.side)
        self.checkBox_2.clicked.connect(self.real)
        self.checkBox_3.clicked.connect(self.pull)
        self.checkBox_4.clicked.connect(self.continuous)
        self.checkBox_6.clicked.connect(self.waypoint_0)
        self.checkBox_7.clicked.connect(self.waypoint_1)
        self.checkBox_8.clicked.connect(self.waypoint_2)
        self.checkBox_9.clicked.connect(self.waypoint_3)
        self.checkBox_10.clicked.connect(self.waypoint_4)
        self.checkBox_11.clicked.connect(self.waypoint_5)
        self.checkBox_12.clicked.connect(self.waypoint_6)
        self.checkBox_13.clicked.connect(self.waypoint_7)
        self.checkBox_14.clicked.connect(self.waypoint_8)
        self.checkBox_15.clicked.connect(self.waypoint_9)
        self.checkBox_16.clicked.connect(self.wall)
        self.checkBox_17.clicked.connect(self.addtask)
        self.checkBox_18.clicked.connect(self.pitch_error)


        # t0 mb_process
        # t3 pc_process

    def set_current(self):
        self.t3.put('current_pose')
        while True:
            time.sleep(1)
            #print(self.t3.qsize())
            pose = self.t3.get()
            if pose == 'current_pose':
                self.t3.put('current_pose')
            else:    
                print('current pose',pose)
                if pose != 0:
                    pose = pose.split(';')
                
                    if len(pose) > 1:
                        tcp = eval(pose[0])
                        jnt = eval(pose[1])
                        
                        self.lineEdit.setText(str(round(float(tcp[0]),2)))
                        self.lineEdit_2.setText(str(round(float(tcp[1]),2)))
                        self.lineEdit_3.setText(str(round(float(tcp[2])+985,2)))
                        self.lineEdit_4.setText(str(round(float(tcp[3]),2)))
                        self.lineEdit_5.setText(str(round(float(tcp[4]),2)))
                        self.lineEdit_6.setText(str(round(float(tcp[5]),2)))
                        
                        self.lineEdit_7.setText(str(round(float(jnt[0]),2)))
                        self.lineEdit_8.setText(str(round(float(jnt[1]),2)))
                        self.lineEdit_9.setText(str(round(float(jnt[2]),2)))
                        self.lineEdit_10.setText(str(round(float(jnt[3]),2)))
                        self.lineEdit_11.setText(str(round(float(jnt[4]),2)))
                        self.lineEdit_12.setText(str(round(float(jnt[5]),2)))
                break
            
    def refresh(self):
        self.t0.put('refresh')

    def play(self):
        self.t3.put('play')

    def clear(self):
        self.t3.put('clear')

    def setting(self):
        self.t3.put('setting')
        m_dist = self.lineEdit_17.text()
        m_height = self.lineEdit_18.text()
        m_speed = self.lineEdit_19.text()
        m_accel = self.lineEdit_20.text()
        m_waypoints = self.lineEdit_22.text()
        p_height = self.lineEdit_23.text()
        l_height = self.lineEdit_24.text()
        
        m_setting = [m_dist, m_height, m_speed, m_accel, m_waypoints, p_height, l_height]
        self.t3.put(m_setting)

    def side(self):
        if self.checkBox.isChecked():
            self.t3.put('side_c')
        else:
            self.t3.put('side_nc')  

    def real(self):
        if self.checkBox_2.isChecked():
            self.t3.put('real')
        else:
            self.t3.put('simul')    

    def pull(self):
        if self.checkBox_3.isChecked():
            self.t3.put('pull')
        else:
            self.t3.put('unpull')

    def continuous(self):
        if self.checkBox_4.isChecked():
            self.t3.put('continuous')
        else:
            self.t3.put('point')

    def wall(self):
        if self.checkBox_16.isChecked():
            self.t3.put('wall')
        else:
            self.t3.put('not_wall')

    def addtask(self):
        if self.checkBox_17.isChecked():
            self.add_task = True
        else:
            self.add_task = False

    def pitch_error(self):
        if self.checkBox_18.isChecked():
            self.t3.put('pitch+')
        else:
            self.t3.put('pitch-')

    def waypoint_0(self):
        if self.checkBox_6.isChecked():
            self.t3.put('waypoint_0')
        else:
            self.t3.put('del_0')

    def waypoint_1(self):
        if self.checkBox_7.isChecked():
            self.t3.put('waypoint_1')
        else:
            self.t3.put('del_1')

    def waypoint_2(self):
        if self.checkBox_8.isChecked():
            self.t3.put('waypoint_2')
        else:
            self.t3.put('del_2')

    def waypoint_3(self):
        if self.checkBox_9.isChecked():
            self.t3.put('waypoint_3')
        else:
            self.t3.put('del_3')

    def waypoint_4(self):
        if self.checkBox_10.isChecked():
            self.t3.put('waypoint_4')
        else:
            self.t3.put('del_4')

    def waypoint_5(self):
        if self.checkBox_11.isChecked():
            self.t3.put('waypoint_5')
        else:
            self.t3.put('del_5')

    def waypoint_6(self):
        if self.checkBox_12.isChecked():
            self.t3.put('waypoint_6')
        else:
            self.t3.put('del_6')

    def waypoint_7(self):
        if self.checkBox_13.isChecked():
            self.t3.put('waypoint_7')
        else:
            self.t3.put('del_7')

    def waypoint_8(self):
        if self.checkBox_14.isChecked():
            self.t3.put('waypoint_8')
        else:
            self.t3.put('del_8')

    def waypoint_9(self):
        if self.checkBox_15.isChecked():
            self.t3.put('waypoint_9')
        else:
            self.t3.put('del_9')

    def adjust_left(self):
        self.t3.put('adjust_left')
        self.point_adjust -= 1
        if self.point_adjust < -5:
            self.point_adjust = -5
        self.lcdNumber.display(str(self.point_adjust))

    def adjust_right(self):
        self.t3.put('adjust_right')
        self.point_adjust += 1
        if self.point_adjust > 5:
            self.point_adjust = 5
        self.lcdNumber.display(str(self.point_adjust))

    def add_trajectory(self):
        self.t3.put('add_trajectory')
        time.sleep(2)
        with open('task.plk', 'rb') as f:
            task = plk.load(f)
            f.close()

        for t in task:
            self.model.appendRow(QStandardItem(str(t)))
        self.model.appendRow(QStandardItem('------------------------------------------------------------------'))
        self.listView.setModel(self.model)

    def delete_task(self):
        with open('task.plk', 'rb') as f:
            task = plk.load(f)
            f.close()

        if task:
            del task[0]

            with open('task.plk', 'wb') as f:
                plk.dump(task, f)
                f.close()

            for t in task:
                self.model.appendRow(QStandardItem(str(t)))
        self.model.appendRow(QStandardItem('-------------------------------------------------------------------'))
        self.listView.setModel(self.model)

    def execute_task(self):
        with open('task.plk', 'rb') as f:
            task = plk.load(f)
            f.close()
        print('num tasks : ', len(task))

        for idx, t in enumerate(task):
            print(t)
            if t[0] == 'robot':
                print('-------------->execute robot')
                self.t3.put('execute_task')
                self.t3.put(idx)
                time.sleep(3)

            elif t[0] == 'mobile':
                print('-------------->execute mobile')

                self.t0.put(t[1])
                time.sleep(0.2)
                print(t[1])

                if t[1] == 'go_forward' or t[1] == 'go_backward':
                    distance = t[2]
                    rpm = t[3]
                    print(distance, rpm)

                    self.pipe.send(distance)
                    self.pipe.send(rpm)
                else:
                    r_a = t[2]
                    s_a = t[3]
                    rpm = t[4]

                    self.pipe.send(r_a)
                    self.pipe.send(s_a)
                    self.pipe.send(rpm)

            while True:
                if self.t3.qsize() != 0:
                    get = self.t3.get()
                    if get == 'completed':
                        print(get)
                        break
                elif self.t0.qsize() != 0:
                    get = self.t0.get()
                    if get == 'completed':
                        print(get)
                        break

                time.sleep(0.1)

            time.sleep(2)

        task = []
        with open('task.plk', 'wb') as f:
            plk.dump(task, f)
            f.close()

    def trajectory_summary(self):
        self.t3.put('trajectory_summary')
        time.sleep(1)
        get = self.t3.get()

        self.model.appendRow(QStandardItem('width :' + str(get['width'])))
        self.model.appendRow(QStandardItem('height :' + str(get['height'])))
        self.model.appendRow(QStandardItem('horizontal :' + str(get['horizontal'])))
        self.model.appendRow(QStandardItem('vertical :' + str(get['vertical'])))
        self.model.appendRow(QStandardItem('--------------------------------------------------------'))
        self.listView_2.setModel(self.model)

    def m_home(self):
        self.t0.put('m_home')

    def autoscan(self):
        self.auto_scan = True

        for i in range(2):
            self.add_trajectory()
            time.sleep(1)

            self.go_backward()
            time.sleep(1)
            while True:
                if self.t0.qsize() != 0:
                    get = self.t0.get()
                    if get == 'completed':
                        print(get)
                        break
                time.sleep(0.1)
            time.sleep(2)

        self.add_trajectory()

        self.auto_scan = False

    def rail_left(self):
        self.t0.put('rail_left')

    def rail_center(self):
        self.t0.put('rail_center')

    def rail_right(self):
        self.t0.put('rail_right')

    def front(self):
        self.t0.put('front')
        self.t3.put('front')

    def rear(self):
        self.t0.put('rear')
        self.t3.put('rear')

    def tower(self):
        self.t0.put('arm')
        self.t3.put('arm')

    def arm(self):
        self.t0.put('arm')
        self.t3.put('arm')

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_Escape:
            self.t1.put('quit')
            self.t0.put('quit')
            qApp.quit()

        elif e.key() == Qt.Key_8:
            self.t0.put('accel')
        elif e.key() == Qt.Key_2:
            self.t0.put('back')
        elif e.key() == Qt.Key_5:
            self.t0.put('break')
        elif e.key() == Qt.Key_6:
            self.t0.put('steer_clock')   
        elif e.key() == Qt.Key_4:
            self.t0.put('steer_clockwise')    
        elif e.key() == Qt.Key_7:
            self.t0.put('steer_minus')  
        elif e.key() == Qt.Key_9:
            self.t0.put('steer_plus')
        elif e.key() == Qt.Key_Enter:
            self.t0.put('emergency')
        elif e.key() == Qt.Key_J:
            self.t5.put('ServoJ')

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.origin = QPoint(e.pos())
            #print(self.origin)
            #self.rubberBand.setGeometry(QRect(self.origin, QSize()))
            #self.rubberBand.show()
            loc = ((e.x() - 100) // 30, (e.y() - 140) // 30)
            self.mouse_tracking.append(loc)
            print(self.mouse_tracking)

    def mouseMoveEvent(self, e):

        #if not self.origin.isNull():
        #    self.rubberBand.setGeometry(QRect(self.origin, e.pos()).normalized())

        loc = ((e.x()-100)//30, (e.y()-140)//30)
        if self.mouse_tracking[-1] != loc:
            self.mouse_tracking.append(loc)
            print(self.mouse_tracking)

    def mouseReleaseEvent(self, e):

        if e.button() == Qt.LeftButton:
            if 0 <= self.origin.x()-90 < 1280 and 0 <= self.origin.y()-140 < 720 and 0 <= e.pos().x()-90 < 1280 and 0 <= e.pos().y()-140 < 720:
                if self.origin != e.pos():
                    roi = [self.origin.x()-100, self.origin.y()-140, e.pos().x()-100, e.pos().y()-140]
                    print('ROI : ', roi)
                    #self.t4.put(roi)
                    #self.t3.put('roi')
                    #self.t3.put(roi)
                else:
                    center = (e.pos().x()-100, e.pos().y()-140)
                    self.t3.put('center')
                    self.t3.put(center)
            self.rubberBand.hide()

    def tracking(self, e):
        self.t3.put('tracking')
        self.t3.put(self.mouse_tracking)
        self.mouse_tracking = []

    def fastlio_map(self, e):
        self.t3.put('fastlio_map')

    def emergency(self):
        self.t0.put('emergency')

    def stop(self):
        self.t0.put('stop')  

    def go_forward(self):
        if self.add_task or self.auto_scan:
            with open('task.plk', 'rb') as f:
                task = plk.load(f)
                f.close()
            task.append(['mobile', 'go_forward', self.lineEdit_13.text(), self.lineEdit_16.text()])
            print('task added : ', ['mobile', 'go_forward', self.lineEdit_13.text(), self.lineEdit_16.text()])
            with open('task.plk', 'wb') as f:
                plk.dump(task, f)
                f.close()

            for t in task:
                self.model.appendRow(QStandardItem(str(t)))
            self.model.appendRow(QStandardItem('---------------------------------------------------------------------'))
            self.listView.setModel(self.model)

        if not self.add_task or self.auto_scan:
            self.t0.put('go_forward')
            distance = self.lineEdit_13.text()
            rpm = self.lineEdit_16.text()
            #print(distance, rpm)

            self.pipe.send(distance)
            self.pipe.send(rpm)

    def go_backward(self):
        if self.add_task:
            with open('task.plk', 'rb') as f:
                task = plk.load(f)
                f.close()
            task.append(['mobile', 'go_backward', self.lineEdit_13.text(), self.lineEdit_16.text()])
            print('task added : ', ['mobile', 'go_backward', self.lineEdit_13.text(), self.lineEdit_16.text()])
            with open('task.plk', 'wb') as f:
                plk.dump(task, f)
                f.close()

            for t in task:
                self.model.appendRow(QStandardItem(str(t)))
            self.model.appendRow(QStandardItem('---------------------------------------------------------------------'))
            self.listView.setModel(self.model)

        if not self.add_task or self.auto_scan:
            self.t0.put('go_backward')
            distance = self.lineEdit_13.text()
            rpm = self.lineEdit_16.text()

            self.pipe.send(distance)
            self.pipe.send(rpm)

    def left_forward(self):
        if self.add_task:
            with open('task.plk', 'rb') as f:
                task = plk.load(f)
                f.close()
            task.append(['mobile', 'left_forward', self.lineEdit_14.text(), self.lineEdit_15.text(), self.lineEdit_16.text()])
            print('task added : ', ['mobile', 'left_forward', self.lineEdit_14.text(), self.lineEdit_15.text(), self.lineEdit_16.text()])
            with open('task.plk', 'wb') as f:
                plk.dump(task, f)
                f.close()

            for t in task:
                self.model.appendRow(QStandardItem(str(t)))
            self.model.appendRow(QStandardItem('---------------------------------------------------------------------'))
            self.listView.setModel(self.model)

        if not self.add_task or self.auto_scan:
            self.t0.put('left_forward')

            r_a = self.lineEdit_14.text()
            s_a = self.lineEdit_15.text()
            rpm = self.lineEdit_16.text()

            self.pipe.send(r_a)
            self.pipe.send(s_a)
            self.pipe.send(rpm)
        

    def right_forward(self):
        if self.add_task:
            with open('task.plk', 'rb') as f:
                task = plk.load(f)
                f.close()
            task.append(['mobile', 'right_forward', self.lineEdit_14.text(), self.lineEdit_15.text(), self.lineEdit_16.text()])
            print('task added : ',
                  ['mobile', 'right_forward', self.lineEdit_14.text(), self.lineEdit_15.text(), self.lineEdit_16.text()])
            with open('task.plk', 'wb') as f:
                plk.dump(task, f)
                f.close()

            for t in task:
                self.model.appendRow(QStandardItem(str(t)))
            self.model.appendRow(QStandardItem('---------------------------------------------------------------------'))
            self.listView.setModel(self.model)

        if not self.add_task or self.auto_scan:
            self.t0.put('right_forward')

            r_a = self.lineEdit_14.text()
            s_a = self.lineEdit_15.text()
            rpm = self.lineEdit_16.text()

            self.pipe.send(r_a)
            self.pipe.send(s_a)
            self.pipe.send(rpm)

    def left_backward(self):
        if self.add_task:
            with open('task.plk', 'rb') as f:
                task = plk.load(f)
                f.close()
            task.append(['mobile', 'left_backward', self.lineEdit_14.text(), self.lineEdit_15.text(), self.lineEdit_16.text()])
            print('task added : ',
                  ['mobile', 'left_backward', self.lineEdit_14.text(), self.lineEdit_15.text(), self.lineEdit_16.text()])
            with open('task.plk', 'wb') as f:
                plk.dump(task, f)
                f.close()

            for t in task:
                self.model.appendRow(QStandardItem(str(t)))
            self.model.appendRow(QStandardItem('---------------------------------------------------------------------'))
            self.listView.setModel(self.model)

        if not self.add_task or self.auto_scan:
            self.t0.put('left_backward')

            r_a = self.lineEdit_14.text()
            s_a = self.lineEdit_15.text()
            rpm = self.lineEdit_16.text()

            self.pipe.send(r_a)
            self.pipe.send(s_a)
            self.pipe.send(rpm)

    def right_backward(self):
        if self.add_task:
            with open('task.plk', 'rb') as f:
                task = plk.load(f)
                f.close()
            task.append(['mobile', 'right_backward', self.lineEdit_14.text(), self.lineEdit_15.text(), self.lineEdit_16.text()])
            print('task added : ',
                  ['mobile', 'right_backward', self.lineEdit_14.text(), self.lineEdit_15.text(),
                   self.lineEdit_16.text()])
            with open('task.plk', 'wb') as f:
                plk.dump(task, f)
                f.close()

            for t in task:
                self.model.appendRow(QStandardItem(str(t)))
            self.model.appendRow(QStandardItem('---------------------------------------------------------------------'))
            self.listView.setModel(self.model)

        if not self.add_task or self.auto_scan:
            self.t0.put('right_backward')

            r_a = self.lineEdit_14.text()
            s_a = self.lineEdit_15.text()
            rpm = self.lineEdit_16.text()

            self.pipe.send(r_a)
            self.pipe.send(s_a)
            self.pipe.send(rpm)

    def quit(self):
        self.t1.put('quit')
        self.t0.put('quit')
        qApp.quit()
        
    def drive(self):
        self.t0.put('drive') 
        self.t3.put('frontview')

    def drive2(self):
        self.t0.put('drive2') 
        self.t3.put('leftview')

    def drive3(self):
        self.t0.put('drive3')    
        self.t3.put('leftview')     

    def auto(self):
        self.t0.put('auto')   
        #self.t3.put('drive3')
    
    def paint(self):
        self.t3.put('paint')

    def IK_pose(self):
        
        text_doosan = 'pose;'+"{\"pose\": [[" + str(float(self.lineEdit.text())) + ',' + str(float(self.lineEdit_2.text())) + ',' + str(float(self.lineEdit_3.text())) + ',' + \
                str(np.deg2rad(float(self.lineEdit_4.text()))) + ',' + \
                str(np.deg2rad(float(self.lineEdit_5.text()))) + ',' + \
                str(np.deg2rad(float(self.lineEdit_6.text()))) + ']]}' 
            
        text_rainbow = 'ik;L;[' + str(float(self.lineEdit.text())) + ',' + str(float(self.lineEdit_2.text())) + ',' + str(float(self.lineEdit_3.text())-985) + ',' + \
                self.lineEdit_4.text() + ',' + self.lineEdit_5.text() + ',' + self.lineEdit_6.text() + ',400, 400]'
        #print(text_rainbow)        

        self.t3.put('IK_pose')
        #self.t0.put(text_doosan) 
        self.t3.put(text_rainbow)

    def FK_pose(self):
    
        text_rainbow = 'fk;J;[' + self.lineEdit_7.text() + ',' + self.lineEdit_8.text() + ',' + self.lineEdit_9.text() + ',' + \
                self.lineEdit_10.text() + ',' + self.lineEdit_11.text() + ',' + self.lineEdit_12.text() + ',20, 20]' 
        #print(text_rainbow)        

        self.t3.put('FK_pose')
        #self.t0.put(text_doosan) 
        self.t3.put(text_rainbow)
    
    def reconnect(self):
        self.t0.put('connect') 

    def robot_home(self):
        self.t3.put('home')

    def plane(self):
        self.t3.put('plane')

    def curved(self):
        self.t3.put('curved')

    def topview(self):
        self.t0.put('topview')
        
    def floorview(self):
        self.t3.put('floorview')

    def ceilingview(self):
        self.t3.put('ceilingview')    

    def frontview(self):
        self.t3.put('frontview')

    def leftview(self):
        self.t3.put('leftview')

    def right_cam(self):
        self.t0.put('right_cam')    

    def arm_tilt(self):
        self.t0.put('arm_tilt')  
        self.t3.put('arm_tilt') 

    def arm_cam_left(self):
        self.t0.put('arm_camera_left')

    def leftwall(self):
        self.t0.put('leftwall')
        self.t3.put('leftwall')

    def leftcorner(self):
        self.t0.put('leftcorner')
        self.t3.put('leftcorner')

    def frontwall(self):
        self.t0.put('frontwall')
        self.t3.put('frontwall')  

    def turn90_left(self):
        self.t0.put('turn90_left')
        self.t0.put(self.lineEdit_21.text())

    def cover_up(self):
        self.t0.put('cover_up')

    def cover_down(self):
        self.t0.put('cover_down')

    def cover_stop(self):
        self.t0.put('cover_stop') 

    def lift_up(self):
        self.t0.put('lift_up')
        self.t0.put(self.lineEdit_25.text())

    def lift_down(self):
        self.t0.put('lift_down')

    def lift_stop(self):
        self.t0.put('lift_stop')   


def viewer(t0, t1, t3, t5, t6, a_pc, b_image, b_vertices, b_robot):
    app = QApplication(sys.argv)
    myWindow = WindowClass(t0, t1, t3, t5, t6, a_pc, b_image, b_vertices, b_robot)
    myWindow.show()
    app.exec_()
    
def sub_loop(t0, t1, t2, t3, t5, t6, a_pc, a_pipe, a_image, a_vertices, a_robot):
    rclpy.init()
    pcs = cmd_Subscriber(t0, t1, t2, t3, t5, t6, a_pc, a_pipe, a_image, a_vertices, a_robot)
    executor = MultiThreadedExecutor(num_threads=1)
    executor.add_node(pcs)
    try:
        executor.spin()
    except Exception as e:
        print(e)
        pcs.destroy_node()
        rclpy.shutdown() 
        print('### ros2 shutdown')

def mb_process(pipe, t0, b_pc, t1, t2, camera):
    process = Mb_process(pipe, t0, b_pc, t1, t2, camera)

def tcp_server(HOST, PORT, a_pc):
    server = Socket(HOST, PORT, a_pc)

if __name__ == '__main__':
    
    pc0 = Process(target=mb_process, args=(b_pipe, t0, b_pc, t1, t2, 'arm'))   
    pc0.start()
    
    v = Process(target=viewer, args=(t0, t1, t3, t5, t6, a_pc, b_image, b_vertices, b_robot))
    v.start()
    s = Process(target=sub_loop, args=(t0, t1, t2, t3, t5, t6, a_pc, a_pipe, a_image, a_vertices, a_robot))
    s.start()

    HOST, PORT = '192.168.1.5', 9999
    s = Process(target=tcp_server, args=(HOST, PORT, a_pc))
    s.start()
