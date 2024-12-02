import numpy as np
import sys
import rclpy
import threading
import math
import statistics
import time
import cv2
import pickle as plk
import os

import std_msgs.msg as std_msgs
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from pypcd4 import PointCloud

from datetime import datetime

from PyQt5.QtWidgets import *
from PyQt5 import uic, QtGui
import pc_submodule
submodule = pc_submodule.submodule()

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt

class ShowImg:

    def __init__(self, pipe, parent, t4):
        self.pipe = pipe
        self.label = parent
        self.t4 = t4
        self.roi = [0, 0, 1280, 720]

    def _2D(self):
        while True:
            img = self.pipe.recv()
            if self.t4.qsize() != 0:
                self.roi = self.t4.get()
            x0, y0, x, y = self.roi[0], self.roi[1], self.roi[2], self.roi[3]
            cv2.rectangle(img, (x0, y0), (x, y), (0, 255, 0), 2)
            h, w, c = img.shape
            qImg = QtGui.QImage(img.data, w, h, w * c, QtGui.QImage.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(qImg)
            self.label.setPixmap(pixmap)
            self.label.resize(pixmap.width(), pixmap.height())


class PlotCanvas(FigureCanvas):

    def __init__(self, type, pipe, parent, width, height, rows, columns, b, t0, t3, t6, b_robot):
        
        self.pipe = pipe
        self.t0 = t0
        self.t3 = t3
        self.t6 = t6
        self.b_robot = b_robot

        self.roi = [0, 0, 1280, 720]
        self.way_points = []
        self.way_angles = []
        self.way_tilts = []
        self.x, self.y, self.z = [], [], []
        self.v, self.p = [[], 0], []
        self.angles, self.tilt_angles = [], []
        self.goals = []
        self.goals_array = []
        
        self.translation = [0, 0.14, 0.435]
        self.rotation = [0.0, 0.0, 0.0]

        self.pan_theta = 0
        self.tilt_theta = 0
        self.roll_theta = 0

        self.roll = np.array([[math.cos(self.roll_theta), -math.sin(self.roll_theta)],
                              [math.sin(self.roll_theta), math.cos(self.roll_theta)]])
        self.tilt = np.array([[math.cos(self.tilt_theta), -math.sin(self.tilt_theta)],
                              [math.sin(self.tilt_theta), math.cos(self.tilt_theta)]])
        self.pan = np.array([[math.cos(self.pan_theta), -math.sin(self.pan_theta)],
                             [math.sin(self.pan_theta), math.cos(self.pan_theta)]])

        self.plane = False
        self.cam_position = 'arm_tilt'
        self.paint_mode = False
        self.play_mode = False
        self.continuous = True
        self.vertical_mode = False
        self.camera = 'front'
        self.turn_angle_mode = False
        self.to_wall = False
        
        self.frontwall = True
        self.floorview = False
        self.leftwall = False
        self.side = False
        self.wall_mode = False
        self.real = False
        self.walk = False

        self.emergency = 0

        gs = gridspec.GridSpec(rows, columns)
        self.fig = plt.figure(figsize=(width, height))
        if type == 'vertices':
            self.ax = self.fig.add_subplot(gs[0, 0], projection='3d')
        else:
            self.ax = self.fig.add_subplot(gs[0, 0])

        FigureCanvas.__init__(self, self.fig)
        self.setParent(parent)
        FigureCanvas.setSizePolicy(self, QSizePolicy.Expanding, QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)


    def Lidar(self):
        prev_yaw = 0
        deg = 0
        prev_dist = 0
        dist = 0
        walked = 0.0
        prev_walk = False
        delay = 0
        go = False
        self.pltConnect = False
        self.pull = 0

        rclpy.init()
        self.msg = std_msgs.String()

        self.pose_node = rclpy.create_node('pose3')
        self.pose_pub = self.pose_node.create_publisher(std_msgs.String, '/robot_pose', 10)
        
        def onmove(event):
            if event.inaxes and self.pltConnect:
                #print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %('double' if event.dblclick else 'single', event.button,event.x, event.y, event.xdata, event.ydata))
                #self.goals.append([event.xdata, event.ydata])
                #self.goals_array = np.array(self.goals)
                #print(self.goals)
                x = event.xdata * 1000
                y = event.ydata * 1000
                #self.msg.data = 'servo;L;[' + str(601.45) + ',' + str(-1.1-x) + ', ' + str(507.97+y) + ', -84.16, 74.62, 5.67, 0.005, 0.025, 0.05, 0.005]'
                self.msg.data = 'servo;L;[' + str(699.98+y) + ',' + str(-1.1 - x) + ', ' + str(
                    -588+self.pull) + ', 90.16, 14.96, -179.97, 0.003, 0.025, 0.08, 0.05]'
                self.pose_pub.publish(self.msg)

        def onclick(event):
            if event.inaxes and event.button == 1:
                if not self.pltConnect:
                    self.pltConnect = True
                else:
                    self.pltConnect = False

            elif event.inaxes and event.button == 3:
                if self.pull == 0:
                    self.pull = 20
                else:
                    self.pull = 0
                x = event.xdata * 1000
                y = event.ydata * 1000
                # self.msg.data = 'servo;L;[' + str(601.45) + ',' + str(-1.1-x) + ', ' + str(507.97+y) + ', -84.16, 74.62, 5.67, 0.005, 0.025, 0.05, 0.005]'
                for i in range(5):
                    self.msg.data = 'servo;L;[' + str(699.98 + y) + ',' + str(-1.1 - x) + ', ' + str(
                        -588 + self.pull) + ', 90.16, 14.96, -179.97, 0.003, 0.025, 0.08, 0.05]'
                    self.pose_pub.publish(self.msg)
                    time.sleep(0.3)

        self.ax.set_xlim(-0.3, 0.3)
        self.ax.set_ylim(0, 0.3)

        while True:

            #yaw = self.pipe.recv()
            #walk = self.pipe.recv()
            
            #rotate = np.array([[math.cos(math.radians(prev_yaw-yaw)), -math.sin(math.radians(prev_yaw-yaw))], [math.sin(math.radians(prev_yaw-yaw)),math.cos(math.radians(prev_yaw-yaw))]])
            #prev_yaw = yaw
            """
            front_x = self.pipe.recv()
            front_y = self.pipe.recv()
            rear_x = self.pipe.recv()
            rear_y = self.pipe.recv()

            def get_norm(center, right, left):

                right1 = right - center
                left1 = left - center
                up1 = np.array([0, 0, 1])

                cross1 = np.cross(right1, up1)
                cross2 = np.cross(up1, left1)

                norm = cross1 + cross2

                return norm

            if prev_walk and not walk:
                del self.goals[0]
                self.goals_array = np.array(self.goals)
                prev_dist = 0
                go = False

            if not prev_walk and walk:
                delay = 1
            if delay > 0:
                delay += 1
                if delay == 8:
                    goal_dist = np.array(front_y[540])
                    constant = goal_dist - dist
                    delay = 0
                    go = True
            
            if go:
                cur_dist = np.array(front_y[540])
                remain = cur_dist - constant
                walked = dist - remain
                #if prev_dist == 0:
                #    prev_dist = cur_dist
                #walked = prev_dist - cur_dist
                #if walked < 0:
                #    walked = 0
                #prev_dist = cur_dist
                #print(walked)
            else:
                walked = 0.0

            prev_walk = walk
            
            self.ax.cla()
            self.ax.set_aspect('equal')
            # self.ax2.axis('off')
            self.ax.set_xlim(-10, 10)
            self.ax.set_ylim(-6, 6)
            self.ax.plot([0, 0], [0, 0.5], color='#ff00ff', linewidth='2')
            self.ax.plot([-0.3, 0.3, 0.3, -0.3, -0.3], [0.1, 0.1, -0.75, -0.75, 0.1], c='#ff00ff')

            self.ax.scatter(front_x, front_y, s=1, c='b') # lidar
            self.ax.scatter(rear_x, rear_y , s=1, c='b') # lidar
            wall_point = [[]]
            cnt = 0
            wall = 0
            prev_vector = np.zeros(3)
            
            for i in range(20, 792, 9):
                valid = True
                cross_vector = np.zeros(3)
                for j in range(-7, 8, 2):
                    v = np.array([front_x[i+j], front_y[i+j], 0])
                    center = np.array([front_x[i+j], front_y[i+j], 0])
                    left = np.array([front_x[i-9+j], front_y[i-9+j], 0])
                    right = np.array([front_x[i+9+j], front_y[i+9+j], 0])

                    if math.dist(center, np.array([-0.3, 0.1, 0])) < 0.7 or \
                            math.dist(left, np.array([-0.3, 0.1, 0])) < 0.7 or \
                            math.dist(right, np.array([-0.3, 0.1, 0])) < 0.7: # or math.dist(right, left) > 0.7:
                        #print('too close')
                        valid = False

                    cross_vector += get_norm(center, right, left)

                if valid:
                    v = np.array([front_x[i], front_y[i], 0])
                    end_position = cross_vector + v
                    vector_len = math.dist(v, end_position)
                    normalized = cross_vector / vector_len
                    p = normalized + v

                    #self.ax.plot([v[0], p[0]], [v[1], p[1]])

                    if cnt == 0:
                        wall_point[wall].append([v, normalized])
                        #print(wall_point)
                        cnt += 1
                    elif 0 < cnt < 5:
                        if np.dot(wall_point[wall][0][1], normalized) > 0.98 and math.dist(prev_vector, v) < 0.3:
                            cnt += 1
                        else:
                            cnt = 1
                            del wall_point[wall][0]
                            wall_point[wall].append([v, normalized])
                    elif cnt == 5:
                        if np.dot(wall_point[wall][0][1], normalized) > 0.98 and math.dist(prev_vector, v) < 0.3:
                            wall_point[wall].append([v, normalized])
                            #print(wall_point)
                            cnt += 1
                        else:
                            cnt = 1
                            del wall_point[wall][0]
                            wall_point[wall].append([v, normalized])
                    else:
                        if np.dot(wall_point[wall][0][1], normalized) > 0.98 and math.dist(prev_vector, v) < 0.3:
                            wall_point[wall][1] = [v, normalized]
                            cnt += 1
                        else:
                            #print(cnt)
                            cnt = 1
                            wall_point.append([])
                            wall += 1
                            wall_point[wall].append([v, normalized])
                            #print(wall_point)

                    prev_vector = v
                    #print(math.degrees(math.acos(0.9)))

            if len(wall_point[-1]) == 1:
                del wall_point[-1]
                #print(wall_point[-1])

            #print(len(wall_point))

            for i in wall_point:
                try:
                    self.ax.plot([i[0][0][0], i[1][0][0]], [i[0][0][1], i[1][0][1]], linewidth='3', c='r')
                    self.msg1.data = str([(i[0][0][0], i[0][0][1]), (i[1][0][0], i[1][0][1])])
                    self.wallpoints_pub1.publish(self.msg1)
                except:
                    a=1
            
            if self.goals:
                goal_x = [0] * (len(self.goals)+1)
                goal_y = [0] * (len(self.goals)+1)
                for idx, goal in enumerate(self.goals_array):
                    rotated = np.matmul(rotate, goal)
                    y = rotated[1] - walked
                    goal_x[idx+1] = rotated[0]
                    goal_y[idx+1] = y
                    self.goals[idx] = [rotated[0], y]
                    self.goals_array = np.array(self.goals)
                self.ax.plot(goal_x, goal_y)
                dist = round(self.goals[0][1], 2)
                try:
                    cos = np.dot(self.goals[0], (0, 1)) / round(math.dist(self.goals[0], [0, 0]), 2)
                    cos = min(cos, 1)
                    deg = round(math.degrees(math.acos(cos)), 2)
                    
                except:
                    print('cos', cos)
                    
                if self.goals[0][0] < 0:
                    deg = 660 - deg
                    print(deg)

                if walk:
                    self.ax.text(self.goals[0][0], self.goals[0][1], str(dist)+'meters')
                else:
                    self.ax.text(self.goals[0][0], self.goals[0][1], str(deg)+'degrees')

            #self.pipe.send((deg, dist, len(self.goals)))
            """
            self.ax.set_aspect('equal')
            # self.ax2.axis('off')

            self.fig.canvas.mpl_connect('button_press_event', onclick)
            self.fig.canvas.mpl_connect('motion_notify_event', onmove)
            self.ax.grid(True)
            self.draw()
            time.sleep(0.1)

    def _3D(self):
        self.center = (-1, -1)
        self.prev_center = (-1, -1)
        self.depth_range = 0.5
        point = (0,0,0)
        m_dist = 0.5
        m_height = 5
        self.m_speed = 800
        self.m_accel = 500
        self.m_waypoints = 3
        teaching_x = []
        teaching_y = []
        teaching_z = []
        teaching_tilt = []
        object_center = [1, 1]
        self.extender = 0.5
        self.point_adjust = 0
        self.p_height = 2
        self.l_height = 0.7
        self.pitch_error = 0.0
        self.add_trajectory = False
        self.execute_task = False
        self.trajectory_summary = {'width': 0, 'height': 0, 'horizontal': 0, 'vertical': 0}

        self.mouse_tracking = []
        self.tracking_mode = False

        self.selected_waypoints = [False, False, False, False, False, False, False, False, False, False]
        self.selected_total = 0

        self.pose_node = rclpy.create_node('pose1')                                 
        self.pose_pub = self.pose_node.create_publisher(std_msgs.String, '/robot_pose', 10)

        self.pc_node = rclpy.create_node('registered_topic')
        self.publisher = self.pc_node.create_publisher(PointCloud2, '/registered_topic', 1)

        self.trajectory_node = rclpy.create_node('trajectory')
        self.trajectory_pub = self.trajectory_node.create_publisher(std_msgs.String, '/trajectory', 10)

        self.transform_node1 = rclpy.create_node('transform_camera')
        self.transform_pub = self.transform_node1.create_publisher(std_msgs.String, '/transform_camera', 10)

        self.view_node = rclpy.create_node('view_mode')
        self.view_pub = self.view_node.create_publisher(std_msgs.String, '/view_mode', 10)

        self.scan_node = rclpy.create_node('scan_mode')
        self.scan_pub = self.scan_node.create_publisher(std_msgs.String, '/auto_scan', 10)

        self.msg = std_msgs.String()

        robot_state = '1'

        self.task = []
        with open('task.plk', 'wb') as f:
            plk.dump(self.task, f)
            f.close()

        time.sleep(2)

        while True:
            #time.sleep(0.2)
            if self.t3.qsize() != 0:
                get = self.t3.get()
                print('pc_command :' , get)
                if get == 'curved':
                    self.plane = False
                elif get == 'plane':
                    self.plane = True
                elif get == 'setting':
                    m_setting = self.t3.get()
                    m_dist = float(m_setting[0]) * 0.01 
                    m_height = int(m_setting[1])
                    self.m_speed = int(m_setting[2]) 
                    self.m_accel = int(m_setting[3])  
                    self.m_waypoints = int(m_setting[4]) 
                    self.p_height = float(m_setting[5])
                    self.l_height = float(m_setting[6])
                elif get == 'current_pose':
                    print(current_pose)
                    self.t3.put(current_pose)
                    time.sleep(2)
                elif get == 'side_c':
                    self.side = True
                elif get == 'side_nc':
                    self.side = False
                elif get == 'wall':
                    self.wall_mode = True
                elif get == 'not_wall':
                    self.wall_mode = False
                elif get == 'real':
                    self.real = True
                elif get == 'simul':
                    self.real = False   
                elif get == 'pull':
                    self.msg.data = 'gripper;1'
                    self.pose_pub.publish(self.msg)
                elif get == 'unpull':
                    self.msg.data = 'gripper;0'
                    self.pose_pub.publish(self.msg)  
                elif get == 'continuous':
                    self.continuous = True
                elif get == 'point':
                    self.continuous = False
                elif get == 'clear':
                    teaching_x = []
                    teaching_y = []
                    teaching_z = []
                    teaching_tilt = []
                elif get == 'fastlio_map':
                    time.sleep(0.5)
                    num = self.t3.get()
                    file_name = '../fastlio/src/FAST_LIO/PCD/scans_{}.pcd'.format(num)
                    print(file_name)
                    pc = PointCloud.from_path(file_name)

                    arrar = pc.numpy(("x", "y", "z", "intensity"))
                    print(pc.fields)
                    length = len(arrar)
                    for i in range(length // 10000):
                        self.pointcloud_publish(arrar[i * 10000:i*10000+10000])   #(i + 1) * 1000])
                        time.sleep(0.02)
                        if self.t3.qsize() != 0:
                            get = self.t3.get()
                            if get == 'cancel':
                                break
                    print('***********Notice***********')
                    print('       end publishing')
                    print('***********Notice***********')
                elif get == 'roi':
                    time.sleep(0.1)
                    self.roi = self.t3.get()
                elif get == 'center':
                    self.center = self.t3.get()

                elif get == 'front':
                    self.camera = get
                    self.translation = [0, 0.14, 0.435]

                elif get == 'rear':
                    self.camera = get
                    self.translation = [0, 0.5, 1.4]

                elif get == 'tower':
                    self.camera = get
                    self.translation = [0, 0.5, 1.4] 

                elif get =='arm':
                    self.camera = get

                elif get == 'drive3':
                    if self.to_wall:
                        self.to_wall = False
                    else:
                        self.to_wall = True
                    print('to_wall', self.to_wall)

                elif get == 'paint':
                    self.paint_mode = True
                elif get == 'play':
                    self.play_mode = True

                elif get == 'tracking':
                    time.sleep(0.3)
                    self.mouse_tracking = self.t3.get()
                    print(self.mouse_tracking)
                    self.tracking_mode = True

                elif get == 'IK_pose':
                    #time.sleep(0.1)
                    get = self.t3.get()
                    print(get)
                    self.msg.data = str(get)
                    self.pose_pub.publish(self.msg)

                    self.check_robotstat()

                elif get == 'FK_pose':
                    get = self.t3.get()
                    self.msg.data = get
                    self.pose_pub.publish(self.msg)

                    self.check_robotstat()

                elif get == 'home':
                    self.msg.data = 'fk;J;[180,100,-140, -20,-150,0,30,30]'
                    self.pose_pub.publish(self.msg)

                elif get == 'frontview':
                    self.msg.data = get
                    self.view_pub.publish(self.msg)

                    self.camera = get
                    self.leftwall = False
                    self.frontwall = False
                    self.extender = 0.5

                    self.msg.data = 'fk;J;[161.07, -6.48, -130.13, 137.02, -71.32, 88.07, 30, 30]'
                    self.pose_pub.publish(self.msg)

                    self.check_robotstat()

                elif get == 'leftview':
                    self.msg.data = get
                    self.view_pub.publish(self.msg)

                    self.camera = get
                    self.leftwall = False
                    self.frontwall = False
                    self.extender = 0.5

                    self.msg.data = 'fk;J;[261.59, -22.38, -138.8, 161.58, -81.96, 89.82, 30, 30]'
                    self.pose_pub.publish(self.msg)

                    self.check_robotstat()

                elif get == 'floorview':
                    self.msg.data = get
                    self.view_pub.publish(self.msg)

                    self.camera = get
                    self.leftwall = False
                    self.frontwall = False
                    self.extender = 0.5

                    self.msg.data = 'fk;J;[166.69, -10.67, -119.48, 40.15, -89.86, 103.05, 30, 30]'
                    self.pose_pub.publish(self.msg)

                    self.check_robotstat()

                elif get == 'ceilingview':
                    self.msg.data = get
                    self.view_pub.publish(self.msg)

                    self.camera = get
                    self.leftwall = False
                    self.frontwall = False
                    self.extender = 0.5

                    self.msg.data = 'fk;J;[168.87, -83.41, -89.21, 82.61, 90.05, -100.73, 30, 30]'
                    self.pose_pub.publish(self.msg)

                    self.check_robotstat()

                elif get == 'waypoint_0':
                    self.selected_waypoints[0] = True
                    self.selected_total += 3
                elif get == 'del_0':
                    self.selected_waypoints[0] = False
                    self.selected_total -= 3

                elif get == 'waypoint_1':
                    self.selected_waypoints[1] = True
                    self.selected_total += 3
                elif get == 'del_1':
                    self.selected_waypoints[1] = False
                    self.selected_total -= 3

                elif get == 'waypoint_2':
                    self.selected_waypoints[2] = True
                    self.selected_total += 3
                elif get == 'del_2':
                    self.selected_waypoints[2] = False
                    self.selected_total -= 3

                elif get == 'waypoint_3':
                    self.selected_waypoints[3] = True
                    self.selected_total += 3
                elif get == 'del_3':
                    self.selected_waypoints[3] = False
                    self.selected_total -= 3

                elif get == 'waypoint_4':
                    self.selected_waypoints[4] = True
                    self.selected_total += 3
                elif get == 'del_4':
                    self.selected_waypoints[4] = False
                    self.selected_total -= 3

                elif get == 'waypoint_5':
                    self.selected_waypoints[5] = True
                    self.selected_total += 3
                elif get == 'del_5':
                    self.selected_waypoints[5] = False
                    self.selected_total -= 3

                elif get == 'waypoint_6':
                    self.selected_waypoints[6] = True
                    self.selected_total += 3
                elif get == 'del_6':
                    self.selected_waypoints[6] = False
                    self.selected_total -= 3

                elif get == 'waypoint_7':
                    self.selected_waypoints[7] = True
                    self.selected_total += 3
                elif get == 'del_7':
                    self.selected_waypoints[7] = False
                    self.selected_total -= 3

                elif get == 'waypoint_8':
                    self.selected_waypoints[8] = True
                    self.selected_total += 3
                elif get == 'del_8':
                    self.selected_waypoints[8] = False
                    self.selected_total -= 3

                elif get == 'waypoint_9':
                    self.selected_waypoints[9] = True
                    self.selected_total += 3
                elif get == 'del_9':
                    self.selected_waypoints[9] = False
                    self.selected_total -= 3

                elif get == 'adjust_left':
                    self.point_adjust -= 1

                elif get == 'adjust_right':
                    self.point_adjust += 1

                elif get == 'add_trajectory':
                    self.add_trajectory = True

                elif get == 'execute_task':
                    self.execute_task = True
                    time.sleep(0.1)
                    task_idx = self.t3.get()

                elif get == 'trajectory_summary':
                    self.t3.put(self.trajectory_summary)
                    time.sleep(2)

                elif get == 'pitch+':
                    self.pitch_error = math.pi / 6

                elif get == 'pitch-':
                    self.pitch_error = 0.0

            if self.point_adjust > 5:
                self.point_adjust = 5
            elif self.point_adjust < -5:
                self.point_adjust = -5

            self.x_data = []
            self.y_data = []
            self.z_data = []
            self.pc_array = []

            #self.position_data = self.pipe.recv()
            #self.position_data2 = self.pipe.recv()
            self.position_data2 = [0,0,self.center]
            
            self.ax.cla()

            self.ax.set_xlim(-1.5, 1.5)
            self.ax.set_ylim(-0.5, 2.5)
            self.ax.set_zlim(0, 3)
            
            """
            if self.emergency == 0 and self.position_data[0] == 1 and self.position_data2[2] < 2:
                self.msg.data = 'yellow:1'
                self.emg_pub.publish(self.msg)
                self.msg.data = 'green:0'
                self.emg_pub.publish(self.msg)
                self.emergency = 1
                self.msg.data = 'sound:2'
                self.emg_pub.publish(self.msg)
            elif self.emergency == 1 and ((self.position_data[0] == 1 and self.position_data2[2] > 2) or (self.position_data[0] == 0)):
                self.msg.data = 'yellow:0'
                self.emg_pub.publish(self.msg)
                self.msg.data = 'green:1'
                self.emg_pub.publish(self.msg)
                self.emergency = 0
                self.msg.data = 'sound:0'
                self.emg_pub.publish(self.msg)
            elif self.emergency == 0 and self.position_data[0] == 0 and self.position_data2[2] < 1.2:
                self.msg.data = 'red:1'
                self.emg_pub.publish(self.msg)
                self.msg.data = 'green:0'
                self.emg_pub.publish(self.msg)
                self.emergency = 2
                self.msg.data = 'sound:1'
                self.emg_pub.publish(self.msg)
                self.t0.put('stop')
            elif self.emergency == 2 and (self.position_data[0] == 0 and self.position_data2[2] > 1.2):
                self.msg.data = 'red:0'
                self.emg_pub.publish(self.msg)
                #self.msg.data = 'green:1'
                #self.emg_pub.publish(self.msg)
                self.emergency = 0
                #self.msg.data = 'sound:0'
                #self.emg_pub.publish(self.msg)
            """

            # print(self.position_data)
            self.t6.put('vertices')
            vertices = np.array(self.pipe.recv())
            current_pose = self.pipe.recv()
            robot_state = current_pose.split(';')[2]
            transform = self.pipe.recv()
            transform[4] += self.pitch_error

            self.msg.data = str(transform)         # to unity
            self.transform_pub.publish(self.msg)

            self.translation = [transform[0], transform[1], transform[2]]
            self.rotation = [transform[3], transform[4], transform[5]]
            self.roll_theta, self.tilt_theta, self.pan_theta = self.rotation[0], self.rotation[1], self.rotation[2]

            self.roll = np.array([[math.cos(self.roll_theta), -math.sin(self.roll_theta)],
                                            [math.sin(self.roll_theta), math.cos(self.roll_theta)]])
            self.tilt = np.array([[math.cos(self.tilt_theta), -math.sin(self.tilt_theta)],
                                            [math.sin(self.tilt_theta), math.cos(self.tilt_theta)]])
            self.pan = np.array([[math.cos(self.pan_theta), -math.sin(self.pan_theta)],
                                            [math.sin(self.pan_theta), math.cos(self.pan_theta)]])   
            #print(transform)

            roi = [self.roi[0] // 30, self.roi[1] // 30, self.roi[2] // 30, self.roi[3] // 30]
            
            object_center = vertices[10, 20]

            if self.center != self.prev_center:
                self.prev_center = self.center
                object_center = vertices[self.center[1] // 30, self.center[0] // 30]
                tilt = np.matmul(self.tilt, np.array([object_center[1], object_center[2]]))
                y, z = tilt[0], tilt[1]
                roll = np.matmul(self.roll, np.array([object_center[0], z]))
                x, z = roll[0], roll[1]
                pan = np.matmul(self.pan, np.array([x, y]))
                x, y = pan[0], pan[1]

                x += self.translation[0]
                y += self.translation[1]
                z += self.translation[2]
                point = (x, y, z)
                #self.ax.scatter(x, y, z, color='r')
                print('**point**', x, y, z)
                if self.continuous:
                    teaching_x.append(x)
                    teaching_y.append(y)
                    teaching_z.append(z)
                    teaching_tilt.append(90)
                    
            if self.play_mode and self.continuous:
                self.publish_pose(teaching_x, teaching_y, teaching_z, 90, teaching_tilt)
                self.play_mode = False
                    
                self.depth_range = 0.5
            elif self.play_mode and not self.continuous:
                if self.camera == 'frontview':
                    self.msg.data = 'ik;L;[' + str(point[1] * 1000-(0.3+m_dist+0.5)*1000) + ',' + str(point[0] * (-1) * 1000) + ',' + str(point[2] * 1000 - 985) + ',' + '-90' + ',' + '90' + ',0,' + str(self.m_speed) + ',' + str(self.m_accel) + ']'
                elif self.camera == 'ceilingview':
                    self.msg.data = 'ik;L;[' + str(point[1] * 1000) + ',' + str(point[0] * (-1) * 1000-30) + ',' + str(point[2] * 1000 - 985-(0.3+m_dist)*1000) + ',' + '-90' + ',' + '0' + ',0,' + str(self.m_speed) + ',' + str(self.m_accel) + ']'
                #print(self.msg.data)
                self.pose_pub.publish(self.msg)
                self.play_mode = False
            #else:
            #    object_center = vertices[24, 43]

            vertices = vertices[roi[1]:roi[3], roi[0]:roi[2]]
            
            object_dist = object_center[1]

            for i, row in enumerate(vertices):
                self.pc_array.append([])
                for j, v in enumerate(row):
                    
                    pc_dist = v[1]
                    roll = np.matmul(self.roll, np.array([v[0], v[2]]))
                    x, z = roll[0], roll[1]
                    tilt = np.matmul(self.tilt, np.array([v[1], z]))
                    y, z = tilt[0], tilt[1]
                    pan = np.matmul(self.pan, np.array([x, y]))
                    x, y = pan[0], pan[1]

                    if object_dist - self.depth_range < pc_dist < object_dist + self.depth_range:

                        self.x_data.append(x + self.translation[0])  # pantilt location (x: 0.26, y: -0.70, z: 1.68)
                        self.y_data.append(y + self.translation[1])
                        self.z_data.append(z + self.translation[2])
                    #else:
                    #    print(i, j, v)
                        if not self.plane:
                            self.pc_array[-1].append(
                                [(x + self.translation[0]), (y + self.translation[1]), (z + self.translation[2])])
                    if self.plane:
                            self.pc_array[-1].append(
                                [(x + self.translation[0]), (y + self.translation[1]), (z + self.translation[2])])    

            self.ax.scatter(self.x_data, self.y_data, self.z_data, s=1, alpha=0.1)
            self.ax.scatter(point[0], point[1], point[2], color='r')
            #self.tight_layout()

            # print(vertices)

            #if not self.plane:
            #    self.ax.plot(self.x, self.y, self.z, color='#FF00FF')
            if not self.plane:

                if self.camera != 'tower' or self.camera == 'arm':
                    #self.get_vector(self.pc_array, m_dist, m_height)
                    #print(v,p)
                    if self.tracking_mode:
                        v_pass = True
                        self.get_track(m_dist)
                    try:
                        if not self.tracking_mode:
                            self.get_vector(m_dist, m_height)
                            #submodule.face_orient(self.pc_array)
                            for v in range(len(self.v_list)):
                                for v1, p1 in zip(self.v_list[v], self.way_points[v]):
                                    self.ax.plot([v1[0], p1[0]], [v1[1], p1[1]], [v1[2], p1[2]], c='gray')
                            v_pass = False

                    except Exception as e:
                        print('pass', e)
                        v_pass = True

                    if not v_pass:

                        trajectory = []

                        if self.execute_task:
                            self.msg.data = 'Execute'
                            self.scan_pub.publish(self.msg)

                            with open('task.plk', 'rb') as f:
                                task = plk.load(f)
                                f.close()

                            self.way_points, self.way_angles, self.way_tilts, self.camera = task[task_idx][1], task[task_idx][2], task[task_idx][3], task[task_idx][4]

                            self.paint_mode = True

                        for idx, row in enumerate(self.way_points):
                            self.x, self.y, self.z = [], [], []
                            self.angles, self.tilt_angles = [], []

                            selected_idx = 0
                            selected_cnt = 1
                            trajectory.append([])

                            for way in zip(self.way_points[idx], self.way_angles[idx], self.way_tilts[idx], self.v_list[idx]):

                                self.x_v, self.y_v, self.z_v = [], [], []
                                self.angles_v, self.tilt_angles_v = [], []

                                self.x.append(way[0][0])
                                self.y.append(way[0][1])
                                self.z.append(way[0][2])

                                self.angles.append(way[1])
                                self.tilt_angles.append(way[2])

                                yaw = 0
                                if self.camera == 'frontview':
                                    yaw = 0
                                elif self.camera == 'leftview':
                                    yaw = 90

                                if not self.wall_mode:
                                    if not self.paint_mode:
                                        data = str((
                                            self.x[-1], self.y[-1], self.z[-1], self.angles[-1],
                                            self.tilt_angles[-1], yaw, way[3][0], way[3][1], way[3][2]))

                                        trajectory[-1].append(data)

                                if self.wall_mode and idx == 0 and self.selected_waypoints[selected_idx]:
                                    if selected_cnt % 2 == 1:
                                        self.x_v.append(way[0][0] - 0.05)
                                        self.y_v.append(way[0][1] - 0.05)
                                        self.z_v.append(self.p_height)
                                        self.angles_v.append(way[1])
                                        self.tilt_angles_v.append(way[2] - 35)

                                        if not self.paint_mode:
                                            self.msg.data = str((
                                                self.x_v[-1], self.y_v[-1], self.z_v[-1], self.angles_v[-1],
                                                self.tilt_angles_v[-1], yaw, self.selected_total))
                                            self.trajectory_pub.publish(self.msg)
                                            time.sleep(0.1)

                                        self.x_v.append(way[0][0])
                                        self.y_v.append(way[0][1])
                                        self.z_v.append((self.p_height + self.l_height) / 2)
                                        self.angles_v.append(way[1])
                                        self.tilt_angles_v.append(way[2])

                                        if not self.paint_mode:
                                            self.msg.data = str((
                                                self.x_v[-1], self.y_v[-1], self.z_v[-1], self.angles_v[-1],
                                                self.tilt_angles_v[-1], yaw, self.selected_total))
                                            self.trajectory_pub.publish(self.msg)
                                            time.sleep(0.03)

                                        self.x_v.append(way[0][0] - 0.05)
                                        self.y_v.append(way[0][1] - 0.05)
                                        self.z_v.append(self.l_height)
                                        self.angles_v.append(way[1])
                                        self.tilt_angles_v.append(way[2] + 35)

                                        if not self.paint_mode:
                                            self.msg.data = str((
                                                self.x_v[-1], self.y_v[-1], self.z_v[-1], self.angles_v[-1],
                                                self.tilt_angles_v[-1], yaw, self.selected_total))
                                            self.trajectory_pub.publish(self.msg)
                                            time.sleep(0.03)

                                        selected_cnt += 1

                                    else:
                                        self.x_v.append(way[0][0] - 0.05)
                                        self.y_v.append(way[0][1] - 0.05)
                                        self.z_v.append(self.l_height)
                                        self.angles_v.append(way[1])
                                        self.tilt_angles_v.append(way[2] + 35)

                                        if not self.paint_mode:
                                            self.msg.data = str((
                                                self.x_v[-1], self.y_v[-1], self.z_v[-1], self.angles_v[-1],
                                                self.tilt_angles_v[-1], yaw, self.selected_total))
                                            self.trajectory_pub.publish(self.msg)
                                            time.sleep(0.03)

                                        self.x_v.append(way[0][0])
                                        self.y_v.append(way[0][1])
                                        self.z_v.append((self.p_height + self.l_height) / 2)
                                        self.angles_v.append(way[1])
                                        self.tilt_angles_v.append(way[2])

                                        if not self.paint_mode:
                                            self.msg.data = str((
                                                self.x_v[-1], self.y_v[-1], self.z_v[-1], self.angles_v[-1],
                                                self.tilt_angles_v[-1], yaw, self.selected_total))
                                            self.trajectory_pub.publish(self.msg)
                                            time.sleep(0.03)

                                        self.x_v.append(way[0][0] - 0.05)
                                        self.y_v.append(way[0][1] - 0.05)
                                        self.z_v.append(self.p_height)
                                        self.angles_v.append(way[1])
                                        self.tilt_angles_v.append(way[2] - 35)

                                        if not self.paint_mode:
                                            self.msg.data = str((
                                                self.x_v[-1], self.y_v[-1], self.z_v[-1], self.angles_v[-1],
                                                self.tilt_angles_v[-1], yaw, self.selected_total))
                                            self.trajectory_pub.publish(self.msg)
                                            time.sleep(0.03)

                                        selected_cnt += 1

                                    if self.paint_mode:
                                        print('selected', selected_cnt)
                                        self.t6.put('state')
                                        time.sleep(0.3)
                                        robot_state = self.b_robot.recv()

                                        print('robot state :', robot_state)
                                        while True:
                                            if robot_state == '1':
                                                print('motion start')
                                                self.publish_pose(self.x_v, self.y_v, self.z_v, self.angles_v, self.tilt_angles_v)
                                                break
                                            else:
                                                self.t6.put('state')
                                                time.sleep(0.3)
                                                robot_state = self.b_robot.recv()
                                                print('robot state :', robot_state)

                                        #time.sleep(1)

                                #print(selected_idx, self.angles_v)

                                selected_idx += 1

                            try:
                                if not self.wall_mode and self.paint_mode:
                                    self.t6.put('state')
                                    time.sleep(0.3)
                                    robot_state = self.b_robot.recv()
                                    while True:
                                        if robot_state == '1':
                                            print('motion start')
                                            if not self.side:
                                                self.publish_pose(self.x, self.y, self.z, self.angles, self.tilt_angles, idx)
                                                #time.sleep(1)
                                                break
                                            elif self.side and idx == 0:
                                                self.publish_pose(self.x, self.y, self.z, self.angles, self.tilt_angles)
                                        else:
                                            self.t6.put('state')
                                            time.sleep(0.3)
                                            robot_state = self.b_robot.recv()
                                            print('robot state :', robot_state)

                                        # print(self.angles, self.tilt_angles)
                            except Exception as e:
                                print(e)

                            self.ax.plot(self.x, self.y, self.z, color='#FF00FF')

                        self.msg.data = str(trajectory)
                        self.trajectory_pub.publish(self.msg)

                        if self.add_trajectory:
                            with open('task.plk', 'rb') as f:
                                self.task = plk.load(f)
                                f.close()

                            self.task.append(['robot', self.way_points, self.way_angles, self.way_tilts, self.camera])

                            print('tasks : ', len(self.task))
                            with open('task.plk', 'wb') as f:
                                plk.dump(self.task, f)
                                f.close()

                            self.msg.data = 'Capture'
                            self.scan_pub.publish(self.msg)

                            self.add_trajectory = False

                        #print(str(trajectory))
                        
                        if self.paint_mode and len(self.task) == 0:
                            if self.camera == 'frontview':
                                self.msg.data = 'fk;J;[161.07, -6.48, -130.13, 137.02, -71.32, 88.07, 30, 30]'
                                self.pose_pub.publish(self.msg)
                            elif self.camera == 'leftview':
                                self.msg.data = 'fk;J;[261.59, -22.38, -138.8, 161.58, -81.96, 89.82, 30, 30]'
                                self.pose_pub.publish(self.msg)

                            self.t6.put('state')
                            time.sleep(0.3)
                            robot_state = self.b_robot.recv()

                            while True:
                                if robot_state == '1':
                                    self.msg.data = 'stop;J;[161.07, -6.48, -130.13, 137.02, -71.32, 88.07, 30, 30]'
                                    self.pose_pub.publish(self.msg)
                                    print('last')
                                    break
                                else:
                                    self.t6.put('state')
                                    time.sleep(0.3)
                                    robot_state = self.b_robot.recv()
                                    print('robot state :', robot_state)
                            self.paint_mode = False
                        
                        if self.paint_mode:
                            self.paint_mode = False
                        if self.execute_task:
                            self.check_robotstat()
                            self.t3.put('completed')
                            self.execute_task = False
                            time.sleep(0.5)

                    self.way_points = []
                    self.way_angles = []
                    self.way_tilts = []

                    time.sleep(0.3)

            elif self.plane:
                try:
                    self.position_data = [700, 350, 130, 120, 1000, 550, False, object_dist]
                    left, right, left_y, right_y, high, low, z_list, x_list = self.get_coner(self.pc_array,
                                                                                             self.position_data)
                    if self.camera == 'floorview':
                        self.ax.plot([left, right, right, left, left], [high, high, low, low, high], [left_y, right_y, right_y, left_y, left_y], color='r')
                    else:    
                        self.ax.plot([left, right, right, left, left], [left_y, right_y, right_y, left_y, left_y],
                                 [high, high, low, low, high], color='r')
                    x, y, z = [], [], []

                    for i in range(0, len(z_list) // m_height):
                        if self.camera == 'floorview':
                            y.append(z_list[i * m_height ])
                            y.append(z_list[i * m_height ])
                        else:
                            z.append(z_list[i * m_height ])
                            z.append(z_list[i * m_height ])
                        if self.camera == 'tower':
                            if i % 2 == 1:
                                x.append(left + 0.3 + 0.6)
                                x.append(right + 0.3 + 0.6)
                                y.append(x_list[i * m_height][0] - 0.15)
                                y.append(x_list[i * m_height][1] + 0.1)
                            else:
                                x.append(right + 0.3 + 0.6)
                                x.append(left + 0.3 + 0.6)
                                y.append(x_list[i * m_height][1] + 0.1)
                                y.append(x_list[i * m_height][0] - 0.15)

                        elif self.camera == 'floorview':
                            if i % 2 == 1:
                                x.append(x_list[i * m_height][0]+0.03)
                                x.append(x_list[i * m_height][1]+0.03)
                                z.append(left_y + m_dist + self.extender + 0.3)
                                z.append(right_y + m_dist + self.extender + 0.3)
                            else:
                                x.append(x_list[i * m_height][1]+0.03)
                                x.append(x_list[i * m_height][0]+0.03)
                                z.append(right_y + m_dist + self.extender + 0.3)
                                z.append(left_y + m_dist + self.extender + 0.3)

                        else:
                            if i % 2 == 1:
                                x.append(x_list[i * m_height][0]+0.03)
                                x.append(x_list[i * m_height][1]+0.03)
                                y.append(left_y - m_dist - self.extender - 0.3)
                                y.append(right_y - m_dist - self.extender - 0.3)
                            else:
                                x.append(x_list[i * m_height][1]+0.03)
                                x.append(x_list[i * m_height][0]+0.03)
                                y.append(right_y - m_dist - self.extender - 0.3)
                                y.append(left_y - m_dist - self.extender - 0.3)

                    self.ax.plot(x, y, z, color='#FF00FF')
                    distance = 0.0
                    theta = 0.0
                    norm_theta = 0
                    norm = 0
                    if self.camera == 'arm':
                        c, g = self.goal_position(left, left_y, right, right_y)  # center, goal
                        if not self.turn_angle_mode:
                            self.ax.plot([c[0], g[0]], [c[1], g[1]], [0, 0])
                            self.ax.plot([g[0], 0], [g[1], 0], [0, 0], color='r')
                            norm = (c[1] - g[1], -c[0] + g[0])

                            distance = round(math.dist(g, (0, 0)), 2)
                            cos_theta = g[1] / distance
                            norm_theta = round(
                                math.acos(np.dot(norm, (0, 1)) / math.sqrt(norm[0] * norm[0] + norm[1] * norm[1])), 2)
                            tilt_angle = math.pi / 2



                        else:
                            self.ax.plot([c[0], 0], [c[1], 0], [0, 0], color='r')
                            norm = (c[1], -c[0])
                            distance = round(math.dist(c, (0, 0)), 2)
                            cos_theta = c[1] / distance

                        self.ax.plot([0, 0], [0, 1], [0, 0], color='black')

                        # norm_theta = round(math.asin(norm[1] / math.sqrt(norm[0]*norm[0] + norm[1] * norm[1])), 2)

                        theta = round(math.acos(cos_theta), 2) * (-1)
                        # degree = math.degrees(theta)
                        if (not self.turn_angle_mode and g[0] < 0) or (self.turn_angle_mode and c[0] < 0):
                            theta = theta * (-1)
                        ##print(distance, theta)
                        distance = c[1]
                        degree = round(math.degrees(norm_theta) - 90, 2)
                        self.ax.set_title('Distance : {}M  Theta : {}(Degrees)'.format(distance, degree))

                    elif self.camera == 'tower':
                        vector = (right - left, right_y - left_y)
                        norm = (-vector[1], vector[0])

                        norm_theta = round(
                            math.acos(np.dot(norm, (0, 1)) / math.sqrt(norm[0] * norm[0] + norm[1] * norm[1])), 2)
                        tilt_angle = 0.0
                        # norm_theta = np.pi / 2

                    if self.paint_mode:  
                        
                        #norm_thetas = np.ones(len(x)) * norm_theta
                        #tilt_angles = np.ones(len(x)) * tilt_angle
                        self.publish_pose(x, y, z, 90, 90)

                        self.angle_cnt = 0
                        self.paint_mode = False
                        print('pose published')


                except Exception as e:
                    # print(e)
                    ##print('devided by zero')
                    a = 1

            self.draw()

    def goal_position(self, x1, y1, x2, y2):
        d = 0
        cx = round((x1 + x2) / 2, 2)
        cy = round((y1 + y2) / 2, 2)
        c = (cx, cy)
        y = cy
        s = (y2 - y1) / (x2 - x1)
        p1 = (cx, cy)
        while d < 1.6:
            y -= 0.1
            p2 = (-s * y + s * cy + cx, y)
            d = math.dist(p1, p2)
        return c, p2

    def publish_pose(self, x, y, z, angles, tilt_angles, row):

        text = "{\"pose\": ["

        if self.camera == 'floorview':

            for idx, p in enumerate(x):
                pose_x, pose_y, pose_z = y[idx], x[idx] * (-1), z[idx]
                pose = "[{:.02}, {}, {}, {}, {}, {}], ".format(round(pose_x, 2), round(pose_y, 2), round(pose_z, 2),-90, 180, 0)
                                                               #tilt_angles[idx], math.pi - angles[idx],
                                                               #tilt_angles[idx])
                text += pose

        elif self.camera == 'frontview':

            for idx, p in enumerate(x):
                pose_x, pose_y, pose_z = y[idx], x[idx] * (-1), z[idx]
                pose = "[{:.02}, {}, {}, {}, {}, {}], ".format(round(pose_x, 2), round(pose_y, 2), round(pose_z, 2),-90, 90, 0)
                                                               #tilt_angles[idx], math.pi - angles[idx],
                                                               #tilt_angles[idx])
                text += pose        

        elif self.camera == 'ceilingview':

            for idx, p in enumerate(x):
                pose_x, pose_y, pose_z = y[idx], x[idx] * (-1), z[idx]
                pose = "[{:.02}, {}, {}, {}, {}, {}], ".format(round(pose_x, 2), round(pose_y, 2), round(pose_z, 2),-90, 0, 0)
                                                               #tilt_angles[idx], math.pi - angles[idx],
                                                               #tilt_angles[idx])
                text += pose        


        else:
            for idx, p in enumerate(x):
                pose_x, pose_y, pose_z = y[idx], x[idx] * (-1), z[idx]
                pose = "[{:.02}, {}, {}, {}, {}, 0.0], ".format(round(pose_x, 2), round(pose_y, 2), round(pose_z, 2),
                                                                0.0, 0) #tilt_angles[idx])
                text += pose

        text = text[:-2] + "]}"
        print(text) 
        
        msg = std_msgs.String()

        Data = eval(text)

        length = len(Data['pose'])

        if self.wall_mode:

            if self.camera == 'frontview':

                for idx, xyz in enumerate(Data['pose']):
                    if idx == 0:
                        msg.data = 'ik;L;[' + str(xyz[0] * 1000) + ',' + str(xyz[1] * 1000) + ',' + str(
                            xyz[2] * 1000 - 985) + ',' + str(-90) + ',' + str(
                            tilt_angles[idx]) + ',' + str(angles[idx]) + ',' + str(1000) + ', 1500]'

                        self.pose_pub.publish(msg)
                        time.sleep(0.3)
                    else:
                        msg.data = 'iks;L;[' + str(xyz[0] * 1000) + ',' + str(xyz[1] * 1000) + ',' + str(
                            xyz[2] * 1000 - 985) + ',' + str(-90) + ',' + str(
                            tilt_angles[idx]) + ',' + str(angles[idx]) + ',' + str(self.m_speed) + ', 500]'

                        self.pose_pub.publish(msg)
                        time.sleep(0.3)

                if self.real:
                    msg.data = 'gripper;1'
                    self.pose_pub.publish(msg)

                time.sleep(0.3)
                msg.data = 'iksr'
                self.pose_pub.publish(msg)
                

            elif self.camera == 'leftview':
                for idx, xyz in enumerate(Data['pose']):
                    if idx == 0:
                        msg.data = 'ik;L;[' + str(xyz[0] * 1000) + ',' + str(xyz[1] * 1000) + ',' + str(
                            xyz[2] * 1000 - 985) + ',' + str(-90) + ',' + str(
                            tilt_angles[idx]) + ',' + str(90 + angles[idx]) + ',' + str(1000) + ', 1500]'

                        self.pose_pub.publish(msg)
                        time.sleep(0.3)
                    else:
                        msg.data = 'iks;L;[' + str(xyz[0] * 1000) + ',' + str(xyz[1] * 1000) + ',' + str(
                            xyz[2] * 1000 - 985) + ',' + str(-90) + ',' + str(
                            tilt_angles[idx]) + ',' + str(90 + angles[idx]) + ',' + str(self.m_speed) + ', 500]'

                        self.pose_pub.publish(msg)
                        time.sleep(0.3)

                if self.real:
                    msg.data = 'gripper;1'
                    self.pose_pub.publish(msg)

                time.sleep(0.6)
                msg.data = 'iksr'
                self.pose_pub.publish(msg)

        elif not self.side and not self.continuous:
            for idx, xyz in enumerate(Data['pose']):
                if self.real:
                    if idx % 2 == 1:
                        gripper = '1'
                    else:
                        gripper = '0'
                else:
                    gripper = '0'
                
                if self.camera == 'floorview':
                    msg.data = 'gripper;'+gripper
                    self.pose_pub.publish(msg)
                    msg.data = 'ik;L;[' + str(xyz[0] * 1000) + ',' + str(xyz[1] * 1000) + ',' + str(xyz[2] * 1000 - 985) + ',' + '-90' + ',' + '180' + ',' + '0,' + str(self.m_speed) + ',' + str(self.m_accel) + ']'    
                elif self.camera == 'ceilingview':
                    msg.data = 'gripper;'+gripper
                    self.pose_pub.publish(msg)
                    msg.data = 'ik;L;[' + str(xyz[0] * 1000) + ',' + str(xyz[1] * 1000) + ',' + str(xyz[2] * 1000 - 985) + ',' + '-90' + ',' + '0' + ',' + '0,' + str(self.m_speed) + ',' + str(self.m_accel) + ']'
                else:    
                    msg.data = 'gripper;'+gripper
                    self.pose_pub.publish(msg)
                    msg.data = 'ik;L;[' + str(xyz[0] * 1000) + ',' + str(xyz[1] * 1000) + ',' + str(xyz[2] * 1000 - 985) + ',' + '-90' + ',' + '90' + ',0,' + str(self.m_speed) + ',' + str(self.m_accel) + ']'
                
                time.sleep(2)
                self.pose_pub.publish(msg)

        elif not self.side and self.continuous:
            for idx, xyz in enumerate(Data['pose']):
                if self.camera == 'floorview':
                    if idx == 0:
                        msg.data = 'ik;L;[' + str(xyz[0] * 1000) + ',' + str(xyz[1] * 1000) + ',' + str(xyz[2] * 1000 - 985) + ',' + str(-90 - angles[idx]) + ',' + str(270 - tilt_angles[idx]) + ',' + '0,' + str(800) + ', 500]'
                        self.pose_pub.publish(msg)
                        time.sleep(0.3)
                    else:
                        msg.data = 'iks;L;[' + str(xyz[0] * 1000) + ',' + str(xyz[1] * 1000) + ',' + str(
                            xyz[2] * 1000 - 985) + ',' + str(-90 - angles[idx]) + ',' + str(
                            270 - tilt_angles[idx]) + ',' + '0,' + str(self.m_speed) + ', 1500]'
                elif self.camera == 'frontview':
                    if idx == 0:
                        msg.data = 'ik;L;[' + str(xyz[0] * 1000) + ',' + str(xyz[1] * 1000) + ',' + str(xyz[2] * 1000 - 985) + ',' + str(-90) + ',' + str(tilt_angles[idx]) + ',' + str(angles[idx]) +',' + str(800) + ', 1500]'
                        self.pose_pub.publish(msg)
                        time.sleep(0.3)
                    else:
                        msg.data = 'iks;L;[' + str(xyz[0] * 1000) + ',' + str(xyz[1] * 1000) + ',' + str(xyz[2] * 1000 - 985) + ',' + str(-90) + ',' + str(tilt_angles[idx]) + ',' + str(angles[idx]) + ',' + str(self.m_speed) + ', 500]'
                elif self.camera == 'leftview':
                    if row % 2 == 0:
                        angle = [120, 60]
                    else:
                        angle = [60, 120]

                    if idx == 0:
                        msg.data = 'ik;L;[' + str(xyz[0] * 1000) + ',' + str(xyz[1] * 1000) + ',' + str(
                            xyz[2] * 1000 - 985) + ',' + str(-90 - angles[idx]) + ',' + str(
                            tilt_angles[idx]) + ',' + str(angle[0]) + ',' + str(800) + ', 1500]'
                        self.pose_pub.publish(msg)
                        time.sleep(0.3)
                    elif idx < len(Data['pose'])-1:
                        msg.data = 'iks;L;[' + str(xyz[0] * 1000) + ',' + str(xyz[1] * 1000) + ',' + str(
                            xyz[2] * 1000 - 985) + ',' + str(-90 - angles[idx]) + ',' + str(
                            tilt_angles[idx]) + ',' + '90,' + str(self.m_speed) + ',' + str(self.m_accel) + ']'
                    else:
                        msg.data = 'iks;L;[' + str(xyz[0] * 1000) + ',' + str(xyz[1] * 1000) + ',' + str(
                            xyz[2] * 1000 - 985) + ',' + str(-90 - angles[idx]) + ',' + str(
                            tilt_angles[idx]) + ',' + str(angle[1]) + ',' + str(self.m_speed) + ',' + str(self.m_accel) + ']'
                elif self.camera == 'ceilingview':
                    if idx == 0:
                        msg.data = 'ik;L;[' + str(xyz[0] * 1000) + ',' + str(xyz[1] * 1000) + ',' + str(xyz[2] * 1000 - 985) + ',' + str(-90 - angles[idx]) + ',' + str(tilt_angles[idx]-90) + ',' + '0,' + str(800) + ', 1500]'
                        self.pose_pub.publish(msg)
                        time.sleep(0.3)
                    else:
                        msg.data = 'iks;L;[' + str(xyz[0] * 1000) + ',' + str(xyz[1] * 1000) + ',' + str(xyz[2] * 1000 - 985) + ',' + str(-90 - angles[idx]) + ',' + str(tilt_angles[idx]-90) + ',' + '0,' + str(self.m_speed) + ', 500]'
                self.pose_pub.publish(msg)
                time.sleep(0.3)
            print('published')

            #if self.real:
            #    msg.data = 'gripper;1'
            #    self.pose_pub.publish(msg)

            msg.data = 'iksr;' + str(self.m_accel) + ';' + str(self.real)
            self.pose_pub.publish(msg)

        time.sleep(0.1)
        #msg.data = 'gripper;0'
        #self.pose_pub.publish(msg)
        #print('gripper stop')

    def get_track(self, m_dist):
        center_position_vectors = []
        tcp = []

        for j in range(len(self.pc_array)):
            center_position_vectors.append([])
            for i in range(len(self.pc_array[j])):
                center_position_vectors[-1].append(np.array(self.pc_array[j][i]))

        for idx, loc in enumerate(self.mouse_tracking):
            cross_vector = np.zeros(3)
            row = loc[1]
            column = loc[0]
            for i in [-2, -1, 0, 1, 2]:  # -5 -3 0 3  5
                for j in [-2, -1, 0, 1, 2]:
                    center = center_position_vectors[row + j][column + i]
                    up = center_position_vectors[row - 1 + j][column + i]
                    right = center_position_vectors[row + j][column + 1 + i]
                    down = center_position_vectors[row + 1 + j][column + i]
                    left = center_position_vectors[row + j][column - 1 + i]
                    cross_vector += self.get_norm(center, up, right, down, left)

            v = center_position_vectors[row][column]
            end_position_vector = cross_vector + v
            vector_length = math.dist(end_position_vector, v)

            if cross_vector[1] > 0:
                pitch = 360 - round(math.degrees(math.acos(cross_vector[2] * -1 / vector_length)), 2)
            else:
                pitch = round(math.degrees(math.acos(cross_vector[2] * -1 / vector_length)), 2)
            yaw = round(math.degrees(math.acos(cross_vector[0] / vector_length)), 2) - 90

            p = cross_vector / vector_length * (0.3 + m_dist + self.extender) + v
            if self.camera == 'frontview':
                tcp.append((p[1] * 1000, -p[0] * 1000, p[2] * 1000 - 985, -90, pitch, -yaw))
            elif self.camera == 'floorview':
                tcp.append((p[1] * 1000, -p[0] * 1000, p[2] * 1000 - 985, -90 + yaw, pitch, 0))

        self.msg.data = 'ik;L;[' + str(tcp[0][0]) + ',' + str(tcp[0][1]) + ',' + str(tcp[0][2]) + ',' + str(tcp[0][3]) + ',' + str(tcp[0][4]) + ',' + str(tcp[0][5]) + ',' + '800, 1500]'
        self.pose_pub.publish(self.msg)
        time.sleep(0.5)
        self.check_robotstat()

        for t in tcp:
            self.msg.data = 'servo;L;[' + str(t[0]) + ',' + str(t[1]) + ',' + str(t[2]) + ',' + str(t[3]) + ',' + str(t[4]) + ',' + str(t[5]) + ', 0.03, 0.025, 0.1, 0.05]'
            self.pose_pub.publish(self.msg)
            time.sleep(0.05)

        self.tracking_mode = False

    def get_vector(self, m_dist, m_height):
        # cx, cy, x0, y0, x2, y2, detect, dist = [t for t in self.position_data]

        center_position_vectors = []
        self.v_list = []
        p_list = []

        #for j in range(len(self.pc_array)):
        #    center_position_vectors.append([])
        #    for i in range(len(self.pc_array[j])):
        #        center_position_vectors[-1].append(np.array(self.pc_array[j][i]))

        way_rows = []
        len_of_first = len(self.pc_array[5])
        for row in range(len(self.pc_array)):
            if len(self.pc_array[row]) > len_of_first-5:
                way_rows.append(row)

        points = (len(self.pc_array[5])-8) // (self.m_waypoints-1)  #26
        loc = 4  #13
        point_loc = [4]  #13

        for i in range(self.m_waypoints-2):
            loc += points
            point_loc.append(loc)
        point_loc.append(len(self.pc_array[5])-5) #13

        #for i in range(14, len(self.pc_array[5])-11, 3):
        #    loc += points
        #    point_loc.append(loc)

        #print(point_loc)
        loc = 6  #13
        height_loc = [6] #13

        while True:
            loc += m_height
            if loc > len(way_rows)-4:  #10
                break
            else:
                height_loc.append(loc)

        #for row in range(4, len(way_rows) - 4):
        #    if row % 7 == 3:
        for row in height_loc:
            self.way_points.append([])
            self.way_angles.append([])
            self.way_tilts.append([])
            self.v_list.append([])

            #cross_vector = np.zeros(3)
            for idx, loc in enumerate(point_loc):
                cross_vector = np.zeros(3)
                loc += self.point_adjust
                for i in [-2, -1, 0, 1, 2]:   #-5 -3 0 3  5
                    for j in [-2, -1, 0, 1, 2]:
                        center = self.pc_array[way_rows[row+j]][loc+i]
                        up = self.pc_array[way_rows[row-1+j]][loc+i]   #4
                        right = self.pc_array[way_rows[row+j]][loc+1+i]
                        down = self.pc_array[way_rows[row+1+j]][loc+i]
                        left = self.pc_array[way_rows[row+j]][loc-1+i]
                        cross_vector += self.get_norm(center, up, right, down, left)

                v = self.pc_array[way_rows[row]][loc]
                end_position_vector = cross_vector + v
                vector_length = math.dist(end_position_vector, v)

                if self.camera == 'frontview' or self.camera == 'leftview':
                    tilt_angle = round(math.acos((cross_vector * (-1))[2] * 1 / vector_length), 2)
                    tilt_theta = round(math.acos((cross_vector * (-1))[2] * 1 / vector_length) - math.pi / 2, 2)
                elif self.camera == 'floorview' or self.camera == 'ceilingview':
                    tilt_angle = round(math.acos(cross_vector[1] * 1 / vector_length), 2)
                    tilt_theta = round(math.acos(cross_vector[1] * 1 / vector_length) - math.pi / 2, 2)
                #if idx == 0 or idx == len(point_loc)-1:
                #    side_back = 0.0
                #elif idx == 1 or idx == len(point_loc)-2:
                #    side_back = 0.0
                #else:
                side_back = 0
                p = cross_vector / vector_length * (0.3 + m_dist + self.extender+side_back) + v
                if self.camera == 'leftview':
                    angle = round(math.pi / 2 - math.acos((cross_vector * (-1))[1] * (-1) / vector_length), 2)
                else:
                    angle = round(math.pi / 2 - math.acos((cross_vector * (-1))[0] * (-1) / vector_length), 2)
                degree_angle = math.degrees(angle)

                self.v_list[-1].append(v)

                #print('robot_distance : ', math.dist(p, (0,0,0.987)))

                #if p[0] != 0 and not np.isnan(p[0]):
                self.way_points[-1].append(p)
                self.way_angles[-1].append(degree_angle)
                self.way_tilts[-1].append(math.degrees(tilt_angle))
                ##print(self.way_points)

            if len(self.way_points) % 2 == 0:
                self.way_points[-1].reverse()
                self.way_angles[-1].reverse()
                self.way_tilts[-1].reverse()
                self.v_list[-1].reverse()

        print('#######################################')

        self.trajectory_summary['width'] = round(math.dist(self.v_list[0][0], self.v_list[0][-1]), 2)
        self.trajectory_summary['height'] = round(math.dist(self.v_list[0][0], self.v_list[-1][0]), 2)
        self.trajectory_summary['horizontal'] = round(math.dist(self.v_list[0][0], self.v_list[0][1]), 2)
        self.trajectory_summary['vertical'] = round(math.dist(self.v_list[0][0], self.v_list[1][-1]), 2)

    def get_norm(self, center, up, right, down, left):

        up1 = np.array(up) - np.array(center)
        right1 = np.array(right) - np.array(center)
        down1 = np.array(down) - np.array(center)
        left1 = np.array(left) - np.array(center)

        cross1 = np.cross(right1, up1)
        cross2 = np.cross(up1, left1)
        cross3 = np.cross(left1, down1)
        cross4 = np.cross(down1, right1)

        norm = cross1 + cross2 + cross3 + cross4

        return norm

    def get_coner(self, pc_array, position_data):

        column_x = []
        column_y = []
        row = []
        data_x = []
        data_y = []
        data_z = []
        z_list = []
        x_list = []
        x = []
        left_check = False
        up_check = False
        right_column = 0
        low_row = 0
        cx, cy, x0, y0, x2, y2, detect, dist = [t for t in position_data]

        # dist_ratio = 1.3 / dist
        dist_ratio = 1
        # row data

        len_pc = len(pc_array)

        for j in range(len_pc):

            for i in range(len(pc_array[j])):

                k = pc_array[j][i][2]
                s1 = pc_array[j][i][1]
                p = pc_array[j][i][0]
                if self.camera == 'floorview':
                    pc_dist = self.translation[2] - pc_array[j][i][2]
                elif self.camera == 'frontview':
                    pc_dist = pc_array[j][i][1] - self.translation[1]
                else:
                    pc_dist = pc_array[j][i][1]
                #print(dist, pc_dist)

                if abs(k) != 0 and dist - 0.5 < pc_dist < dist + 0.5:

                    if self.camera == 'floorview':
                        data_z.append(s1)
                    else:
                        data_z.append(k)

                    if self.camera == 'tower':
                        x.append(s1)
                    else:
                        x.append(p)

            if len(data_z) > 10 * dist_ratio and not up_check:
                high = statistics.mean(data_z)
                #print(high)
                up_check = True
                high_row = len(row)
                z_list.append(high)

                x_list.append((x[0], x[-1]))

            elif len(data_z) > 10 * dist_ratio and up_check:
                low_row = len(row)
                z_list.append(statistics.mean(data_z))
                x_list.append((x[0], x[-1]))
            row.append(data_z)
            # print(len(data_z), len(row))
            data_z = []
            x = []
        #print(high_row, low_row)
        low = statistics.mean(row[low_row])

        # column data

        cnt = 0

        for j in range(len(pc_array[0])):

            for i in range(high_row, low_row):

                s = pc_array[i][j][0]
                s1 = pc_array[i][j][1]
                s2 = pc_array[i][j][2]

                if self.camera == 'tower':
                    pc_dist = s * (-1)
                else:
                    if self.camera == 'floorview':
                        pc_dist = self.translation[2] - s2
                    elif self.camera == 'frontview':
                        pc_dist = s1 - self.translation[1]
                    else:
                        pc_dist = s1
                #print(dist, pc_dist)
                if abs(s) != 0 and dist - 0.5 < pc_dist < dist + 0.5:
                    if self.camera == 'floorview':
                        data_y.append(s2)
                    else:
                        data_y.append(s1)

                    data_x.append(s)

            if len(data_x) > 15 * dist_ratio and not left_check:

                #print(data_x)
                left = np.mean(data_x)
                left_y = statistics.mean(data_y)
                left_check = True

            elif len(data_x) > 15 * dist_ratio and left_check:
                right_column = len(column_x)

            column_x.append(data_x)
            column_y.append(data_y)
            # print(len(data_x), len(column_x), left)
            data_x = []
            data_y = []

        right = statistics.mean(column_x[right_column])
        right_y = statistics.mean(column_y[right_column])

        return left, right, left_y, right_y, high, low, z_list, x_list

    def check_robotstat(self):
        print('check robot status')
        self.t6.put('state')
        time.sleep(0.3)
        robot_state = self.b_robot.recv()
        print('robot state :', robot_state)

        while True:
            if robot_state == '1':
                self.msg.data = 'stop;J;[161.07, -6.48, -130.13, 137.02, -71.32, 88.07, 30, 30]'
                self.pose_pub.publish(self.msg)
                break
            else:
                self.t6.put('state')
                time.sleep(0.3)
                robot_state = self.b_robot.recv()
                print('robot state :', robot_state)

    def pointcloud_publish(self, pc_array):
        pc_data = np.array(pc_array, dtype=np.float32).flatten()

        pc2_msg = PointCloud2()
        pc2_msg.header = Header()
        pc2_msg.header.stamp = self.pc_node.get_clock().now().to_msg()

        pc2_msg.header.frame_id = 'fastlio2_link'

        pc2_msg.height = 1
        pc2_msg.width = len(pc_data) // 4
        pc2_msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        pc2_msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        pc2_msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
        pc2_msg.fields.append(PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1))
        #pc2_msg.fields.append(
        #    PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1))  # color
        pc2_msg.point_step = 16
        pc2_msg.row_step = pc2_msg.point_step * pc2_msg.width
        pc2_msg.is_dense = True
        pc2_msg.is_bigendian = False
        pc2_msg.data = pc_data.tobytes()

        # return pc2_msg
        self.publisher.publish(pc2_msg)
        #print('==================')
        #print(datetime.now(), 'Publishing PointCloud2 message from fastlio')
        #print('==================')