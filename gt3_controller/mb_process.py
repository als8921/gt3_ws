import json
import pickle as plk
import os
import matplotlib.pyplot as plt
import numpy as np
import time
from datetime import datetime
import math
import rclpy
from rclpy.qos import qos_profile_sensor_data
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
from lift_srv.srv import LiftCommand
#import gt3_distributor.msg as gt3
#import rcl_interfaces.msg as rcl_msgs
from rclpy.clock import Clock


class Mb_process():
    def __init__(self, pipe, t0, b_pc, t1, t2, camera):
        
        self.pipe = pipe
        self.t0 = t0                  # q : pc_server
        self.t1 = t1
        self.t2 = t2
        self.server_pipe = b_pc

        self.wheelradius = 0.133
        self.wheeldist = 0.594
        self.steerdist = 0.73
        
        self.yaw_origin = -1000
        self.start_angle = 0
        self.steer_angle = 0.0
        self.mobile_front = (0, 0.3)
        self.pitch = 0
        self.x_origin = 0.0
        self.y_origin = 0.0
        self.speed = 0.0
        self.orient = 1
        self.front_obstacle_dist = 0
        self.ob_left = 0
        self.ob_right = 0
        self.x = 0.0
        self.y = 0.0
        
        self.emergency = 0

        self.turn_mode = False
        self.turnleft_mode = False
        self.emergency_mode = False
        self.walk = False

        self.degrees = []
        self.degree = 0
        
        rclpy.init()
        
        #self.node_1 = rclpy.create_node('gt3')
        #self.gt3_pub = self.node_1.create_publisher(gt3.Camera, '/camera_data', 10)

        self.emergency_node = rclpy.create_node('emergency_mb')
        self.emergency_pub = self.emergency_node.create_publisher(std_msgs.Bool, '/emergency', 10)

        #self.steer_node = rclpy.create_node('steer')
        #self.steer_pub = self.steer_node.create_publisher(gt3.Camera2, '/camera_data2', 10)
        
        self.rotate_node = rclpy.create_node('rotate')
        self.rotate_pub = self.rotate_node.create_publisher(geometry_msgs.Twist, '/cmd_vel', 10)
        
        self.report_node = rclpy.create_node('report')
        self.report_pub = self.report_node.create_publisher(std_msgs.String, '/mobile_to_robot', 10)

        self.pose_node = rclpy.create_node('pose')
        self.pose_pub = self.pose_node.create_publisher(std_msgs.String, '/robot_pose', 10)

        self.gtm_node = rclpy.create_node('gtm_cmd')
        self.gtm_pub = self.gtm_node.create_publisher(std_msgs.String, '/gtm/controlcmd', 10)

        self.unity_node = rclpy.create_node('unity_cmd2')
        self.unity_pub = self.unity_node.create_publisher(std_msgs.String, '/unity/cmd', 10)

        self.rail_node = rclpy.create_node('rail_cmd')
        self.rail_pub = self.rail_node.create_publisher(std_msgs.String, '/rcs/rail_refpos', 10)
        self.client = self.rail_node.create_client(LiftCommand, 'lift_command')

        #while not self.client.wait_for_service(timeout_sec=1.0):
        #    print('서비스를 기다리는 중...')
        
        self.msg = std_msgs.String()
        self.e_msg = std_msgs.Bool()

        #self.steer_msg = gt3.Camera2()
        #self.steer_msg.distance = 1.0
        #self.steer_msg.theta = self.steer_angle
    
        self.rotate_msg = geometry_msgs.Twist()
        self.rotate_msg.linear.x = 0.0
        self.rotate_msg.linear.y = 0.0

        self.msg.data = 'green:1'
        self.gtm_pub.publish(self.msg)
        
        self.msg.data = 'p'

        self.P = 0.148
        self.I = 0.05
        self.D = 0.2
        self.max_state = 20.0

        self.min_state = 0.0
        self.pre_state = 0.0
        self.dt = 0.0
        self.integrated_state = 0.0
        self.pre_time = Clock().now().to_msg()

        self.cur_theta = 0.0
        self.goal_theta = 0.0

        self.goal_degree = 0

        
        while True:
            
            if self.t0.qsize() != 0:
                get = self.t0.get()
                #time.sleep(0.1)
                print('COMMAND : ', get)
                if get == 'paint':
                    self.msg.data = 'paint'
                    self.report_pub.publish(self.msg)

                elif get == 'emergency':
                    if self.emergency_mode:
                        self.e_msg.data = False
                        self.emergency_mode = False
                        print('EMERGENCY OFF~~')
                    else:
                        self.e_msg.data = True
                        self.emergency_mode = True
                        print('EMERGENCY!!!')
                    self.emergency_pub.publish(self.e_msg)
                    self.speed = 0
                    self.steer_angle = 0

                elif get == 'connect':
                    self.msg.data = 'connect'
                    self.report_pub.publish(self.msg)    

                elif get == 'home':
                    self.msg.data = 'home'
                    self.report_pub.publish(self.msg)
                    
                    self.msg.data = 'fk;J;[180,100,-140, -20,-150,0,30,30]'
                    self.pose_pub.publish(self.msg)

                elif get == 'm_home':
                    self.m_home()

                elif get == 'refresh':
                    self.msg.data = get
                    self.report_pub.publish(self.msg)

                elif get == 'drive':
                    #self.go_straight(0.3, 10.0)
                    self.go_backward(0.5, 10.0)
                    self.stop()
                    self.msg.data = 'fk;J;[161.07, -6.48, -130.13, 137.02, -71.32, 88.07, 30, 30]'
                    self.pose_pub.publish(self.msg)
                    time.sleep(3.0)                    
                    self.steer(-45.0, 45.0, 10.0, -1)
                    self.go_backward(1.0, 10.0)
                    self.steer(41.0, 45.0, 10.0, -1)
                    time.sleep(0.3)
                    #self.detect()
                    #self.go_backward(0.2, 10.0)
                    self.stop()
                    #time.sleep(0.5)
                    #self.go_straight(1.3, 10.0)
                    #self.stop()

                elif get == 'drive2':
                    self.autonomous_turn(90.0)
                    self.stop()
                    self.msg.data = 'fk;J;[183.18, -11.34, -123.97, 142.6, -4.54, 81.16, 30, 30]'
                    self.pose_pub.publish(self.msg)
                    time.sleep(0.5)
                    self.go_straight(0.4, 10.0)
                    self.stop()

                elif get == 'drive3':
                    self.go_backward(1.3, 10.0)
                    self.stop()
                    self.go_straight(0.2, 10.0)
                    self.steer(41.0, 45.0, 10.0, 1)
                    self.go_straight(0.6, 10.0)
                    self.steer(-45.0, 45.0, 10.0, 1)
                    time.sleep(0.5)
                    self.msg.data = 'fk;J;[183.18, -11.34, -123.97, 142.6, -4.54, 81.16, 30, 30]'
                    self.pose_pub.publish(self.msg)
                    time.sleep(3.0)
                    self.go_straight(0.5, 10.0)
                    self.stop()

                elif get == 'rail_left':
                    #time.sleep(0.5)
                    self.msg.data = '0.85'
                    self.rail_pub.publish(self.msg)

                elif get == 'rail_center':
                    #time.sleep(0.5)
                    self.msg.data = '0.0'
                    self.rail_pub.publish(self.msg)

                elif get == 'rail_right':
                    #time.sleep(0.5)
                    self.msg.data = '-0.85'
                    self.rail_pub.publish(self.msg)

                elif get == 'accel':
                    self.orient = 1
                    if self.speed >= 0:
                        self.speed += 10.0

                        if self.speed > 30:
                            self.speed = 30.0

                        if self.steer_angle != 0.0:
                            fast, low = self.wheel_velocity(abs(self.steer_angle))
                            print(fast, low)
                        else:
                            fast, low = 1.0, 1.0

                        if self.steer_angle <= 0:
                            self.rotate_msg.linear.x = self.speed * low * self.orient
                            self.rotate_msg.linear.y = self.speed * fast * self.orient
                        else:
                            self.rotate_msg.linear.x = self.speed * fast * self.orient
                            self.rotate_msg.linear.y = self.speed * low * self.orient


                        #speed = self.speed * self.orient
                        #self.rotate_msg.linear.x = speed
                        #self.rotate_msg.linear.y = speed
                        self.rotate_pub.publish(self.rotate_msg)
                        print('set SPEED : ', self.speed)

                elif get == 'cover_up':
                    self.msg.data = 'cover:1'
                    self.gtm_pub.publish(self.msg)
                elif get == 'cover_down':
                    self.msg.data = 'cover:-1'
                    self.gtm_pub.publish(self.msg)    
                elif get == 'cover_stop':
                    self.msg.data = 'cover:0'
                    self.gtm_pub.publish(self.msg)
                elif get == 'lift_up':
                    self.msg.data = 'lift:1'
                    self.gtm_pub.publish(self.msg)
                    time.sleep(0.2)
                    value = float(self.t0.get())
                    print('Lift MOVE to {}'.format(value))
                    response = self.send_request("MOVE", value)
                elif get == 'lift_down':
                    self.msg.data = 'lift:-1'
                    self.gtm_pub.publish(self.msg)
                    response = self.send_request("HOMING", 0.0)
                elif get == 'lift_stop':
                    self.msg.data = 'lift:0'
                    self.gtm_pub.publish(self.msg)
                    response = self.send_request("STOP", 0.0)
                elif get == 'break':
                    if self.speed == 0:
                        self.steer_angle = 0
                        if self.emergency == 3:
                            self.msg.data = 'yellow:0'
                            self.gtm_pub.publish(self.msg)
                            self.msg.data = 'green:1'
                            self.gtm_pub.publish(self.msg)
                            self.emergency = 0
                            self.msg.data = 'sound:0'
                            self.gtm_pub.publish(self.msg)
                    if self.speed > 0:
                        self.speed -= 10.0
                    elif self.speed < 0:
                        self.speed += 10

                    if self.speed > 0:
                        self.orient = 1
                    elif self.speed < 0:
                        self.orient = -1
                    if self.steer_angle != 0.0:
                        fast, low = self.wheel_velocity(abs(self.steer_angle))
                        print(fast, low)
                    else:
                        fast, low = 1.0, 1.0        
                    
                    if self.steer_angle <= 0:
                        self.rotate_msg.linear.x = abs(self.speed) * low * self.orient
                        self.rotate_msg.linear.y = abs(self.speed) * fast * self.orient
                    else:
                        self.rotate_msg.linear.x = abs(self.speed) * fast * self.orient
                        self.rotate_msg.linear.y = abs(self.speed) * low * self.orient
                        
                    self.rotate_pub.publish(self.rotate_msg)  
                    print('set SPEED : ', self.speed)

                elif get == 'back':
                    self.orient = -1
                    if self.speed <= 0:
                        if self.emergency == 0:
                            self.msg.data = 'red:0'
                            self.gtm_pub.publish(self.msg)
                            self.msg.data = 'yellow:1'
                            self.gtm_pub.publish(self.msg)
                            self.msg.data = 'green:0'
                            self.gtm_pub.publish(self.msg)
                            self.emergency = 3
                            #self.msg.data = 'sound:3'
                            #self.gtm_pub.publish(self.msg)
                        self.speed -= 10.0

                        if self.speed < -30:
                            self.speed = -30.0

                        if self.steer_angle != 0.0:
                            fast, low = self.wheel_velocity(abs(self.steer_angle))
                            print(fast, low)
                        else:
                            fast, low = 1.0, 1.0

                        if self.steer_angle <= 0:
                            self.rotate_msg.linear.x = abs(self.speed) * low * self.orient
                            self.rotate_msg.linear.y = abs(self.speed) * fast * self.orient
                        else:
                            self.rotate_msg.linear.x = abs(self.speed) * fast * self.orient
                            self.rotate_msg.linear.y = abs(self.speed) * low * self.orient

                            # speed = self.speed * self.orient
                        # self.rotate_msg.linear.x = speed
                        # self.rotate_msg.linear.y = speed
                        self.rotate_pub.publish(self.rotate_msg)

                elif get == 'steer_clock':
                    if self.speed == 0:
                        self.speed = 5
                    self.steer_angle -= 10.0
                    if self.steer_angle < -80:
                        self.steer_angle = -80.0
                    #self.steer_msg.distance = 1.0
                    #self.steer_msg.theta = self.steer_angle
                    #self.steer_pub.publish(self.steer_msg)
                    if self.steer_angle != 0.0:
                        fast, low = self.wheel_velocity(abs(self.steer_angle))
                    else:
                        fast, low = 1.0, 1.0

                    if self.steer_angle > 0:
                        self.rotate_msg.linear.x = abs(self.speed) * fast * self.orient
                        self.rotate_msg.linear.y = abs(self.speed) * low * self.orient
                    else:
                        self.rotate_msg.linear.x = abs(self.speed) * low * self.orient
                        self.rotate_msg.linear.y = abs(self.speed) * fast * self.orient

                    self.rotate_pub.publish(self.rotate_msg)
                    print('steer angle : ', self.steer_angle)

                elif get == 'steer_clockwise':
                    if self.speed == 0:
                        self.speed = 5
                    self.steer_angle += 10.0
                    if self.steer_angle > 80:
                        self.steer_angle = 80.0
                    #self.steer_msg.distance = 1.0
                    #self.steer_msg.theta = self.steer_angle
                    #self.steer_pub.publish(self.steer_msg)
                    if self.steer_angle != 0.0:
                        fast, low = self.wheel_velocity(abs(self.steer_angle))
                    else:
                        fast, low = 1.0, 1.0

                    if self.steer_angle > 0:
                        self.rotate_msg.linear.x = abs(self.speed) * fast * self.orient
                        self.rotate_msg.linear.y = abs(self.speed) * low * self.orient
                    else:
                        self.rotate_msg.linear.x = abs(self.speed) * low * self.orient
                        self.rotate_msg.linear.y = abs(self.speed) * fast * self.orient

                    self.rotate_pub.publish(self.rotate_msg)
                    print('steer angle : ', self.steer_angle)

                elif get == 'stop':
                    self.stop()    

                elif get == 'go_forward':
                    dist = float(self.server_pipe.recv())
                    rpm = float(self.server_pipe.recv())
                    print('distance : ', dist, 'rpm : ', rpm)
                    self.go_straight(dist, rpm)   

                    #self.stop()

                elif get == 'go_backward':
                    dist = float(self.server_pipe.recv())
                    rpm = float(self.server_pipe.recv())
                    print('distance : ', dist, 'rpm : ', rpm)
                    self.go_backward(dist, rpm)      

                    #self.stop()

                elif get == 'left_forward':
                    
                    #self.go_straight(0.2, 10.0)
                    rotate_angle = float(self.server_pipe.recv())*(-1)
                    steer_angle = float(self.server_pipe.recv())
                    rpm = float(self.server_pipe.recv())
                    print('rotate angle : ', rotate_angle, 'steer angle : ', steer_angle)
                    self.steer(rotate_angle, steer_angle, rpm, 1)

                    #self.stop()

                elif get == 'right_forward':
                    #self.go_straight(0.2, 10.0)
                    rotate_angle = float(self.server_pipe.recv())
                    steer_angle = float(self.server_pipe.recv())
                    rpm = float(self.server_pipe.recv())
                    print('rotate angle : ', rotate_angle, 'steer angle : ', steer_angle)
                    self.steer(rotate_angle, steer_angle, rpm, 1)

                    #self.stop()

                elif get == 'left_backward':
                    #self.go_backward(0.2, 10.0)
                    rotate_angle = float(self.server_pipe.recv())*(-1)
                    steer_angle = float(self.server_pipe.recv())
                    rpm = float(self.server_pipe.recv())
                    print('rotate angle : ', rotate_angle, 'steer angle : ', steer_angle)
                    self.steer(rotate_angle, steer_angle, rpm, -1)

                    #self.stop()

                elif get == 'right_backward':
                    #self.go_backward(0.2, 10.0)
                    rotate_angle = float(self.server_pipe.recv())
                    steer_angle = float(self.server_pipe.recv())
                    rpm = float(self.server_pipe.recv())
                    print('rotate angle : ', rotate_angle, 'steer angle : ', steer_angle)
                    self.steer(rotate_angle, steer_angle, rpm, -1)

                elif get == 'quit':
                    self.stop()  
                    plt.close()
                    break

                elif get == 'front':
                    self.msg.data = get
                    self.report_pub.publish(self.msg)

                elif get == 'rear':
                    self.msg.data = get
                    self.report_pub.publish(self.msg)

                elif get == 'tower':
                    self.msg.data = get
                    self.report_pub.publish(self.msg)

                elif get == 'arm':
                    self.msg.data = get
                    self.report_pub.publish(self.msg)

                elif get == 'plane':
                    self.msg.data = 'plane'
                    self.report_pub.publish(self.msg)

                elif get == 'curved':
                    self.msg.data = 'curved'
                    self.report_pub.publish(self.msg)    

                elif get == 'topview':
                    self.msg.data = 'topview'
                    self.report_pub.publish(self.msg)  
                    
                    self.msg.data = 'ik;L;[600,0,800,0,90,0,300,300]'
                    self.pose_pub.publish(self.msg)
                    
                elif get == 'right_cam':
                    self.msg.data = 'right_cam'
                    self.report_pub.publish(self.msg)  

                elif get == 'arm_tilt':
                    self.msg.data = 'arm_tilt'
                    self.report_pub.publish(self.msg)   

                elif get == 'leftwall':
                    self.msg.data = 'leftwall'
                    self.report_pub.publish(self.msg) 

                elif get == 'leftcorner':
                    self.msg.data = 'leftcorner'
                    self.report_pub.publish(self.msg)   

                elif get == 'frontwall':
                    self.msg.data = 'frontwall'
                    self.report_pub.publish(self.msg)           


                elif get == 'arm_camera_left':
                    self.msg.data = get
                    self.report_pub.publish(self.msg)   
                    
                    self.msg.data = 'fk;J;[161.14,-29.25,-134.53,163.82,-71.51,-0.12,30,30]'
                    self.pose_pub.publish(self.msg)

                elif get == 'turn90_left':
                    time.sleep(0.1)
                    degree = int(self.t0.get())
                    self.autonomous_turn(degree)
                    """
                    self.get_imu()
                    self.goal_theta = self.cur_theta + 90.0
                    #if 270 < self.cur_theta < 360:
                    #    self.goal_theta += 360
                    if self.goal_theta >= 360 :
                        self.goal_theta -= 360
                    e_theta = self.goal_theta - self.cur_theta
                    if e_theta < -10:
                        e_theta += 360 

                    while e_theta > 3.0:
                        self.get_imu()
                        e_theta = self.goal_theta - self.cur_theta
                        if e_theta < -10:
                            e_theta += 360    
                        pid_out = self.pid(e_theta)
                        print('self.goal_theta = ',self.goal_theta) 
                        print('self.cur_theta = ',self.cur_theta)
                        print('e_theta = ',e_theta)
                        #print('pid_out = ',pid_out)
                        self.rotate_msg.linear.x = pid_out
                        self.rotate_msg.linear.y = -pid_out
                        self.rotate_pub.publish(self.rotate_msg)
                        if self.t0.qsize() != 0:
                            get = self.t0.get()
                            if get == 'stop': 
                                self.stop()
                                break 
                    """
                elif get == 'auto':
                    for i in range(self.goal_num):
                        print('start turn')
                        self.autonomous_turn(self.goal_degree)
                        print('stop turn')
                        self.walk = True
                        for i in range(10):
                            self.get_imu()
                        
                        print('start walk')
                        self.autonomous_walk(self.goal_dist)
                        print('stop walk')
                        for i in range(10):
                            self.get_imu()

                    
            get = ''

            if self.yaw_origin == -1000:
                while True:
                    self.t1.put('imu')
                    #rear_scan = self.pipe.recv()
                    #front_scan = self.pipe.recv()
                    imu = self.pipe.recv().split(';')
                    #if rear_scan == 0 or front_scan == 0:
                    #    continue

                    #imu = [0,0,0,0,0,0]

                    if float(imu[5]) >= 0:
                        self.yaw_origin = float(imu[5])
                    else:
                        self.yaw_origin = float(imu[5]) + 360

                    self.x_origin = float(imu[0])
                    self.y_origin = float(imu[1])
                    break  
            
            self.pitch = int(float(imu[4])) * (-1)
            #print(self.pitch)
            
            self.get_imu()

            time.sleep(0.05)


    def send_request(self, command, value):
        request = LiftCommand.Request()
        request.command = command
        request.value = value
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.rail_node, self.future)
        return self.future.result()

    def autonomous_walk(self, goal_dist):
        self.rotate_msg.linear.x = 10.0
        self.rotate_msg.linear.y = 10.0
        self.rotate_pub.publish(self.rotate_msg)

        while True:
            if not self.walk:
                self.rotate_msg.linear.x = 0.0
                self.rotate_msg.linear.y = 0.0
                self.rotate_pub.publish(self.rotate_msg)
                break
            else:
                self.get_imu()
                time.sleep(0.1)
                if (self.ob_left < 1.5 or self.ob_right < 1.5) and self.speed > 0:
                    #print(self.front_obstacle_dist)
                    self.msg.data = 'red:1'
                    self.gtm_pub.publish(self.msg)
                    self.msg.data = 'sound:1'
                    self.gtm_pub.publish(self.msg)
                    self.rotate_msg.linear.x = 0.0
                    self.rotate_msg.linear.y = 0.0
                    self.speed = 0
                    self.rotate_pub.publish(self.rotate_msg)
                    break


    def autonomous_turn(self, goal_degree):
        orient = 1
        print(goal_degree)
        if 180 < goal_degree < 360:
            goal_degree = 360 - goal_degree
            cur_degree = self.yaw
        else:
            orient = -1
            cur_degree = 360 - self.yaw
        print(orient)
        goal = cur_degree + goal_degree
        
        if goal >= 360 :
            goal -= 360

        e = goal - cur_degree

        if e < -10:
            e += 360 

        while e > 1.0:
            self.get_imu()
            if orient == 1:
                cur_degree = self.yaw
            else:
                cur_degree = 360 - self.yaw
            e = goal - cur_degree
            if e < -10:
                e += 360    
            pid_out = self.pid(e)
            if pid_out > 5:
                pid_out = 5.0
            elif pid_out < 1:
                pid_out = 1.0
            #print('self.goal_theta = ',goal) 
            #print('self.cur_theta = ',cur_degree)
            #print('e_theta = ',e)
            #print('pid_out = ',pid_out)
            self.rotate_msg.linear.x = pid_out * orient
            self.rotate_msg.linear.y = -pid_out * orient
            self.rotate_pub.publish(self.rotate_msg)
            if self.t0.qsize() != 0:
                get = self.t0.get()
                if get == 'stop': 
                    self.stop()
                    break 

        self.rotate_msg.linear.x = 0.0
        self.rotate_msg.linear.y = 0.0
        self.rotate_pub.publish(self.rotate_msg)  
        #self.walk = True

    #def autonomous_walk(self, goal_dist):
    #    self.remain_dist

    def detect(self):

        self.msg.data = 'detect'
        self.report_pub.publish(self.msg)
        time.sleep(10)
        self.rotate_msg.linear.x = 10.0
        self.rotate_msg.linear.y = 10.0
        self.rotate_pub.publish(self.rotate_msg)  

        while True:

            self.get_imu()

            if self.t2.qsize() != 0:
                get = self.t2.get()
                print(get)
                if get == 'detected':
                    time.sleep(1)
                    self.steer(-90, 45.0, 1)
                    break

            time.sleep(0.05)
    
    def go_straight(self, d, r):

        #order_time = self.dist_to_time(d, r)
        #start_time = datetime.now()
        start_position = (self.x, self.y)

        self.rotate_msg.linear.x = r
        self.rotate_msg.linear.y = r
        self.rotate_pub.publish(self.rotate_msg)  
        #self.steer_msg.distance = 1.0
        #self.steer_msg.theta = 0.0
        #self.steer_pub.publish(self.steer_msg)
        adjust_yaw = False

        while True:
            
            self.get_imu()
            """
            if self.pitch >= 2:
                
                print('pitch : ', self.pitch)
                #self.rotate_msg.linear.x = 10.0 * (1 + self.pitch * 10 / 45) 
                self.rotate_msg.linear.x = 30.0
                self.rotate_msg.linear.y = 30.0
                self.rotate_pub.publish(self.rotate_msg) 
            else:
                self.rotate_msg.linear.x = r
                self.rotate_msg.linear.y = r
                self.rotate_pub.publish(self.rotate_msg) 
            """
            #current_time = datetime.now()
            #elasped_time = current_time - start_time
            elapsed_walk = math.dist(start_position, (self.x, self.y))
            #print(elapsed_walk)
            #if elasped_time.seconds > order_time:

            if d - elapsed_walk < 0.2:

                self.rotate_msg.linear.x = 5.0
                self.rotate_msg.linear.y = 5.0

                self.rotate_pub.publish(self.rotate_msg)

            if d - elapsed_walk < 0.01:
                break

            if self.t0.qsize() != 0:
                get = self.t0.get()
                if get == 'stop': 
                    self.stop()
                    break
                elif get == 'pause':
                    while True:
                        self.rotate_msg.linear.x = 0.0
                        self.rotate_msg.linear.y = 0.0
                        self.rotate_pub.publish(self.rotate_msg)
                        time.sleep(0.5)
                        get = self.t0.get()
                        if get == 'resume':
                            self.rotate_msg.linear.x = 10.0
                            self.rotate_msg.linear.y = 10.0
                            self.rotate_pub.publish(self.rotate_msg)
                            break

            time.sleep(0.05)
        self.stop()
        self.t0.put('completed')
        self.msg.data = 'completed'
        time.sleep(1)
        self.unity_pub.publish(self.msg)

    def go_backward(self, d, r):   

        #order_time = self.dist_to_time(d, r)
        #start_time = datetime.now()
        start_position = (self.x, self.y)

        #self.steer_msg.distance = 1.0
        #self.steer_msg.theta = 0.0
        #self.steer_pub.publish(self.steer_msg)

        self.rotate_msg.linear.x = -1 * r
        self.rotate_msg.linear.y = -1 * r
        self.rotate_pub.publish(self.rotate_msg)    

        while True:

            self.get_imu()

            #current_time = datetime.now()
            #elasped_time = current_time - start_time

            #if elasped_time.seconds > order_time:
            #    break

            elapsed_walk = math.dist(start_position, (self.x, self.y))
            print(elapsed_walk)
            # if elasped_time.seconds > order_time:
            if d - elapsed_walk < 0.2:
                """
                if 1 < self.yaw < 5:
                    self.rotate_msg.linear.x = -5.0
                    self.rotate_msg.linear.y = -4.5
                    print('adjust yaw --')
                elif 355 < self.yaw < 359:
                    self.rotate_msg.linear.x = -4.5
                    self.rotate_msg.linear.y = -5.0
                    print('adjust yaw ++')
                else:
                """
                self.rotate_msg.linear.x = -5.0
                self.rotate_msg.linear.y = -5.0
                self.rotate_pub.publish(self.rotate_msg)

            if d - elapsed_walk < 0.01:
                break

            if self.t0.qsize() != 0:
                get = self.t0.get()
                if get == 'stop': 
                    self.stop()
                    break
                elif get == 'pause':
                    while True:
                        self.rotate_msg.linear.x = 0.0
                        self.rotate_msg.linear.y = 0.0
                        self.rotate_pub.publish(self.rotate_msg)
                        time.sleep(0.5)
                        get = self.t0.get()
                        if get == 'resume':
                            self.rotate_msg.linear.x = 10.0
                            self.rotate_msg.linear.y = 10.0
                            self.rotate_pub.publish(self.rotate_msg)
                            break

            time.sleep(0.05)

        self.stop()
        self.t0.put('completed')
        self.msg.data = 'completed'
        time.sleep(1)
        self.unity_pub.publish(self.msg)

    def dist_to_time(self, d, rpm):
        t = d / (rpm * 2 * np.pi * self.wheelradius / 60)
        return t
    
    def wheel_velocity(self, angle):
        icr = self.steerdist / math.tan(np.deg2rad(angle)) 
        print(icr)
        fast = (icr + self.wheeldist/2)/icr
        low = (icr - self.wheeldist/2)/icr
        #print(fast, low)

        return fast, low

    def steer(self, degree, steer_angle, rpm, orient):
        
        self.start_angle = self.yaw
        print('start angle', self.start_angle)

        fast, low = self.wheel_velocity(steer_angle)
        
        if degree > 0:
            self.steer_angle -= steer_angle
            print('steer', self.steer_angle, 'published')
            ##self.steer_msg.distance = 1.0
            #elf.steer_msg.theta = self.steer_angle
            #self.steer_pub.publish(self.steer_msg)
            #self.steer_angle += steer_angle
            
            self.rotate_msg.linear.x = rpm * low * orient
            self.rotate_msg.linear.y = rpm * fast * orient
            
            self.rotate_pub.publish(self.rotate_msg)

        elif degree < 0:
            self.steer_angle += steer_angle
            print('steer',self.steer_angle, 'published')
            #self.steer_msg.distance = 1.0
            #self.steer_msg.theta = self.steer_angle
            #self.steer_pub.publish(self.steer_msg)
            #self.steer_angle -= steer_angle
            
            self.rotate_msg.linear.x = rpm * fast * orient
            self.rotate_msg.linear.y = rpm * low * orient
            
            self.rotate_pub.publish(self.rotate_msg)

        if orient == -1:
            degree *= -1

        rotated = self.calculate_rotated_angle(degree, fast, low, orient, rpm)
        self.stop()
        self.t0.put('completed')
        self.msg.data = 'completed'
        time.sleep(1)
        self.unity_pub.publish(self.msg)
        """
        if rotated:     
            
            self.steer_angle = 0.0
            self.steer_msg.distance = 1.0 
            self.steer_msg.theta = self.steer_angle
            self.st
            self.rotate_msg.linear.x = rpm * orient
            self.rotate_msg.linear.y = rpm * orient

            self.rotate_pub.publish(self.rotate_msg)

            time.sleep(2)
        """
    
        print('steer mode end', self.yaw)
      
    def stop(self):
        #time.sleep(2)
        self.speed = 0
        self.rotate_msg.linear.x = 0.0
        self.rotate_msg.linear.y = 0.0    
        self.rotate_pub.publish(self.rotate_msg)

    def m_home(self):
        asin = math.asin(self.x/math.dist((0, 0), (self.x, self.y)))
        d = 360 - (math.degrees(asin) + 90 - self.yaw)
        print(asin, d)
        self.autonomous_turn(d)
        time.sleep(0.5)
        self.go_straight(math.dist((0, 0), (self.x, self.y)), 10.0)
        time.sleep(0.5)
        d = math.degrees(asin) + 90
        self.autonomous_turn(d)
        time.sleep(0.5)
    
    def get_imu(self):
        self.t1.put('imu')
        #rear_scan = self.pipe.recv()
        #front_scan = self.pipe.recv()
        recv = self.pipe.recv()
        #self.msg.data = recv
        #self.unity_pub.publish(self.msg)
        imu = recv.split(';')
        #imu = [0,0,0,0,0,0]
        #print(imu[3], imu[4])

        self.pitch = int(float(imu[4])) * (-1)  # imu[1]

        if float(imu[5]) >= 0:
            self.yaw = float(imu[5]) - self.yaw_origin
        else:
            self.yaw = float(imu[5]) + 360 - self.yaw_origin

        if self.yaw < 0:
            self.yaw += 360

        self.x = float(imu[0]) - self.x_origin
        self.y = float(imu[1]) - self.y_origin




        #front_x, front_y, rear_x, rear_y = self.lidar_plot_data(front_scan, rear_scan)
        """
        if (self.ob_left < 1.5 or self.ob_right < 1.5) and self.speed > 0:
            #print(self.front_obstacle_dist)
            self.msg.data = 'red:1'
            self.gtm_pub.publish(self.msg)
            self.msg.data = 'sound:1'
            self.gtm_pub.publish(self.msg)
            self.rotate_msg.linear.x = 0.0
            self.rotate_msg.linear.y = 0.0
            self.speed = 0
            self.rotate_pub.publish(self.rotate_msg)
        
        if self.degree > 1.5 and self.speed == 10.0:
            self.rotate_msg.linear.x = 10.0
            self.rotate_msg.linear.y = 11.0
            self.rotate_pub.publish(self.rotate_msg)
            print('turn right')
        elif self.degree < -1.5 and self.speed == 10.0:
            self.rotate_msg.linear.x = 11.0
            self.rotate_msg.linear.y = 10.0
            self.rotate_pub.publish(self.rotate_msg)
            print('turn left')
        else:
            if self.rotate_msg.linear.x == 11.0 or self.rotate_msg.linear.y == 11.0:
                self.rotate_msg.linear.x = 10.0
                self.rotate_msg.linear.y = 10.0
                self.rotate_pub.publish(self.rotate_msg)
        
        self.server_pipe.send(self.yaw)
        self.server_pipe.send(self.walk)
        self.server_pipe.send(front_x)
        self.server_pipe.send(front_y)
        self.server_pipe.send(rear_x)
        self.server_pipe.send(rear_y)
        """
        #goal = self.server_pipe.recv()
        #print(goal)
        #self.goal_degree, self.goal_dist, self.goal_num = goal[0], goal[1], goal[2]
        #if self.goal_dist < 0.1:
        #    self.walk = False

        #print('atan: ',math.atan(22/460))

    def calculate_rotated_angle(self, degree, fast, low, orient, rpm):

        #degree = math.radians(degree)
        
        if degree > 0:
            turn_right = True
            print('turn right')
        else:
            turn_right = False
            print('turn left')

        while True:

            self.get_imu()

            yaw_rad = math.radians(self.yaw)
            print('now', self.yaw)

            if turn_right: 
                elasped_angle = round(self.start_angle - self.yaw, 2)
            else:
                elasped_angle = round(self.yaw - self.start_angle, 2)
                degree = abs(degree)
            print('elapsed angle', elasped_angle)

            #print(degree, self.yaw_rad, self.start_angle)

            if elasped_angle < -200:
                elasped_angle += 360
                print('elapsed angle+++', elasped_angle)   #math.pi * 2

            target_left = degree - elasped_angle
            if target_left < -200:
                target_left += 360
                
            print('target left', target_left)
            #left_degree = target_left * 180 / math.pi

            #print(elasped_angle, left_degree, 'left')

            if target_left < 7.0:
                break

            if self.t0.qsize() != 0:
                get = self.t0.get()
                if get == 'stop': 
                    self.stop()
                    break
                elif get == 'pause':
                    while True:
                        self.rotate_msg.linear.x = 0.0
                        self.rotate_msg.linear.y = 0.0
                        self.rotate_pub.publish(self.rotate_msg)
                        time.sleep(0.5)
                        get = self.t0.get()
                        if get == 'resume':
                            if degree > 0:
                                self.rotate_msg.linear.x = rpm * low * orient
                                self.rotate_msg.linear.y = rpm * fast * orient
                            else:
                                self.rotate_msg.linear.x = rpm * fast * orient
                                self.rotate_msg.linear.y = rpm * low * orient
                            self.rotate_pub.publish(self.rotate_msg)
                            break

            time.sleep(0.03)
                
        return True  

    def pid(self, state):
        cur_time = Clock().now().to_msg()
        #print("cur_time = ",cur_time.nanosec)
        #print("self.pre_time = ",self.pre_time.nanosec)
        dt = cur_time.sec - self.pre_time.sec
        if self.dt == 0.0:
            state_D = 0.0
        else:
            state_D = (state - self.pre_state) / self.dt

        state_I = state + self.integrated_state

        #out = self.P*state + self.D*state_D + self.I*state_I * self.dt
        out = self.P*state

        if out > self.max_state:
            out = self.max_state
        elif out < self.min_state:
            out = self.min_state

        self.pre_state = state
        self.integrated_state = state_I
        self.pre_time = Clock().now().to_msg()

        return out   
    
    def lidar_plot_data(self, front_scan, rear_scan):
        rear_min_angle = rear_scan[-2]
        rear_increment_angle = rear_scan[-1]
        del rear_scan[-1]
        del rear_scan[-1]
        #print(math.radians(225.5))

        rear_lidar_x = []
        rear_lidar_y = []

        front_min_angle = front_scan[-2]
        front_increment_angle = front_scan[-1]
        del front_scan[-1]
        del front_scan[-1]

        front_lidar_x = []
        front_lidar_y = []

        degrees = [(((rear_min_angle + rear_increment_angle*index)*180/math.pi)) for index, value in enumerate(rear_scan)]
        rear_rotate = np.array([[math.cos(math.radians(225.5)), -math.sin(math.radians(225.5))], [math.sin(math.radians(225.5)),math.cos(math.radians(225.5))]]) #좌표계 회전 변환을 위한 행렬 선언
        front_rotate = np.array([[math.cos(math.radians(45)), -math.sin(math.radians(45))], [math.sin(math.radians(45)),math.cos(math.radians(45))]])

        for i in range(0,len(rear_scan)):
            rad = math.radians(degrees[i])
            lidar_rotated = np.matmul(rear_rotate, np.array([(rear_scan[i]+0.03)*math.cos(rad), (rear_scan[i]+0.03)*math.sin(rad)])) #행렬 곱 연산
            rear_lidar_x.append(lidar_rotated[0]*(-1))
            rear_lidar_y.append(lidar_rotated[1])  

        rear_lidar_x = np.array(rear_lidar_x)
        rear_lidar_y = np.array(rear_lidar_y)

        for i in range(0,len(front_scan)):
            rad = math.radians(degrees[i])
            lidar_rotated = np.matmul(front_rotate, np.array([(front_scan[i]+0.03)*math.cos(rad), (front_scan[i]+0.03)*math.sin(rad)])) #행렬 곱 연산
            front_lidar_x.append(lidar_rotated[0]*(-1))
            front_lidar_y.append(lidar_rotated[1]) 

        front_lidar_x = np.array(front_lidar_x)
        front_lidar_y = np.array(front_lidar_y)

        #front_rotated = np.matmul(self.rotate, np.array([-0.295, 0.1]))
        #rear_rotated = np.matmul(self.rotate, np.array([0.295, -0.75]))

        front_x = front_lidar_x - 0.295 #+ front_rotated[0]
        front_y = front_lidar_y + 0.1 #front_rotated[1]
        rear_x = rear_lidar_x + 0.295 #rear_rotated[0]
        rear_y = rear_lidar_y -0.75 #+ rear_rotated[1] 
        left_y = front_y[540]
        right_y = rear_y[0]
        if left_y > 0.3 and right_y > 0.3:
            self.front_obstacle_dist = (left_y + right_y) / 2
            self.ob_right = right_y
            self.ob_left = left_y

            diff_y = left_y - right_y
            degree = math.degrees(math.atan(diff_y / 0.59))

            if len(self.degrees) < 10:
                self.degrees.append(degree)
            elif len(self.degrees) == 10:
                self.degree = np.mean(self.degrees)
                del self.degrees[0]
                #print(self.degree)


        return front_x, front_y, rear_x, rear_y    
