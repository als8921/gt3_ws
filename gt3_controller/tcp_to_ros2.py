import socket
import rclpy
import std_msgs.msg as std_msgs
from datetime import datetime
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from pypcd4 import PointCloud
import time
from rclpy.clock import Clock

class Socket():
    def __init__(self, HOST, PORT, a_pc):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((HOST, PORT))
        #self.pipe = a_pc
        print('tcp ready!')

        rclpy.init()

        odom_node = rclpy.create_node('fastlio_odom')
        odom_pub = odom_node.create_publisher(std_msgs.String, '/fastlio_odom', 10)
        joint_node = rclpy.create_node('robotis_joint')
        joint_pub = joint_node.create_publisher(std_msgs.String, '/robotis_joint', 10)
        odom_pub2 = odom_node.create_publisher(Odometry, '/odom3', 10)
        
        msg = std_msgs.String()
        msg_odom = Odometry()
        pc_array = []

        self.socket.listen()

        print('wait')
        self.con, addr = self.socket.accept()

        print('==================')
        print('ROS1 connected!!!', addr)
        print('==================')

        while True:
            r = self.con.recv(1024).decode()
            #print(r)
            data = r.split(':')
            q = data[1].split(';')
            if data[0] == 'odom':
                msg.data = data[1]
                odom_pub.publish(msg)
                try:
                
                    time_now = Clock().now().to_msg()
                    msg_odom.header.stamp = time_now

                    msg_odom.pose.pose.position.x = float(q[0])
                    msg_odom.pose.pose.position.y = float(q[1])
                    msg_odom.pose.pose.position.z = float(q[2])
                    msg_odom.pose.pose.orientation.x = float(q[3])
                    msg_odom.pose.pose.orientation.y = float(q[4])
                    msg_odom.pose.pose.orientation.z = float(q[5])
                    msg_odom.pose.pose.orientation.w = float(q[6])
                    msg_odom.twist.twist.linear.x = float(q[7])
                    msg_odom.twist.twist.linear.y = float(q[8])
                    msg_odom.twist.twist.angular.z = float(q[9])
                    
                    odom_pub2.publish(msg_odom)
                except Exception as e:
                    print(e)
                
            elif data[0] == 'joint':
                msg.data = data[1]
                joint_pub.publish(msg)
