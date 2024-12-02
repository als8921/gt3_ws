import rospy
from std_msgs.msg import String, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
import std_msgs.msg as std_msgs
import math
import socket
from multiprocessing import Process, Pipe, Queue
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation

import time

a_pipe, b_pipe = Pipe()
a_pc, b_pc = Pipe()

t0 = Queue()
t1 = Queue()

i = 0

def listener(a_pipe, a_pc, t0, t1):

    def odm_callback(data):
        # rospy.loginfo("I heard: %s", data.pose.pose.position)
        # rospy.loginfo("I heard: %s", data.pose.pose.orientation.z)

        p_x = str(round(data.pose.pose.position.x, 2))
        p_y = str(round(data.pose.pose.position.y, 2))
        p_z = str(round(data.pose.pose.position.z, 2))
        r_x = str(round(data.pose.pose.orientation.x, 2))
        r_y = str(round(data.pose.pose.orientation.y, 2))
        r_z = str(round(data.pose.pose.orientation.z, 2))
        r_w = str(round(data.pose.pose.orientation.w, 2))
        t_x = str(round(data.twist.twist.linear.x, 2))
        t_y = str(round(data.twist.twist.linear.y, 2))
        t_z = str(round(data.twist.twist.angular.z, 2))

        data = 'odom:' + p_x + ';' + p_y + ';' + p_z + ';' + r_x + ';' + r_y + ';' + r_z + ';' + r_w + ';' + t_x + ';' + t_y + ';' + t_z
        #print(data.twist)
        t0.put(data)
        #print(data)
        #a_pipe.send(data.pose.pose.orientation.w)

    def master_callback(msg):
        r = msg.data
        joint = []
        joint.append(np.degrees(r[0]))
        joint.append(np.degrees(r[1]))
        joint.append(np.degrees(r[2]))
        joint.append(np.degrees(r[3]))
        joint.append(r[4])

        #print(r[4])
        data = 'joint:' + str(joint)
        t0.put(data)

    # 노드 초기화
    rospy.init_node('listener', anonymous=True)

    # 구독자 설정
    print('node initialized')
    rospy.Subscriber('Odometry', Odometry, odm_callback)
    rospy.Subscriber('/goal_joint_position',  Float64MultiArray, master_callback)

    # 계속해서 콜백을 받기 위해 스핀
    rospy.spin()

def client_socket(b_pipe, t0, t1):
    Client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while True:
        try:
            Client_socket.connect(('192.168.1.5', 9999))
            break
        except Exception as e:
            a = 1

    while True:
        if t0.qsize() != 0:
            data = t0.get()
            Client_socket.send(data.encode('utf-8'))
            #time.sleep(0.02)
            #Client_socket.send('end'.encode('utf-8'))
            #time.sleep(0.02)

def pc_draw(b_pc):
    gs = gridspec.GridSpec(1, 1)
    fig = plt.figure(figsize=(13, 8))
    ax = fig.add_subplot(gs[0, 0], projection='3d')
    ax.set_xlim(-17, 17)
    ax.set_ylim(-15, 20)
    ax.set_zlim(-15, 20)
    ax.set_aspect('equal')

    cloud_arr = b_pc.recv()
    pc_len = len(cloud_arr)

    x, y, z = [], [], []

    for i in range(pc_len):
        if i%320 == 0 and i < pc_len-320:
            x.append(cloud_arr[i][1]*-1)
            y.append(cloud_arr[i][0])
            z.append(cloud_arr[i][2])
            #for k in range(0, 4):
            #    print(cloud_arr[k+i])
            #print('-------------------------')




    lidar = ax.scatter(x, y, z, s=0.2,c='r')
    print(pc_len, len(x), len(x)*5)
    """
    def update(f):
        print(f)
        lidar = ax.scatter(x[f], y[f], z[f], s=0.2, c='r')
        ax.figure.canvas.draw()
        return lidar,

    anim = animation.FuncAnimation(fig, update,
                                   frames=len(x)-1,
                                   interval=10, repeat=False)
   """

    plt.show()


if __name__ == '__main__':
    pc1 = Process(target=client_socket, args=(b_pipe, t0, t1))
    pc1.start()
    pc0 = Process(target=listener, args=(a_pipe, a_pc, t0, t1))
    pc0.start()
    pc2 = Process(target=pc_draw, args=(b_pc,))
    pc2.start()

