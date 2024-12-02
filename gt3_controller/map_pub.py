import numpy as np
import sys
import rclpy

import time

import os

import std_msgs.msg as std_msgs
from std_msgs.msg import Header
import std_msgs.msg as std_msgs
from sensor_msgs.msg import PointCloud2, PointField
from pypcd4 import PointCloud

rclpy.init()
pc_node = rclpy.create_node('registered_topic3')
publisher = pc_node.create_publisher(PointCloud2, '/registered_topic', 1)

msg = std_msgs.String()


def pointcloud_publish(pc_array):
    pc_data = np.array(pc_array, dtype=np.float32).flatten()

    pc2_msg = PointCloud2()
    pc2_msg.header = Header()
    pc2_msg.header.stamp = pc_node.get_clock().now().to_msg()

    pc2_msg.header.frame_id = 'fastlio2_link'

    pc2_msg.height = 1
    pc2_msg.width = len(pc_data) // 4
    pc2_msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
    pc2_msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
    pc2_msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
    pc2_msg.fields.append(PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1))
    # pc2_msg.fields.append(
    #    PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1))  # color
    pc2_msg.point_step = 16
    pc2_msg.row_step = pc2_msg.point_step * pc2_msg.width
    pc2_msg.is_dense = True
    pc2_msg.is_bigendian = False
    pc2_msg.data = pc_data.tobytes()

    # return pc2_msg
    publisher.publish(pc2_msg)
    # print('==================')
    # print(datetime.now(), 'Publishing PointCloud2 message from fastlio')
    # print('==================')

pc = PointCloud.from_path('../fastlio/src/FAST_LIO/PCD/scans_1.pcd')
                    #pc = PointCloud.from_path('scans_1.pcd')
arrar = pc.numpy(("x", "y", "z", "intensity"))
print(pc.fields)
length = len(arrar)
print(length//1000)

for i in range(length // 10000):
    #print(arrar[i * 1000:i*1000+200])
    pointcloud_publish(arrar[i * 10000:i*10000+10000])   #(i + 1) * 1000])
    time.sleep(0.01)


print('***********Notice***********')
print('        end publishing')
print('***********Notice***********')
