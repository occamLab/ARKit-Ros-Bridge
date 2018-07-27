#!/usr/bin/env python
"""
A simple echo server
"""

import socket
import rospy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
import tf


rospy.init_node("intrinsics_server")
camera_name = rospy.get_param('~camera_name')
port = rospy.get_param('~port_number')
pub_camera_info = rospy.Publisher('/' + camera_name + '/camera_info', CameraInfo, queue_size=10)

host = ''
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((host,port))

msg = CameraInfo()

def handle_intrinsics():
    while True:

        data = sock.recv(1024)
        intrinsics_vals = [float(x) for x in data.split(',')]

        ROS_time = rospy.Time.now()

        msg = CameraInfo(header=Header(stamp=rospy.Time.now()),
                        width=intrinsics_vals[5],
                        height=intrinsics_vals[6],
                        distortion_model='plumb_bob',
                        D=[0, 0, 0, 0, 0],
                        K=[intrinsics_vals[0], 0, intrinsics_vals[2],
     		 	 0, intrinsics_vals[1], intrinsics_vals[3], 0, 0, 1],
                        P =[intrinsics_vals[0], 0, intrinsics_vals[2], 0,
     		 	 0, intrinsics_vals[1], intrinsics_vals[3], 0, 0, 0, 1, 0])
        print(msg)
    pub_camera_info.publish(msg)


while True:
    handle_intrinsics()
