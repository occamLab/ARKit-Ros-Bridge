#!/usr/bin/env python

'''
OccamLab: ARKit ROS Bridge - Pose Server
author: @danielconnolly
'''

import sys
import numpy as np
from math import pi
import rospy
from geometry_msgs.msg import PoseStamped, Point32
from std_msgs.msg import Float64, Float64MultiArray, String, Int32
from tf.transformations import quaternion_multiply, quaternion_about_axis, quaternion_from_matrix, translation_from_matrix
import tf
from handle_udp import extractUDP

class PoseServer:
    """ Pose server for iOS to ROS """

    def __init__(self):
        ''' Initialize the server to enable the collection and publication of pose data.'''
        rospy.init_node('iOS_pose')
        self.port = rospy.get_param('~port_number')
        self.pose_topic = rospy.get_param('~pose_topic')
        self.coordinate_frame = rospy.get_param('~coordinate_frame')
        self.pub_pose = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        self.pub_clock = rospy.Publisher('/ios_clock', Float64, queue_size=10)
        self.br = tf.TransformBroadcaster()
        self.ios_clock_valid = False
        self.ios_clock_offset = -1.0
        self.last_timestamp = rospy.Time.now()
        self.msg = PoseStamped()
        self.msg.header.frame_id = self.coordinate_frame
        self.pose_vals = None
        self.ios_timestamp = None
        self.rotation_matrix = None
        self.pose_data = None

    def get_data(self):
        ''' Get pose data from the udp port. '''
        self.pose_data = extractUDP(udp_port=self.port)
        self.pose_vals = self.pose_data.split(",")

    def handle_ios_clock(self):
        ''' Handle differences in time between the iOS device and ROS. '''
        self.ios_timestamp = self.pose_vals[16]
        ROS_timestamp = rospy.Time.now()
        if not self.ios_clock_valid:
            self.ios_clock_offset = ROS_timestamp.to_time() - float(self.ios_timestamp)
        self.pub_clock.publish(self.ios_clock_offset)
        self.msg.header.stamp = rospy.Time(self.ios_clock_offset + float(self.ios_timestamp))

    def process_pose(self):
        ''' Get the position and orientation of the device from the iOS device. '''
        self.pose_vals = [float(x) for x in self.pose_vals[:16]]
        #Get the transformation matrix from the server and transpose it to row-major order
        self.rotation_matrix = np.matrix([self.pose_vals[0:4], self.pose_vals[4:8], self.pose_vals[8:12], self.pose_vals[12:16]]).T
        #Changing from the iOS coordinate space to the ROS coordinate space.
        change_basis = np.matrix([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
        change_basis2 = np.matrix([[0, 0, -1, 0], [0, -1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])
        #Left multiplying swaps rows, right multiplying swaps columns
        new_mat = change_basis*self.rotation_matrix*change_basis2
        new_mat = new_mat.A

        #Get the position and orientation from the transformed matrix.
        trans = translation_from_matrix(new_mat)
        quat = quaternion_from_matrix(new_mat)

        self.msg.pose.position.x = trans[0]
        self.msg.pose.position.y = trans[1]
        self.msg.pose.position.z = trans[2]

        self.msg.pose.orientation.x = quat[0]
        self.msg.pose.orientation.y = quat[1]
        self.msg.pose.orientation.z = quat[2]
        self.msg.pose.orientation.w = quat[3]


    def run(self):
        ''' Publish pose data and enable visualization of the device and camera axes. '''
        while not rospy.is_shutdown():
            self.get_data()
            self.handle_ios_clock()
            self.process_pose()
            self.br.sendTransform([self.msg.pose.position.x, self.msg.pose.position.y, self.msg.pose.position.z],
                                  [self.msg.pose.orientation.x, self.msg.pose.orientation.y, self.msg.pose.orientation.z, self.msg.pose.orientation.w],
                                  self.msg.header.stamp,
                                  "real_device",
                                  "odom")

            # Convert from ROS device coordinate frame to ROS camera frame
            camera_quat1 = quaternion_about_axis(pi/2, [0, 1, 0])
            camera_quat2 = quaternion_about_axis(pi/2, [0, 0, -1])

            self.br.sendTransform([0.0, 0.0, 0.0],
                                  quaternion_multiply(camera_quat1, camera_quat2),
                                  self.msg.header.stamp,
                                  "camera",
                                  "real_device")

            self.pub_pose.publish(self.msg)

if __name__ == '__main__':
    node = PoseServer()
    node.run()
