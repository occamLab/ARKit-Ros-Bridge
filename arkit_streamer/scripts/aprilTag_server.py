#!/usr/bin/env python

'''
OccamLab: ARKit ROS Bridge - April Tag Server
author: @danielconnolly
'''

import sys
import socket
import numpy as np
from math import pi
import rospy
from apriltags_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Point32
from std_msgs.msg import Float64, Float64MultiArray, String, Int32
from tf.transformations import quaternion_multiply, quaternion_about_axis, quaternion_from_matrix, translation_from_matrix
import tf
from handle_udp import extractUDP

class AprilTagServer:
    """ April Tag server for iOS to ROS """

    def __init__(self):
        ''' Initialize the server to enable the collection and publication of april tag data.'''
        rospy.init_node('aprilTag_server')
        self.port = rospy.get_param('~port_number')
        self.ios_clock_offset = 0
        self.pub_tags = rospy.Publisher('/april_tags_ios', AprilTagDetectionArray, queue_size=10)
        self.clock_sub = rospy.Subscriber('/ios_clock', Float64, self.handle_ios_clock)
        self.br = tf.TransformBroadcaster()
        self.sequence_number = 0
        self.last_timestamp = rospy.Time.now()
        self.april_tags = []
        self.pose = PoseStamped()
        self.msg = AprilTagDetectionArray()
        self.pose_data = None
        self.tag_time = None
        UDP_IP = "0.0.0.0"
        self.sock = socket.socket(socket.AF_INET, #Internet
                                  socket.SOCK_DGRAM) #UDP
        self.sock.bind((UDP_IP, self.port))

    def handle_ios_clock(self, msg):
        ''' Get the offset between iOS time and ROS time from the clock Subscriber. '''
        self.ios_clock_offset = msg.data

    def process_tag_pose(self):
        ''' Get the position and orientation of detected april tags from the iOS device. '''
        for tags in self.april_tags:
            ar = tags.split(",")
            current_tag = AprilTagDetection()
            current_tag.id = int(ar[0])
            current_tag.size = 0.165
            current_tag.pose.header.stamp = rospy.Time(float(self.ios_clock_offset) + float(ar[17]))
            current_tag.pose.header.frame_id = "camera"
            tag = [float(x) for x in ar[1:17]]
            mat = np.matrix([tag[0:4], tag[4:8], tag[8:12], tag[12:16]])
            new_mat = mat.A
            trans = translation_from_matrix(new_mat)
            quat = quaternion_from_matrix(new_mat)
            current_tag.pose.pose.position.x = trans[0]
            current_tag.pose.pose.position.y = trans[1]
            current_tag.pose.pose.position.z = trans[2]

            current_tag.pose.pose.orientation.x = quat[0]
            current_tag.pose.pose.orientation.y = quat[1]
            current_tag.pose.pose.orientation.z = quat[2]
            current_tag.pose.pose.orientation.w = quat[3]
            self.msg.detections.append(current_tag)

    def run(self):
        ''' Publish april tag data and enable visualization of the april tag poses. '''
        while not rospy.is_shutdown():
            self.pose_data, addr = self.sock.recvfrom(1024)
            if len(self.pose_data) > 5:
                self.april_tags = self.pose_data.split(",TAG,")[1:]
            self.process_tag_pose()
            for tag in self.msg.detections:
                self.br.sendTransform([tag.pose.pose.position.x, tag.pose.pose.position.y, tag.pose.pose.position.z],
                                      [tag.pose.pose.orientation.x, tag.pose.pose.orientation.y, tag.pose.pose.orientation.z, tag.pose.pose.orientation.w],
                                       tag.pose.header.stamp,
                                      "tag_" + str(tag.id),
                                      "camera")

            self.pub_tags.publish(self.msg)
            self.sequence_number += 1
            self.april_tags = []
            self.msg.detections = []

if __name__ == '__main__':
    node = AprilTagServer()
    node.run()
