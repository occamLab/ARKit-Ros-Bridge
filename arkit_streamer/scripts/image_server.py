#!/usr/bin/env python3

'''
OccamLab: ARKit ROS Bridge - Image Server
author: @danielconnolly
'''

from handle_udp import extractUDP
import rospy
import time
import socket
import struct
import numpy as np
import cv2
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from copy import deepcopy

class ImageServer:
    ''' Image Server for iOS to ROS '''

    def __init__(self):
        ''' Initialize the server to enable the collection and publication of image and camera data. '''
        rospy.init_node('image_server')
        self.port = rospy.get_param('~port_number')
        self.camera_name = rospy.get_param('~camera_name')
        self.ios_clock_offset = 0
        self.bridge = CvBridge()
        self.clock_sub = rospy.Subscriber('/ios_clock', Float64, self.handle_ios_clock)
        self.pub_camera = rospy.Publisher('/' + self.camera_name + '/image_raw/compressed', CompressedImage, queue_size=10)
        self.pub_lowres = rospy.Publisher('/' + self.camera_name + '_lowres/image_raw/compressed', CompressedImage, queue_size=10)
        self.pub_camera_info = rospy.Publisher('/' + self.camera_name + '/camera_info', CameraInfo, queue_size=10)
        self.pub_lowres_info = rospy.Publisher('/' + self.camera_name + '_lowres/camera_info', CameraInfo, queue_size=10)
        self.cvmsg = CompressedImage()
        self.image_data = {}
        self.msg = CompressedImage()
        self.intrinsic_msg = CameraInfo()
        self.updated_intrinsics = None
        self.last_packet_timestamp = rospy.Time(0.0)
        UDP_IP = "0.0.0.0"
        self.sock = socket.socket(socket.AF_INET, #Internet
                                  socket.SOCK_DGRAM) #UDP
        self.sock.bind((UDP_IP, self.port))

    def handle_ios_clock(self, msg):
        ''' Get the offset between iOS time and ROS time from the clock Subscriber. '''
        self.ios_clock_offset = msg.data

    def complete_packet_assembly(self, image_number):
        ''' Finish assembling an image when all of its packets have been received. '''
        self.image_data[image_number]['payload'].sort()
        image = bytearray()
        for packet in self.image_data[image_number]['payload']:
            image += packet[1]

        self.msg.header.stamp  = rospy.Time(float(self.ios_clock_offset) + float(self.image_data[image_number]['timestamp']))
        self.msg.header.frame_id = self.camera_name
        self.msg.data = image
        self.msg.format = 'jpeg'

        # Convert the iOS image to a cv2 image and lower the resolution
        resize_factor = 1/3.
        cv_image = self.bridge.compressed_imgmsg_to_cv2(self.msg)
        flipped_image = cv2.transpose(cv_image) # Transpose and flip the image so it is aligned with the correct camera axes
        flipped_image = cv2.flip(flipped_image, 1)
        full_res = self.bridge.cv2_to_compressed_imgmsg(flipped_image)
        self.msg.data = full_res.data
        lower_res = cv2.resize(flipped_image, (0,0), fx=resize_factor, fy=resize_factor)
        image = self.bridge.cv2_to_compressed_imgmsg(lower_res)
        self.cvmsg.data = image.data
        self.cvmsg.format = image.format
        self.cvmsg.header.stamp  = rospy.Time(float(self.ios_clock_offset) + float(self.image_data[image_number]['timestamp']))
        self.cvmsg.header.frame_id = self.camera_name

        # Update the camera intrinsics for the flipped images
        self.updated_intrinsics = CameraInfo()
        self.updated_intrinsics.K = [v * resize_factor if v != 1.0 else v for v in self.image_data[image_number]['intrinsics_message'].K]
        self.updated_intrinsics.P = [v * resize_factor if v != 1.0 else v for v in self.image_data[image_number]['intrinsics_message'].P]
        self.updated_intrinsics.width = self.image_data[image_number]['intrinsics_message'].width * resize_factor
        self.updated_intrinsics.height = self.image_data[image_number]['intrinsics_message'].height * resize_factor
        self.updated_intrinsics.header.stamp  = rospy.Time(float(self.ios_clock_offset) + float(self.image_data[image_number]['timestamp']))
        self.updated_intrinsics.header.frame_id = self.camera_name


    def run(self):
        ''' Publish image and camera intrinsics data. '''
        while not rospy.is_shutdown():
            data, addr = self.sock.recvfrom(1600)
            data = bytearray(data)
            packet_offset = 0
            image_number, packet_number = struct.unpack('<BB', data[packet_offset:packet_offset + 2])
            packet_offset += 2

            # Check if the packet is the first in a given image
            if packet_number == 0:
                total_packets = data[packet_offset]
                packet_offset += 1
                self.image_data[image_number] = {}
                self.image_data[image_number]['packets_expected'] = total_packets
                self.image_data[image_number]['packets_received'] = 1

                time_bytes = data[packet_offset]
                packet_offset += 1
                intrinsic_bytes = data[packet_offset]
                packet_offset += 1
                stampedTime = data[packet_offset:packet_offset+time_bytes]  # TODO: remove magic numbers of 5 and 4
                intrinsic_data = data[packet_offset+time_bytes:packet_offset+time_bytes+intrinsic_bytes].decode('utf-8')
                intrinsics_vals = [float(x) for x in intrinsic_data.split(',')]

                self.image_data[image_number]['timestamp'] = stampedTime
                self.image_data[image_number]['payload'] = [(packet_number, data[packet_offset+time_bytes+intrinsic_bytes:])]

                # Set the camera intrinsics for each image
                # The camera images are always transmitted from iOS in a landscape format, even when the camera is in portrait mode. Thus,we will utilize cv2 to flip and transpose the image
                self.intrinsic_msg = CameraInfo(header=Header(stamp=rospy.Time(float(self.ios_clock_offset) + float(stampedTime))),
                                width=intrinsics_vals[6], # Preemptively swap the width and the height since the image will be flipped
                                height=intrinsics_vals[5],
                                distortion_model='plumb_bob',
                                D=[0, 0, 0, 0, 0], #The iPhone automatically adjusts the image to remove distortion.
                                K=[intrinsics_vals[0], 0, intrinsics_vals[3], # Preemptively switch the principal point offsets since the image will be transposed and flipped
             		 	 0, intrinsics_vals[1], intrinsics_vals[2], 0, 0, 1], # vals out of order because iOS uses column-major order
                                P =[intrinsics_vals[0], 0, intrinsics_vals[3], 0,
             		 	 0, intrinsics_vals[1], intrinsics_vals[2], 0, 0, 0, 1, 0])
                self.image_data[image_number]['intrinsics_message'] = self.intrinsic_msg

            # Check if a partially completed image exists
            elif image_number in self.image_data.keys():
                self.image_data[image_number]['packets_received'] += 1
                self.image_data[image_number]['payload'] += [(packet_number, data[packet_offset:])]

                # Check if all of the packets for an image have been received
                if self.image_data[image_number]['packets_received'] == self.image_data[image_number]['packets_expected']:
                    self.complete_packet_assembly(image_number)
                    # Ensure images are not published if an image with a later timestamp has already been published
                    if self.last_packet_timestamp == 0.0 or self.last_packet_timestamp < self.msg.header.stamp:
                        self.pub_camera.publish(self.msg)
                        self.pub_camera_info.publish(self.image_data[image_number]['intrinsics_message'])
                        self.pub_lowres.publish(self.cvmsg)
                        self.pub_lowres_info.publish(self.updated_intrinsics)
                        self.last_packet_timestamp = self.msg.header.stamp

if __name__ == '__main__':
    node = ImageServer()
    node.run()
