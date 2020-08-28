'''
OccamLab: ARKit ROS Bridge - Extract UDP Data
'''

import socket

def extractUDP(udp_port):
    ''' Gets UDP data from a given port. '''

    UDP_IP = "0.0.0.0"
    UDP_PORT = udp_port

    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))

    while True:
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        return(data.decode('utf-8'))
