#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This script handles the data coming from the HyperIMU app from an android 
device and publish the /cmd_vel topic with linear (x) and angular (z)
velocities. The goal is to drive the robot using the pitch and roll signals
from the HyperIMU app and the phone's orientation sensor. In the app you 
should point the data to the IP address defined in the variable 'host' below.
Pitch and roll data are considered with 45° corresponding to the maximum
possible linear or angular velocity. A dead-band has been defined for angles
smaller than 5°
'''

import numpy as np
import roslib; roslib.load_manifest('mulfy_ros')
import rospy
import tf.transformations
from geometry_msgs.msg import Twist

import socket
import time
import os

host = '192.168.22.11' # The IP address of the machine running this code!
port = 5555

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind((host, port))

xp = [-45,45]
fp = [-1,1]

pub_freq = 10
hyperimu_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=50)
rospy.init_node('hyperimu_cmd_vel_publisher', anonymous=True)
rate = rospy.Rate(pub_freq)

def main():
    message, address = s.recvfrom(8192)
    azimuth, pitch, roll = read_msg(message)
    #print([azimuth,pitch,roll])
    if np.abs(pitch) <= 5:
        x = 0
    else:
        x = np.interp(float(pitch), xp, fp)
    if np.abs(roll) <= 5:
        z = 0
    else:
        z = np.interp(float(roll), xp, fp)
    hyperimu_msg = Twist()
    hyperimu_msg.linear.x = x
    hyperimu_msg.linear.y = 0.0
    hyperimu_msg.linear.z = 0.0
    hyperimu_msg.angular.x = 0.0
    hyperimu_msg.angular.y = 0.0
    hyperimu_msg.angular.z = z
    hyperimu_pub.publish(hyperimu_msg)
    #rate.sleep(0.1)
    time.sleep(0.1)

def read_msg(s):

   line = s.split(',')
   azimuth = int(float(line[0]))
   pitch = int(float(line[1]))
   int_var = line[2]
   int_var = int_var[:-1]
   roll = int(float(int_var))
   return azimuth, pitch, roll

if __name__ == '__main__':
    try:
        while True:
            main()
    except rospy.ROSInterruptException:
        pass

# -------------------------------------------------------
