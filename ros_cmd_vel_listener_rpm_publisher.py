#!/usr/bin/env python

'''
This script listens to cmd_vel and publishes a "fake" rpm value to be used for odometry.
As this is innacurate and may diverge in time, the robot should be provided with good
orientation using a MPU6050 for example and we are also using viusla odometry from the
kinect sensor
'''

import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3Stamped

wheel_diameter = 0.080
track_width = 0.205
factor_linearX = 1.0
factor_angularZ = 1.0
resolution_vitesse = 0.25
x = 0.0
z = 0.0

pub_freq = 25.0
rospy.init_node('cmd_vel_listener_rpm_publisher', anonymous=True)
rpm_pub = rospy.Publisher('rpm', Vector3Stamped, queue_size=50)
rate = rospy.Rate(pub_freq)

def callback(msg):

	x = msg.linear.x
	z = msg.angular.z
    #print(x,z)

	rpm1 = (x*60.0*factor_linearX/(np.pi*wheel_diameter)-z*factor_angularZ*track_width*60.0/(wheel_diameter*np.pi*2.0))*resolution_vitesse
	rpm2 = (x*60.0*factor_linearX/(np.pi*wheel_diameter)+z*factor_angularZ*track_width*60.0/(wheel_diameter*np.pi*2.0))*resolution_vitesse
	#print('RPM 1 = '+str(rpm1),'RPM 2 = '+str(rpm2))

	rpm_msg = Vector3Stamped()
	rpm_msg.header.stamp = rospy.Time.now()
	rpm_msg.vector.x = rpm1
	rpm_msg.vector.y = rpm2
	rpm_msg.vector.z = 1.0/pub_freq #dt
    rpm_pub.publish(rpm_msg)

	#rate.sleep()

def listener_and_pub():
	rospy.Subscriber("/cmd_vel", Twist, callback) #/cmd_vel /key_vel /ps3_vel /joy
	rospy.spin()
	
if __name__ == '__main__':
	try:
		listener_and_pub()
	except rospy.ROSInterruptException:
		pass
