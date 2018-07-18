#!/usr/bin/env python

'''
This script listens to /cmd_vel topic and sends the linear (x) and angular (z) 
velocities commands to an Arduino connected trough i2c at the specified slave adress.
The values sent to the Arduino are transformed as int and ranges from 0 to 200.
'''

import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from smbus import SMBus

I2C_Bus = SMBus(1) # Remplacer 0 par 1 pour Raspberry Pi 3
SLAVE_ADD = 0x8 # Direction esclave pour commande vitesse
command = 999
type_cmd = 1 # (0) Ps3 (1) cmd_vel
wheel_diameter = 0.080
track_width = 0.205
factor_linearX = 1.0
factor_angularZ = 1.0
x = 0.0
z = 0.0

rospy.init_node('ros_rpi3_arduino', anonymous=True)

def callback(msg):
	#print('LinearX = '+str(msg.linear.x),'AngularZ = '+str(msg.angular.z))

	# If using cmd_vel or key_vel:
	x = int(msg.linear.x*100.0*factor_linearX+100.0)
	z = int(msg.angular.z*100.0*factor_angularZ+100.0)

	#print('LinearX = '+str(x),'AngularZ = '+str(z))

	# If using joy:
	#x = int(msg.axes[1]*100.0*factor_linearX+100.0)
	#z = int(msg.axes[0]*100.0*factor_angularZ+100.0)

	# Writing to Arduino using i2c
	I2C_Bus.write_i2c_block_data(SLAVE_ADD,command,[type_cmd,x,z])

def listener():
	rospy.Subscriber("/cmd_vel", Twist, callback) #/cmd_vel /key_vel /ps3_vel /joy
	rospy.spin()
	
if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
