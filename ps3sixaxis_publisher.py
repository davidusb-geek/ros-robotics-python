#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This script handles the data from a ps3 six axis controller and publishes
velocities commands to the ps3_vel topic for ROS. The velocities commands
come from the analog joystick of the ps3 controller.
Other possible option has been commented out, but for example a buzzer
connected to GPIO 27 could be activated by pressing the triangle button, 
or an mp3 file may be played pressing the cross button. 
'''

import roslib; roslib.load_manifest('mulfy_ros')
import rospy
import tf.transformations
from geometry_msgs.msg import Twist

from triangula.input import SixAxis, SixAxisResource

import traceback
import time
import os
import RPi.GPIO as GPIO
#import pygame

GPIO.setmode(GPIO.BCM) # or 'GPIO.board' pour numerotation 1, 2, 3, 4, ...
configuration=GPIO.getmode()
GPIO.setup(27,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(21,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(20,GPIO.OUT,initial=GPIO.LOW)

GPIO.output(27,GPIO.HIGH)
time.sleep(0.1)
GPIO.output(27,GPIO.LOW)

time.sleep(0.1)

GPIO.output(27,GPIO.HIGH)
time.sleep(0.1)
GPIO.output(27,GPIO.LOW)

#i = 1

#pygame.mixer.init()
#pygame.mixer.music.load("groot.mp3")

pub_freq = 10
ps3_pub = rospy.Publisher("/ps3_vel", Twist, queue_size=50)
rospy.init_node('ps3_vel_publisher', anonymous=True)
rate = rospy.Rate(pub_freq)

def main():
	with SixAxisResource(bind_defaults=True) as joystick:
		# Register buttons handlers
		#joystick.register_button_handler(handler_TR, SixAxis.BUTTON_TRIANGLE)
		#joystick.register_button_handler(handler_CR, SixAxis.BUTTON_CROSS)
		#joystick.register_button_handler(handler_L1, SixAxis.BUTTON_L1)
		#joystick.register_button_handler(handler_R1, SixAxis.BUTTON_R1)
		x = joystick.axes[0].corrected_value()
		y = joystick.axes[1].corrected_value()
		#print(x,y)
		new_x = float(y)
		new_z = float(x)
		ps3_msg = Twist()
		ps3_msg.linear.x = new_x
		ps3_msg.linear.y = 0.0
		ps3_msg.linear.z = 0.0
		ps3_msg.angular.x = 0.0
		ps3_msg.angular.y = 0.0
		ps3_msg.angular.z = new_z
		ps3_pub.publish(ps3_msg)
		# Code pour augmenter/diminuer la vitesse
		#if i==4:
		#	GPIO.output(20,GPIO.LOW)
		#	GPIO.output(21,GPIO.LOW)
		#if i==3:
		#	GPIO.output(20,GPIO.HIGH)
		#	GPIO.output(21,GPIO.LOW)
		#if i==2:
		#	GPIO.output(20,GPIO.LOW)
		#	GPIO.output(21,GPIO.HIGH)
		#if i==1:
		#	GPIO.output(20,GPIO.HIGH)
		#	GPIO.output(21,GPIO.HIGH)
		rate.sleep(0.1)


#def handler_TR(button): # Claxon !!!
#   print('Button {} pressed'.format(button))
#   GPIO.output(27,GPIO.HIGH)
#   time.sleep(0.1)
#   GPIO.output(27,GPIO.LOW)

#def handler_R1(button): # Vitesse UP !!!
#   global i
#   print('Button {} pressed'.format(button))
#   if i>=1 and i<4:
#      i+=1
#      time.sleep(0.1)

#def handler_L1(button): # Vitesse DOWN !!!
#   global i
#   print('Button {} pressed'.format(button))
#   if i>1 and i<=4:
#      i-=1
#      time.sleep(0.1)

#def handler_CR(button): # Mulfy parle !!!
#   print('Button {} pressed'.format(button))
#   pygame.mixer.music.play()
#   while pygame.mixer.music.get_busy() == True:
#      continue

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
