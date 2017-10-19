# -*- coding: utf-8 -*-
# -------------------------------------------------------
#!/usr/bin/env python

import roslib; roslib.load_manifest('mulfy_ros')
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import Vector3

from triangula.input import SixAxis, SixAxisResource

import traceback
import time
import os
import RPi.GPIO as GPIO
#import datetime as dt
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

i = 1

pub_freq = 10
rospy.init_node('ps3_vel_publisher')
ps3_pub = rospy.Publisher("/ps3_vel", Twist)
rate = rospy.Rate(pub_freq)

#pygame.mixer.init()
#pygame.mixer.music.load("groot.mp3")

def main():
   with SixAxisResource(bind_defaults=True) as joystick:
      # Register a button handler for the square button
      joystick.register_button_handler(handler_TR, SixAxis.BUTTON_TRIANGLE)
      joystick.register_button_handler(handler_CR, SixAxis.BUTTON_CROSS)
      joystick.register_button_handler(handler_L1, SixAxis.BUTTON_L1)
      joystick.register_button_handler(handler_R1, SixAxis.BUTTON_R1)
      while 1:
         try:
            # Read the x and y axes of the left hand stick, the right hand stick has axes 2 and 3
            x = joystick.axes[0].corrected_value()
            z = joystick.axes[1].corrected_value()
            x = float(x)
            z = float(z)
            print(x,z)
            ps3_msg = Twist() #Vector3() ???
            ps3_msg.linear.x = x
            ps3_msg.linear.y = 0.0
            ps3_msg.linear.z = 0.0
            ps3_msg.angular.x = 0.0
            ps3_msg.angular.y = 0.0
            ps3_msg.angular.z = z
            ps3_pub.publish(ps3_msg)
            
            # Code pour augmenter/diminuer la vitesse
            if i==4:
                GPIO.output(20,GPIO.LOW)
                GPIO.output(21,GPIO.LOW)
            if i==3:
                GPIO.output(20,GPIO.HIGH)
                GPIO.output(21,GPIO.LOW)
            if i==2:
                GPIO.output(20,GPIO.LOW)
                GPIO.output(21,GPIO.HIGH)
            if i==1:
                GPIO.output(20,GPIO.HIGH)
                GPIO.output(21,GPIO.HIGH)
                
            time.sleep(0.1)
            rospy.spin()
         except (KeyboardInterrupt, SystemExit):
            raise
         except:
            traceback.print_exc()

def handler_TR(button): # Claxon !!!
   print('Button {} pressed'.format(button))
   GPIO.output(27,GPIO.HIGH)
   time.sleep(0.1)
   GPIO.output(27,GPIO.LOW)

def handler_R1(button): # Vitesse UP !!!
   global i
   print('Button {} pressed'.format(button))
   if i>=1 and i<4:
      i+=1
      time.sleep(0.1)

def handler_L1(button): # Vitesse DOWN !!!
   global i
   print('Button {} pressed'.format(button))
   if i>1 and i<=4:
      i-=1
      time.sleep(0.1)

#def handler_CR(button): # Mulfy parle !!!
#   print('Button {} pressed'.format(button))
#   pygame.mixer.music.play()
#   while pygame.mixer.music.get_busy() == True:
#      continue

if __name__ == '__main__':
   main()

# -------------------------------------------------------