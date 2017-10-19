#!/usr/bin/env python

import roslib; roslib.load_manifest('mulfy_ros')
import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped
from smbus import SMBus

I2C_Bus = SMBus(1) # Remplacer 0 par 1 pour Raspberry Pi 3
SLAVE_ADD = 0x8 # Direction esclave pour commande vitesse
command = 999
type_cmd = 1 # (0) Ps3 (1) cmd_vel
wheel_diameter = 0.080
track_width = 0.205
factor_linearX = 0.5
factor_angularZ = 4.0
x = 0.0
z = 0.0
pub_freq = 10

def callback(msg):
    global x,z,factor_linearX,factor_linearZ
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    x = int(msg.linear.x*100.0*factor_linearX+100.0)
    z = int(msg.angular.z*100.0*factor_angularZ+100.0)
    #print(x,z)
    # Writing to Arduino using i2c
    I2C_Bus.write_i2c_block_data(SLAVE_ADD,command,[type_cmd,x,z]) # Exemple pour lire un message: reponse = bus.read_byte(address)
    x = (float(x) - 100.0)/(100.0*factor_linearX)
    z = (float(z) - 100.0)/(100.0*factor_angularZ)
    #print('LinearX = '+str(x),'AngularZ = '+str(z))

def listener_and_pub():
    global x,z,factor_linearX,factor_linearZ
    
    rospy.init_node('cmd_vel_listener_rpm_publisher')
    rospy.Subscriber("/ps3_vel", Twist, callback) #/cmd_vel #/key_vel
    rpm_pub = rospy.Publisher('rpm', Vector3Stamped, queue_size=50)
    rate = rospy.Rate(pub_freq)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    rospy.loginfo("waiting for device...")
    while not rospy.is_shutdown():
        # Compute rpm and dt HERE!
        current_time = rospy.Time.now()
        dt = current_time.to_sec() - last_time.to_sec()
        rpm1 = x*60.0*factor_linearX/(np.pi*wheel_diameter)-z*factor_angularZ*track_width*60.0/(wheel_diameter*np.pi*2.0)
        rpm2 = x*60.0*factor_linearX/(np.pi*wheel_diameter)+z*factor_angularZ*track_width*60.0/(wheel_diameter*np.pi*2.0)
        #print('RPM 1 = '+str(rpm1),'RPM 2 = '+str(rpm2))
        #time.sleep(1)

        rpm_msg = Vector3Stamped()
        rpm_msg.header.stamp = rospy.Time.now()
        rpm_msg.vector.x = rpm1
        rpm_msg.vector.y = rpm2
        rpm_msg.vector.z = dt
        rpm_pub.publish(rpm_msg)
        last_time = current_time

    rospy.spin()

if __name__ == '__main__':
    try:
        listener_and_pub()
    except rospy.ROSInterruptException:
        pass