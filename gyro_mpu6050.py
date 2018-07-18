#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This script handles the data coming from a mpu6050 sensor and publishes a
gyroscope and an IMU message to ROS topics gyro and imu respectively.
It uses the mpu6050 class developed by MrTijn/Tijndagamer (slightly modified
by adding the additional function gyro_data_for_ros)
'''

import mpu6050
import rospy
import socket
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from math import sqrt,atan2,cos,sin,pi


def imu_publisher():

    theta = 0
    gyro_x_offset = 0.0
    gyro_y_offset = 0.0
    gyro_z_offset = 0.0
    pub_freq = 10
    count = 0
    num_callibration_itrs = 2000
    debug = False

    linearaccel_pub = rospy.Publisher('linearaccel', Vector3, queue_size=50)
    gyro_pub = rospy.Publisher('gyro', Vector3, queue_size=50)
    imu_pub = rospy.Publisher('imu', Imu, queue_size=50)
    rospy.init_node('imu_publisher', anonymous=True)
    rate = rospy.Rate(pub_freq)

    if rospy.has_param('~num_callibration_itrs'):
        num_callibration_itrs = rospy.get_param('~num_callibration_itrs')
    if rospy.has_param('~debug'):
        debug = rospy.get_param('~debug')

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    
    rospy.loginfo("waiting for device...")
    while not rospy.is_shutdown():
        gyro_data = mpu6050.gyro_data_for_ros()
        if len(gyro_data) >1:  #received complete packet
            current_time = rospy.Time.now()
            gyro_x = float(gyro_data['x'])
            gyro_y = float(gyro_data['y'])
            gyro_z = float(gyro_data['z'])

            if count < num_callibration_itrs:
                gyro_x_offset += gyro_x
                gyro_y_offset += gyro_y
                gyro_z_offset += gyro_z
                count += 1
            elif count == num_callibration_itrs and num_callibration_itrs != 0:
                gyro_x_offset /= num_callibration_itrs
                gyro_y_offset /= num_callibration_itrs
                gyro_z_offset /= num_callibration_itrs
                rospy.loginfo("finished callibrating yaw")
                count += 1

            #publish ros Imu message
            else:
                gyro_x -= gyro_x_offset
                gyro_y -= gyro_y_offset
                gyro_z -= gyro_z_offset

                #discretize readings to round off noise
                #gyro_x = float(int(gyro_x*100))/100.0;
                #gyro_y = float(int(gyro_y*100))/100.0;
                #gyro_z = float(int(gyro_z*100))/100.0;
                if debug:
                    rospy.loginfo('x %s y %s z %s', gyro_x, gyro_y, gyro_z)
                gyro_msg = Vector3()
                gyro_msg.x = gyro_x
                gyro_msg.y = gyro_y
                gyro_msg.z = gyro_z
                gyro_pub.publish(gyro_msg)

                dt = current_time.to_sec() - last_time.to_sec()
                theta += dt*gyro_z
                imu_msg = Imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = '/base_link'
                q = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
                imu_msg.orientation.x = q[0]
                imu_msg.orientation.y = q[1]
                imu_msg.orientation.z = q[2]
                imu_msg.orientation.w = q[3]
                imu_msg.orientation_covariance = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0] #[1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
                imu_msg.angular_velocity_covariance[0] = -1
                imu_msg.linear_acceleration_covariance[0] = -1
                imu_pub.publish(imu_msg)
            last_time = current_time
            #rate.sleep()
        else:
            rospy.loginfo("received incomplete data from mpu6050")
            continue
             

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass
