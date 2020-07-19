# ros-robotics-python

Some example files using python for publishing and subscribing to topics in ROS for some basic robotics. 

These files are used on a real robot combining ROS on a RPi3 and a client computer, a kinect sensor, an mpu6050 sensor for orientation, an Arduino Nano connected trough i2c to the RPi3 controlling two stepper motors and driven by two DRV8825 drivers, a smartphone with the HyperIMU app installed.

These examples includes:

**gyro_mpu6050.py**

Handling the data coming from a mpu6050 sensor and publishes a
gyroscope and an IMU message to ROS topics gyro and imu respectively.

**hyperimu_cmd_vel.py**

Handling the data coming from the HyperIMU app from an android 
device and publish the /cmd_vel topic with linear (x) and angular (z)
velocities. The goal is to drive the robot using the pitch and roll signals
from the HyperIMU app and the phone's orientation sensor. 

**mpu6050.py**

A slighly modified class used by gyro_mpu6050.py. The original class
is the work of MrTijn/Tijndagamer

**ps3sixaxis_publisher.py**

Handling the data from a ps3 six axis controller and publishes
velocities commands to the ps3_vel topic for ROS. The velocities commands
come from the analog joystick of the ps3 controller.

**ros_cmd_vel_listener_rpm_publisher.py**

Listens to cmd_vel and publishes a "fake" rpm value to be used for odometry.
As this is innacurate and may diverge in time, the robot should be provided with good
orientation using a MPU6050 for example and we are also using visual odometry from the
kinect sensor

**ros_rpi3_arduino.py**

Listens to /cmd_vel topic and sends the linear (x) and angular (z) 
velocities commands to an Arduino connected trough i2c at the specified slave adress.
