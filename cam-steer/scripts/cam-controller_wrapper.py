#!/usr/bin/env python

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

import numpy as np
import rospy


from camera_algo import *
from cam_controller import Controller
from sensor_fusion import SensorFusion

def cam_callback(data):
    # Get camera yaw estimate
    print("cam angle : ", np.rad2deg(float(data.data)))
    print("odom angle", np.rad2deg(fusion_filter.odom))

    # Full kalman step according to the motion model and sensor readings
    fusion_filter.take_step(float(data.data))

    # Controller update
    ctrl =  controller.update(fusion_filter.x[0][0])

    # Publish the result
    print("Filtered angle:", np.rad2deg(fusion_filter.x[0][0]))
    twist = Twist()
    twist.angular.z = 0
    twist.linear.x = 0

    pub.publish(twist)
    print("control signal", ctrl)

def odom_callback(data):
    # Save odometer reading
    fusion_filter.update_odom(data.pose.pose.position.z)

def node():
    rospy.Subscriber('cam_yaw', Float32, cam_callback)
    rospy.Subscriber('odom',Odometry, odom_callback)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        # ---------- Initialization ------------ #

        # Discrete time step
        Ts = 0.1

        # Initialize PID controller
        P = 0.5
        I = 0
        D = 0

        controller = Controller(P,I,D,Ts)

        # Initialize sensor fusion algorithm
        x_0 = np.array([[0],[0]])
        P_0 = np.array([[1,0],[0,1]])
        Q = np.array([[1, 0],[0, 1]])
        R_c = 5
        R_0 = 5

        fusion_filter = SensorFusion(x_0,P_0,Q,R_c,R_0,Ts)

        # Initialize ros publisher
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Initialize controller node
        rospy.init_node('cam_ctrl')

        # Save last odometer reading
        last_odom = 0;

        # Start node
        node()

    except rospy.ROSInterruptException:
        pass
