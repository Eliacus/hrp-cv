#!/usr/bin/env python

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

import numpy as np
import rospy
import time
import csv

from camera_algo import *
from cam_controller import Controller
from sensor_fusion import SensorFusion

def cam_callback(data):

    # Full kalman step according to the motion model and sensor readings
    fusion_filter.take_step(float(data.data))


    # Printing raw measurements
    print("---------------------------------")
    print("Raw camera angle: ", float(data.data))
    print("Raw odom angle: ", fusion_filter.odom)
    # Printing  HP filtered measurements
    print("HP-filtered camera angle: ", np.rad2deg(fusion_filter.old_yc))
    print("HP-filtered odom angle: ", np.rad2deg(fusion_filter.old_yo))

    # Controller update
    ctrl =  controller.update(fusion_filter.x[0][0])

    # Publish the result
    print("Kalman filtered angle:", np.rad2deg(fusion_filter.x[0][0]))
    twist = Twist()
    twist.angular.z = 0
    twist.linear.x = 0.2

    pub.publish(twist)
    print("control signal", ctrl)


    # Writing to csv file
    with open('log.csv', 'a') as file:
        writer = csv.writer(file)
        writer.writerow([time.time()-start, float(data.data), fusion_filter.odom,
        fusion_filter.old_yc, fusion_filter.old_yo, fusion_filter.x[0][0]])

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
        P = 1
        I = 0.02
        D = 0

        controller = Controller(P,I,D,Ts)

        # ------------ Initialize sensor fusion algorithm -----------------
        # Initialize prior
        x_0 = np.array([[0],[0]])
        P_0 = np.array([[1,0],[0,1]])

        # Initialize Process and Measurement noise
        Q = np.array([[0.1, 0],[0, 0.1]])
        R_c = 10
        R_0 = 10

        # Initialize High-Pass filter alphas
        alpha_c = 0.9
        alpha_o = 0.9

        # Create the fusion filter
        fusion_filter = SensorFusion(x_0,P_0,Q,R_c,R_0,Ts)

        # Initialize ros publisher
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Initialize controller node
        rospy.init_node('cam_ctrl')

        # Save last odometer reading
        last_odom = 0;

        # Start time
        with open('log.csv', 'w+') as file:
            writer = csv.writer(file)
            writer.writerow(["time","Raw cam yaw", "Raw odom yaw",
             "HP-filtered cam yaw",'HP-filtered odom yaw',
             'Kalman filtered fused yaw'])

        start = time.time()

        # Start node
        node()

    except rospy.ROSInterruptException:
        pass
