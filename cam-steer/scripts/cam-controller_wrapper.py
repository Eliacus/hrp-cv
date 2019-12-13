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
    # Get
    cam_yaw = controller.update(float(data.data))
    print(" Angle : ", np.rad2deg(float(data.data)))

    fusion_filter.take_step(cam_yaw,last_odom)

    print("Filtered angle:", fusion_filter.x[0])
    twist = Twist()
    twist.angular.z = fusion_filter.x[0]

    twist.linear.x = 0.1

    pub.publish(twist)

def odom_callback(data):

    last_odom = data.pose.pose.position.z
    print(last_odom)


def node():
    rospy.Subscriber('cam_yaw', Float32, cam_callback)
    rospy.Subscriber('odom',Odometry, odom_callback)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        rate.sleep()





if __name__ == '__main__':
    try:
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
        R_c = 1
        R_0 = 10

        fusion_filter = SensorFusion(x_0,P_0,Q,R_c,R_0,Ts)

        # Initialize ros publisher
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Initialize controller node
        rospy.init_node('cam_ctrl')

        # Save last odometer reading
        last_odom = 0;
        
        node()
    except rospy.ROSInterruptException:
        pass
