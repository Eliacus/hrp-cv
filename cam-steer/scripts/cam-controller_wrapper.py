#!/usr/bin/env python

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

from camera_algo import *
import numpy as np
import rospy
from cam_controller import Controller


def callback(data):
    vel = controller.update(float(data.data))
    print("velocity command: ", vel)
    print(" Angle : ", np.rad2deg(float(data.data)))
    twist = Twist()
    twist.angular.z = vel
    twist.linear.x = 0.1

    pub.publish(twist)

def node():
    rospy.init_node('cam_ctrl')
    rospy.Subscriber('cam_yaw', Float32, callback)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        P = 0.5
        I = 0
        D = 0
        Ts = 0.1
        controller = Controller(P,I,D,Ts)
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        node()
    except rospy.ROSInterruptException:
        pass
