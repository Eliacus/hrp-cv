#!/usr/bin/env python

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from camera_algo import *

import rospy
import numpy as np
import cv2 as cv
import time

# CompressedImage message
#std_msgs/Header header
#  uint32 seq
#  time stamp
#  string frame_id
# string format
# uint8[] data


# Create subscriber


# Create publisher

def callback(data):
    start_time = time.time()

    # Convert image buffer
    np_arr = np.fromstring(data.data, np.uint8)
    im = cv.imdecode(np_arr, 0)

    # Use im to calculate yaw
    tracker.run(im)
    pub.publish(tracker.euler_angles[1])    # Change this to yaw variable

    rospy.loginfo(rospy.get_caller_id() + "Yaw:    %s", tracker.euler_angles[1])
def node():
    rospy.init_node('yaw_estimator')
    rospy.Subscriber('raspicam_node/image/compressed', CompressedImage, callback)
    rate = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('cam_yaw', Float32, queue_size=1)
        tracker = FeatureTracker()
        node()
    except rospy.ROSInterruptException:
        pass










    # Setup publishers
    #self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # Setup subscriber
    # rospySubscriber('raspicam_node/image/compressed', CompressedImage)  # DOUBLE CHECK THIS
