#!/usr/bin/env python

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

from std_msgs.msg import Float32
from camera_algo import *
from cv_bridge import CvBridge

import rospy
import numpy as np
import cv2 as cv

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
    # Convert image buffer
    #np_arr = np.fromstring(data.data, np.uint8)
    #im = cv.imdecode(np_arr, 0)
    im = bridge.imgmsg_to_cv2(data, "bgr8")
    rospy.loginfo(rospy.get_caller_id() + "The converted Image Data is:    %s", im)
    # Use im to calculate yaw
    tracker.run(im)
def node():
    pub = rospy.Publisher('cam_yaw', Float32, queue_size=1)
    rospy.init_node('cam_yaw')
    rospy.Subscriber('raspicam_node/image', Image, callback)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pub.publish(tracker.euler_angles[1])    # Change this to yaw variable
        rate.sleep()


if __name__ == '__main__':
    try:
        bridge = CvBridge()
        tracker = FeatureTracker()
        node()
    except rospy.ROSInterruptException:
        pass



#def init(self):







    # Setup publishers
    #self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # Setup subscriber
    # rospySubscriber('raspicam_node/image/compressed', CompressedImage)  # DOUBLE CHECK THIS
