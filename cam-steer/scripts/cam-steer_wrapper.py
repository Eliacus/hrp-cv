#!/usr/bin/env python

# Message imports
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# Import packages
from cv_bridge import CvBridge
import rospy
import numpy as np
import cv2 as cv
import time

from src.camera_algo import *

def callback(data):

    # Compressded Image
    #np_arr = np.fromstring(data.data, np.uint8)
    #im = cv.imdecode(np_arr, 0)

    # Regular image
    im = bridge.imgmsg_to_cv2(data, "mono8")

    #rospy.loginfo(rospy.get_caller_id() + "The converted Image Data is:    %s", np.size(im,1))
    #cv.imshow("image",im)
    #cv.waitKey(3)

    tracker.run(im)
def node():
    rospy.init_node('yaw_estimator')
    rospy.Subscriber('raspicam_node/image', Image, callback)
    #rospy.Subscriber('raspicam_node/image/compressed', CompressedImage, callback)
    rate = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('cam_yaw', Float32, queue_size=1)
        bridge = CvBridge()
        tracker = FeatureTracker()
        node()
    except rospy.ROSInterruptException:
        pass







    # Setup publishers
    #self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # Setup subscriber
    # rospySubscriber('raspicam_node/image/compressed', CompressedImage)  # DOUBLE CHECK THIS
