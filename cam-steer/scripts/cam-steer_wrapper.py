#!/usr/bin/env python

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from camera_algo import *

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
    np_arr = np.fromstring(data.data, np.uint8)
    im = cv.imdecode(np_arr, 0)

    # Use im to calculate yaw
    app.run(im)
    # rospy.loginfo(rospy.get_caller_id() + "The Compressed Image Data is:    %s", np.shape(im))

def node():

    pub = rospy.Publisher('cam_yaw', Float32, queue_size=1)
    rospy.init_node('cam_yaw')
    rospy.Subscriber('raspicam_node/image/compressed', CompressedImage, callback)
    rate = rospy.Rate(10) # 10hz
    app = App()

    while not rospy.is_shutdown():
        pub.publish(app.euler_angles[1])    # Change this to yaw variable
        rate.sleep()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass



#def init(self):







    # Setup publishers
    #self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # Setup subscriber
    # rospySubscriber('raspicam_node/image/compressed', CompressedImage)  # DOUBLE CHECK THIS
