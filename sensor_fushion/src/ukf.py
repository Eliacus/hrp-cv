#!/usr/bin/env python

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from am_driver.msg import WheelEncoder
from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import String
import numpy as np

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
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    v = data.twist.twist.linear.x
    theta = data.pose.pose.position.z
    T = data.twist.twist.linear.y
    w = data.twist.twist.angular.z
    states_move = np.array([x,y,v,theta,w])
    rospy.loginfo(rospy.get_caller_id() + "The WheelEncoder data is:  %s", x)

def node():
    rospy.init_node('sensor_fushion')
    rospy.Subscriber('odom', Odometry, callback)
    rate = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #twist = Twist()
        #twist.linear.x = 1
        #twist.angular.z = 0.1
        #pub.publish(twist)
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
