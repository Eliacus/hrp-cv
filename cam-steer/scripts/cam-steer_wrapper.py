 #!/usr/bin/env python

from geometry.msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

import rospy


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
    rospy.loginfo(rospy.get_caller_id() + "The cmd_vel is:  %s", data.data)
def node():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('cam_steer')
    rospy.subscriber('cmd_vel', Twist, callback)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = 1
        tist.angular.z = 1
        pub.publish(twist)
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
