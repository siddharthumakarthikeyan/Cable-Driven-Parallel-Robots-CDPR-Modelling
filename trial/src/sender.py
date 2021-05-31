#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import LinkState

def talker():
    pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=10)
    ppp = LinkState()
    rospy.init_node('talker', anonymous=True)
    
    rate = rospy.Rate(100) # 10hz
    i = 1
    while not rospy.is_shutdown():
        ppp.link_name = "platform"
        ppp.pose.position.x = 0.1
        ppp.pose.position.y = 0.1
        ppp.pose.position.z = 1
        ppp.pose.orientation.x = 0
        ppp.pose.orientation.y = 0
        ppp.pose.orientation.z = 0
        ppp.pose.orientation.w = 0
        i = i+1
        rospy.loginfo(ppp)
        pub.publish(ppp)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
