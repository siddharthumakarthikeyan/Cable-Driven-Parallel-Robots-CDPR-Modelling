#!/usr/bin/python

import pylab as pl
import roslib, rospy, rospkg
from std_msgs.msg import Float32MultiArray


if __name__ == '__main__':
    '''
    Begin of main code
    '''    
    
    rospy.init_node('dummy_HAB')
    
    
    msg = Float32MultiArray()
    msg.data = [0] * 32
    pub = rospy.Publisher('barycenter', Float32MultiArray, queue_size=1)
    
    while not rospy.is_shutdown():
        
        for i in xrange(8):
            # random H
            msg.data[4*i] = pl.randn()
            msg.data[4*i+1] = pl.randn()
                    
            # random A
            msg.data[4*i+2] = -1000 - 1000*pl.rand()   
        
            # random B higher than A
            msg.data[4*i+3] = msg.data[4*i+2] + 2000 + 1000*pl.rand()
            
        pub.publish(msg)       
        
        rospy.sleep(1)
