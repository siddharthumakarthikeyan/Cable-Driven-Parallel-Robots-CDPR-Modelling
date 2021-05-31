import roslib
# roslib.load_manifest('pr2_gazebo')
import rospy
import time
from gazebo_msgs.msg import *
from gazebo.srv import *

if __name__ == '__main__':
  rospy.init_node('gripper_')
  rospy.wait_for_service('/gazebo/apply_joint_effort')
  time.sleep(10)
  apply_joint_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
  joint_left = "grip_left_slide"
  effort_left = 100
  joint_right = "grip_right_slide"
  effort_right = -100
  start_time = rospy.Duration.from_sec(0)
  duration = rospy.Duration.from_sec(2)
  for i in range(10):
    try:
      resp1 = apply_joint_effort(joint_left, effort_left, start_time, duration)
      resp2 = apply_joint_effort(joint_right, effort_right, start_time, duration)
    except rospy.ServiceException, e:
      print "Service did not process request: %s"%str(e)
    time.sleep(3)