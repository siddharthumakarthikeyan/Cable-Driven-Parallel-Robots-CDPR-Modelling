import roslib
# roslib.load_manifest('pr2_gazebo')
import rospy
import time
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *


if __name__ == '__main__':
  # rospy.set_param("/Tra/position/B",[0,0,1])
  rospy.init_node('gripper_')
  rospy.wait_for_service('/gazebo/apply_joint_effort')
  print("waiting...")
  time.sleep(2)
  print("Wait is over")
  apply_joint_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
  joint_left = "grip_left_slide"
  effort_left = 50
  joint_right = "grip_right_slide"
  effort_right = 50
  start_time = rospy.Duration.from_sec(0)
  duration = rospy.Duration.from_sec(100)
  for i in range(10):
    try:
      if(i%2==0):
        print("Applying positive forces")
        resp1 = apply_joint_effort(joint_left, effort_left, start_time, duration)
        resp2 = apply_joint_effort(joint_right, effort_right, start_time, duration)
      else:
        print("Applying negative forces")
        resp1 = apply_joint_effort(joint_left, -effort_left, start_time, duration)
        resp2 = apply_joint_effort(joint_right, -effort_right, start_time, duration)
    except rospy.ServiceException, e:
      print "Service did not process request: %s"%str(e)
    time.sleep(100)
