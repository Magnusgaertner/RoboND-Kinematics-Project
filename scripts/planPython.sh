#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
## END_SUB_TUTORIAL

from std_msgs.msg import String

def move_group_python_interface_tutorial():
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("arm_group")
  group.clear_pose_targets()
  ## Then, we will get the current set of joint values for the group
  group_variable_values = group.get_current_joint_values()
  print "============ Joint valuess: ", group_variable_values
	joint_state = JointState()
	joint_state.header = Header()
	joint_state.header.stamp = rospy.Time.now()
	saved_states = group.get_remembered_joint_values()
	middle_right = saved_states['middle_right'] 	
	print "============ Joint values: " middle_right
	joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
	joint_state.position = [-0.399785047899, 0.547865794198, -0.53408570763, -1.53819544195, -0.400005105505, -4.7477817452]
	moveit_robot_state = RobotState()
	moveit_robot_state.joint_state = joint_state
	group.set_start_state(moveit_robot_state)
  group.set_joint_value_target([0.41523042671, 0.740434785089, -1.26737863948, -3.86136759133,  -0.658526198241, -2.53533941067])
  plan2 = group.plan()
  moveit_commander.roscpp_shutdown()
  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass


