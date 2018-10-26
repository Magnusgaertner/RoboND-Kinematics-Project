#!/usr/bin/env python
import rospy
import moveit_msgs.msg


def print_current_jointstate_for_srdf(goal):
  joint = [constraint.position for constraint in goal.goal.request.goal_constraints[0].joint_constraints]
  print('''
  <group_state name="unique_name" group="arm_group">
    <joint name="joint_1" value="''' + str(joint[0]) + '''" />
    <joint name="joint_2" value="''' + str(joint[1]) + '''" />
    <joint name="joint_3" value="''' + str(joint[2]) + '''" />
    <joint name="joint_4" value="''' + str(joint[3]) + '''" />
    <joint name="joint_5" value="''' + str(joint[4]) + '''" />
    <joint name="joint_6" value="''' + str(joint[5]) + '''" />
  </group_state>''')


def listener():
  # In ROS, nodes are uniquely named. If two nodes with the same
  # node are launched, the previous one is kicked off. The
  # anonymous=True flag means that rospy will choose a unique
  # name for our 'listener' node so that multiple listeners can
  # run simultaneously.
  rospy.init_node('goalPositionFormater', anonymous=True)
  rospy.Subscriber("/move_group/goal", moveit_msgs.msg.MoveGroupActionGoal, print_current_jointstate_for_srdf)

  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()


if __name__ == '__main__':
  listener()