#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import yaml
import os
## END_SUB_TUTORIAL

from std_msgs.msg import String

robot = 0
scene = 0
group = 0


def test_load_save_plan(plan, name):
  global group
  file_path = os.path.join(os.path.expanduser('~'), 'saved_trajectories', str(name) + '.yaml')
  with open(file_path, 'w') as file_save:
    yaml.dump(plan, file_save, default_flow_style=True)

  with open(file_path, 'r') as file_open:
    loaded_plan = yaml.load(file_open)

  group.execute(loaded_plan)


def get_sweep_jointstate(level, angle):
  levels = [[1.566188518849043, 0.028112826490181726, -0.7158366157503401, 0.05348143637055464, 0.6971889170190857, -0.051167993750760365],
            [1.5553869265248506, -0.3815680363554752, 0.1786823083911969, 0.2115026962534538, 0.21633464251561565, -0.21695941887818584],
            [1.5510595921175467, -0.35326833544152686, 0.6844425743756631, 0.19001021912683286, 0.2003505781786184, -0.2196496755326847],
            [1.5217529156156142, -0.5544466230138799, 1.1179864510344133, -2.461132637666335, -0.0915636406473192, 2.408606578045072],
            [1.5335834242422266, 0.3401343068477073, 0.9245577738873176, -3.2110975349675073, -1.8279829922846291, 3.1306786676693084]]
  value = levels[level]
  value[0] = angle
  return value


def get_stored_jointstates(name):
  dic = {(1, 1): [0.405590817250955, 0.6258868345394196, -1.1897619160598405, -0.6767938075084625, 0.6815055784257186, 0.5578166100184557],
         (1, 2): [-0.007116423280140886, 0.38015576796362593, -0.8334587100406742, 0.016293199360760697, 0.45335701429008735, -0.014653366012149624],
         (1, 3): [-0.3893949244128473, 0.5995486647522825, -1.1488911775726698, 0.6661193422157059, 0.6615935778844636, -0.555097049371991],
         (2, 1): [0.4181910104410044, 0.45261003723788706, -0.40243619941061115, -1.6831761885550423, 0.4209943622500738, 1.6938191807655303],
         (2, 2): [-0.01955187315304623, 0.2953043150800145, -0.13641297363912105, -3.2646785013945134, 0.16008026201141282, 3.2631145459913347],
         (2, 3): [-0.39271544667235997, 0.6527434366702103, -0.11037253155468367, -3.8178938013483856, 0.6578963139611251, 3.7073808384484264],
         (3, 1): [0.6732631480717193, 1.2349721934571025, -0.70363518808994, -2.136770342897361, 0.8311627088674939, 2.3266307204744017],
         (3, 2): [0.8132184238355311, 0.6254913853427008, -1.1605673475259293, -2.9748216908475973, 0.9164551702481022, 2.234643232502442],
         (3, 3): [-0.1358717272130872, -0.08424582747511959, -0.28198130170369134, -0.028766405008148886, -1.0240832405108982, 0.14862440750870906]}
  return dic[name]


def init_globals():
  global robot
  global scene
  global group
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("arm_group")
  group.clear_pose_targets()


def follow_waypoints(waypoints, name=None):
  global group
  i = 0
  for waypoint in waypoints:
    plan = get_plan(waypoint)
    #if name is not None:
    test_load_save_plan(plan, name + "_"+str(i))
    # else:
    #  group.execute(plan)
    i += 1


def execute_saved_plans(name):
  global group
  i = 0
  try:
    while True:
      file_path = os.path.join(os.path.expanduser('~'), 'saved_trajectories', str(name) + "_" + str(i) + '.yaml')
      if not os.path.exists(file_path):
        return
      with open(file_path, 'r') as file_open:
        loaded_plan = yaml.load(file_open)
      group.execute(loaded_plan)
      i += 1
  finally:
    None


def get_plan(end, start=None, max_planning_time=100):
  global group
  if not start:
    start = group.get_current_joint_values()
  joint_state = JointState()
  joint_state.header = Header()
  joint_state.header.stamp = rospy.Time.now()
  joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
  joint_state.position = start
  moveit_robot_state = RobotState()
  moveit_robot_state.joint_state = joint_state
  group.set_start_state(moveit_robot_state)
  group.set_joint_value_target(end)
  group.set_planning_time(max_planning_time)
  return group.plan()


def get_robot_scene_group():
  global robot
  global scene
  global group
  return robot, scene, group


robot, scene, group = get_robot_scene_group()


def move_group_python_interface_tutorial(max_planning_time=100):
  global robot
  global scene
  global group
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("arm_group")
  group.clear_pose_targets()

  joint_state = JointState()
  joint_state.header = Header()
  joint_state.header.stamp = rospy.Time.now()
  joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
  joint_state.position = [-0.399785047899, 0.547865794198, -0.53408570763, -1.53819544195, -0.400005105505,
                          -4.7477817452]
  moveit_robot_state = RobotState()
  moveit_robot_state.joint_state = joint_state
  group.set_start_state(moveit_robot_state)
  group.set_joint_value_target(
    [0.41523042671, 0.740434785089, -1.26737863948, -3.86136759133, -0.658526198241, -2.53533941067])
  group.set_planning_time(max_planning_time)
  plan2 = group.plan()
  group.set_pose_targets()
  moveit_commander.roscpp_shutdown()
  # print("============ STOPPING")


if __name__ == '__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
