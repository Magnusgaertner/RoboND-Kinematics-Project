#!/usr/bin/env python
import os
import subprocess
import sys
import pathlib
import moveit_msgs.srv
import time

from lxml import etree
import rospy
from ruamel.yaml import YAML
from std_srvs import srv
from time import sleep
import signal
import sys
import psutil
import signal
import time




terminal_session = None


def kill_movegroup():
  global terminal_session
  try:
    terminal_session.terminate
    subprocess.Popen(["killall", "move_group"])
  finally:
    return


def kill_gazebo():
  try:
    global terminal_session
    terminal_session.terminate #subprocess.Popen(["killall", "gzserver", "gzclient"])
  finally:
   return


def start_moveit():
  global terminal_session
  terminal_session = subprocess.Popen(["./start_moveit.sh"])

def start_moveit_with_handle():
  global roscore
  global move_group_launch

  roscore = subprocess.Popen(
    ["gnome-terminal", "-e",  "/bin/bash -c  roscore; exec /bin/bash -i"])
  # roscore = subprocess.Popen(["roscore"], shell=True)
  print("started roscore")
  #time.sleep(2)
  # subprocess.check_output(["rosparam", "load", "stomp_std_devs.yaml"])
  # print("loaded stomp std_dev params")
  time.sleep(3)
  move_group_launch = subprocess.Popen(["roslaunch", "kr210_claw_moveit", "demo.launch"], shell=True)
  time.sleep(10)




def set_voxelsize(size):
  kill_movegroup()
  if size > 1:
    set_move_group_param("voxblox/tsdf_voxel_size", float(size)/100)
  else:
    set_move_group_param("voxblox/tsdf_voxel_size", size)

def set_move_group_param(name, value):
  kill_movegroup()
  path = '/home/magnus/kr210_ws/src/RoboND-Kinematics-Project/' \
         'kr210_claw_moveit/launch/move_group.launch'
  config = etree.parse(path)
  entry = config.find(".//param[@name='" + str(name) + "']")
  entry.attrib['value'] = str(value)
  config.write(path)
  print(str(name) + ": " + str(value))


def set_move_group_arg(name, value):
  kill_movegroup()
  path = '/home/magnus/kr210_ws/src/RoboND-Kinematics-Project/' \
         'kr210_claw_moveit/launch/move_group.launch'
  config = etree.parse(path)
  entry = config.find(".//arg[@name='" + str(name) + "']")
  entry.attrib['default'] = str(value)
  config.write(path)
  print(str(name) + ": " + str(value))


# must be called before start moveit
def set_stomp_std_dev(std_dev):
    kill_movegroup()
    yaml = YAML()
    path = pathlib.Path('/home/magnus/kr210_ws/src/RoboND-Kinematics-Project/'
                        'kr210_claw_moveit/config/stomp_planning.yaml')
    params = yaml.load(path)

    params['stomp/arm_group']['task']['noise_generator'][0]['stddev'] = std_dev
    yaml.dump(params, path)


def set_stomp_optimization_params(params):
  kill_movegroup()
  yaml = YAML()
  path = pathlib.Path('/home/magnus/kr210_ws/src/RoboND-Kinematics-Project/'
                      'kr210_claw_moveit/config/stomp_planning.yaml')
  stomp = yaml.load(path)
  for key, value in params.iteritems():
    stomp['stomp/arm_group']['optimization'][key] = value

  yaml.dump(stomp, path)

# usage: set_simulation_cameras_enabled([0, 1, 1, 0])
def set_simulation_cameras_enabled(enabled):
  kill_gazebo()
  valid_topic = "/depth/points"
  invalid_topic = "/trash_"
  path = '/home/magnus/kr210_ws/src/RoboND-Kinematics-Project/' \
         'kuka_arm/urdf/kr210.gazebo.xacro'
  gazebo_config = etree.parse(path)
  for camera_number in range(0, 4):
    sensor = gazebo_config.find(".//sensor[@type='depth'][@name='camera" + str(camera_number+1)+"']")
    topic = sensor.find(".//pointCloudTopicName")
    if enabled[camera_number]:
      topic.text = valid_topic
    else:
      topic.text = invalid_topic
  gazebo_config.write(path)


def load_map(path):
  rospy.wait_for_service('/move_group/load_map', timeout=30)
  rospy.ServiceProxy('/move_group/load_map', moveit_msgs.srv.LoadMap)('voxblox_map_voxblox_' + str(path) + 'cm.vxblx')

def save_map(path):
  rospy.wait_for_service('/move_group/save_map',timeout=30)
  rospy.ServiceProxy('/move_group/save_map', moveit_msgs.srv.SaveMap)('voxblox_map_voxblox_' + str(path) + 'cm.vxblx')



def get_out():
  global move_group_launch
  return move_group_launch.std_out


def clear_map():
  try:
    clear = rospy.ServiceProxy('/move_group/voxblox/clear_map', srv.Empty)
    clear()
  except rospy.ServiceException, e:
    print(e)

def pause_gazebo():
  try:
    pause = rospy.ServiceProxy('/gazebo/pause_physics "{}"', srv.Empty)
    pause()
  except rospy.ServiceException, e:
    print(e)



def unpause_gazebo():
  try:
    unpause = rospy.ServiceProxy('/gazebo/unpause_physics "{}"', srv.Empty)
    unpause()
  except rospy.ServiceException, e:
    print(e)


def get_move_group_ram_usage():
  pid = int(subprocess.check_output(["pidof", "-s", "move_group"]))
  process = psutil.Process(pid)
  return process.memory_info()[0]


