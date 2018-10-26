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

import yaml


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


def set_octomap_resolution(size):
  kill_movegroup()
  if size > 1:
    set_param("octomap_resolution", float(size)/100, '/home/magnus/kr210_ws/src/RoboND-Kinematics-Project/kr210_claw_moveit/launch/sensor_manager.launch.xml')
  else:
    set_param("octomap_resolution", size, '/home/magnus/kr210_ws/src/RoboND-Kinematics-Project/kr210_claw_moveit/launch/sensor_manager.launch.xml')


def set_voxelsize(size):
  kill_movegroup()
  if size > 1:
    set_move_group_param("voxblox/tsdf_voxel_size", float(size)/100)
  else:
    set_move_group_param("voxblox/tsdf_voxel_size", size)


def set_param(name, value, path):
  config = etree.parse(path)
  entry = config.find(".//param[@name='" + str(name) + "']")
  entry.attrib['value'] = str(value)
  config.write(path)
  print(str(name) + ": " + str(value))


def set_move_group_param(name, value):
  kill_movegroup()
  path = '/home/magnus/kr210_ws/src/RoboND-Kinematics-Project/' \
         'kr210_claw_moveit/launch/move_group.launch'
  set_param(name, value, path)


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


def set_esdf_enabled():
  kill_gazebo()
  set_move_group_param("load_octomap_monitor", "false")
  with open("/home/magnus/kr210_ws/src/RoboND-Kinematics-Project/kr210_claw_moveit/config/scan.yaml", "r") as file_open:
    data = yaml.load(file_open)

  data['sensors'][0]['sensor_plugin'] = "occupancy_map_monitor/PointCloudEsdfUpdater"
  with open("/home/magnus/kr210_ws/src/RoboND-Kinematics-Project/kr210_claw_moveit/config/scan.yaml",
            "w") as  file_open:
    yaml.dump(data, file_open)


def set_octomap_enabled():
  kill_gazebo()
  set_move_group_param("load_octomap_monitor", "true")
  with open("/home/magnus/kr210_ws/src/RoboND-Kinematics-Project/kr210_claw_moveit/config/scan.yaml", "r") as file_open:
    data = yaml.load(file_open)

  data['sensors'][0]['sensor_plugin'] = "occupancy_map_monitor/PointCloudOctomapUpdater"
  with open("/home/magnus/kr210_ws/src/RoboND-Kinematics-Project/kr210_claw_moveit/config/scan.yaml",
            "w") as  file_open:
    yaml.dump(data, file_open)


def load_map(path, type='vxblx'):
  rospy.wait_for_service('/move_group/load_map', timeout=30)
  rospy.ServiceProxy('/move_group/load_map', moveit_msgs.srv.LoadMap)('maps_for_planning/' + type + "_" + str(path) + 'cm.' + type)


def save_map(path,type='vxblx'):
  rospy.wait_for_service('/move_group/save_map',timeout=30)
  rospy.ServiceProxy('/move_group/save_map', moveit_msgs.srv.SaveMap)('maps_for_planning/' + type + "_" + str(path) + 'cm.' + type)


def get_out():
  global move_group_launch
  return move_group_launch.std_out


def clear_map():
  try:
    clear = rospy.ServiceProxy('/move_group/voxblox/clear_map', srv.Empty)
    clear()
  except rospy.ServiceException as e:
    print(e)

def pause_gazebo():
  try:
    pause = rospy.ServiceProxy('/gazebo/pause_physics', srv.Empty)
    pause()
  except rospy.ServiceException as e:
    print(e)


def unpause_gazebo():
  try:
    unpause = rospy.ServiceProxy('/gazebo/unpause_physics', srv.Empty)
    unpause()
  except rospy.ServiceException as e:
    print(e)


def get_move_group_ram_usage():
  pid = int(subprocess.check_output(["pidof", "-s", "move_group"]))
  process = psutil.Process(pid)
  return process.memory_info()[0]


