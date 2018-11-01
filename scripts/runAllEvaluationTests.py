#!/usr/bin/env python
import datetime

from utils import *
from planPython import *

def getTime():
  return datetime.datetime.now().strftime("%m-%d_%H%M")

def doOneRunWithimulation(init_function, function, name):
  try:
    in_ = init_function()

    FNULL = open(os.devnull, 'w')
    roscore = subprocess.Popen(["roscore"], stdout=FNULL, stderr=subprocess.STDOUT)
    time.sleep(2)
    cafe = subprocess.Popen(["roslaunch", "kuka_arm", "cafe.launch"], stdout=FNULL, stderr=subprocess.STDOUT)
    time.sleep(2)

    inverse_kinematics = subprocess.Popen(["roslaunch", "kuka_arm", "inverse_kinematics.launch"], stdout=FNULL, stderr=subprocess.STDOUT)
    time.sleep(2)
    swri = subprocess.Popen(["roslaunch", "swri_profiler", "profiler.launch"], stdout=FNULL,stderr=subprocess.STDOUT)
    time.sleep(10)

    rosbag = subprocess.Popen(["rosbag",
                                           "record",
                                           "/profiler/data",
                                           "/profiler/index",
                                           "/rosout_agg", "-O", getTime()+name],
                                          stdout=FNULL, stderr=subprocess.STDOUT)
    time.sleep(10)
    init_globals()
    time.sleep(10)
    robot, scene, group = get_robot_scene_group()
    function(robot, scene, group, in_)

    # rospy.Subscriber("/rosout", Log, ros_info)

    # get_plan(get_stored_jointstates((1, 1)), get_stored_jointstates((2, 3)), max_planning_time=600)
    time.sleep(20)

  except Exception as e:
    print(e)
  finally:
    try:
      rosbag.kill()
      time.sleep(10)
    finally:
      None
    try:
      rosbag.terminate()
    finally:
      None
    try:
      inverse_kinematics.terminate()
    finally:
      None
    try:
      cafe.terminate()
    finally:
      None
    try:
      roscore.terminate()
    finally:
      None


def scane_scene_init_(esdf, size, name):
  if esdf:
    set_esdf_enabled()
    set_voxelsize(size)
  else:
    set_octomap_enabled()
    set_octomap_resolution(size)
  return name




def scan_scene_function(robot, scene, group, in_ ):
  pause_gazebo()
  time.sleep(10)
  before = get_move_group_ram_usage()
  unpause_gazebo()
  time.sleep(5)
  execute_saved_plans("scan")
  time.sleep(30)
  pause_gazebo()
  time.sleep(30)
  after = get_move_group_ram_usage()
  print(after - before)
  save_map(getTime()+ in_["name"])


doOneRunWithimulation(lambda :scane_scene_init_(True, 0.1, "voxblox_10"), lambda robot, scene, group, in_:scan_scene_function(robot, scene, group, in_), "voxblox_10")
