#!/usr/bin/env python
import datetime
import time
from planPython import *
import utils


init_globals()
robot, scene, group = get_robot_scene_group()

utils.pause_gazebo()
time.sleep(10)

before = utils.get_move_group_ram_usage()

utils.unpause_gazebo()
time.sleep(5)
execute_saved_plans("scan")
time.sleep(30)
utils.pause_gazebo()
time.sleep(30)
after = utils.get_move_group_ram_usage()
print(after-before)

utils.save_map(datetime.datetime.now().strftime("%m-%d_%H%M")+"_3","vxblx")