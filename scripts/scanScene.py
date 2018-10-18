#!/usr/bin/env python

from planPython import *

init_globals()
robot, scene, group = get_robot_scene_group()
execute_saved_plans("sweep")
