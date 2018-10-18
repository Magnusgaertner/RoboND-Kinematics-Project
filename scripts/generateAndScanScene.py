#!/usr/bin/env python

from planPython import *

init_globals()
robot, scene, group = get_robot_scene_group()
follow_waypoints([get_sweep_jointstate(0, 1.5),
                  get_sweep_jointstate(0, 0),
                  get_sweep_jointstate(0, -1.5),
                  get_sweep_jointstate(1, -1.5),
                  get_sweep_jointstate(1, 0),
                  get_sweep_jointstate(1, 1.5),
                  get_sweep_jointstate(2, 1.5),
                  get_sweep_jointstate(2, 0),
                  get_sweep_jointstate(2, -1.5),
                  get_sweep_jointstate(3, -1.5),
                  get_sweep_jointstate(3, 0),
                  get_sweep_jointstate(3, 1.5),
                  get_sweep_jointstate(4, 1.5),
                  get_sweep_jointstate(4, 0),
                  get_sweep_jointstate(4, -1.5)], "sweep")
