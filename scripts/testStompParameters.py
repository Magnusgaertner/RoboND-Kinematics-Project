#!/usr/bin/env python
import datetime
import random

from planPython import *
from utils import *
from rosgraph_msgs.msg import Log
import re
import rospy

results = []
first_valid_iteration = -1
num_iterations = -1
costs = []
overall_time = -1

def reset_run_vars():
  global first_valid_iteration
  global num_iterations
  global costs
  global overall_time
  first_valid_iteration = -1
  num_iterations = -1
  costs = []
  overall_time = -1


def ros_info(info):
  global first_valid_iteration
  global num_iterations
  global costs
  global overall_time

  if info.line == 290:
    nums = re.findall('[\d.]+', info.msg)
    print(nums) # found valid solution after n seconds
    overall_time = nums[0]
  elif info.line == 253:
    nums = re.findall('[\d.]+', info.msg)
    # iteration complete
    num_iterations = nums[0]
    costs.append(nums[1])
  elif info.line == 259:
    nums = re.findall('[\d.]+', info.msg)
    print(nums) # valid iteration complete
    if first_valid_iteration == -1:
      first_valid_iteration = num_iterations
# roscore = subprocess.Popen(["roscore"])


def evaluate(std_devs, params):
  try:
    set_stomp_std_dev(std_devs)
    set_stomp_optimization_params(params)
    set_voxelsize(10)
    FNULL = open(os.devnull, 'w')
    move_group_launch = subprocess.Popen(["roslaunch", "kr210_claw_moveit", "demo.launch"], stdout=FNULL, stderr=subprocess.STDOUT)
    time.sleep(10)
    load_map(10)
    time.sleep(10)
    init_globals()
    rospy.Subscriber("/rosout", Log, ros_info)
    robot, scene, group = get_robot_scene_group()
    get_plan(get_stored_jointstates((1, 1)), get_stored_jointstates((2, 3)), max_planning_time=600)
    time.sleep(10)
  except Exception as e:
    print(e)
  finally:
    move_group_launch.terminate()
  print(results)
  return {'std_devs': std_devs, 'params':params.copy(),  'first_valid_iteration': first_valid_iteration, 'time': overall_time, 'cost': costs}

params = {'num_timesteps': 5, 'num_iterations':50, 'num_iterations_after_valid':3, 'num_rollouts': 20, 'max_rollouts': 23}


#for dev1 in [0.15, 0.2, 0.4]:
#  for dev2 in [0.15, 0.2, 0.4]:
#    for dev3 in [0.15, 0.2, 0.4]:
#      for dev4 in [0.1, 0.2, 0.4]:
#        for num_rollouts in [20, 60, 100]:
#          for num_timesteps in [5]:
#
#            params['num_rollouts'] = num_rollouts
#            params['max_rollouts'] = num_rollouts+5
#            params['num_timesteps'] = num_timesteps
#            std_devs = [dev1, dev2, dev3, dev4, 0.05, 0.05]
#            print(str(params)+str(std_devs))
#            # output = evaluate([dev1, dev2, dev3, dev4, 0.05, 0.05], params)
#            # results.append(output)
#            reset_run_vars()

start_time = time.time()
for i in range(0,10):
  params['num_rollouts'] = 50
  params['max_rollouts'] = 120
  params['num_timesteps'] = 10
  std_devs = [random.randint(0,100)*0.002+0.2,
              1.0+random.randint(-100,100)*0.002,
              0.8+random.randint(-100,100)*0.002,
              0.4+random.randint(-100,100)*0.0005,
              0.2+random.randint(-100,100)*0.001,
              0.2+random.randint(0,100)*0.0005]
  #print(str(params)+str(std_devs))
  output = evaluate(std_devs, params)
  results.append(output)
  print(output)
  reset_run_vars()




# print(chr(27) + "[2J")
#print("\033[H\033[J")
#print(results)

output_name = "ergebnisse1711.yaml" # + datetime.datetime.now().strftime("%m-%d_%H%M") + ".yaml"
path = open(output_name, "w+")
yaml.dump(results, path)

# roscore.terminate()