#! /bin/bash
gnome-terminal --tab -e "/bin/bash -c \

'roscore;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 2;\
rosrun rqt_console rqt_console;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 1;\
roslaunch kuka_arm target_description.launch;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 3;\
roslaunch kuka_arm gazebo.launch;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 3;\
env CPUPROFILE=/tmp/wtf.prof LD_PRELOAD=/usr/lib/libprofiler.so.0 roslaunch kuka_arm cafe_replay.launch;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 2;\
roslaunch swri_profiler profiler.launch;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 2;\
roslaunch kuka_arm inverse_kinematics.launch;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 3;\
rosbag record /profiler/data /profiler/index /rosout_agg;\
exec /bin/bash -i'" 
# 
