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
roslaunch kuka_arm cafe.launch;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 3;\
roslaunch kuka_arm inverse_kinematics.launch;\
exec /bin/bash -i'" 
# env CPUPROFILE=/tmp/cafe.prof LD_PRELOAD=/usr/lib/libprofiler.so.0 
