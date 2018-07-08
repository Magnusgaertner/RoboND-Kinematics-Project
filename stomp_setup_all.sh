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
roslaunch kuka_arm spawn_target.launch;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 5;\
roslaunch kuka_arm inverse_kinematics.launch;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 5;\
roslaunch kuka_arm self_filter.launch;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 5;\
roslaunch voxblox_ros kr210.launch;\
exec /bin/bash -i'"