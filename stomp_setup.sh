#! /bin/bash
gnome-terminal --tab -e "/bin/bash -c \


'sleep 2;\
rosrun rqt_console rqt_console;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 5;\
roslaunch kuka_arm inverse_kinematics.launch;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'roslaunch kuka_arm self_filter.launch;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 5;\
roslaunch voxblox_ros kr210.launch;\
exec /bin/bash -i'"