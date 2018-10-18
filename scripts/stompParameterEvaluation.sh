#! /bin/bash
gnome-terminal --tab -e "/bin/bash -c \

'roscore;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 3;\
rosparam load stomp_std_devs.yaml;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 3;\
roslaunch kr210_claw_moveit demo.launch|tee log.txt;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 12;\
./loadWorld.sh;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 12;\
./planPython.py;\
exec /bin/bash -i'" 
