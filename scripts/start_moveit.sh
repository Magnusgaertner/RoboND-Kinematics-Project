#! /bin/bash
gnome-terminal --tab -e "/bin/bash -c \

'roscore;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 2;\
rosrun rqt_console rqt_console;\
exec /bin/bash -i'" --tab -e "/bin/bash -c \

'sleep 3;\
roslaunch kr210_claw_moveit demo.launch|tee log.txt;\
exec /bin/bash -i'"
