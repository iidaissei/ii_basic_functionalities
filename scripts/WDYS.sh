xterm -geometry 80x5+0+560 -e "/opt/ros/kinetic/bin/rosrun get_distance_pcl get_distance_pcl" &
sleep 2s
xterm -geometry 80x5+0+670 -e "/opt/ros/kinetic/bin/rosrun object_fake_tf object_fake_tf" &
sleep 2s
xterm -geometry 80x5+0+670 -e "/opt/ros/kinetic/bin/rosrun ii_basic_functionalities navigation.py" &
sleep 2s
xterm -geometry 80x10+2400+120 -e "/opt/ros/kinetic/bin/rosrun ii_basic_functionalities move_close_human.py" &
sleep 2s
xterm -geometry 80x10+2400+350 -e "/opt/ros/kinetic/bin/rosrun ii_basic_functionalities what_did_you_say.py" &
