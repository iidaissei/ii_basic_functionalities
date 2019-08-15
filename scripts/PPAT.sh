xterm -geometry 80x10+2400+0 -e "/opt/ros/kinetic/bin/rosrun ii_basic_functionalities navigation.py" &
sleep 2s
xterm -geometry 80x10+2400+200 -e "/opt/ros/kinetic/bin/rosrun ii_basic_functionalities avoid_that.py" &
sleep 2s
xterm -geometry 80x10+2400+350 -e "/opt/ros/kinetic/bin/rosrun ii_basic_functionalities pick_and_place.py"
