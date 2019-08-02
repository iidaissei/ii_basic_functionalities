xterm -geometry 80x5+0+0 -e "/opt/ros/kinetic/bin/roscore" &
sleep 3s
xterm -geometry 80x5+0+120 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_bringup minimal.launch" &
sleep 4s
xterm -geometry 80x5+0+230 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_bringup 3dsensor.launch" &
sleep 4s
xterm -geometry 80x5+0+340 -e "/opt/ros/kinetic/bin/roslaunch realsense2_camera rs_rgbd.launch" 
