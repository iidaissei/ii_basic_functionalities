xterm -geometry 80x5+0+0 -e "/opt/ros/kinetic/bin/roscore" &
sleep 3s
xterm -geometry 80x5+0+120 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_bringup minimal.launch" &
sleep 3s
xterm -geometry 80x5+0+220 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_bringup 3dsensor.launch" &
sleep 3s
xterm -geometry 80x5+0+320 -e "/opt/ros/kinetic/bin/roslaunch realsense2_camera rs_rgbd.launch" &
sleep 3s
xterm -geometry 80x5+0+420 -e "/opt/ros/kinetic/bin/rosrun picotts picotts.exe" &
sleep 3s
xterm -geometry 80x5+0+620 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/athome/map/sisoujyou2.yaml" &
sleep 5s
