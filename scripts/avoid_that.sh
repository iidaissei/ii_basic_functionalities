xterm -geometry 80x20+0+0 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_bringup minimal.launch" &
sleep 3s
xterm -geometry 80x20+0+120 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_bringup 3dsensor.launch" &
sleep 3s
xtera -geometry 80x20+600+0 -e "/opt/ros/kinetic/bin/roslaunch realsense2_camera rs_rgbd.launch" &
sleep 3s
xterm -geometry 80x20+600+0 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_navigation amcl_demo.launch map_file:=~/home/athome/map/sisoujyou2.yaml"
sleep 5s
xterm -geometry 80x20+10+120 -e "/opt/ros/kinetic/"
xterm -geometry 80x20+0+620 -e "/opt/ros/kinetic/bin/roslaunch camera_tf start" &
sleep 7s
xterm -geometry 80x20+600+120 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 3s
xterm -geometry 80x20+0+240 -e "/opt/ros/kinetic/bin/rosrun ii_basic_fanctionalities navigation.py" &
sleep 5s
xterm -geometry 80x20+0+360 -e "/opt/ros/kinetic/bin/rosrun ii_basic_fanctionalities avoid_that.py"

