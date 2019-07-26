xterm -geometry 80x20+0+0 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_bringup minimal.launch" &
<<<<<<< HEAD
sleep 7s
xterm -geometry 80x20+600+0 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_bringup 3dsensor.launch" &
sleep 7s
xtera -geometry 80x20+0+240 -e "/opt/ros/kinetic/bin/roslaunch realsense2_camera rs_rgbd.launch" &
sleep 7s
xterm -geometry 80x20+0+360 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_navigation amcl_demo.launch map_file:=~/home/athome/map/sisoujyou2.yaml"
sleep 7s
#xterm -geometry 80x5+600+120 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_rviz_launchers view_navigation.launch" &
#sleep 7s
xterm -geometry 80x20+600+620 -e "connectJetson"
sleep 7s
xterm -geometry 80x5+0+620 -e "/opt/ros/kinetic/bin/roslaunch e_manipulation motor_setup.launch" &
sleep 7s
#xterm -geometry 80x5+600+220 -e "/opt/ros/kinetic/bin/rosrun test_storing_groceries navigation.py" &
#sleep 7s
#xterm -geometry 80x5+0+320 -e "/opt/ros/kinetic/bin/rosrun e_object_recognizer object_recognizer.py" &
#sleep 3s
#xterm -geometry 80x5+0+420 -e "/opt/ros/kinetic/bin/rosrun e_grasping_position_detector e_grasping_position_detector" &
#sleep 3s
#xterm -geometry 80x5+0+520 -e "/opt/ros/kinetic/bin/rosrun e_manipulation manipulation.py" &
#sleep 3s
=======
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

>>>>>>> 945b22067adf5831e16a359dad9b101862d3277c
