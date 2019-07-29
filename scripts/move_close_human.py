#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf 
import math
import actionlib
import subprocess
from std_srvs.srv import Empty
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Twist, Quaternion, Point, PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from math import pi
from get_distance_pcl.msg import Coordinate_xyz


class MimiControlClass():
    def __init__(self):
        #Publisher
        self.m5_pub = rospy.Publisher('/m5_controller/command', Float64, queue_size = 1)
        self.m6_pub = rospy.Publisher('/m6_controller/command', Float64, queue_size = 1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)

    def motorControl(self, motor_name, value):
        if motor_name == 6:
            m6_angle = Float64()
            m6_angle = value
            rospy.sleep(0.1)
            self.m6_pub.publish(m6_angle)
            rospy.loginfo(" Changing m6 pose")
    
    def linearControl(self, linear_num):
        twist_cmd = Twist()
        twist_cmd.linear.x = linear_num
        self.cmd_vel_pub.publish(twist_cmd)
        twist_cmd.linear.x = 0

    def angularControl(self, angular_num):
        twist_cmd = Twist()
        twist_cmd.angular.z = angular_num
        self.cmd_vel_pub.publish(twist_cmd)
        twist_cmd.angular.z = 0

    def speak(self, sentense):
        voice_cmd = '/usr/bin/picospeaker -r -25 -p 5 %s' %sentense
        subprocess.call(voice_cmd.strip().split(' '))


class MoveClosePerosn():
    def __init__(self):
        #Publisher
        self.move_close_person_pub = rospy.Publisher('/move_close_person/stop', String, queue_size = 1)
        self.human_detect_pub = rospy.Publisher('/move_close_human/human_detect_flag', Bool , queue_size = 1)
 
        #Subscriber
        rospy.Subscriber('/move_close_person/start', String, self.getStartFlgCB)
        rospy.Subscriber('/recog_obj', String, self.recogPerson)
        self.human_dist_sub = rospy.Subscriber('get_distance_pcl/Coordinate_xyz', Coordinate_xyz, self.getObject_xy)
        #Service
        rospy.wait_for_service('move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty)


        self.start_flg = 'Null'
        self.person_flg = False
        self.sub_tf_flg = False
        self.object_xy_flg = False
        self.human_detect_flg = False
        self.mimi = MimiControlClass()

    def getStartFlgCB(self, receive_msg):
        self.start_flg = receive_msg.data

    def recogPerson(self, receive_msg):
        obj = receive_msg.data
        obj_list = obj.split(" ")
        rospy.sleep(0.1)
        for i in range(len(obj_list)):
            if obj_list[i] == 'person':
                self.person_flg = True
 
    def getObject_xy(self, receive_msg):
        self.object_coordinate_x = receive_msg.world_x
        self.object_coordinate_y = receive_msg.world_y
        self.object_coordinate_z = receive_msg.world_z
        self.object_xy_flg = True

    def waitTopic(self):#-----------------------------------------------------state0
        while not rospy.is_shutdown():
            if self.start_flg == 'start':
                return 1
            else:
                return 0

    def findPerson(self):#----------------------------------------------------state1
        try:
            rospy.loginfo(" Start  the state1")
            rospy.sleep(0.1)
            self.mimi.motorControl(6, -0.1)
            self.person_flg = False
            #while not rospy.is_shutdown() and self.person_flg == False:
            while not rospy.is_shutdown() and self.object_xy_flg == False:
                for i in range(24):
                    if i <= 8 and self.object_xy_flg  == False:
                        self.mimi.angularControl(0.61)
                        rospy.sleep(0.5)
                    elif i >8 and self.object_xy_flg == False:
                        self.mimi.angularControl(-0.61)
                        rospy.sleep(0.5)
                    if i == 24 and self.object_xy_flg == False:
                        rospy.loginfo(" Could not find person")
                        for j in range(8):
                            self.mimi.angularControl(0.61)
                        break
                break
            self.person_flg = False
            rospy.sleep(0.1)
            rospy.loginfo(" Find Person")
            rospy.loginfo(" Finished the state1")
            return 2
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

    def getHumanCoordinate(self):#----------------------------------------------state2
        try:
            rospy.sleep(1.0)
            rospy.loginfo(" Start the state2")
            data = Bool()
            data.data = True
            rospy.sleep(1.0)
            self.human_detect_pub.publish(data)
            while not rospy.is_shutdown() and self.object_xy_flg == False:
                self.human_detect_pub.publish(data)
                rospy.sleep(0.5)
            rospy.sleep(1.0)
            print self.object_coordinate_x
            print self.object_coordinate_y
            rospy.sleep(0.1)
            self.mimi.motorControl(6, 0.3)
            rospy.sleep(3.0)
            rospy.loginfo(" Get human coordinate")
            rospy.loginfo(" Finished the state2")
            return 3
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass
 

    def navigation(self):#---------------------------------------------------state3
        try:
            rospy.loginfo(" Start the state3")
            rospy.sleep(0.2)
            ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            while not ac.wait_for_server(rospy.Duration(5.0)) and not rospy.is_shutdown():
                rospy.loginfo(" Waiting for action client comes up...")
            rospy.loginfo(" The server comes up")
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = self.object_coordinate_x
            goal.target_pose.pose.position.y = self.object_coordinate_y
            q = tf.transformations.quaternion_from_euler(0, 0, 0.5)
            goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
            rospy.sleep(0.5)
            self.mimi.speak("Move to the point of human")
            rospy.loginfo(" Send Goal")
            rospy.sleep(0.5)
            ac.send_goal(goal)
            while not rospy.is_shutdown():
                if ac.get_state() == 1:
                    rospy.loginfo(" Got out of the obstacle")
                    rospy.sleep(1.5)
                elif ac.get_state() == 3:
                    rospy.loginfo(" Has arrived")
                    rospy.sleep(1.0)
                    self.mimi.speak("I found a human")
                    rospy.loginfo("Finished the state3")
                    rospy.loginfo(" Finished move_close_person")
                    result = String()
                    result.data = 'stop'
                    rospy.sleep(0.1)
                    self.move_close_person_pub.publish(result)
                    return 4
                elif ac.get_state() == 4:
                    rospy.loginfo(" Buried in obstacle")
                    self.clear_costmaps()
                    print 'clear'
                    return 3
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

if __name__ == '__main__':
    rospy.init_node("move_close_person", anonymous = True)
    try:
        state = 0
        mcp = MoveClosePerosn()
        while not rospy.is_shutdown() and not state == 4:
            if state == 0:
                state = mcp.waitTopic()
            elif state == 1:
                state = mcp.findPerson()
            elif state == 2:
                state = mcp.getHumanCoordinate()
            elif state == 3:
                state = mcp.navigation()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Interrupted")
        pass
