#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf 
import math
import actionlib
import subprocess
import rosparam
import time
from math import pi
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String, Bool, Float64
from get_distance_pcl.msg import Coordinate_xyz
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Quaternion, Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class MimiControlClass():
    def __init__(self):
        #Publisher
        self.m5_pub = rospy.Publisher('/m5_controller/command', Float64, queue_size = 1)
        self.m6_pub = rospy.Publisher('/m6_controller/command', Float64, queue_size = 1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)
        self.tts_pub = rospy.Publisher('/tts', String, queue_size = 1)

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

    def ttsSpeak(self, sentense):
        data = String()
        data.data = sentense
        rospy.sleep(0.1)
        self.tts_pub.publish(data)
        rospy.sleep(1.0)

class Navigation():
    def __init__(self):
        #Publisher
        self.simple_goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped , queue_size = 1)
        #Subscriber
        rospy.Subscriber('/odom', Odometry, self.getTfCB)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.getStatusCB) 
        #Service
        rospy.wait_for_service('move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty)

        self.mimi = MimiControlClass()

    def getTfCB(self, receive_msg):#向きのみを購読
        if receive_msg.header.frame_id == 'odom':
            self.tf_pose_w = receive_msg.pose.pose.orientation.w
            self.tf_pose_z = receive_msg.pose.pose.orientation.z
            self.sub_tf_flg = True

    def getStatusCB(self, receive_msg):
        try:
            self.get_status = receive_msg.status_list[0].status
        except IndexError:
            pass

    def navigation(self, object_coordinate_x, object_coordinate_y):#---------------------------------------------------state3
        try:
            rospy.loginfo(" Start the state3")
            rospy.sleep(0.1)
            self.mimi.motorControl(6, 0.3)
            rospy.sleep(2.0)
            #rosparam.set_param('/move_base/DWAPlannerROS/xy_goal_tolerance', 0.50)
            print 'set goal_tolerance'
            while not rospy.is_shutdown() and self.sub_tf_flg == False:
                rospy.sleep(1.0)
            self.sub_tf_flg = False
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = object_coordinate_x
            goal.pose.position.y = object_coordinate_y
            goal.pose.orientation.z = self.tf_pose_z
            goal.pose.orientation.w = self.tf_pose_w
            rospy.loginfo(" Approach person")
            self.mimi.ttsSpeak("Approach person")
            self.clear_costmaps()
            rospy.sleep(0.5)
            self.get_status = 0
            self.simple_goal_pub.publish(goal)
            print goal
            while not rospy.is_shutdown() and not self.get_status == 10:
                #self.simple_goal_pub.publish(goal)
                if self.get_status == 1:
                    rospy.loginfo(" Got out of the obstacle")
                    rospy.sleep(1.5)
                elif self.get_status == 3:
                    rospy.loginfo(" Has arrived")
                    self.get_status = 10
                    break
                elif self.get_status == 4:
                    rospy.loginfo(" Buried in obstacle")
                    self.clear_costmaps()
                    print 'clear'
            rospy.sleep(2.0)
            rospy.loginfo(" Finished navigation")
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

class MoveClosePerosn():
    def __init__(self):
        #Publisher
        self.move_close_person_pub = rospy.Publisher('/move_close_person/stop', String, queue_size = 1)
        self.human_detect_pub = rospy.Publisher('/move_close_human/human_detect_flag', Bool , queue_size = 1)
        #Subscriber
        rospy.Subscriber('/move_close_person/start', String, self.getStartFlgCB)
        rospy.Subscriber('/recog_obj', String, self.recogPersonCB)
        self.human_dist_sub = rospy.Subscriber('get_distance_pcl/Coordinate_xyz', Coordinate_xyz, self.getObject_xy)

        self.start_flg = 'Null'
        self.person_flg = False
        self.object_xy_flg = False
        self.human_detect_flg = False
        self.mimi = MimiControlClass()
        self.nav = Navigation()

    def getStartFlgCB(self, receive_msg):
        self.start_flg = receive_msg.data

    def recogPersonCB(self, receive_msg):
        obj = receive_msg.data
        obj_list = obj.split(" ")
        rospy.sleep(0.1)
        for i in range(len(obj_list)):
            if obj_list[i] == 'person':
                self.person_flg = True
 
    def turnPerson(self):
        try:
            rospy.loginfo(" Turn to person")
            self.mimi.motorControl(6, -0.2)
            rospy.sleep(2.0)
            self.person_flg = False
            while not rospy.is_shutdown() and self.person_flg == False:
                for i in range(24):
                    if i <= 8 and self.person_flg  == False:
                        self.mimi.angularControl(0.61)
                        rospy.sleep(0.3)
                    elif i >8 and self.person_flg == False:
                        self.mimi.angularControl(-0.61)
                        rospy.sleep(0.3)
                    if i == 24 and self.person_flg == False:
                        rospy.loginfo(" Could not find person")
                        for j in range(24):
                            self.mimi.angularControl(0.61)
                        break
                break
            self.person_flg = False
            rospy.sleep(1.0)
            rospy.loginfo(" Turned to person")
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass
 
    def getObject_xy(self, receive_msg):
        self.object_coordinate_x = receive_msg.world_x
        self.object_coordinate_y = receive_msg.world_y
        self.object_xy_flg = True

    def waitTopic(self):#-----------------------------------------------------state0
        while not rospy.is_shutdown():
            if self.start_flg == 'start':
                return 1
            else:
                return 0

    def findPerson(self):#----------------------------------------------------state1
        try:
            print '-'*80
            rospy.loginfo(" Start  the state1")
            rospy.sleep(0.1)
            self.turnPerson()
            rospy.sleep(1.0)
            rospy.loginfo(" Finished the state1")
            return 2
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

    def getHumanCoordinate(self):#--------------------------------------------state2
        try:
            print '-'*80
            rospy.loginfo(" Start the state2")
            data = Bool()
            data.data = True
            rospy.sleep(1.0)
            self.human_detect_pub.publish(data)
            while not rospy.is_shutdown() and self.object_xy_flg == False:
                self.human_detect_pub.publish(data)
                rospy.sleep(0.5)
            print self.object_coordinate_x
            print self.object_coordinate_y
            rospy.loginfo(" Get human coordinate")
            rospy.loginfo(" Finished the state2")
            return 3
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

    def ApproachPerson(self):#------------------------------------------------state3
        try:
            rospy.loginfo(" Start the state3")
            rospy.sleep(0.5)
            self.nav.navigation(self.object_coordinate_x, self.object_coordinate_y)
            rospy.sleep(2.0)
            #self.turnPerson()
            #rospy.sleep(2.0)
            self.mimi.motorControl(6, 0.5)
            rospy.sleep(1.5)
            rospy.loginfo(" I found person")
            self.mimi.ttsSpeak("I found person")
            rospy.loginfo(" Finished the state3")
            result = String()
            result.data = 'stop'
            rospy.sleep(0.5)
            self.move_close_person_pub.publish(result)
            rospy.loginfo(" Finish move_close_person")
            return 4
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
                state = mcp.ApproachPerson()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Interrupted")
        pass
