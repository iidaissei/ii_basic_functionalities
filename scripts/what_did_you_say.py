#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import time
import sys
import std_srvs.srv
import math
import actionlib
import subprocess
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from std_msgs.msg import String, Bool, Int8, Float64


class KobukiControlClass():
    def __init__(self):
        #Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)#kobukiの前進後進
        #Subscriber
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.getLaserCB)
        
        self.min_laser_dist = 999.9
        self.front_laser_dist = 999.9
        self.twist_cmd = Twist()
        
    def getLaserCB(self, laser_scan):
        self.laser_dist = laser_scan.ranges
        self.min_laser_dist = min(laser_scan.ranges[180:540])
        self.front_laser_dist = laser_scan.ranges[359]
        
    def linearControl(self, linear_num):
        self.twist_cmd.linear.x = linear_num
        self.cmd_vel_pub.publish(self.twist_cmd)
        self.twist_cmd.linear.x = 0

    def angularControl(self, angular_num):
        self.twist_cmd.angular.z = angular_num
        self.cmd_vel_pub.publish(self.twist_cmd)
        self.twist_cmd.angular.z = 0


class MimiControlClass():
    def __init__(self):
        #Publisher
        self.m5_pub = rospy.Publisher('/m5_controller/command', Float64, queue_size = 1)
        self.m6_pub = rospy.Publisher('m6_controller/command', Float64, queue_size = 1)
        self.changing_pose_pub = rospy.Publisher('/arm/changing_pose_req', String, queue_size = 1)#manipulateしたあとの変形

    def motorControl(self, motor_name, value):
        if motor_name == 5:
            self.m5_angle.data = Float64()
            self.m5_angle.data = value
            self.m5_pub.publish(self.m5_angle)
        elif motor_name == 6:
            self.m6_angle.data = Float64()
            self.m6_angle.data = value
            self.m6_pub.publish(self.m6_angle)

    def speak(self, sentense):
        voice_cmd = '/usr/bin/picospeaker %s' %sentense
        subprocess.call(voice_cmd.strip().split(' '))

    def armChangingPose(self, receive_msg):
        pose_req = String()
        pose_req.data = receive_msg
        self.changing_pose_pub.publish(pose_req)
        rospy.sleep(1.0)

    
class NavigationClass():
    def __init__(self):
        #Publisher
        self.navigation_memorize_pub = rospy.Publisher('/navigation/memorize_place', String, queue_size = 1)#目的地を記憶
        self.navigation_command_pub = rospy.Publisher('/navigation/move_place', String, queue_size = 1)#ナビゲーション開始の命令
        #Subscriber
        self.navigation_res_sub = rospy.Subscriber('/navigation/result', Bool, self.setPlace, self.movePlace)
        
        self.navigation_result_flg = False
        self.mimi = MimiControlClass()
        
    def getNavigationResultCB(self, result_msg):
        self.navigation_result_flg = result_msg.data

    def setPlace(self, receive_msg): 
        place_name = String()
        place_name = receive_msg
        rospy.loginfo(" Memorizing...")
        self.navigation_memorize_pub.publish(place_name)
        rospy.loginfo(" Publeshed topic")
        while self.navigation_result_flg.data == False and not rospy.is_shutdown():
            self.navigation_memorize_pub.publish(place_name)
            rospy.loginfo(" Waiting for result")
            time.sleep(2.0)
        self.navigation_result_flg.data = False
        rospy.loginfo(" Memorization complete!")
        self.mimi.speak("I remembered the location og the " + place_name)

    def movePlace(self, receive_msg):
        place_name = String()
        place_name = receive_msg
        self.navigation_command_pub.publish(place_name)
        rospy.loginfo(" Moving...")
        while self.navigation_result_flg.data == False and not rospy.is_shutdown():
            rospy.sleep(3.0)
        self.navigation_result_flg = False
        rospy.loginfo(" Has arrived!")
 
class WhatDidYouSay():
    def __init__(self):
        #Publisher
        self.conversation_start_pub = rospy.Publisher('conversation/start', Bool, queue_size = 1)
        #Subscriber
        self.conversation_stop_sub = rospy.Subscriber('conversation/stop', Bool, self.conversation_stopCB)
        
        self.nav = NavigationClass()
        self.mimi = MimiControlClass()
        self.conversation_stop_flg = False

    def conversation_stopCB(self, receive_msg):
        self.conversation_stop_flg = receive_msg

    def conversationMethod(self):
        try:
            question_number = 0
            result = Bool()
            result.data = True
            while not rospy.is_shutdown() and not question_number == 3:
                rospy.loginfo(" Quaternion Number: " + question_number + 1)
                self.conversation_start_pub.publish(result)
                rospy.loginfo(" Published 'conversation/start' Topic")
                while self.conversation_stop_flg == False:
                    rospy.loginfo(" Waiting for topic...")
                rospy.loginfo(" Published 'conversation/stop' Topic")
                question_number += 1
            rospy.loginfo(" Finish conversation")
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

    def enterRoom(self):
        try:
            print '-' *80
            rospy.loginfo(" Start the state0")
            self.nav.movePlace('')#開始場所を指定
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

    def moveClosePeople(self):
        try:
            print '-' *80
            rospy.loginfo(" Start the state1")
            self.mimi.motorControl(6, 0.2)#顔の角度を指定
            #人を発見する処理は保留
            self.mimi.speak("Hello. I'm mimi")
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

    def startConversation(self):
        try:
            print '-' *80
            rospy.loginfo(" Start the state2")
            self.mimi.speak("Let's start conversation")
            self.conversationMethod()
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

    def exitRoom(self):
        try:
            print '-' *80
            rospy.loginfo(" Start the state3")
            self.nav.movePlace('door')
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass


if __name__ == '__main__':
    rospy.init_node("what_did_you_say", anonymous = True)
    try:
        state = 0
        wds = WhatDidYouSay()
        while not rospy.is_shutdown() and not state == 4:
            if state == 0:
                state = wds.enterRoom()
            elif state == 1:
                state = wds.moveClosePeople()
            elif state == 2:
                state = wds.startConversation()
            elif state == 3:
                state = wds.exitRoom()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Interrupted")
        pass
