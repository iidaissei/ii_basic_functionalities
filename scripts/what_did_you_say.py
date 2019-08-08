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

class MimiControlClass():
    def __init__(self):
        #Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)#kobukiの前進後進
        self.m6_pub = rospy.Publisher('/m6_controller/command', Float64, queue_size = 1)
        self.tts_pub = rospy.Publisher('/tts', String, queue_size = 1)
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

    def motorControl(self, motor_name, value):
        if motor_name == 6:
            m6_angle = Float64()
            m6_angle = value
            rospy.sleep(0.1)
            self.m6_pub.publish(m6_angle)

    def speak(self, sentense):
        voice_cmd = '/usr/bin/picospeaker -r -25 -p 5 %s' %sentense
        subprocess.call(voice_cmd.strip().split(' '))

    def ttsSpeak(self, sentense):
        data = String()
        data.data = sentense
        rospy.sleep(0.1)
        self.tts_pub.publish(data)
        rospy.sleep(0.5)

class NavigationClass():
    def __init__(self):
        #Publisher
        self.navigation_memorize_pub = rospy.Publisher('/navigation/memorize_place', String, queue_size = 1)#目的地を記憶
        self.navigation_command_pub = rospy.Publisher('/navigation/move_place', String, queue_size = 1)#ナビゲーション開始の命令
        #Subscriber
        self.navigation_res_sub = rospy.Subscriber('/navigation/result', Bool, self.getNavigationResultCB)
        
        self.navigation_result_flg = False
        self.mimi = MimiControlClass()
 
    def getNavigationResultCB(self, result_msg):
        self.navigation_result_flg = result_msg.data

    def movePlace(self, receive_msg):
        self.mimi.motorControl(6, 0.3)
        rospy.sleep(1.5)
        place_name = String()
        place_name.data = receive_msg
        rospy.loginfo(" Move to " + str(place_name.data))
        self.mimi.ttsSpeak("I move to " + str(place_name.data))
        rospy.sleep(0.1)
        self.navigation_command_pub.publish(place_name)
        while self.navigation_result_flg == False and not rospy.is_shutdown():
            rospy.sleep(2.5)
            rospy.loginfo(" Moving...")
        rospy.sleep(0.5)
        self.navigation_result_flg = False
        rospy.loginfo(" Arrived " + str(place_name.data))
        self.mimi.ttsSpeak("I arrived " + str(place_name.data))
        rospy.sleep(1.0)

class WhatDidYouSay():
    def __init__(self):
        #Publisher
        self.conversation_start_pub = rospy.Publisher('conversation/start', Bool, queue_size = 1)
        self.move_close_person_pub = rospy.Publisher('/move_close_person/start', String, queue_size = 1)
        #Subscriber
        rospy.Subscriber('conversation/stop', Bool, self.conversation_stopCB)
        rospy.Subscriber('/move_close_person/stop', String, self.move_close_personCB)
        
        self.nav = NavigationClass()
        self.mimi = MimiControlClass()
        self.conversation_stop_flg = Bool()
        self.move_close_person_flg = 'Null'
        self.conversation_stop_flg = True

    def conversation_stopCB(self, receive_msg):
        self.conversation_stop_flg = receive_msg.data

    def move_close_personCB(self, result_msg):
        self.move_close_person_flg = result_msg.data

    def conversationMethod(self):
        try:
            self.mimi.ttsSpeak("Let's start conversation!")
            question_number = 1
            result = Bool()
            result.data = False
            while not rospy.is_shutdown() and not question_number == 5:
                rospy.sleep(1.0)
                rospy.loginfo(" Question Number: " + str(question_number))
                self.mimi.ttsSpeak("Please give me a question")
                rospy.sleep(0.1)
                self.conversation_start_pub.publish(result)
                rospy.loginfo(" Published 'conversation/start' Topic")
                while not rospy.is_shutdown() and  self.conversation_stop_flg == True:
                    rospy.loginfo(" Waiting for topic...")
                    rospy.sleep(1.5)
                self.conversation_stop_flg = True
                rospy.sleep(0.5)
                rospy.loginfo(" Subscribed 'conversation/stop' Topic")
                self.mimi.ttsSpeak("I finished answering")
                question_number += 1
            self.mimi.ttsSpeak("I answered all questions")
            rospy.loginfo(" Finish conversation")
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

    def enterRoom(self):#--------------------------------------------------------state0
        try:
            print '-' *80
            rospy.loginfo(" Start the state0")
            rospy.sleep(1.0)
            self.mimi.ttsSpeak("Start What Did You Say")
            rospy.sleep(1.0)
            #self.nav.movePlace('entrance')#スタート地点に移動
            rospy.sleep(1.0)
            rospy.loginfo(" Finished the state0")
            return 1
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

    def moveClosePerson(self):#--------------------------------------------------state1
        try:
            print '-' *80
            rospy.sleep(1.5)
            rospy.loginfo(" Start the state1")
            data = String()
            data.data = 'start'
            self.move_close_person_pub.publish(data)
            while not rospy.is_shutdown() and not self.move_close_person_flg == 'stop':
                #rospy.loginfo(" Waiting for topic")
                rospy.sleep(0.1)
            self.move_close_person_flg = 'Null'
            rospy.sleep(2.0)
            self.mimi.ttsSpeak("My name is mimi")
            rospy.sleep(1.0)
            self.mimi.ttsSpeak("Nice to meet you")
            rospy.sleep(2.0)
            rospy.loginfo(" Finished the state1")
            return 2
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

    def startConversation(self):#------------------------------------------------state2
        try:
            print '-' *80
            rospy.loginfo(" Start the state2")
            rospy.sleep(0.5)
            self.conversationMethod()
            rospy.sleep(1.0)
            rospy.loginfo(" Finished the state2")
            return 3
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

    def exitRoom(self):#---------------------------------------------------------state3
        try:
            print '-' *80
            rospy.loginfo(" Start the state3")
            rospy.sleep(1.0)
            self.nav.movePlace('entrance')#大会時変更
            rospy.sleep(2.0)
            rospy.loginfo(" Finished the state3")
            rospy.loginfo(" Finished What Did You Say")
            self.mimi.ttsSpeak("Finished What Did You Say")
            return 4
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
                state = wds.moveClosePerson()
            elif state == 2:
                state = wds.startConversation()
            elif state == 3:
                state = wds.exitRoom()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Interrupted")
        pass
