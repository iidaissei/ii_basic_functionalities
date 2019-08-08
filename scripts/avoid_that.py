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
        self.m6_pub = rospy.Publisher('/m6_controller/command', Float64, queue_size = 1)
        self.tts_pub = rospy.Publisher('/tts', String, queue_size = 1)

    def motorControl(self, motor_name, value):
        if motor_name == 6:
            m6_angle = Float64()
            m6_angle = value
            rospy.sleep(0.1)
            self.m6_pub.publish(m6_angle)
            rospy.loginfo(" Changing m6 pose")

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
        self.navigation_command_pub = rospy.Publisher('/navigation/move_place', String, queue_size = 1)
        #Subscriber
        self.navigation_res_sub = rospy.Subscriber('/navigation/result', Bool, self.getNavigationResultCB)
        
        self.mimi = MimiControlClass()
        self.navigation_result_flg = Bool()

    def getNavigationResultCB(self, result_msg):
        self.navigation_result_flg = result_msg.data

    def movePlace(self, receive_msg):
        self.mimi.motorControl(6, 0.3)
        place_name = String()
        place_name.data = receive_msg
        print place_name
        rospy.loginfo(" Move to " + str(place_name.data))
        self.mimi.ttsSpeak("I move to " + str(place_name.data))
        rospy.sleep(0.1)
        self.navigation_result_flg = False
        self.navigation_command_pub.publish(place_name)
        while self.navigation_result_flg == False and not rospy.is_shutdown():
            rospy.sleep(2.5)
            rospy.loginfo(" Moving...")
        rospy.sleep(0.5)
        self.navigation_result_flg = False
        rospy.loginfo(" Arrived " + str(place_name.data))
        self.mimi.ttsSpeak("I arrived " + str(place_name.data))
        rospy.sleep(1.0)

class AvoidThat():
    def __init__(self):
        #Subscriber
        rospy.Subscriber('/task_2_start', Bool , self.task_2_startCB)

        self.task_2_start_flg = Bool()
        self.nav = NavigationClass()
        self.mimi = MimiControlClass()

    def task_2_startCB(self, receive_msg):
        self.task_2_start_flg = receive_msg.data

    def waitingTopic(self):#---------------------state0
        while not rospy.is_shutdown():
            if self.task_2_start_flg == True:
                rospy.sleep(2.0)
                self.task_2_start_flg = False
                return 1
            else:
                return 0

    def moveStartPosition(self):#--------------------------------state1
        try:
            print '-' *80
            rospy.loginfo(" Start the state1")
            rospy.sleep(1.0)
            self.mimi.ttsSpeak("Start Avoid That")
            #rospy.sleep(1.0)
            #self.nav.movePlace('shelf')
            rospy.sleep(1.0)
            rospy.loginfo(" Finished the state1")
            rospy.sleep(1.0)
            return 2
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass
    
    def moveDestination(self):#----------------------------------state2
        try:
            print '-' *80
            rospy.loginfo(" Start the state2")
            #self.mimi.ttsSpeak("Start Avoid That")
            rospy.sleep(1.0)
            self.nav.movePlace('entrance')
            rospy.sleep(1.0)
            rospy.loginfo(" Finished the state2")
            rospy.sleep(1.0)
            self.mimi.ttsSpeak("Finished Avoid That")
            rospy.loginfo(" Finished Avoid That")
            return 3
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

if __name__ == '__main__':
    rospy.init_node("avoid_taht", anonymous = True)
    try:
        state = 0
        at = AvoidThat()
        while not rospy.is_shutdown() and not state == 3:
            if state == 0:
                state = at.waitingTopic()
            elif state == 1:
                state = at.moveStartPosition()
            elif state == 2:
                state = at.moveDestination()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Interrupted")
        pass
