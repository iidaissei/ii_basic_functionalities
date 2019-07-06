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
    
class AvoidThat():
    def __init__(self):
        self.nav = NavigationClass()
        self.mimi = MimiControlClass()

    def moveStartPosition(self):#--------------------------------state0
        try:
            print '-' *80
            rospy.loginfo(" Start the state0")
            self.nav.setPlace('start_position')
            rospy.sleep(3.0)
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass
    
    def moveDestination(self):#----------------------------------state1
        try:
            print '-' *80
            rospy.loginfo(" Start the state1")
            self.nav.movePlace('destination')
            rospy.sleep(3.0)
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
                state = at.moveStartPosition()
            elif state == 1:
                state = at.moveDestination()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Interrupted")
        pass
