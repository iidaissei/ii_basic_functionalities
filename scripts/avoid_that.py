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

    def motorControl(self, motor_name, value):
        if motor_name == 6:
            m6_angle = Float64()
            m6_angle = value
            rospy.sleep(0.1)
            self.m6_pub.publish(m6_angle)
            rospy.loginfo(" Changing m6 pose")
        rospy.sleep(1.0)

    def speak(self, sentense):
        voice_cmd = '/usr/bin/picospeaker -r -15 -p 4 %s' %sentense
        subprocess.call(voice_cmd.strip().split(' '))


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
        self.navigation_result_flg = False
        place_name = String()
        place_name = receive_msg
        self.navigation_command_pub.publish(place_name)
        while self.navigation_result_flg == False and not rospy.is_shutdown():
            rospy.sleep(4.5)
            rospy.loginfo(" Mooving...")
            self.mimi.speak("Moving")
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
<<<<<<< HEAD
            self.mimi.motorControl(6, 0.3)#正面を向く
=======
            #self.mimi.motorControl(6, 0.3)#正面を向く
>>>>>>> 945b22067adf5831e16a359dad9b101862d3277c
            #self.mimi.speak("Move to the front of the shelf")
            #rospy.loginfo(" Move to the shelf")
            #rospy.sleep(1.0)
            #self.nav.movePlace('shelf')
            #rospy.sleep(1.0)
            #self.mimi.speak(" I arrived the shelf")
            rospy.loginfo(" Finished the state0")
            rospy.sleep(1.0)
            return 1
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass
    
    def moveDestination(self):#----------------------------------state1
        try:
            print '-' *80
            rospy.loginfo(" Start the state1")
            self.mimi.Control(6, 0.3)
            rospy.sleep(1.0)
            self.mimi.speak("Move to the entrance")
            rospy.loginfo(" Move to the entrance")
            rospy.sleep(0.5)
            self.nav.movePlace('entrance')
            rospy.sleep(1.0)
            self.mimi.speak("I arrived the entrance")
            rospy.loginfo(" Finished the state1")
            rospy.sleep(1.0)
            self.mimi.speak("Finished Avoid That")
            return 2
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass


if __name__ == '__main__':
    rospy.init_node("avoid_taht", anonymous = True)
    try:
        state = 1
        at = AvoidThat()
        while not rospy.is_shutdown() and not state == 2:
            if state == 0:
                state = at.moveStartPosition()
            if state == 1:
                state = at.moveDestination()
        rospy.loginfo(" Finished Avoid_That")
    except rospy.ROSInterruptException:
        rospy.loginfo(" Interrupted")
        pass
