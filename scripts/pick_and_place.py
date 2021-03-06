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

    def getLaserCB(self, laser_scan):
        self.laser_dist = laser_scan.ranges
        self.min_laser_dist = min(laser_scan.ranges[180:540])
        self.front_laser_dist = laser_scan.ranges[359]
        
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
        place_name = String()
        place_name.data = receive_msg
        print place_name
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

class ObjectRecognizeClass():
    def __init__(self):
        #Publisher
        self.object_recog_req_pub = rospy.Publisher('/object/recog_req', String, queue_size = 1)#searchの開始
        self.object_list_req_pub = rospy.Publisher('/object/list_req', Bool, queue_size = 1)#objectのリストをもらう
        self.object_count_req_pub = rospy.Publisher('/object/count_req', Bool, queue_size = 1)#objectの個数を要求
        #Subscriber
        self.object_recog_res_sub = rospy.Subscriber('/object/recog_res', Bool, self.getObjectRecognizeResultCB)
        self.object_list_res_sub = rospy.Subscriber('/object/list_res', String, self.getObjectListCB)
        self.object_count_res_sub = rospy.Subscriber('/object/count_res', Int8, self.getObjectCountCB)
        
        self.object_recog_flg = False
        self.object_list = []
        self.object_list_flg = False
        self.object_num = -1

    def getObjectCountCB(self, result_msg):
        self.object_num = result_msg.data
       
    def getObjectListCB(self, result_msg):
        print result_msg.data
        self.object_list = result_msg.data.split(" ")
        self.object_list[-1:] = []
        print self.object_list
        self.object_num = len(self.object_list)
        self.object_list_flg = True

    def getObjectRecognizeResultCB(self, result_msg):
        self.object_recog_flg = result_msg.data

class MoveObject():#---------------------------------------------------state0
    def __init__(self):
        self.object_req = ObjectRecognizeClass()
        self.nav = NavigationClass()
        self.mimi = MimiControlClass()

    def doorOpenStart(self):
        try:
            while not rospy.is_shutdown() and self.mimi.front_laser_dist == 999.9:
                rospy.sleep(1.0)
            initial_distance = self.mimi.front_laser_dist
            print initial_distance
            self.mimi.ttsSpeak("Please open the door")
            while not rospy.is_shutdown() and self.mimi.front_laser_dist <= initial_distance + 0.88:#試走場のドアの幅を参考
                rospy.loginfo(" Waiting for door open")
                rospy.sleep(2.0)
            rospy.sleep(2.0)
            self.mimi.ttsSpeak("Thank you")
            rospy.sleep(0.1)
            for i in range(15):
                self.mimi.linearControl(0.3)#linear0.3を15回で80cm前進
                rospy.sleep(0.2)
            rospy.sleep(0.5)
            rospy.loginfo(" Enter the room")
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass
 
    def master(self):
        try:
            print '-' *80
            rospy.loginfo(" Start the state0")
            rospy.sleep(1.0)
            self.mimi.ttsSpeak("Start Pick and place")
            rospy.sleep(1.0)
            self.mimi.motorControl(6, 0.3)
            rospy.sleep(1.0)
            self.doorOpenStart()
            rospy.sleep(0.5)
            self.nav.movePlace('Table')#objectの置き場所を送る
            rospy.sleep(0.5)
            rospy.loginfo(" Finish the state0")
            return 1
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

class PickObject():#----------------------------------------------------state1
    def __init__(self):
        #Publisher
        self.object_grasp_req_pub = rospy.Publisher('/object/grasp_req', String, queue_size = 10)#manipulationの開始
        #Subscriber
        self.object_grasp_res_sub = rospy.Subscriber('/object/grasp_res', Bool, self.getObjectGraspResultCB)
        
        self.object_grasp_result_flg = False    
        self.object_name = String()
        self.object_req = ObjectRecognizeClass()
        self.mimi = MimiControlClass()
    
    def getObjectGraspResultCB(self, result_msg):
        self.object_grasp_result_flg = result_msg.data

    def findObject(self):
        list_req = Bool()
        list_req.data = True
        rospy.sleep(0.5)
        self.object_req.object_list_req_pub.publish(list_req)
        rospy.sleep(2.0)
        print self.object_req.object_list
        while not rospy.is_shutdown() and self.object_req.object_list_flg == False:
            for i in range(10):
                self.mimi.angularControl(0.3)
                if i == 10:
                    #self.mimi.ttsSpeak("I can't find object")
                    #rospy.sleep(1.0)
                    #self.mimi.ttsSpeak("Please help me")
                    self.mimi.angularControl(0)
                    break
        self.object_list_flg = False
        self.object_name = self.object_req.object_list[0]
        self.mimi.ttsSpeak("I find "+ self.object_name)
        rospy.loginfo(" Find " + self.object_name)
        rospy.sleep(1.0)
        
    def startObjectGrasp(self):
        rospy.sleep(0.5)
        rospy.loginfo("Grasp " + self.object_name)
        self.mimi.ttsSpeak("I grasp " + self.object_name)
        grasp_req = String()
        grasp_req.data = self.object_name
        rospy.sleep(0.1)
        self.object_grasp_req_pub.publish(grasp_req)
        while not rospy.is_shutdown() and self.object_grasp_result_flg == False:
            rospy.loginfo(" Waiting for grasping...")
            rospy.sleep(2.5)
        self.object_grasp_result_flg = False
        self.object_req.object_list = []

    def master(self):
        try:
            print '-' *80
            rospy.loginfo(" Start the state1")
            self.mimi.motorControl(6, -0.05)
            rospy.sleep(1.0)
            self.findObject()
            rospy.sleep(1.0)
            self.startObjectGrasp()
            rospy.sleep(0.5)
            rospy.loginfo(" Finish the state1")
            return 2
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

class PlaceObject():#-------------------------------------------------------------------------------state2
    def __init__(self):
        #Publisher
        self.object_place_req_pub = rospy.Publisher('/arm/changing_pose_req', String, queue_size=1)#objectを置く
        self.task_2_start_pub = rospy.Publisher('/task_2_start', Bool , queue_size = 1)
        #Subscriber
        rospy.Subscriber('/arm/changing_pose_res', Bool, self.getObjectPlaseCB)
        self.object_place_res_sub = rospy.Subscriber('/arm/changing_pose_res', Bool, self.getObjectPlaseCB)
        
        self.object_place_flg = False
        self.nav = NavigationClass()
        self.mimi = MimiControlClass()

    def getObjectPlaseCB(self, result_msg):
        self.object_place_flg = result_msg.data
        print self.object_place_flg

    def statrObjectPlace(self):
        place_req = String()
        place_req.data = 'place'
        rospy.loginfo(" Place object")
        self.object_place_req_pub.publish(place_req)
        while not rospy.is_shutdown() and self.object_place_flg == False:
            rospy.loginfo(" Waiting for placing...")
            rospy.sleep(2.5)
        rospy.sleep(0.5)
        self.object_place_flg = False

    def master(self):
        try:
            print '-' *80
            rospy.loginfo(" Start the state2")
            rospy.sleep(0.1)
            self.mimi.motorControl(6, 0.3)
            rospy.sleep(1.5)
            while not rospy.is_shutdown():
                if self.object_name == 'Glass Noodle' or 'Blue Noodle':
                    self.nav.movePlace('Drawer')
                elif self.object_name == 'Blue Stick' or 'Potato Chips':
                    self.nav.movePlace('Cupboard')
                elif self.object_name == 'Lemon Tea':
                    self.nav.movePlace('Couch')
                else:
                    self.nav.movePlace('Cupboard')
            rospy.sleep(1.5)
            self.statrObjectPlace()
            rospy.sleep(1.0)
            rospy.loginfo(" Finish the state2")
            rospy.loginfo(" Finished pick and place")
            self.mimi.ttsSpeak("Finished pick and place")
            rospy.sleep(2.0)
            data = Bool()
            data.data = True
            rospy.sleep(0.1)
            self.task_2_start_pub.publish(data)
            rospy.loginfo(" Published '/task_2_start' topic")
            return 4
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

if __name__ == '__main__':
    rospy.init_node("pick_and_place", anonymous = True)
    try:
        state = 0
        moveO = MoveObject()
        pickO = PickObject()
        placeO = PlaceObject()
        while not rospy.is_shutdown() and not state == 4:
            if state == 0:
                state = moveO.master()
            elif state == 1:
                state = pickO.master()
            elif state == 2:
                state = placeO.master()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Interrupted")
        pass
