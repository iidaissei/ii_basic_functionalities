#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf 
from math import pi
import actionlib
from  std_srvs.srv import Empty
import time
import actionlib
from math import pi
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Quaternion
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan

class Navigation:
    def __init__(self):
        rospy.Subscriber('/navigation/memorize_place', String, self.getMemorizePlaceCB)
        rospy.Subscriber('/navigation/move_place', String, self.getDestinationCB)
        self.sub_tf  = rospy.Subscriber('/tf', TFMessage, self.getTfCB)
        
        self.navigation_result_pub = rospy.Publisher('/navigation/result', Bool, queue_size = 1)

        rospy.wait_for_service('move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty)

        self.location_name = 'Null'
        self.location_list = [['Table', 2.87, 1.9, -0.715, 0.698], ['Drawer', 1.81, 0.16, -0.680, 0.732], ['Cupboard', 2.61, 0.08, -0.680, 0.732 ], ['Couch', 3.34, 0.216, -0.680, 0.732 ], ['WhatDidYouSay startposition', 1.87, 4.3, 0.000, 0.999], ['Exit', 4.41, 5.76, -0.999, 0.041]]#要素3・4は/odom orientationのz,wを参考
        self.location_pose_x = 0
        self.location_pose_y = 0
        self.location_pose_w = 0
        self.destination = 'Null'
        self.sub_tf_flg = False

    def getMemorizePlaceCB(self, receive_msg):
        self.location_name = receive_msg.data

    def getDestinationCB(self, receive_msg):
        self.destination = receive_msg.data

    def getTfCB(self, receive_msg):
        self.sub_tf = receive_msg
        self.sub_tf_flg = True

    def waitTopic(self):#------------------------------------------------------state 0
        while not rospy.is_shutdown():
            if self.location_name != 'Null':
                rospy.loginfo("*Start LocationList setup*")
                return 1
            elif self.destination != 'Null':
                rospy.loginfo("*Start navigation*")
                return 2
            else :
                return 0

    def setLocationList(self):#------------------------------------------------state 1
        #pose = self.sub_tf
        while not rospy.is_shutdown() and self.sub_tf_flg == False:
            self.getTFCB()
        if self.sub_tf.transforms[0].header.frame_id == 'odom':
            self.location_pose_x = self.sub_tf.transforms[0].transform.translation.x
            self.location_pose_y = self.sub_tf.transforms[0].transform.translation.y
            self.location_pose_w = self.sub_tf.transforms[0].transform.rotation.z
            self.location_list.append([self.location_name, self.location_pose_x, self.location_pose_y, self.location_pose_w])
            rospy.loginfo("Add *" + self.location_name + "* to the LocationList")
            self.location_name = 'Null'
            result = Bool()
            result.data = True
            self.navigation_result_pub.publish(result)
            rospy.loginfo("Published result")
            return 0

    def navigateToDestination(self):#------------------------------------------state 2
        location_num = -1
        for i in range(len(self.location_list)):
            if self.destination == self.location_list[i][0]:
                rospy.loginfo("Destination is " + self.destination)
                location_num = i
        if location_num == -1:
            rospy.loginfo("Not found Destination")
            result = Bool()
            result.data = False
            self.navigation_result_pub.publish(result)
            return 0
        ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        while not ac.wait_for_server(rospy.Duration(5.0)) and not rospy.is_shutdown():
            rospy.loginfo("Waiting for action client comes up...")
        rospy.loginfo("The server comes up")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.location_list[location_num][1]
        goal.target_pose.pose.position.y = self.location_list[location_num][2]
        goal.target_pose.pose.orientation.z = self.location_list[location_num][3]
        goal.target_pose.pose.orientation.w = self.location_list[location_num][4]
        rospy.sleep(1.0)
        self.clear_costmaps()
        rospy.sleep(0.5)
        ac.send_goal(goal)
        rospy.loginfo("Sended Goal")
        while not rospy.is_shutdown():
            num = ac.get_state()
            print num
            if num == 1:
                rospy.loginfo("Got out of the obstacle")
                rospy.sleep(2.0)
            elif num == 3:
                rospy.loginfo("Goal")
                self.destination = 'Null'
                result = Bool()
                result.data = True
                rospy.sleep(0.3)
                self.navigation_result_pub.publish(result)
                rospy.loginfo("Published result")
                num = 0
                rospy.sleep(2.0)
                result.data = False
                rospy.sleep(0.1)
                self.navigation_result_pub.publish(result)
                return 0
            elif num == 4:
                rospy.loginfo("Buried in obstacle")
                self.clear_costmaps()
                rospy.loginfo(" Clear Costmaps")
                rospy.sleep(1.0)
                return 2


if __name__ == '__main__':
    rospy.init_node('sg_navigation', anonymous = True)
    try:
        nav = Navigation()
        state = 0
        while not rospy.is_shutdown():
            if state == 0:
                state = nav.waitTopic()
            elif state == 1:
                rospy.sleep(1.0)
                state = nav.setLocationList()
            elif state == 2:
                state = nav.navigateToDestination()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Interrupted")
        pass
