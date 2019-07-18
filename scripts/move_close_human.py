#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf 
import math
import actionlib
import std_srvs.srv
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Twist, Quaternion, Point, PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from math import pi


class MimiControlClass():
    def __init__(self):
        #Publisher
        self.m5_pub = rospy.Publisher('/m5_controller/command', Float64, queue_size = 1)
        self.m6_pub = rospy.Publisher('/m6_controller/command', Float64, queue_size = 1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)
    
        #self.twist_cmd = Twist()

    def motorControl(self, motor_name, value):
        if motor_name == 6:
            m6_angle = Float64()
            m6_angle = value
            self.sleep(0.1)
            self.m6_pub.publish(m6_angle)
            rospy.loginfo(" Changing m6 pose")
    
    def linearControl(self, linear_num):
        twist_cmd = Twist()
        twist_cmd.linear.x = linear_num
        self.cmd_vel_pub.publish(twist_cmd)
        twist_cmd.linear.x = 0

    def angularControl(self, angular_num):
        twist_cmd.angular.z = angular_num
        self.cmd_vel_pub.publish(twist_cmd)
        twist_cmd.angular.z = 0

    def speak(self, sentense):
        voice_cmd = '/usr/bin/picospeaker %s' %sentense
        subprocess.call(voice_cmd.strip().split(' '))


class MoveClosePerosn():
    def __init__(self):
        #Publisher
        self.move_close_person_flg = rospy.Publisher('/move_close_person/stop', String, queue_size = 1)
        #Subscriber
        rospy.Subscriber('/move_close_person/start', String, self.getStartFlgCB)
        rospy.Subscriber('/recog_obj', String, self.recogPerson)
        rospy.Subscriber('/object/xyz_centroid', Point , self.getObject_xy)
        #rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.navigation, callback_args = 3) 
        self.sub_tf  = rospy.Subscriber('/tf', TFMessage, self.getTfCB)

        self.start_flg = 'Null'
        self.person_flg = False
        self.sub_tf_flg = False
        self.object_xy_flg = False
        self.location_pose_x = 0
        self.location_pose_y = 0
        self.location_pose_w = 0
        self.object_x = 999
        self.object_y = 999
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
 
    def getTfCB(self, receive_msg):
        self.sub_tf = receive_msg
        self.sub_tf_flg = True

    def getObject_xy(self, receive_msg):
        self.object_x = receive_msg.x
        self.object_y = receive_msg.y
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
            self.mimi.motorControl(6, -0.1)
            while not rospy.is_shutdown() and self.person_flg == False:
                for i in range(24):
                    if i <= 8 and self.person_flg == False:
                        self.mimi.angularControl(0.61)
                        rospy.sleep(0.1)
                        rospy.sleep(0.5)
                    elif i >8 and self.person_flg == False:
                        self.mimi.angularControl(-0.61)
                        rospy.sleep(0.1)
                        rospy.sleep(0.5)
                    if i == 24 and self.person_flg == False:
                        rospy.loginfo(" Could not find person")
                        #self.mimi.linearControl(2.0)
                        for i in range(16):
                            self.mimi.angularControl(0.61)
                        i == 1
            self.person_flg = False
            #self.mimi.speak("I found a human")
            rospy.loginfo(" Find Person")
            rospy.loginfo(" Finished the state1")
            rospy.sleep(1.0)
            return 2
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

    def getMimiPosition(self):#----------------------------------------------state2
        try:
            rospy.loginfo(" Start the state1")
            rospy.sleep(1.0)
            while not rospy.is_shutdown() and self.sub_tf_flg = True:
                self.getMimiPosition()
            self.sub_tf_flg = False
            if pose.transforms[0].header.frame_id == 'odom':
                self.location_pose_x = self.sub_tf.transforms[0].transform.translation.x
                self.location_pose_y = self.sub_tf.transforms[0].transform.translation.y
                self.location_pose_w = self.sub_tf.transforms[0].transform.rotation.z
                #self.location_pose_w += 1.5 * self.location_pose_w * self.location_pose_w *self.location_pose_w
            rospy.loginfo(" Get mimi position")
            rospy.loginfo(" Finished the state1")
            return 3
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass
 

    def navigation(self):#---------------------------------------------------state3
        try:
            rospy.loginfo(" Start the state3")
            rospy.sleep(2.0)
            while not rospy.is_shutdown() and self.object_xy_flg = True:
                self.getObject_xy()
            self.location_pose.x += self.object_x
            self.location_pose.y += self.object_y
            rospy.sleep(1.0)
            ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            while not ac.wait_for_server(rospy.Duration(5.0)) and not rospy.is_shutdown():
                rospy.loginfo(" Waiting for action client comes up...")
            rospy.loginfo(" The server comes up")
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = self.location_pose_x
            goal.target_pose.pose.position.y = self.location_pose_y
            q = tf.transformations.quaternion_from_euler(0, 0, self.location_pose_w)
            goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
            self.mimi.speak("Move to the point of questioner")
            rospy.loginfo(" Send Goal")
            ac.send_goal(goal)
            while not rospy.is_shutdown():
                if ac.get_state() == 1:
                    rospy.loginfo(" Got out of the obstacle")
                    rospy.sleep(1.5)
                elif ac.get_state() == 3:
                    self.speak("Has arrived")
                    rospy.loginfo(" Has arrived")
                    rospy.sleep(0.2)
                    rospy.loginfo(" Finished the state 3")
                    result = String()
                    result.data = 'stop'
                    rospy.sleep(0.2)
                    self.move_close_person_pub.publish(result)
                    self.mimi.speak("I found a human")
                    rospy.loginfo(" Finished move_close_person")
                    return 4
                elif ac.get_state() == 4:
                    rospy.loginfo(" Buried in obstacle")
                    rospy.sleep(1.5)
                    return 3
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass

if __name__ == '__main__':
    rospy.init_node("move_close_person", anonymous = True)
    try:
        state = 2
        mcp = MoveClosePerosn()
        while not rospy.is_shutdown() and not state == 4:
            if state == 0:
                state = mcp.waitTopic()
            elif state == 1:
                state = mcp.findPerson()
            elif state == 2:
                state = mcp.getMimiPosition()
            elif state == 3:
                state = mcp.navigation()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Interrupted")
        pass
