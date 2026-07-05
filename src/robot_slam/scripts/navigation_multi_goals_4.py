#!/usr/bin/env python

#coding: utf-8

import rospy

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi
from std_msgs.msg import String

from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
from geometry_msgs.msg import Twist
from geometry_msgs.msg  import Point
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import os
music1_path="~/'01.mp3'"
music2_path="~/'02.mp3'"
music3_path="~/'03.mp3'"
music4_path="~/'04.mp3'"
music5_path="~/'05.mp3'"
music6_path="~/'06.mp3'"
music7_path="~/'07.mp3'"
music8_path="~/'08.mp3'"
time = 0
find_id = 0
id = 0


class navigation_demo:
    def __init__(self):
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.arrive_pub = rospy.Publisher('/voiceWords',String,queue_size=10)
        self.find_sub = rospy.Subscriber('/object_position', Point, self.find_cb);
        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb);
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
	self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1000)
    
    def end(self):
        global time
    	msg = Twist()
    	msg.linear.x = 0.3
    	msg.linear.y = 0.3
    	msg.linear.z = 0.0
    	msg.angular.x = 0.0
    	msg.angular.y = 0.0
    	msg.angular.z = 0.0
	while(time <= 20):
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time = time + 1

    def ar_cb(self, data):
        global id  
        for marker in data.markers:
            id = marker.id

    def find_cb(self, data):
	global find_id
        point_msg = data
	if(point_msg.z>=1 and point_msg.z<=10):
	    find_id = 1
	elif(point_msg.z>=11 and point_msg.z<=20):
	    find_id = 2
	elif(point_msg.z>=21 and point_msg.z<=30):
	    find_id = 3
	elif(point_msg.z>=31 and point_msg.z<=40):
	    find_id = 4
	elif(point_msg.z>=41 and point_msg.z<=50):
	    find_id = 5
	elif(point_msg.z>=51 and point_msg.z<=60):
	    find_id = 6
	elif(point_msg.z>=61 and point_msg.z<=70):
	    find_id = 7
	elif(point_msg.z>=71 and point_msg.z<=80):
	    find_id = 8

    def set_pose(self, p):
        if self.move_base is None:
            return False

        x, y, th = p

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, th/180.0*pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        self.set_pose_pub.publish(pose)
        return True

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s"%(status, result))
        arrive_str = "arrived to traget point"
        self.arrive_pub.publish(arrive_str)

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        msg = feedback
        #rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)

    def goto(self, p):
        rospy.loginfo("[Navi] goto %s"%p)
        #arrive_str = "going to next point"
        #self.arrive_pub.publish(arrive_str)
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p)
        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True
if __name__ == "__main__":
    rospy.init_node('navigation_demo',anonymous=True)
    goalListX = rospy.get_param('~goalListX', '2.0, 2.0')
    goalListY = rospy.get_param('~goalListY', '2.0, 4.0')
    goalListYaw = rospy.get_param('~goalListYaw', '0, 90.0')

    goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","),goalListY.split(","),goalListYaw.split(","))]
    print ('Please 1 to continue: ')
    input = raw_input()
    print (goals)
    r = rospy.Rate(1)
    r.sleep()
    navi = navigation_demo()
    if (input == '1'):
	navi.end()
#1
#        navi.goto(goals[0])
#        rospy.sleep(4) 
        if (find_id == 1 or id == 1 ):
	    #os.system('mplayer %s' % music1_path)
            navi.goto(goals[1])
            rospy.sleep(2)  
	elif (find_id == 2 or id == 2):
	    #os.system('mplayer %s' % music2_path)   
            navi.goto(goals[2])
            rospy.sleep(2)      
        else:
             print "no track"
#2
        navi.goto(goals[9])
        rospy.sleep(4) 
        if (find_id == 3 or id == 3 ):
	    #os.system('mplayer %s' % music3_path)
            navi.goto(goals[3])
            rospy.sleep(2)  
	elif (find_id == 4 or id == 4):
	    #os.system('mplayer %s' % music4_path)   
            navi.goto(goals[4])
            rospy.sleep(2)      
        else:
             print "no track"
#3
        navi.goto(goals[10])
        rospy.sleep(4) 
        if (find_id == 5 or id == 5 ):
	    #os.system('mplayer %s' % music5_path)
            navi.goto(goals[5])
            rospy.sleep(2)  
	elif (find_id == 6 or id == 6):
	    #os.system('mplayer %s' % music6_path)   
            navi.goto(goals[6])
            rospy.sleep(2)      
        else:
             print "no track"
#4
        navi.goto(goals[11])
        rospy.sleep(4) 
        if (find_id == 7 or id == 7 ):
	    #os.system('mplayer %s' % music7_path)
            navi.goto(goals[7])
            rospy.sleep(2)  
	elif (find_id == 8 or id == 8): 
	    #os.system('mplayer %s' % music8_path)  
            navi.goto(goals[8])
            rospy.sleep(2)      
        else:
             print "no track"
        navi.goto(goals[12])
	navi.end()
        r.sleep()

