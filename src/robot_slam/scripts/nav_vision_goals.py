#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""比赛场地视觉点位导航脚本。
依次导航到 4 个视觉识别点 + 4 个任务目标点。
用法: rosrun robot_slam nav_vision_goals.py
"""

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
import sys
reload(sys)
sys.setdefaultencoding('utf-8')


def calc_quat(yaw_rad):
    q = quaternion_from_euler(0, 0, yaw_rad)
    return q[0], q[1], q[2], q[3]


class NavVisionTest(object):
    def __init__(self):
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('[NavTest] Waiting for move_base...')
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo('[NavTest] move_base ready')

    def set_pose(self, x, y, yaw_rad=0.0):
        p = PoseWithCovarianceStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = 'map'
        p.pose.pose.position.x = x
        p.pose.pose.position.y = y
        qx, qy, qz, qw = calc_quat(yaw_rad)
        p.pose.pose.orientation.x = qx
        p.pose.pose.orientation.y = qy
        p.pose.pose.orientation.z = qz
        p.pose.pose.orientation.w = qw
        p.pose.covariance[0] = 0.25
        p.pose.covariance[7] = 0.25
        p.pose.covariance[35] = 0.068
        self.set_pose_pub.publish(p)
        rospy.loginfo('[NavTest] Set initial pose: (%.2f, %.2f, yaw=%.2f)', x, y, yaw_rad)

    def goto(self, x, y, yaw_rad=0.0, timeout=90.0):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        qx, qy, qz, qw = calc_quat(yaw_rad)
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw

        rospy.loginfo('[NavTest] Navigating to (%.2f, %.2f, yaw=%.2f)...', x, y, yaw_rad)
        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_result(rospy.Duration(timeout))

        if not finished:
            self.move_base.cancel_goal()
            rospy.logwarn('[NavTest] TIMEOUT at (%.2f, %.2f)', x, y)
            return False

        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo('[NavTest] Arrived!')
            return True
        else:
            rospy.logwarn('[NavTest] Failed with state: %d', state)
            return False


if __name__ == '__main__':
    rospy.init_node('nav_vision_test')

    # 点位列表: (x, y, yaw_rad, 描述)
    waypoints = [
        # 视觉识别点 (贴墙)
        (0.0,  1.40,  1.57,  'Cell5-北墙视觉'),
        (-1.40, 0.0,  3.14,  'Cell37-西墙视觉'),
        (1.40,  0.0,  0.0,   'Cell45-东墙视觉'),
        (0.0,  -1.40, -1.57, 'Cell77-南墙视觉'),
        # 任务目标点
        (0.0,  0.0,  0.0,   'Cell41-任务'),
        (0.4,  0.4,  0.0,   'Cell33-任务'),
        (0.0,  -0.4, 0.0,   'Cell50-任务'),
        (-0.4, 0.4,  0.0,   'Cell31-任务'),
    ]

    nav = NavVisionTest()

    # 设初始位姿 (Cell 1)
    nav.set_pose(-1.6, 1.6, 0.0)
    rospy.sleep(2)

    print '\n' + '='*50
    print '  比赛场地 8 点位导航测试'
    print '='*50

    for i, (x, y, yaw, desc) in enumerate(waypoints):
        print '\n>>> [%d/8] %s' % (i+1, desc)
        nav.goto(x, y, yaw)
        rospy.sleep(1)

    print '\n' + '='*50
    print '  全部 8 个点位导航完成!'
    print '='*50
