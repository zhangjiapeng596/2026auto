#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    rospy.init_node('publish_initialpose_once', anonymous=False)

    x = rospy.get_param('~x', -1.6)
    y = rospy.get_param('~y', 1.6)
    yaw = rospy.get_param('~yaw', 1.5708)
    delay = rospy.get_param('~delay', 3.0)
    repeat = int(rospy.get_param('~repeat', 5))

    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
    rospy.sleep(delay)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0
    q = tft.quaternion_from_euler(0.0, 0.0, yaw)
    msg.pose.pose.orientation.x = q[0]
    msg.pose.pose.orientation.y = q[1]
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]
    msg.pose.covariance[0] = 0.04
    msg.pose.covariance[7] = 0.04
    msg.pose.covariance[35] = 0.017

    rate = rospy.Rate(2)
    for _ in range(max(1, repeat)):
        if rospy.is_shutdown():
            break
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo('Published initialpose x=%.3f y=%.3f yaw=%.4f', x, y, yaw)


if __name__ == '__main__':
    main()
