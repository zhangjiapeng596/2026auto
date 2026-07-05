#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""等待 rplidarNode 注册后自动调用 /start_motor 启动雷达电机。

由 rplidar.launch 启动，等待 /start_motor 服务就绪后调用一次即退出。
"""
import rospy
import time
from std_srvs.srv import Empty

if __name__ == '__main__':
    rospy.init_node('start_lidar_motor', anonymous=True)
    max_wait = 15.0
    deadline = time.time() + max_wait

    while time.time() < deadline and not rospy.is_shutdown():
        try:
            rospy.wait_for_service('/start_motor', timeout=1.0)
            start_motor = rospy.ServiceProxy('/start_motor', Empty)
            start_motor()
            rospy.loginfo('[LiDAR Motor] Started successfully')
            break
        except rospy.ROSException:
            rospy.logwarn_throttle(3, '[LiDAR Motor] Waiting for /start_motor service...')
        except Exception as e:
            rospy.logerr('[LiDAR Motor] Failed: %s (retrying...)', e)
        time.sleep(1.0)
    else:
        rospy.logerr('[LiDAR Motor] Timeout after %.0fs', max_wait)
