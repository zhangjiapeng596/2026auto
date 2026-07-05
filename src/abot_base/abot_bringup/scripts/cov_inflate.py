#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""将 /wheel_odom 协方差按维度分别放大后重新发布为 /wheel_odom_inflated。

robot_pose_ekf 没有内置传感器权重参数，仅使用消息中的协方差矩阵决定
各数据源信任度。本节点对轮式里程计协方差做各向异性缩放：
  - 前进方向 (xx)：factor=999x → 不信任麦克纳姆轮前向（打滑）
  - 横移方向 (yy)：factor=999x → 不信任麦克纳姆轮侧滑
  - 旋转 (yawyaw)：factor=1x → 信任旋转（麦克纳姆轮旋转可靠）

数学: 对角缩放矩阵 S = diag(√fwd, √lat, 1, 1, 1, √yaw)
做 congruence transform: C_new = S * C * S^T
等价于 C[i,j] *= √(factor[i] * factor[j])，保持协方差正半定性。
"""
import rospy
from nav_msgs.msg import Odometry


class CovInflate(object):
    def __init__(self):
        # 前进方向 (xx)：轮子打滑，极大降权
        self.fwd_factor = rospy.get_param('~forward_factor', 999.0)
        # 横移方向 (yy)：小辊子侧滑，极大降权
        self.lat_factor = rospy.get_param('~lateral_factor', 999.0)
        # 旋转 (yawyaw)：麦克纳姆轮旋转可信，不降权
        self.yaw_factor = rospy.get_param('~yaw_factor', 1.0)

        # 6 维度的缩放因子: (x, y, z, roll, pitch, yaw)
        # z/roll/pitch 在 2D EKF 中不使用，设为 1.0 (不缩放)
        dim = [
            self.fwd_factor,   # 0: x   (前进)
            self.lat_factor,   # 1: y   (横移)
            1.0,               # 2: z   (不关心)
            1.0,               # 3: roll  (不关心)
            1.0,               # 4: pitch (不关心)
            self.yaw_factor,   # 5: yaw (旋转)
        ]

        # 预计算 6×6 逐元素缩放因子
        # factor[i,j] = sqrt(dim[i] * dim[j])
        self._pose_factors = [0.0] * 36
        self._twist_factors = [0.0] * 36
        for i in range(6):
            for j in range(6):
                idx = i * 6 + j
                factor = (dim[i] * dim[j]) ** 0.5
                self._pose_factors[idx] = factor
                self._twist_factors[idx] = factor

        self.pub = rospy.Publisher('/wheel_odom_inflated', Odometry, queue_size=5)
        self.sub = rospy.Subscriber('/wheel_odom', Odometry, self._cb)
        rospy.loginfo('[CovInflate] fwd_x%d / lat_x%d / yaw_x%d -> /wheel_odom_inflated',
                      int(self.fwd_factor), int(self.lat_factor), int(self.yaw_factor))

    def _cb(self, msg):
        # covariance 是 tuple（不可变），需构建新 tuple 后赋值
        msg.pose.covariance = tuple(
            c * self._pose_factors[i] for i, c in enumerate(msg.pose.covariance))
        msg.twist.covariance = tuple(
            c * self._twist_factors[i] for i, c in enumerate(msg.twist.covariance))
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('cov_inflate')
    CovInflate()
    rospy.spin()
