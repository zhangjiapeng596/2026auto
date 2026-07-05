#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""amcl_tf_bridge 单元测试：验证 ExtrapolationException 回退行为

当前 bug: _on_amcl_pose 收到 ExtrapolationException 时直接放弃，
导致 amcl_tf_bridge 永远停留在 bootstrap pose，move_base 无法定位。

在 WSL 中运行: source /opt/ros/melodic/setup.bash && python test_amcl_tf_bridge.py
"""
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

import unittest
import mock
import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped


class TestAmclTfBridgeExtrapolationBug(unittest.TestCase):
    """验证 _on_amcl_pose 遇到 ExtrapolationException 时的行为"""

    def setUp(self):
        """创建 AmclTfBridge 实例，注入 mock tf_buffer，完全隔离 ROS"""
        self._patches = [
            mock.patch('rospy.Subscriber'),
            mock.patch('rospy.Timer'),
            mock.patch('rospy.loginfo'),
            mock.patch('rospy.logwarn_throttle'),
            mock.patch('rospy.Time.now', return_value=rospy.Time(2000)),
        ]
        for p in self._patches:
            p.start()

        # Mock tf2_ros
        self.mock_buffer = mock.MagicMock()
        self._tf_patches = [
            mock.patch('tf2_ros.Buffer', return_value=self.mock_buffer),
            mock.patch('tf2_ros.TransformListener'),
            mock.patch('tf2_ros.TransformBroadcaster'),
        ]
        for p in self._tf_patches:
            p.start()

        # 构造 AMCL pose（时间戳很早，比 TF 缓存早）
        self.amcl_msg = PoseWithCovarianceStamped()
        self.amcl_msg.header.stamp = rospy.Time(1000)   # old timestamp
        self.amcl_msg.header.frame_id = 'map'
        self.amcl_msg.pose.pose.position.x = -1.5
        self.amcl_msg.pose.pose.position.y = 1.5
        self.amcl_msg.pose.pose.position.z = 0.0
        self.amcl_msg.pose.pose.orientation.x = 0.0
        self.amcl_msg.pose.pose.orientation.y = 0.0
        self.amcl_msg.pose.pose.orientation.z = 0.0
        self.amcl_msg.pose.pose.orientation.w = 1.0

        from amcl_tf_bridge import AmclTfBridge
        self.bridge = AmclTfBridge()

    def tearDown(self):
        for p in self._patches + self._tf_patches:
            p.stop()

    # ================================================================
    # Test 1: 确认当前 BUG — ExtrapolationException 被吞掉
    # ================================================================
    def test_has_amcl_stays_false_on_extrapolation(self):
        """BUG: _on_amcl_pose 遇到 ExtrapolationException 后 has_amcl=False"""
        self.mock_buffer.lookup_transform.side_effect = \
            tf2_ros.ExtrapolationException(
                'Lookup would require extrapolation into the past. '
                'Requested time 1000.000 but the earliest data is at '
                'time 2000.000')

        self.bridge._on_amcl_pose(self.amcl_msg)

        self.assertFalse(self.bridge.has_amcl,
                         'BUG 确认: ExtrapolationException 后 has_amcl=False，'
                         'map->odom 永远停留在 bootstrap pose')

    # ================================================================
    # Test 2: 期望行为 — 应该回退到 rospy.Time(0)
    # ================================================================
    def test_should_recover_from_extrapolation(self):
        """期望: ExtrapolationException → 回退 rospy.Time(0) → has_amcl=True"""
        from geometry_msgs.msg import TransformStamped

        odom_tf = TransformStamped()
        odom_tf.header.stamp = rospy.Time(2000)
        odom_tf.header.frame_id = 'odom'
        odom_tf.child_frame_id = 'base_link'
        odom_tf.transform.rotation.w = 1.0  # identity quaternion

        # 第1次: ExtrapolationException, 第2次: 成功
        self.mock_buffer.lookup_transform.side_effect = [
            tf2_ros.ExtrapolationException('extrapolation'),
            odom_tf
        ]

        self.bridge._on_amcl_pose(self.amcl_msg)

        self.assertTrue(self.bridge.has_amcl,
                        '期望: 回退到 rospy.Time(0) 后 has_amcl=True，'
                        '当前代码做不到（需修复）')


if __name__ == '__main__':
    unittest.main()
