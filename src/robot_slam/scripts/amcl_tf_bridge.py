#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""AMCL TF 桥接：收到 /amcl_pose 时计算一次 map→odom，10Hz 持续发布。
解决 AMCL 1.16.7 不发布 map→odom TF 的 bug。

map→odom = map→base_link * inverse(odom→base_link)
在 _on_amcl_pose 回调中计算保证时间对齐，timer 只负责持续发布。
"""
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped

INIT_X = -1.5
INIT_Y = 1.5
INIT_YAW = 1.5708


def _invert_transform(t, q):
    """inverse(T): t_inv = conj(q) * (-t) * q, q_inv = conj(q)"""
    q_inv = tft.quaternion_conjugate(q)
    t_neg = [-t[0], -t[1], -t[2], 0]
    t_inv = list(tft.quaternion_multiply(
        tft.quaternion_multiply(q_inv, t_neg), q
    ))[:3]
    return t_inv, q_inv


def _compose_transforms(t1, q1, t2, q2):
    """T3 = T1 * T2"""
    q3 = tft.quaternion_multiply(q1, q2)
    t2q = [t2[0], t2[1], t2[2], 0]
    t2_rot = list(tft.quaternion_multiply(
        tft.quaternion_multiply(q1, t2q),
        tft.quaternion_conjugate(q1)
    ))[:3]
    t3 = [t1[0] + t2_rot[0], t1[1] + t2_rot[1], t1[2] + t2_rot[2]]
    return t3, q3


def _pose_to_tq(pose):
    p = pose.position
    o = pose.orientation
    t = [p.x, p.y, p.z]
    q = [o.x, o.y, o.z, o.w]
    return t, q


def _tq_to_transform(t, q, stamp):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = 'map'
    msg.child_frame_id = 'odom'
    msg.transform.translation.x = t[0]
    msg.transform.translation.y = t[1]
    msg.transform.translation.z = t[2]
    msg.transform.rotation.x = q[0]
    msg.transform.rotation.y = q[1]
    msg.transform.rotation.z = q[2]
    msg.transform.rotation.w = q[3]
    return msg


class AmclTfBridge(object):
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                                     self._on_amcl_pose)

        # 初始引导位姿
        q0 = tft.quaternion_from_euler(0, 0, INIT_YAW)
        self.current_tf = _tq_to_transform(
            [INIT_X, INIT_Y, 0.0], q0, rospy.Time.now())
        self.has_amcl = False

        rospy.loginfo('[AmclTfBridge] Bootstrapping at (%.2f, %.2f, %.2f)',
                      INIT_X, INIT_Y, INIT_YAW)

        # 10Hz 持续发布，timer 只负责发布，不做计算
        self.timer = rospy.Timer(rospy.Duration(0.1), self._publish_tf)

    def _on_amcl_pose(self, msg):
        """收到 AMCL 位姿时，在回调中同步计算 map→odom（保证时间对齐）

        启动阶段 AMCL 首条 pose 时间戳可能早于 odom→base_link TF 缓存，
        此时回退到 rospy.Time(0) 取最新 TF，打破 "无定位→不动→无更新" 死锁。
        """
        try:
            # 用 AMCL 消息时间戳查里程计（精确时间对齐）
            odom_to_base = self.tf_buffer.lookup_transform(
                'odom', 'base_link', msg.header.stamp, rospy.Duration(0.3))
        except tf2_ros.ExtrapolationException:
            # 启动阶段时间戳回溯失败 → 回退到最新可用 TF
            try:
                odom_to_base = self.tf_buffer.lookup_transform(
                    'odom', 'base_link', rospy.Time(0), rospy.Duration(0.3))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn_throttle(5,
                    '[AmclTfBridge] TF lookup failed (fallback): %s', e)
                return
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            rospy.logwarn_throttle(5, '[AmclTfBridge] TF lookup failed: %s', e)
            return

        map_t, map_q = _pose_to_tq(msg.pose.pose)
        odom_t = [odom_to_base.transform.translation.x,
                  odom_to_base.transform.translation.y,
                  odom_to_base.transform.translation.z]
        odom_q = [odom_to_base.transform.rotation.x,
                  odom_to_base.transform.rotation.y,
                  odom_to_base.transform.rotation.z,
                  odom_to_base.transform.rotation.w]

        inv_t, inv_q = _invert_transform(odom_t, odom_q)
        result_t, result_q = _compose_transforms(map_t, map_q, inv_t, inv_q)

        self.current_tf = _tq_to_transform(result_t, result_q, rospy.Time.now())
        if not self.has_amcl:
            self.has_amcl = True
            rospy.loginfo('[AmclTfBridge] First AMCL update: map->odom = (%.3f, %.3f, %.3f)',
                          result_t[0], result_t[1], result_t[2])

    def _publish_tf(self, event):
        """仅发布当前缓存的 map→odom，不做计算"""
        self.current_tf.header.stamp = rospy.Time.now()
        self.br.sendTransform(self.current_tf)


if __name__ == '__main__':
    rospy.init_node('amcl_tf_bridge')
    AmclTfBridge()
    rospy.spin()
