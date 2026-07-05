#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ABOT 导航远程监控 — 订阅所有关键话题，聚合成 JSON 行输出。
运行方式:
  ssh abot@172.16.25.154 'source ... && rosrun robot_slam nav_monitor.py'
Claude Code 用 Monitor 工具 watch 输出流，实时诊断异常。
"""
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import rospy
import json
import math
import time
from collections import deque

# 话题
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String

TOPIC_AMCL = '/amcl_pose'
TOPIC_SCAN = '/scan_filtered'
TOPIC_ODOM = '/odom'
TOPIC_CMD = '/cmd_vel'
TOPIC_GOAL = '/move_base/goal'
TOPIC_STATUS = '/move_base/status'
TOPIC_SAFETY = '/safety_status'
TOPIC_MISSION = '/mission_state'

# 异常阈值
SCAN_STALE_S = 2.0         # 雷达断流
AMCL_STALE_S = 3.0         # 定位断流
AMCL_STD_MAX = 0.8          # 定位发散
ODOM_STALE_S = 1.5          # 里程计断流
STUCK_DIST_M = 0.02         # 卡死判定位移 (m)
STUCK_TIME_S = 10.0         # 卡死判定时长 (s)
MOTIONLESS_TIME_S = 20.0    # 导航中无运动超时
STATE_STALE_S = 30.0        # 状态机不跳转
MAP_LOAD_TIMEOUT_S = 8.0    # 地图加载等待

# 每 STATUS_INTERVAL_S 输出一行状态摘要
STATUS_INTERVAL_S = 2.0

# 上次运动位姿
_last_motion = {'x': None, 'y': None, 'yaw': 0.0, 't': 0.0, 'stuck_warned': False}


def yaw_from_quat(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))


def now():
    return rospy.Time.now().to_sec()


class NavMonitor(object):
    def __init__(self):
        # ---- 接收时间戳 (用于断流检测) ----
        self.t_scan = 0.0
        self.t_amcl = 0.0
        self.t_odom = 0.0
        self.t_cmd = 0.0
        self.t_map = 0.0
        self.t_safety = 0.0
        self.t_mission = 0.0
        self.t_start = now()

        # ---- 最新数据 ----
        self.amcl_std_x = -1.0
        self.amcl_std_y = -1.0
        self.amcl_std_yaw = -1.0
        self.amcl_x = 0.0
        self.amcl_y = 0.0
        self.amcl_yaw = 0.0

        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_vw = 0.0

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vw = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_active = False

        self.nav_status = -1  # -1=unknown
        self.nav_status_text = 'UNKNOWN'

        self.safety_msg = ''
        self.mission_state = 'UNKNOWN'
        self.last_mission_state = 'UNKNOWN'
        self.t_mission_changed = 0.0

        # 历史位置 (卡死检测)
        self.pos_history = deque(maxlen=20)

        # ---- 一次性事件 ----
        self.map_loaded = False
        self._emitted = set()  # 已发过的告警去重

        # ---- 订阅 ----
        rospy.Subscriber(TOPIC_ODOM, Odometry, self._cb_odom)
        rospy.Subscriber(TOPIC_SCAN, LaserScan, self._cb_scan)
        rospy.Subscriber(TOPIC_AMCL, PoseWithCovarianceStamped, self._cb_amcl)
        rospy.Subscriber(TOPIC_CMD, Twist, self._cb_cmd)
        rospy.Subscriber(TOPIC_GOAL, MoveBaseActionGoal, self._cb_goal)
        rospy.Subscriber(TOPIC_STATUS, GoalStatusArray, self._cb_status)
        rospy.Subscriber(TOPIC_SAFETY, String, self._cb_safety)
        rospy.Subscriber(TOPIC_MISSION, String, self._cb_mission)

        # /map 只取一次确认加载
        rospy.Subscriber('/map', rospy.AnyMsg, self._cb_map)

        # ---- 定期输出 ----
        self._timer = rospy.Timer(rospy.Duration(STATUS_INTERVAL_S), self._tick)

    # ============================
    #  回调
    # ============================
    def _cb_scan(self, msg):
        self.t_scan = now()

    def _cb_amcl(self, msg):
        self.t_amcl = now()
        cov = msg.pose.covariance
        self.amcl_std_x = math.sqrt(max(cov[0], 0))
        self.amcl_std_y = math.sqrt(max(cov[7], 0))
        self.amcl_std_yaw = math.sqrt(max(cov[35], 0))
        self.amcl_x = msg.pose.pose.position.x
        self.amcl_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.amcl_yaw = yaw_from_quat(q)

        # 记录位置历史
        self.pos_history.append((self.amcl_x, self.amcl_y, self.t_amcl))

    def _cb_odom(self, msg):
        self.t_odom = now()
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_vy = msg.twist.twist.linear.y
        self.odom_vw = msg.twist.twist.angular.z

    def _cb_cmd(self, msg):
        self.t_cmd = now()
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_vw = msg.angular.z

    def _cb_goal(self, msg):
        self.goal_x = msg.goal.target_pose.pose.position.x
        self.goal_y = msg.goal.target_pose.pose.position.y
        self.goal_active = True
        self._emit('event', 'NAV_GOAL', 'New goal: ({:.2f}, {:.2f})'.format(self.goal_x, self.goal_y))

        # 重置卡死检测
        global _last_motion
        _last_motion = {'x': None, 'y': None, 'yaw': 0.0, 't': 0.0, 'stuck_warned': False}

    def _cb_status(self, msg):
        if msg.status_list:
            s = msg.status_list[-1]
            prev_status = self.nav_status
            self.nav_status = s.status
            self.nav_status_text = s.text
            # 导航结果 — 只在状态变化时触发，避免重复刷屏
            if s.status != prev_status:
                if s.status == 3:  # SUCCEEDED
                    self._emit('event', 'NAV_DONE', 'Goal reached')
                    self.goal_active = False
                elif s.status in (4, 5, 9):  # ABORTED, REJECTED, LOST
                    self._emit('alert', 'NAV_FAIL', 'Goal failed (status={}): {}'.format(s.status, s.text))
                    self.goal_active = False

    def _cb_safety(self, msg):
        self.t_safety = now()
        text = msg.data.strip()
        if text and text != self.safety_msg:
            self.safety_msg = text
            if any(kw in text for kw in ('COLLISION', 'collision', 'EMERGENCY', 'ABORT', 'CRITICAL')):
                self._emit('alert', 'SAFETY', text)
            else:
                self._emit('event', 'SAFETY', text)

    def _cb_mission(self, msg):
        self.t_mission = now()
        new_state = msg.data.strip()
        if new_state and new_state != self.mission_state:
            self.last_mission_state = self.mission_state
            self.mission_state = new_state
            self.t_mission_changed = now()
            self._emit('event', 'MISSION', '{} -> {}'.format(self.last_mission_state, self.mission_state))

            # 重置运动检测
            global _last_motion
            _last_motion = {'x': None, 'y': None, 'yaw': 0.0, 't': 0.0, 'stuck_warned': False}

    def _cb_map(self, msg):
        if not self.map_loaded:
            self.t_map = now()
            self.map_loaded = True
            self._emit('event', 'MAP_LOADED', 'Map received ({:.1f}s after start)'.format(self.t_map - self.t_start))

    # ============================
    #  定时检查
    # ============================
    def _tick(self, event):
        t = now()
        elapsed = t - self.t_start
        alerts = []

        # --- 地图加载 ---
        if not self.map_loaded and elapsed > MAP_LOAD_TIMEOUT_S:
            alerts.append(('CRITICAL', 'MAP_MISSING', 'Map not loaded after {:.0f}s'.format(elapsed)))

        # --- 雷达断流 ---
        scan_age = t - self.t_scan if self.t_scan > 0 else 999
        if scan_age > SCAN_STALE_S:
            alerts.append(('ERROR', 'LIDAR_STALE', 'LiDAR no data for {:.1f}s'.format(scan_age)))

        # --- 定位断流 ---
        amcl_age = t - self.t_amcl if self.t_amcl > 0 else 999
        if amcl_age > AMCL_STALE_S:
            alerts.append(('ERROR', 'AMCL_STALE', 'AMCL no data for {:.1f}s'.format(amcl_age)))

        # --- 定位发散 ---
        if self.amcl_std_x > AMCL_STD_MAX or self.amcl_std_y > AMCL_STD_MAX:
            alerts.append(('WARN', 'AMCL_DIVERGE',
                           'Covariance high (std_x={:.2f}, std_y={:.2f})'.format(self.amcl_std_x, self.amcl_std_y)))

        # --- 里程计断流 ---
        odom_age = t - self.t_odom if self.t_odom > 0 else 999
        if odom_age > ODOM_STALE_S:
            alerts.append(('ERROR', 'ODOM_STALE', 'Odometry no data for {:.1f}s'.format(odom_age)))

        # --- 导航卡死 ---
        if self.goal_active and len(self.pos_history) >= 5:
            x0, y0, t0 = self.pos_history[0]
            x1, y1, t1 = self.pos_history[-1]
            dt_pos = t1 - t0
            if dt_pos > 0:
                dist = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
                if dist < STUCK_DIST_M and dt_pos > STUCK_TIME_S:
                    alerts.append(('ERROR', 'NAV_STUCK',
                                   'Stuck {:.1f}s, moved {:.3f}m at ({:.2f},{:.2f})'.format(dt_pos, dist, x1, y1)))

        # --- 底盘无运动 ---
        if 'NAVIGATE' in self.mission_state.upper() or 'NAVIGATE' in self.mission_state:
            cmd_speed = math.sqrt(self.cmd_vx**2 + self.cmd_vy**2)
            cmd_age = t - self.t_cmd if self.t_cmd > 0 else 0
            if cmd_speed < 0.005 and cmd_age > MOTIONLESS_TIME_S:
                alerts.append(('WARN', 'NO_MOTION',
                               'No cmd_vel for {:.1f}s in state {}'.format(cmd_age, self.mission_state)))

        # --- 状态机卡住 ---
        mission_age = t - self.t_mission_changed if self.t_mission_changed > 0 else elapsed
        if mission_age > STATE_STALE_S and self.mission_state not in ('DONE', 'UNKNOWN'):
            alerts.append(('WARN', 'STATE_STALE',
                           'State {} unchanged for {:.0f}s'.format(self.mission_state, mission_age)))

        # --- 输出 ---
        # 告警
        for level, code, msg in alerts:
            key = '{}-{}'.format(code, msg[:20])
            if key not in self._emitted:
                self._emitted.add(key)
                self._out({'ts': t, 'type': 'alert', 'level': level, 'code': code, 'msg': msg})

        # 定期状态行
        self._out({
            'ts': t,
            'type': 'status',
            'elapsed': round(elapsed, 1),
            'mission': self.mission_state,
            'nav': self.nav_status_text,
            'goal_active': self.goal_active,
            'amcl_pos': [round(self.amcl_x, 3), round(self.amcl_y, 3), round(self.amcl_yaw, 2)],
            'amcl_std': [round(self.amcl_std_x, 3), round(self.amcl_std_y, 3), round(self.amcl_std_yaw, 3)],
            'amcl_age': round(amcl_age, 1),
            'scan_age': round(scan_age, 1),
            'odom_age': round(odom_age, 1),
            'cmd_vel': [round(self.cmd_vx, 3), round(self.cmd_vy, 3), round(self.cmd_vw, 3)],
            'odom_vel': [round(self.odom_vx, 3), round(self.odom_vy, 3), round(self.odom_vw, 3)],
            'safety': self.safety_msg,
        })

    def _emit(self, category, code, msg):
        self._out({'ts': now(), 'type': category, 'code': code, 'msg': msg})

    @staticmethod
    def _out(obj):
        sys.stdout.write(json.dumps(obj, ensure_ascii=False) + '\n')
        sys.stdout.flush()


if __name__ == '__main__':
    rospy.init_node('nav_monitor', anonymous=True)
    NavMonitor()
    rospy.spin()
