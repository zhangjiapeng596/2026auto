#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""地面巡航比赛任务状态机。统一管理比赛流程。"""
import os
import sys
# Python 2 中文兼容：设置默认编码为 UTF-8
reload(sys)
sys.setdefaultencoding('utf-8')
import rospy
import time
import json
import math
import threading

from std_msgs.msg import String, Empty
from std_srvs.srv import Empty as EmptySrv
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib
import tf as ros_tf
import tf.transformations as tft

# Add common scripts to path for imports
_common_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'common', 'scripts')
if os.path.isdir(_common_dir):
    sys.path.insert(0, _common_dir)

from config_loader import load_config, get_cell_center_xy, check_footprint_in_region
from mission_logger import MissionLogger


class MissionState(object):
    """Python 2 兼容的任务状态。支持 == 比较、.value 访问、set 成员。"""

    def __init__(self, value):
        self.value = value

    def __eq__(self, other):
        if isinstance(other, MissionState):
            return self.value == other.value
        return NotImplemented

    def __ne__(self, other):
        result = self.__eq__(other)
        if result is NotImplemented:
            return result
        return not result

    def __hash__(self):
        return hash(self.value)

    def __repr__(self):
        return 'MissionState(%r)' % self.value


# 正常状态实例
MissionState.IDLE = MissionState('IDLE')
MissionState.WAIT_FOR_WAKEUP = MissionState('WAIT_FOR_WAKEUP')
MissionState.START_ANNOUNCE = MissionState('START_ANNOUNCE')
MissionState.NAVIGATE_TO_FINISH = MissionState('NAVIGATE_TO_FINISH')
MissionState.ARRIVE_FINISH = MissionState('ARRIVE_FINISH')
MissionState.FINISH_ANNOUNCE = MissionState('FINISH_ANNOUNCE')
MissionState.DONE = MissionState('DONE')
# 异常状态实例
MissionState.ABORT_COLLISION_RISK = MissionState('ABORT_COLLISION_RISK')
MissionState.ABORT_TIMEOUT = MissionState('ABORT_TIMEOUT')
MissionState.ABORT_LOCALIZATION_LOST = MissionState('ABORT_LOCALIZATION_LOST')
MissionState.ABORT_PERCEPTION_FAILED = MissionState('ABORT_PERCEPTION_FAILED')
MissionState.ABORT_NAVIGATION_FAILED = MissionState('ABORT_NAVIGATION_FAILED')
MissionState.MANUAL_STOP_REQUESTED = MissionState('MANUAL_STOP_REQUESTED')


def _task_image_state(cls, phase, step_name):
    """构建带序号的阶段状态，如 SEARCH_TASK_IMAGE_1"""
    return MissionState('{}_{}'.format(step_name, phase))


MissionState.task_image_state = classmethod(_task_image_state)


class MissionStateMachine(object):
    """比赛任务状态机。统一管理比赛流程。"""

    ABORT_STATES = {
        MissionState.ABORT_COLLISION_RISK,
        MissionState.ABORT_TIMEOUT,
        MissionState.ABORT_LOCALIZATION_LOST,
        MissionState.ABORT_PERCEPTION_FAILED,
        MissionState.ABORT_NAVIGATION_FAILED,
        MissionState.MANUAL_STOP_REQUESTED,
    }

    TASK_PHASE_STEPS = ['SEARCH_TASK_IMAGE', 'NAVIGATE_TO_VISION',
                        'RECOGNIZE_TASK_IMAGE', 'NAVIGATE_TO_TASK', 'ARRIVE_TASK', 'ANNOUNCE_TASK',
                        'SKIP_TASK']

    def __init__(self, sim_mode=False):
        self.sim_mode = sim_mode
        self.logger = MissionLogger()

        # 加载配置
        self.field_cfg = load_config('competition_field.yaml')
        self.mission_cfg = load_config('mission.yaml')
        self.voice_cfg = load_config('voice_text.yaml')
        self.robot_cfg = load_config('robot.yaml')

        # 状态机
        self.state = MissionState.IDLE
        self.task_index = 0           # 当前序号 (0-3，共 4 个)
        self.target_cell = None        # 当前目标任务点网格号
        self.task_cells_done = []      # 已完成的任务点列表
        self.vision_phase = True       # True=视觉采集阶段, False=任务执行阶段
        self.collected_targets = []    # 视觉阶段收集的目标格子号列表
        self.state_start_time = time.time()
        self.mission_start_time = time.time()
        self.perception_retry_count = 0
        self.navigation_retry_count = 0
        self.task_skip_count = 0         # 不可达任务点跳过计数
        self.footprint_retry_count = 0  # 防止 footprint 检查死循环
        self.finish_nav_retry_count = 0
        self.recognition_in_progress = False
        self.seen_image_ids = []  # 已识别的图像 ID，防止重复
        self.last_nav_goal = None  # 最近一次发送给 move_base 的 (x, y, yaw)

        # ROS 接口
        # TF 位姿 fallback: AMCL /amcl_pose 发布有 bug (仅发1条), 用 TF 作为备胎
        self.tf_listener = ros_tf.TransformListener()

        # 先创建 heartbeat publisher + Timer，防止 safety 在 wait_for_server 期间误判超时
        self.heartbeat_pub = rospy.Publisher('/mission_heartbeat', String, queue_size=1)
        self.heartbeat_pub.publish(String(data='init'))
        self.heartbeat_timer = rospy.Timer(rospy.Duration(2.0), self._publish_heartbeat)
        self.voice_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', EmptySrv)
        rospy.loginfo('[Mission] Waiting for move_base action server...')
        connected = self.move_base_client.wait_for_server(rospy.Duration(10.0))
        if not connected:
            rospy.logwarn('[Mission] move_base not available, navigation disabled')

        # 识别结果跟踪
        self.vision_result = None
        self.vision_result_event = threading.Event()

        # 当前位姿（用于精准到点判定，必须是 map 坐标系）
        self.current_pose = None  # (x, y, yaw) in map frame
        # 位姿源优先级仲裁：数值越大优先级越高，低优先级源不得覆盖高优先级源。
        #   amcl_pose(实机 map 系全局定位) > abot/pose(预留) > odom(仿真兜底)
        # 仿真: sim_robot 只发 /odom 且已是 map 系真值, 收不到 /amcl_pose, 自动用 odom, 行为不变。
        # 实机: AMCL 发 /amcl_pose(map 系), 用它做 footprint 判定才坐标系一致。
        self._POSE_RANK = {'odom': 1, 'odom_tf': 2, 'abot_pose': 2, 'amcl_pose': 3}
        self._pose_source_rank = 0  # 当前 current_pose 的来源等级, 0=尚无位姿
        # AMCL 协方差监控状态 (仅实机有 /amcl_pose 时更新; 仿真用 odom, 这些保持初值不触发)
        self._amcl_pos_std = 0.0
        self._amcl_yaw_std = 0.0
        self._amcl_last_time = 0.0
        self._localization_lost_since = None  # 定位发散起始时刻, None=未发散
        # 位姿写锁：必须在注册位姿订阅之前创建，否则回调线程可能早于锁初始化触发
        self._pose_lock = threading.Lock()

        # 订阅
        rospy.Subscriber('/start', String, self._on_wakeup)
        rospy.Subscriber('/vision_result', String, self._on_vision_result)
        rospy.Subscriber('/safety_status', String, self._on_safety_status)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self._on_amcl_pose)
        rospy.Subscriber('/abot/pose', PoseStamped, self._on_pose)
        rospy.Subscriber('/odom', Odometry, self._on_odom)
        rospy.Subscriber('/tts_done', String, self._on_tts_done)

        # TTS 播报完成事件 + 防 stale 的 pending 文本
        self.tts_done_event = threading.Event()
        self.tts_done_event.set()  # 初始非等待状态
        self._tts_pending = None   # 正在等待的播报文本，防陈旧 /tts_done 误触发

        rospy.loginfo('[Mission] State machine initialized, sim_mode=%s', sim_mode)

    def _publish_heartbeat(self, event):
        """心跳定时器回调。"""
        self.heartbeat_pub.publish(String(data='alive'))

    def _check_aborted(self):
        """检查是否已被安全监控或其他线程设为 abort 状态。"""
        return self.state in self.ABORT_STATES

    def transition(self, new_state):
        """状态跳转，记录日志。"""
        old = self.state.value
        self.state = new_state
        self.state_start_time = time.time()
        rospy.loginfo('[Mission] %s -> %s', old, new_state.value)
        self.logger.log_state_transition(old, new_state.value)

    # ========== 主循环 ==========

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        self.mission_start_time = time.time()
        self.transition(MissionState.WAIT_FOR_WAKEUP)

        while not rospy.is_shutdown():
            try:
                # heartbeat 由 rospy.Timer 独立发布
                self._check_global_timeouts()

                if self.state == MissionState.WAIT_FOR_WAKEUP:
                    self._handle_wait_for_wakeup()
                elif self.state == MissionState.START_ANNOUNCE:
                    self._handle_start_announce()
                elif self.state == MissionState.NAVIGATE_TO_FINISH:
                    self._handle_navigate_to_finish()
                elif self.state == MissionState.ARRIVE_FINISH:
                    self._handle_arrive_finish()
                elif self.state == MissionState.FINISH_ANNOUNCE:
                    self._handle_finish_announce()
                elif self.state == MissionState.DONE:
                    rospy.loginfo('[Mission] All tasks completed successfully!')
                    break
                elif self.state in self.ABORT_STATES:
                    self._handle_abort()
                    break
                else:
                    self._handle_task_phase()

                rate.sleep()
            except Exception as e:
                import traceback
                rospy.logerr('[Mission] Unhandled exception: %s', str(e))
                rospy.logerr('[Mission] Traceback:\n%s', traceback.format_exc())
                rospy.sleep(1.0)
                raise

    # ========== Callbacks ==========

    def _on_wakeup(self, msg):
        if self.state == MissionState.WAIT_FOR_WAKEUP:
            if self.sim_mode and msg.data == 'sim_wakeup':
                rospy.loginfo('[Mission] Simulated wakeup received')
                self.transition(MissionState.START_ANNOUNCE)
            elif not self.sim_mode and msg.data == 'True':
                rospy.loginfo('[Mission] Wake word detected!')
                self.transition(MissionState.START_ANNOUNCE)

    def _on_vision_result(self, msg):
        if not self.recognition_in_progress:
            return
        try:
            result = json.loads(msg.data)
            confidence = result.get('confidence', 0)
            min_conf = self.mission_cfg['confidence']['min_confidence']
            image_id = result.get('image_id', '')

            rospy.loginfo('[Mission] Vision result: cell=%s, confidence=%.2f, id=%s',
                          result.get('target_cell'), confidence, image_id)

            phase = self.task_index + 1
            if image_id and image_id in self.seen_image_ids:
                rospy.logwarn('[Mission] Phase %d: Duplicate image %s, retrying...', phase, image_id)
                self._retry_perception(phase)
                return

            if confidence >= min_conf:
                self.target_cell = result['target_cell']
                if image_id:
                    self.seen_image_ids.append(image_id)
                self.logger.log_perception(result)
                self.perception_retry_count = 0
                self.recognition_in_progress = False
                self.vision_result = result
                self.vision_result_event.set()
            else:
                rospy.logwarn('[Mission] Low confidence %.2f < %.2f, retrying...',
                              confidence, min_conf)
                self._retry_perception(phase)
        except (ValueError, KeyError, TypeError) as e:
            rospy.logerr('[Mission] Invalid vision result: %s', str(e))

    def _update_pose(self, source, x, y, yaw):
        """按优先级仲裁更新 current_pose：高优先级源一旦出现，低优先级源不再覆盖。

        实机用 amcl_pose(map 系)，仿真用 odom(已是 map 系真值)，保证 footprint
        判定始终在 map 坐标系下进行。
        """
        rank = self._POSE_RANK[source]
        with self._pose_lock:
            if rank < self._pose_source_rank:
                return  # 已有更高优先级位姿源，忽略本次低优先级数据
            self.current_pose = (x, y, yaw)
            if rank > self._pose_source_rank:
                self._pose_source_rank = rank
                rospy.loginfo('[Mission] Pose source -> %s (map frame)', source)

    def _on_amcl_pose(self, msg):
        """接收 AMCL 定位结果（/amcl_pose，map 系全局位姿）。实机精准到点的首选位姿源。

        同时记录协方差(6x6, 行优先): cov[0]=var(x), cov[7]=var(y), cov[35]=var(yaw)，
        供定位丢失监控判断粒子群是否发散。
        """
        q = msg.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        cov = msg.pose.covariance
        try:
            pos_std = math.sqrt(max(cov[0], 0.0) + max(cov[7], 0.0))  # x,y 合成标准差
            yaw_std = math.sqrt(max(cov[35], 0.0))
        except (IndexError, ValueError):
            pos_std, yaw_std = 0.0, 0.0
        with self._pose_lock:
            self._amcl_pos_std = pos_std
            self._amcl_yaw_std = yaw_std
            self._amcl_last_time = time.time()
        self._update_pose('amcl_pose', msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _on_pose(self, msg):
        """接收机器人当前位姿（/abot/pose topic，预留扩展位姿源）。"""
        q = msg.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._update_pose('abot_pose', msg.pose.position.x, msg.pose.position.y, yaw)

    def _on_odom(self, msg):
        """接收里程计数据，作为备选位姿来源（仿真兜底；实机有 amcl_pose 时不采用）。"""
        q = msg.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._update_pose('odom', msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _on_tts_done(self, msg):
        """TTS 播报完成回调。仅当文本匹配时放行，防止陈旧信号误触发。"""
        if self._tts_pending is not None and msg.data == self._tts_pending:
            rospy.loginfo('[Mission] TTS done signal received (matched)')
            self._tts_pending = None
            self.tts_done_event.set()
        else:
            rospy.logwarn('[Mission] TTS done signal ignored (stale or mismatched): expected=%s, got=%s',
                          self._tts_pending, msg.data[:30])

    def _get_current_pose(self):
        """返回最新的机器人位姿 (x, y, yaw)，若无数据则返回 (None, None, None)。"""
        with self._pose_lock:
            if self.current_pose is not None:
                return self.current_pose
        return None, None, None

    def _angle_diff(self, a, b):
        """返回两个角度之间的最短差值绝对值。"""
        return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

    def _pose_near_goal(self, x, y, yaw, xy_tolerance, yaw_tolerance):
        """用当前位姿判断是否已经足够接近目标点。"""
        rx, ry, ryaw = self._get_current_pose()
        if rx is None:
            return False
        dist = math.sqrt((rx - x)**2 + (ry - y)**2)
        yaw_diff = self._angle_diff(ryaw, yaw)
        return dist <= xy_tolerance and yaw_diff <= yaw_tolerance

    def _get_task_footprint_status(self):
        """返回当前任务点 footprint 校验结果。"""
        rx, ry, ryaw = self._get_current_pose()
        if rx is None:
            return None, None
        footprint = self.robot_cfg.get('footprint',
            [[-0.175, -0.15], [-0.175, 0.15], [0.175, 0.15], [0.175, -0.15]])
        in_region, detail = check_footprint_in_region(
            rx, ry, ryaw, footprint, self.target_cell, self.field_cfg)
        detail['robot_pose'] = (rx, ry, ryaw)
        return in_region, detail

    def _get_task_center_distance(self):
        """返回当前位姿到任务点中心的距离。"""
        rx, ry, _ = self._get_current_pose()
        if rx is None or self.target_cell is None:
            return None, None
        cx, cy = get_cell_center_xy(self.target_cell, self.field_cfg)
        dist = math.sqrt((rx - cx)**2 + (ry - cy)**2)
        return dist, (cx, cy)

    def _on_safety_status(self, msg):
        """接收安全监控状态。"""
        if not msg.data.startswith('ESTOP'):
            return
        reason = msg.data[len('ESTOP:'):] if msg.data.startswith('ESTOP:') else ''
        rospy.logerr('[Mission] Safety ESTOP received: %s', msg.data)

        if reason == 'collision_risk':
            self.transition(MissionState.ABORT_COLLISION_RISK)
        elif reason == 'manual':
            self.transition(MissionState.MANUAL_STOP_REQUESTED)
        elif reason == 'heartbeat_lost':
            self.transition(MissionState.ABORT_TIMEOUT)
        elif reason == 'no_motion_after_start':
            self.transition(MissionState.ABORT_TIMEOUT)
        else:
            self.transition(MissionState.ABORT_TIMEOUT)

    # ========== Phase Handlers ==========

    def _handle_wait_for_wakeup(self):
        rospy.loginfo_throttle(5, '[Mission] Waiting for wake word...')
        elapsed = time.time() - self.state_start_time
        # 仿真模式下延迟 5s 自动触发；真实比赛模式下 30s 后按唤醒成功兜底。
        if self.sim_mode and elapsed > 5.0:
            rospy.loginfo('[Mission] Auto-wakeup triggered (sim mode)')
            self.transition(MissionState.START_ANNOUNCE)
        elif (not self.sim_mode) and elapsed > 30.0:
            rospy.loginfo('[Mission] Auto-wakeup triggered (30s fallback)')
            self.transition(MissionState.START_ANNOUNCE)

    def _handle_start_announce(self):
        text = self.voice_cfg['voice_text']['wakeup_detected']
        self._speak(text)
        if self._check_aborted():
            return
        self.logger.log_voice(text, 'start')
        # AMCL 冷启动收敛旋转: 原地旋转产生多方向激光配准，加速粒子群收敛 (~5s)
        if not self.sim_mode:
            self._convergence_rotate()
            self._startup_backoff()
        if self._check_aborted():
            return  # 旋转期间收到安全 ESTOP，保留 abort 状态
        self.task_index = 0
        self.perception_retry_count = 0
        self.vision_phase = True
        self.collected_targets = []
        self.target_cell = None
        phase = self.task_index + 1
        self.transition(MissionState.task_image_state(phase, 'SEARCH_TASK_IMAGE'))

    def _handle_task_phase(self):
        """处理 4 轮任务中的当前阶段。"""
        phase = self.task_index + 1

        current_step = None
        for step in self.TASK_PHASE_STEPS:
            expected_state = MissionState.task_image_state(phase, step)
            if self.state == expected_state:
                current_step = step
                break

        if current_step is None:
            rospy.logerr('[Mission] Unknown task state: %s', self.state.value)
            return

        if current_step == 'SEARCH_TASK_IMAGE':
            self._handle_search_task_image(phase)
        elif current_step == 'NAVIGATE_TO_VISION':
            self._handle_navigate_to_vision(phase)
        elif current_step == 'RECOGNIZE_TASK_IMAGE':
            self._handle_recognize_task_image(phase)
        elif current_step == 'NAVIGATE_TO_TASK':
            self._handle_navigate_to_task(phase)
        elif current_step == 'ARRIVE_TASK':
            self._handle_arrive_task(phase)
        elif current_step == 'ANNOUNCE_TASK':
            self._handle_announce_task(phase)
        elif current_step == 'SKIP_TASK':
            self._handle_skip_task(phase)

    def _handle_search_task_image(self, phase):
        """发送导航目标到视觉位置，然后切换到导航等待状态。"""
        vision_cells = self.field_cfg.get('vision_positions', [5, 37, 45, 77])
        vision_cell = vision_cells[phase - 1]
        x, y = get_cell_center_xy(vision_cell, self.field_cfg)

        # 获取车头朝向 (对墙拍照)
        v2t = self.field_cfg.get('vision_to_task', {})
        vinfo = v2t.get(vision_cell, {})
        if isinstance(vinfo, dict):
            yaw = vinfo.get('yaw_rad', 0.0)
            offset_m = vinfo.get('offset_m', 0.0)
        else:
            yaw = 0.0  # 兼容旧格式 (纯数字)
            offset_m = 0.0

        # 向远离围墙方向退后 offset_m (避免车体碰撞墙角)
        if offset_m > 0:
            x -= offset_m * math.cos(yaw)
            y -= offset_m * math.sin(yaw)

        rospy.loginfo('[Mission] Phase %d: Navigating to vision position cell %d (%.3f, %.3f, yaw=%.2f)%s',
                      phase, vision_cell, x, y, yaw,
                      (' offset=%.2fm' % offset_m) if offset_m > 0 else '')

        # 先播报再发导航目标，语音与动作同步开始
        text = self.voice_cfg['voice_text']['task_image_searching'].format(index=phase)
        self._speak(text)
        self._stop_robot()
        self._send_nav_goal(x, y, yaw)
        self.transition(MissionState.task_image_state(phase, 'NAVIGATE_TO_VISION'))

    def _handle_navigate_to_vision(self, phase):
        """轮询等待机器人到达视觉位置，到达后切换到 RECOGNIZE。

        与 _handle_recognize_task_image 拆分：导航归导航，识别归识别。
        """
        timeout_s = self.mission_cfg['timeouts'].get('navigation_goal_timeout_s', 60)
        deadline = time.time() + timeout_s
        arrived = False

        while time.time() < deadline:
            if self._check_aborted():
                self.move_base_client.cancel_goal()
                return
            state = self.move_base_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                nav_cfg = self.mission_cfg.get('navigation', {})
                gx, gy, gyaw = self.last_nav_goal if self.last_nav_goal is not None else (None, None, None)
                if gx is not None and self._pose_near_goal(
                        gx, gy, gyaw,
                        nav_cfg.get('vision_xy_tolerance_m', 0.04),
                        nav_cfg.get('vision_yaw_tolerance_rad', 0.12)):
                    rospy.loginfo('[Mission] Phase %d: Vision position reached (move_base success + pose verified)',
                                  phase)
                    arrived = True
                    break
                rx, ry, ryaw = self._get_current_pose()
                if rx is not None and gx is not None:
                    dist = math.sqrt((rx - gx)**2 + (ry - gy)**2)
                    yaw_diff = self._angle_diff(ryaw, gyaw)
                    rospy.logwarn('[Mission] Phase %d: move_base succeeded but vision pose not verified '
                                  '(dist=%.3f, yaw_diff=%.3f), retrying',
                                  phase, dist, yaw_diff)
                else:
                    rospy.logwarn('[Mission] Phase %d: move_base succeeded but no pose for vision verification, retrying',
                                  phase)
                self._retry_perception(phase)
                return
            if self.last_nav_goal is not None:
                nav_cfg = self.mission_cfg.get('navigation', {})
                gx, gy, gyaw = self.last_nav_goal
                if self._pose_near_goal(
                        gx, gy, gyaw,
                        nav_cfg.get('vision_xy_tolerance_m', 0.04),
                        nav_cfg.get('vision_yaw_tolerance_rad', 0.12)):
                    rospy.loginfo('[Mission] Phase %d: Vision pose reached by tolerance before move_base success',
                                  phase)
                    self.move_base_client.cancel_goal()
                    arrived = True
                    break
            if state in (GoalStatus.ABORTED, GoalStatus.REJECTED,
                         GoalStatus.RECALLED, GoalStatus.PREEMPTED, GoalStatus.LOST):
                rospy.logwarn('[Mission] Phase %d: Nav to vision failed (state=%d), retrying',
                              phase, state)
                self._retry_perception(phase)
                return
            poll_interval = self.mission_cfg.get('waits', {}).get('nav_poll_interval_s', 1.0)
            rospy.sleep(poll_interval)

        if not arrived:
            rospy.logwarn('[Mission] Phase %d: Nav to vision timed out (%.1fs), retrying',
                          phase, timeout_s)
            self.move_base_client.cancel_goal()
            self._retry_perception(phase)
            return

        self._stop_robot()
        # 到达后稳定等待，确保机器人完全停稳再拍照（避免运动模糊）
        stabilize_s = self.mission_cfg.get('waits', {}).get('arrival_stabilize_s', 1.0)
        rospy.sleep(stabilize_s)
        self.transition(MissionState.task_image_state(phase, 'RECOGNIZE_TASK_IMAGE'))

    def _handle_recognize_task_image(self, phase):
        """机器人已到达视觉位置，触发 VLM 拍照识别，获取任务区号。"""
        trigger_delay = self.mission_cfg.get('waits', {}).get('vision_trigger_delay_s', 0.5)
        rospy.sleep(trigger_delay)
        self.recognition_in_progress = True
        self.vision_result_event.clear()
        rospy.set_param('/top_view_shot_node/im_flag', 1)
        rospy.loginfo('[Mission] Phase %d: VLM triggered at vision position', phase)

        # 等待识别结果
        result_timeout = 10.0
        detected = self.vision_result_event.wait(timeout=result_timeout)

        if not detected or self.recognition_in_progress:
            rospy.logwarn('[Mission] Phase %d: No vision result (%.1fs timeout), retrying',
                          phase, result_timeout)
            self._retry_perception(phase)
            return

        rospy.loginfo('[Mission] Phase %d: Target cell=%d', phase, self.target_cell)
        text = self.voice_cfg['voice_text']['task_image_recognized'].format(
            index=phase, target_cell=self.target_cell)
        self._speak(text, wait=True)
        if self._check_aborted():
            return

        # 视觉阶段：收集 target_cell，不立即导航
        self.collected_targets.append(self.target_cell)
        self.task_cells_done.append(self.target_cell)
        rospy.loginfo('[Mission] Collected targets so far: %s', self.collected_targets)

        if len(self.collected_targets) >= 4:
            # 4 个视觉点全部完成，切换到任务执行阶段
            self.vision_phase = False
            self.task_index = 0
            self.target_cell = self.collected_targets[0]
            rospy.loginfo('[Mission] === Vision phase done, starting task execution: %s ===',
                          self.collected_targets)
            self.transition(MissionState.task_image_state(1, 'NAVIGATE_TO_TASK'))
        else:
            # 继续下一个视觉点
            self.task_index += 1
            next_phase = self.task_index + 1
            self.target_cell = None
            self.perception_retry_count = 0
            self.transition(MissionState.task_image_state(next_phase, 'SEARCH_TASK_IMAGE'))

    def _handle_navigate_to_task(self, phase):
        self.footprint_retry_count = 0  # 新任务，重置 footprint 重试计数
        if self.target_cell is None:
            rospy.logerr('[Mission] Phase %d: No target cell set!', phase)
            self.transition(MissionState.ABORT_PERCEPTION_FAILED)
            return

        x, y = get_cell_center_xy(self.target_cell, self.field_cfg)
        text = self.voice_cfg['voice_text']['navigating_to_task'].format(target_cell=self.target_cell)
        self._speak(text)
        if self._check_aborted():
            return
        rospy.loginfo('[Mission] Phase %d: Navigating to cell %d (%.3f, %.3f)',
                      phase, self.target_cell, x, y)

        self._stop_robot()
        self._send_nav_goal(x, y, 0.0)
        self.logger.log_navigation(
            {'cell': self.target_cell, 'x': x, 'y': y}, None, True)

        self.transition(MissionState.task_image_state(phase, 'ARRIVE_TASK'))

    def _handle_arrive_task(self, phase):
        timeout_s = self.mission_cfg['timeouts'].get('navigation_goal_timeout_s', 60)
        stuck_timeout = self.mission_cfg['timeouts'].get('nav_stuck_timeout_s', 10.0)
        deadline = time.time() + timeout_s

        # Polling loop: 按配置间隔检查导航进度，检测卡死
        last_x, last_y, last_yaw = None, None, None
        stuck_since = None
        check_interval = self.mission_cfg.get('waits', {}).get('nav_poll_interval_s', 1.0)
        arrived_by_footprint = False
        arrived_by_center = False

        while time.time() < deadline:
            # 检查安全 abort 和全局超时
            if self._check_aborted():
                self.move_base_client.cancel_goal()
                rospy.logwarn('[Mission] Phase %d: Aborted during navigation poll', phase)
                return
            self._check_global_timeouts()
            if self._check_aborted():
                self.move_base_client.cancel_goal()
                return

            state = self.move_base_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                break
            in_region, _ = self._get_task_footprint_status()
            if in_region:
                rospy.loginfo('[Mission] Phase %d: Footprint reached before move_base success', phase)
                arrived_by_footprint = True
                self.move_base_client.cancel_goal()
                break
            center_dist, _ = self._get_task_center_distance()
            center_tol = self.mission_cfg.get('navigation', {}).get('task_center_tolerance_m', 0.04)
            if center_dist is not None and center_dist <= center_tol:
                rospy.loginfo('[Mission] Phase %d: Task center reached by tolerance before move_base success (dist=%.3f <= %.3f)',
                              phase, center_dist, center_tol)
                arrived_by_center = True
                self.move_base_client.cancel_goal()
                break
            if state in (GoalStatus.ABORTED, GoalStatus.REJECTED,
                         GoalStatus.RECALLED, GoalStatus.PREEMPTED, GoalStatus.LOST):
                break

            # 检查运动进度（卡死检测 + 振荡检测）
            rx, ry, ryaw = self._get_current_pose()

            # 卡死检测（需要历史数据）
            if rx is not None and last_x is not None:
                dist = ((rx - last_x)**2 + (ry - last_y)**2) ** 0.5
                yaw_diff = abs(ryaw - last_yaw) if (ryaw is not None and last_yaw is not None) else 0.0
                if dist >= 0.02 or yaw_diff >= 0.05:
                    stuck_since = None
                else:
                    if stuck_since is None:
                        stuck_since = time.time()
                    elif time.time() - stuck_since > stuck_timeout:
                        rospy.logwarn('[Mission] Phase %d: Robot stuck (no progress for %.1fs, dist=%.3f, yaw_diff=%.3f)',
                                      phase, stuck_timeout, dist, yaw_diff)
                        self.move_base_client.cancel_goal()
                        break
            last_x, last_y, last_yaw = rx, ry, ryaw

            rospy.sleep(check_interval)

        state = self.move_base_client.get_state()
        arrived = arrived_by_footprint or arrived_by_center or (state == GoalStatus.SUCCEEDED)

        if not arrived:
            rospy.logwarn('[Mission] Phase %d: Navigation failed (state=%d)',
                          phase, state)
            self.navigation_retry_count += 1
            max_retries = self.mission_cfg['timeouts']['navigation_retry_limit']
            if self.navigation_retry_count <= max_retries:
                rospy.loginfo('[Mission] Nav retry %d/%d', self.navigation_retry_count, max_retries)
                self.move_base_client.cancel_goal()
                self.transition(MissionState.task_image_state(phase, 'NAVIGATE_TO_TASK'))
                return
            else:
                max_skips = self.mission_cfg['timeouts'].get('max_task_skips', 1)
                if self.task_skip_count < max_skips:
                    rospy.logwarn('[Mission] Max nav retries exceeded, skipping task %d (skip %d/%d)',
                                  self.target_cell, self.task_skip_count + 1, max_skips)
                    self.task_skip_count += 1
                    self.transition(MissionState.task_image_state(phase, 'SKIP_TASK'))
                    return
                else:
                    rospy.logerr('[Mission] Max nav retries AND max skips (%d) exceeded', max_skips)
                    self.transition(MissionState.ABORT_NAVIGATION_FAILED)
                    return

        self.navigation_retry_count = 0
        self._stop_robot()
        stabilize_s = self.mission_cfg.get('waits', {}).get('arrival_stabilize_s', 1.0)
        rospy.sleep(stabilize_s)

        # 精准到点判定：检查 footprint 是否完全进入任务点区域
        in_region, detail = self._get_task_footprint_status()
        if in_region is None:
            rospy.logwarn('[Mission] Phase %d: No pose available, skipping footprint check', phase)
        else:
            if not in_region:
                rx, ry, ryaw = detail['robot_pose']
                center_dist, _ = self._get_task_center_distance()
                center_tol = self.mission_cfg.get('navigation', {}).get('task_center_tolerance_m', 0.04)
                if center_dist is not None and center_dist <= center_tol:
                    rospy.logwarn('[Mission] Phase %d: Footprint outside but task center tolerance reached '
                                  '(dist=%.3f <= %.3f), accepting position',
                                  phase, center_dist, center_tol)
                    in_region = True
                else:
                    rospy.logwarn('[Mission] Phase %d: Footprint NOT fully inside task region! '
                                  'Outside points: %d, task_center=(%.3f,%.3f), robot=(%.3f,%.3f,%.2f)',
                                  phase, len(detail['points_outside']),
                                  detail['task_center'][0], detail['task_center'][1],
                                  rx, ry, ryaw)
                    self.footprint_retry_count += 1
                    cx, cy = detail['task_center']
                    max_footprint_retries = self.mission_cfg['timeouts'].get('footprint_retry_limit', 2)
                    if self.footprint_retry_count <= max_footprint_retries:
                        self.state_start_time = time.time()
                        rospy.loginfo('[Mission] Phase %d: Footprint correction %d/%d',
                                      phase, self.footprint_retry_count, max_footprint_retries)
                        # Snap yaw to nearest valid orientation (0=east or π=west) for task region fit
                        ryaw = math.atan2(math.sin(ryaw), math.cos(ryaw))  # normalize to [-π, π)
                        target_yaw = 0.0 if abs(ryaw) < math.pi / 2 else math.pi
                        self._send_nav_goal(cx, cy, target_yaw)
                        return
                    else:
                        cx, cy = detail['task_center']
                        rospy.logwarn('[Mission] Phase %d: Footprint retry limit reached (%d), trying open-loop',
                                      phase, max_footprint_retries)
                        self._openloop_approach(cx, cy)
                        rospy.sleep(0.3)
                        rospy.logwarn('[Mission] Phase %d: Accepting position after open-loop attempt', phase)
            else:
                rospy.loginfo('[Mission] Phase %d: Footprint verified inside task region', phase)

        rospy.loginfo('[Mission] Phase %d: Arrived at task point %d', phase, self.target_cell)
        self.transition(MissionState.task_image_state(phase, 'ANNOUNCE_TASK'))

    def _handle_announce_task(self, phase):
        text = self.voice_cfg['voice_text']['task_arrived'].format(target_cell=self.target_cell)
        self._speak(text, wait=True)
        if self._check_aborted():
            return
        self.logger.log_voice(text, 'task_arrived')

        self.task_index += 1
        if self.task_index >= len(self.collected_targets):
            rospy.loginfo('[Mission] All %d tasks done, heading to finish', len(self.collected_targets))
            self.transition(MissionState.NAVIGATE_TO_FINISH)
        else:
            self.target_cell = self.collected_targets[self.task_index]
            next_phase = self.task_index + 1
            self.perception_retry_count = 0
            rospy.loginfo('[Mission] Next task: cell=%d (phase %d)', self.target_cell, next_phase)
            self.transition(MissionState.task_image_state(next_phase, 'NAVIGATE_TO_TASK'))

    def _handle_skip_task(self, phase):
        """任务点不可达：静默跳过，进下一任务点。"""
        self.move_base_client.cancel_goal()
        self._stop_robot()

        text = self.voice_cfg['voice_text'].get('task_skip', u'跳过').format(target_cell=self.target_cell)
        self._speak(text, wait=True)
        rospy.loginfo('[Mission] Phase %d: Task cell %d skipped (%d/%d skips used)',
                      phase, self.target_cell, self.task_skip_count,
                      self.mission_cfg['timeouts'].get('max_task_skips', 1))
        self.logger.log_voice(text, 'task_skip')

        # 重置计数，进下一任务
        self.navigation_retry_count = 0
        self.task_index += 1
        if self.task_index >= len(self.collected_targets):
            rospy.loginfo('[Mission] All %d tasks processed (some skipped), heading to finish',
                          len(self.collected_targets))
            self.transition(MissionState.NAVIGATE_TO_FINISH)
        else:
            self.target_cell = self.collected_targets[self.task_index]
            next_phase = self.task_index + 1
            self.perception_retry_count = 0
            rospy.loginfo('[Mission] Next task after skip: cell=%d (phase %d)',
                          self.target_cell, next_phase)
            self.transition(MissionState.task_image_state(next_phase, 'NAVIGATE_TO_TASK'))

    # ========== Finish Phase Handlers ==========

    def _handle_navigate_to_finish(self):
        finish_cell = self.field_cfg['finish_cell']
        x, y = get_cell_center_xy(finish_cell, self.field_cfg)
        # 终点墙角偏移: 正值=向场地中心退, 负值=向墙角靠
        offset = self.field_cfg.get('finish_offset_m', 0.0)
        if abs(offset) > 0.001:
            x -= offset * (1 if x > 0 else -1) if abs(x) > 0.01 else 0
            y -= offset * (1 if y > 0 else -1) if abs(y) > 0.01 else 0
        rospy.loginfo('[Mission] Navigating to finish cell %d (%.3f, %.3f)%s',
                      finish_cell, x, y,
                      (' offset=%.2fm' % offset) if abs(offset) > 0.001 else '')

        text = self.voice_cfg['voice_text']['navigating_to_finish']
        self._speak(text)
        if self._check_aborted():
            return

        self._stop_robot()
        # 终点朝向：保持自然进入 yaw（≈0/朝东），不强制转
        finish_yaw = 0.0
        self._send_nav_goal(x, y, finish_yaw)
        self.transition(MissionState.ARRIVE_FINISH)

    def _handle_arrive_finish(self):
        timeout_s = self.mission_cfg['timeouts'].get('navigation_goal_timeout_s', 60)
        stuck_timeout = self.mission_cfg['timeouts'].get('nav_stuck_timeout_s', 10.0)
        deadline = time.time() + timeout_s
        last_x, last_y, last_yaw = None, None, None
        stuck_since = None
        check_interval = self.mission_cfg.get('waits', {}).get('nav_poll_interval_s', 1.0)
        arrived_by_pose = False

        while time.time() < deadline:
            if self._check_aborted():
                self.move_base_client.cancel_goal()
                return
            self._check_global_timeouts()
            if self._check_aborted():
                self.move_base_client.cancel_goal()
                return

            state = self.move_base_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                break
            if self.last_nav_goal is not None:
                nav_cfg = self.mission_cfg.get('navigation', {})
                gx, gy, gyaw = self.last_nav_goal
                if self._pose_near_goal(
                        gx, gy, gyaw,
                        nav_cfg.get('finish_xy_tolerance_m', 0.05),
                        nav_cfg.get('finish_yaw_tolerance_rad', 0.20)):
                    rospy.loginfo('[Mission] Finish pose reached by tolerance before move_base success')
                    arrived_by_pose = True
                    self.move_base_client.cancel_goal()
                    break
            if state in (GoalStatus.ABORTED, GoalStatus.REJECTED,
                         GoalStatus.RECALLED, GoalStatus.PREEMPTED, GoalStatus.LOST):
                break

            rx, ry, ryaw = self._get_current_pose()
            if rx is not None and last_x is not None:
                dist = ((rx - last_x)**2 + (ry - last_y)**2) ** 0.5
                yaw_diff = abs(ryaw - last_yaw) if (ryaw is not None and last_yaw is not None) else 0.0
                if dist < 0.02 and yaw_diff < 0.05:
                    if stuck_since is None:
                        stuck_since = time.time()
                    elif time.time() - stuck_since > stuck_timeout:
                        rospy.logwarn('[Mission] Finish: Robot stuck, retrying')
                        self.move_base_client.cancel_goal()
                        break
                else:
                    stuck_since = None
            last_x, last_y, last_yaw = rx, ry, ryaw
            rospy.sleep(check_interval)

        arrived = arrived_by_pose or (self.move_base_client.get_state() == GoalStatus.SUCCEEDED)

        if not arrived:
            rospy.logwarn('[Mission] Finish navigation failed (state=%d)',
                          self.move_base_client.get_state())
            self.finish_nav_retry_count += 1
            max_retries = self.mission_cfg['timeouts']['navigation_retry_limit']
            if self.finish_nav_retry_count <= max_retries:
                rospy.loginfo('[Mission] Finish nav retry %d/%d',
                              self.finish_nav_retry_count, max_retries)
                self.move_base_client.cancel_goal()
                self.transition(MissionState.NAVIGATE_TO_FINISH)
                return
            else:
                rospy.logwarn('[Mission] Max finish nav retries exceeded, trying open-loop fallback')
                self.move_base_client.cancel_goal()
                rospy.sleep(0.5)
                if self.last_nav_goal:
                    self._openloop_approach(self.last_nav_goal[0], self.last_nav_goal[1])
        else:
            self.finish_nav_retry_count = 0

        self._stop_robot()
        text = self.voice_cfg['voice_text']['finish_arrived']
        self._speak(text, wait=True)
        if self._check_aborted():
            return
        self.transition(MissionState.FINISH_ANNOUNCE)

    def _handle_finish_announce(self):
        text = self.voice_cfg['voice_text']['finish']
        self.logger.log_voice(text, 'finish')
        self.transition(MissionState.DONE)

    # ========== Abort ==========

    def _handle_abort(self):
        rospy.logerr('[Mission] ABORT: %s', self.state.value)
        self.logger.log_system('abort', self.state.value)
        self.move_base_client.cancel_all_goals()
        self._stop_robot()

        # Speak appropriate abort message
        vt = self.voice_cfg['voice_text']
        abort_texts = {
            MissionState.ABORT_TIMEOUT: vt['abort_timeout'],
            MissionState.ABORT_COLLISION_RISK: vt['abort_collision'],
            MissionState.ABORT_LOCALIZATION_LOST: vt['abort_localization_lost'],
            MissionState.ABORT_PERCEPTION_FAILED: vt['abort_perception_failed'],
            MissionState.ABORT_NAVIGATION_FAILED: vt['abort_navigation_failed'],
            MissionState.MANUAL_STOP_REQUESTED: vt['abort_manual_stop'],
        }
        text = abort_texts.get(self.state, u'任务终止')
        self._speak(text)
        # 在 abort handler 中不需要再检查 abort（已经处于 abort 状态）

    # ========== Helpers ==========

    def _openloop_approach(self, target_x, target_y):
        """开环逼近：转正车头→直行，仅用于 <0.3m 的末端微调。

        当 move_base 因 AMCL 定位漂移无法完成最后一步时，用定时速度指令做
        开环逼近。仅使用 x 轴直行（Mecanum 最可靠方向：四轮同向驱动）。

        Returns:
            bool: True 如果执行了开环运动，False 如果跳过（距离太远/无位姿）
        """
        rx, ry, ryaw = self._get_current_pose()
        if rx is None:
            rospy.logwarn('[Mission] Open-loop: no current pose, skipping')
            return False

        dx = target_x - rx
        dy = target_y - ry
        dist = math.sqrt(dx*dx + dy*dy)

        if dist > 0.30:
            rospy.logwarn('[Mission] Open-loop: distance %.3f > 0.30m, skipping', dist)
            return False
        if dist < 0.015:
            rospy.loginfo('[Mission] Open-loop: already at target (%.3fm)', dist)
            return True

        rospy.loginfo('[Mission] Open-loop: approaching (%.3fm, %.3f deg)',
                      dist, math.degrees(math.atan2(dy, dx)))

        # 1) 转正车头对准目标方向
        target_heading = math.atan2(dy, dx)
        yaw_err = self._angle_diff(ryaw, target_heading)
        if yaw_err > 0.15:
            twist = Twist()
            twist.angular.z = 1.0 if math.sin(target_heading - ryaw) > 0 else -1.0
            rotate_duration = min(yaw_err / 1.0, 2.0)
            t0 = time.time()
            while time.time() - t0 < rotate_duration and not rospy.is_shutdown() and not self._check_aborted():
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(0.05)
            self._stop_robot()
            rospy.sleep(0.3)

        # 2) 直行
        twist = Twist()
        twist.linear.x = 0.10  # 0.10 m/s, 极慢速度最小化打滑
        drive_duration = max(0.5, min(dist / 0.10, 3.0))
        t0 = time.time()
        while time.time() - t0 < drive_duration and not rospy.is_shutdown() and not self._check_aborted():
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.05)
        self._stop_robot()

        # 验证
        rx2, ry2, _ = self._get_current_pose()
        if rx2 is not None:
            final_dist = math.sqrt((target_x - rx2)**2 + (target_y - ry2)**2)
            rospy.loginfo('[Mission] Open-loop done: final distance %.3fm', final_dist)

        return True

    def _speak(self, text, wait=False):
        """发送 TTS 播报。

        wait=False: 发布即返回（fire-and-forget，用于过渡语音）。
        wait=True:  等待 /tts_done 回执，超时 8s 不卡死（用于关键节点语音）。
        """
        try:
            msg = String()
            msg.data = text
            self.voice_pub.publish(msg)
        except Exception as e:
            rospy.logerr('[Mission] TTS publish failed: %s', str(e))
            return

        rospy.loginfo('[Mission] TTS: %s', text)

        if wait:
            self._tts_pending = text
            self.tts_done_event.clear()
            if not self.tts_done_event.wait(timeout=8.0):
                rospy.logwarn('[Mission] TTS wait timeout: %s', text[:30])
            self._tts_pending = None

    def _stop_robot(self):
        """确保机器人完全停止。"""
        self.cmd_vel_pub.publish(Twist())

    def _convergence_rotate(self):
        """冷启动收敛旋转：帮助 AMCL 粒子群快速收敛 (~5s)。

        原地旋转 ±90° 产生多方向激光配准数据，加速 KLD 采样收敛。
        仅实机模式执行（sim_mode=false），仿真跳过。
        """
        rospy.loginfo('[Mission] AMCL convergence rotate: starting (~5s)')
        twist = Twist()

        # 第一阶段: 正转 ~90° (约 2s @ 0.8 rad/s = ~92°)
        twist.angular.z = 0.8
        t0 = time.time()
        while time.time() - t0 < 2.0 and not rospy.is_shutdown() and not self._check_aborted():
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.05)

        # 第二阶段: 反转 ~90° (约 2s)，回到大致初始朝向
        twist.angular.z = -0.8
        t0 = time.time()
        while time.time() - t0 < 2.0 and not rospy.is_shutdown() and not self._check_aborted():
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.05)

        self._stop_robot()
        rospy.sleep(0.5)  # 停稳
        rospy.loginfo('[Mission] AMCL convergence rotate: done')

    def _startup_backoff(self):
        """开局定位旋转后后退约 10cm，再开始第一个识别点导航。"""
        rospy.loginfo('[Mission] Startup backoff: backing up ~0.10m')
        twist = Twist()
        twist.linear.x = -0.05
        t0 = time.time()
        while time.time() - t0 < 2.0 and not rospy.is_shutdown() and not self._check_aborted():
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.05)

        self._stop_robot()
        rospy.sleep(0.3)
        rospy.loginfo('[Mission] Startup backoff: done')

    def _send_nav_goal(self, x, y, yaw=0.0):
        """通过 move_base actionlib 发送导航目标。"""
        self._clear_costmaps('before_nav_goal')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        quat = tft.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        self.move_base_client.send_goal(goal)
        self.last_nav_goal = (x, y, yaw)
        rospy.loginfo('[Mission] Nav goal sent: (%.3f, %.3f, %.2f rad)', x, y, yaw)

    def _clear_costmaps(self, reason):
        """清理动态障碍层，防止上一个任务点的局部障碍残留影响下一段规划。"""
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=0.5)
            self.clear_costmaps_srv()
            rospy.loginfo('[Mission] Cleared move_base costmaps (%s)', reason)
        except Exception as e:
            rospy.logwarn('[Mission] Clear costmaps failed (%s): %s', reason, e)

    def _check_global_timeouts(self):
        """检查比赛全局超时。"""
        mission_elapsed = time.time() - self.mission_start_time
        max_time = self.mission_cfg['mission']['max_time_s']
        if mission_elapsed > max_time:
            rospy.logerr('[Mission] Mission timeout: %.1fs > %ds', mission_elapsed, max_time)
            self.transition(MissionState.ABORT_TIMEOUT)
            return

        state_elapsed = time.time() - self.state_start_time
        max_state_time = self.mission_cfg['timeouts']['no_state_change_s']
        if state_elapsed > max_state_time:
            rospy.logwarn('[Mission] State timeout in %s: %.1fs > %ds',
                          self.state.value, state_elapsed, max_state_time)
            self.transition(MissionState.ABORT_TIMEOUT)
            return

        self._check_localization()
        self._update_pose_from_tf()  # 当 AMCL /amcl_pose 停发时用 TF 更新位姿

    def _update_pose_from_tf(self):
        """当 AMCL /amcl_pose 停发时从 TF (map→base_link) 获取位姿作为 fallback。

        AMCL 1.16.7 有 bug: /amcl_pose 仅发1条后停止，但 map→odom TF 正常发布。
        本方法在 AMCL pose 可用时不干预(rank 低于 amcl_pose 不会被接受)。
        """
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                'map', 'base_link', rospy.Time(0))
            x, y = trans[0], trans[1]
            _, _, yaw = tft.euler_from_quaternion(rot)
            self._update_pose('odom_tf', x, y, yaw)
        except (ros_tf.LookupException, ros_tf.ConnectivityException,
                ros_tf.ExtrapolationException, TypeError):
            pass

    def _check_localization(self):
        """监控 AMCL 定位是否发散（粒子群协方差过大且持续）。

        仅在位姿源已升级为 amcl_pose 时生效：
          - 仿真用 odom (rank=1)，收不到 /amcl_pose，self._pose_source_rank 永远 < 3，
            直接 return，绝不误触发。
          - 实机当 AMCL 协方差(位置/朝向标准差)超阈值并持续 lost_duration_s 才 ABORT。
        """
        loc_cfg = self.mission_cfg.get('localization', {})
        if not loc_cfg.get('monitor_enabled', True):
            return

        # 在同一锁内读取位姿源等级与协方差快照，避免 AMCL 升级瞬间的竞态窗口
        with self._pose_lock:
            rank = self._pose_source_rank
            pos_std = self._amcl_pos_std
            yaw_std = self._amcl_yaw_std
            last_time = self._amcl_last_time

        # 关键守卫：只有真正在用 amcl_pose 才监控，否则(仿真/未定位)跳过
        if rank < self._POSE_RANK['amcl_pose']:
            return

        # AMCL 长时间无更新也视为定位异常
        stale_s = loc_cfg.get('amcl_stale_s', 3.0)
        max_pos = loc_cfg.get('max_pos_std_m', 0.5)
        max_yaw = loc_cfg.get('max_yaw_std_rad', 0.5)
        now = time.time()
        diverged = False
        stale_triggered = False
        std_triggered = False
        if stale_s > 0 and last_time > 0 and (now - last_time) > stale_s:
            diverged = True
            stale_triggered = True
        if pos_std > max_pos or yaw_std > max_yaw:
            diverged = True
            std_triggered = True

        if not diverged:
            self._localization_lost_since = None
            return

        # 需持续超过 lost_duration_s 才判定丢失，避免单次抖动误触发
        if self._localization_lost_since is None:
            self._localization_lost_since = now
            rospy.logwarn('[Mission] Localization diverging: pos_std=%.3fm(max=%.3f) yaw_std=%.3frad(max=%.3f) stale=%s(%.1f/%.1fs)',
                          pos_std, max_pos, yaw_std, max_yaw,
                          stale_triggered, now - last_time if last_time > 0 else 0.0, stale_s)
            return
        lost_duration = loc_cfg.get('lost_duration_s', 3.0)
        if now - self._localization_lost_since > lost_duration:
            rospy.logerr('[Mission] Localization LOST (>%.1fs): pos_std=%.2fm yaw_std=%.2frad',
                         lost_duration, pos_std, yaw_std)
            self.transition(MissionState.ABORT_LOCALIZATION_LOST)

    def _retry_perception(self, phase):
        """处理识别重试：重置旋转计数器，从头开始搜索。"""
        max_retries = self.mission_cfg['timeouts']['perception_retry_limit']
        self.perception_retry_count += 1
        if self.perception_retry_count > max_retries:
            rospy.logerr('[Mission] Max perception retries (%d) exceeded', max_retries)
            self.transition(MissionState.ABORT_PERCEPTION_FAILED)
            return
        rospy.loginfo('[Mission] Perception retry %d/%d, re-navigating to vision position',
                      self.perception_retry_count, max_retries)
        text = self.voice_cfg['voice_text']['task_image_failed']
        self._speak(text)
        self.recognition_in_progress = False
        self.vision_result_event.clear()
        self.transition(MissionState.task_image_state(phase, 'SEARCH_TASK_IMAGE'))


if __name__ == '__main__':
    rospy.init_node('mission_state_machine')
    sim_mode = rospy.get_param('~sim_mode', False)
    fsm = MissionStateMachine(sim_mode=sim_mode)
    fsm.run()
