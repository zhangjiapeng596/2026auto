#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""仿真机器人节点：发布带噪声的 mock odometry、laser scan、TF，替代真实硬件。

使用方法:
  rosrun mission_manager sim_robot.py _init_x:=-1.6 _init_y:=1.6

发布的 Topic:
  /odom            (nav_msgs/Odometry, 含噪声)
  /scan_filtered   (sensor_msgs/LaserScan, 含噪声 + 动态障碍物)
  /joint_states    (sensor_msgs/JointState)
  /initialpose     (geometry_msgs/PoseWithCovarianceStamped, 给 AMCL 初始化)
  TF: odom -> base_footprint (含噪声里程计, map->odom 由 AMCL 发布)

噪声参数 (仿真 AMCL 定位):
  ~odom_noise_linear  里程计线位移噪声系数 (默认 0.1, noise=系数×每步位移)
  ~odom_noise_angular 里程计角位移噪声系数 (默认 0.1)
  ~laser_noise_std    激光测距高斯噪声 (m, 默认 0.01)

障碍物:
  从 competition_field.yaml 的 obstacles 字段读取 cell 编号,
  在激光扫描中注入短距读数 (仿真 LiDAR 检测障碍物)
"""
import os
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import rospy
import math
import random
import yaml
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, PoseWithCovarianceStamped
try:
    from visualization_msgs.msg import Marker, MarkerArray
    HAS_MARKER = True
except ImportError:
    HAS_MARKER = False

# config 路径
CONFIG_PATHS = [
    os.path.join(os.path.dirname(__file__), '..', '..', '..', 'config'),
    os.path.join(os.path.expanduser('~'), '3X2KY2', 'config'),
    os.path.join(os.environ.get('WS_PATH', ''), 'config'),
]


def _find_config(filename):
    for d in CONFIG_PATHS:
        p = os.path.join(os.path.abspath(d), filename)
        if os.path.isfile(p):
            return p
    return None


def _cell_to_xy(cell, rows=9, cols=9, cell_sz=0.4, field_sz=3.6):
    """网格编号 → map 坐标"""
    n = cell - 1
    row = n // cols
    col = n % cols
    x = (col - cols / 2.0) * cell_sz + cell_sz / 2.0
    y = (rows / 2.0 - row) * cell_sz - cell_sz / 2.0
    return x, y


def _board_dimensions(cfg):
    """返回挡板可视化/碰撞线段的 (length, thickness)。"""
    size = cfg.get('obstacles_config', {}).get('board_size_m', [0.01, 0.40])
    if not isinstance(size, list) or len(size) < 2:
        size = [0.01, 0.40]
    thickness = min(float(size[0]), float(size[1]))
    length = max(float(size[0]), float(size[1]))
    return length, thickness


def _obstacle_pose(obs, rows, cols, cell_sz):
    """把 {cell, edge} 转成内部网格边上的挡板中心和朝向。"""
    cell = obs['cell']
    if cell < 1 or cell > rows * cols:
        raise ValueError('obstacle cell %s is outside 1..%d' % (cell, rows * cols))
    n = cell - 1
    row = n // cols
    col = n % cols
    edge = str(obs.get('edge', '')).upper()
    if edge not in ('N', 'S', 'E', 'W'):
        raise ValueError('obstacle cell %s must set edge=N/S/E/W' % cell)
    if ((edge == 'N' and row == 0) or
            (edge == 'S' and row == rows - 1) or
            (edge == 'W' and col == 0) or
            (edge == 'E' and col == cols - 1)):
        raise ValueError('obstacle cell %s edge %s is outer boundary' % (cell, edge))

    cx, cy = _cell_to_xy(cell, rows, cols, cell_sz)
    if edge == 'N':
        cy += cell_sz / 2.0
        yaw = 0.0
    elif edge == 'S':
        cy -= cell_sz / 2.0
        yaw = 0.0
    elif edge == 'E':
        cx += cell_sz / 2.0
        yaw = math.pi / 2.0
    else:
        cx -= cell_sz / 2.0
        yaw = math.pi / 2.0
    return cx, cy, yaw


class SimRobot(object):
    """仿真机器人：响应 /cmd_vel 更新位姿，发布 odom + laser(含障碍物) + TF。"""

    def __init__(self):
        init_x = rospy.get_param('~init_x', -1.6)
        init_y = rospy.get_param('~init_y', 1.6)
        init_yaw = rospy.get_param('~init_yaw', 0.0)

        self.x = init_x
        self.y = init_y
        self.yaw = init_yaw

        # 传感器噪声参数 (仿真 AMCL 定位)
        # noise = coefficient * distance_per_step + floor
        # 例: 0.1 × 0.01m = 0.001m/step → 100步(1m)累积 ≈ 1cm RMS
        self.odom_noise_linear = rospy.get_param('~odom_noise_linear', 0.1)    # 线位移噪声系数
        self.odom_noise_angular = rospy.get_param('~odom_noise_angular', 0.1)  # 角位移噪声系数
        # 激光噪声: RPLidar A1 典型 ~1cm @ 1m
        self.laser_noise_std = rospy.get_param('~laser_noise_std', 0.01)        # m

        # 加载障碍物 — 放在内圈网格边上，建模为线段
        self.obstacle_segments = []
        try:
            config_path = _find_config('competition_field.yaml')
            if config_path:
                with open(config_path, 'r') as f:
                    cfg = yaml.safe_load(f)
                field = cfg['field']
                rows, cols = field['grid_rows'], field['grid_cols']
                cell_sz = field['cell_size_m']
                board_len, board_thickness = _board_dimensions(cfg)
                half = board_len / 2.0
                for obs in (cfg.get('obstacles') or []):
                    cx, cy, def_yaw = _obstacle_pose(obs, rows, cols, cell_sz)
                    yaw = math.radians(obs.get('yaw_deg', math.degrees(def_yaw)))
                    ax = cx - half * math.cos(yaw)
                    ay = cy - half * math.sin(yaw)
                    bx = cx + half * math.cos(yaw)
                    by = cy + half * math.sin(yaw)
                    self.obstacle_segments.append(
                        (ax, ay, bx, by, cx, cy, board_len, board_thickness))
                if self.obstacle_segments:
                    rospy.loginfo('[SimRobot] %d obstacles loaded', len(self.obstacle_segments))
        except Exception as e:
            rospy.logwarn('[SimRobot] Obstacle load failed: %s', e)
            self.obstacle_segments = []

        # 场地四周围栏 (AMCL 匹配静态地图必须)
        field_w = 3.6
        half_f = field_w / 2.0
        wall_thickness = 0.01
        self.obstacle_segments.extend([
            (-half_f,  half_f,  half_f,  half_f,  0.0,  half_f,  field_w, wall_thickness),   # 北
            (-half_f, -half_f,  half_f, -half_f,  0.0, -half_f,  field_w, wall_thickness),   # 南
            ( half_f, -half_f,  half_f,  half_f,  half_f,  0.0,  field_w, wall_thickness),   # 东
            (-half_f, -half_f, -half_f,  half_f, -half_f,  0.0,  field_w, wall_thickness),   # 西
        ])

        # 发布
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.scan_pub = rospy.Publisher('/scan_filtered', LaserScan, queue_size=10)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        if HAS_MARKER:
            self.obs_marker_pub = rospy.Publisher('/sim_obstacles', MarkerArray, queue_size=10)
            self._publish_obstacle_markers()
        self.tf_br = tf.TransformBroadcaster()

        # 订阅 cmd_vel 模拟运动
        rospy.Subscriber('/cmd_vel', Twist, self._on_cmd_vel)
        self.last_time = rospy.Time.now()

        # 发布初始位姿给 AMCL, 避免粒子滤波器冷启动发散
        self._publish_initial_pose()

        rospy.loginfo('[SimRobot] Init at (%.2f, %.2f, %.2f)  obstacles=%d',
                      self.x, self.y, self.yaw, len(self.obstacle_segments))

    def _publish_initial_pose(self):
        """发布初始位姿到 /initialpose (latch)，AMCL 随时订阅都能收到。
        只发一次 — 重复发布会导致 AMCL 反复重置粒子滤波器无法收敛。"""
        ip = PoseWithCovarianceStamped()
        ip.header.frame_id = 'map'
        ip.header.stamp = rospy.Time.now()
        ip.pose.pose.position.x = self.x
        ip.pose.pose.position.y = self.y
        q = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        ip.pose.pose.orientation = Quaternion(*q)
        # 3×3 对角协方差 (x, y, yaw) — 仿真初始位姿为真值，小协方差
        ip.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                              0, 0.01, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0.01]
        # latch=True: AMCL 即使晚启动也能在订阅时收到最后一条
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
        pub.publish(ip)
        rospy.loginfo('[SimRobot] Initial pose sent to AMCL (%.2f, %.2f, %.2f)',
                      self.x, self.y, self.yaw)

    def _on_cmd_vel(self, msg):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        if dt <= 0 or dt > 0.5:
            dt = 0.05
        self.last_time = now

        # 全向轮运动学积分
        linear_dist = math.sqrt(msg.linear.x**2 + msg.linear.y**2) * dt
        angular_dist = abs(msg.angular.z) * dt

        self.x += msg.linear.x * math.cos(self.yaw) * dt
        self.y += msg.linear.x * math.sin(self.yaw) * dt
        self.x -= msg.linear.y * math.sin(self.yaw) * dt
        self.y += msg.linear.y * math.cos(self.yaw) * dt
        self.yaw += msg.angular.z * dt

        # 叠加高斯噪声模拟轮滑/地面不平整 (噪声与位移成正比)
        self.x += random.gauss(0, self.odom_noise_linear * linear_dist + 0.0001)
        self.y += random.gauss(0, self.odom_noise_linear * linear_dist + 0.0001)
        self.yaw += random.gauss(0, self.odom_noise_angular * angular_dist + 0.00005)

    def _publish_odom(self):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        q = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        msg.pose.pose.orientation = Quaternion(*q)
        msg.pose.covariance = [0.001, 0, 0, 0, 0, 0,
                               0, 0.001, 0, 0, 0, 0,
                               0, 0, 1e6, 0, 0, 0,
                               0, 0, 0, 1e6, 0, 0,
                               0, 0, 0, 0, 1e6, 0,
                               0, 0, 0, 0, 0, 0.001]
        self.odom_pub.publish(msg)

    def _publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'laser_link'
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.pi / 180.0
        msg.time_increment = 0
        msg.scan_time = 0.1
        msg.range_min = 0.15
        msg.range_max = 12.0
        num_readings = 360
        ranges = [12.0] * num_readings

        # 线段障碍物: 射线-线段求交 (纸板, 不是圆桶)
        for ax, ay, bx, by, cx, cy, board_len, board_thickness in self.obstacle_segments:
            # 线段向量
            sx = bx - ax
            sy = by - ay
            seg_len_sq = sx * sx + sy * sy
            if seg_len_sq < 1e-10:
                continue

            for i in range(num_readings):
                ray_angle = msg.angle_min + i * msg.angle_increment + self.yaw
                rdx = math.cos(ray_angle)
                rdy = math.sin(ray_angle)
                rx = self.x
                ry = self.y

                # 射线-线段求交 (2D cross product)
                cross_rs = rdx * sy - rdy * sx
                if abs(cross_rs) < 1e-10:
                    continue  # 平行

                # t = 射线参数, u = 线段参数
                dx_ar = rx - ax
                dy_ar = ry - ay
                t = (dx_ar * sy - dy_ar * sx) / (-cross_rs)
                u = -(dx_ar * rdy - dy_ar * rdx) / cross_rs

                if t > 0.01 and 0.0 <= u <= 1.0:
                    if t < ranges[i]:
                        ranges[i] = t

        # 叠加高斯噪声模拟 LiDAR 测距抖动
        if self.laser_noise_std > 0:
            for i in range(num_readings):
                ranges[i] = max(msg.range_min, min(msg.range_max,
                    ranges[i] + random.gauss(0, self.laser_noise_std)))

        msg.ranges = ranges
        msg.intensities = [0.0] * num_readings
        self.scan_pub.publish(msg)

    def _publish_obstacle_markers(self):
        """发布障碍物可视化 Marker (薄板, RViz 可看到纸板形状和朝向)。"""
        ma = MarkerArray()
        for i, (ax, ay, bx, by, cx, cy, board_len, board_thickness) in enumerate(self.obstacle_segments):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = rospy.Time.now()
            m.ns = 'sim_obstacles'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.15
            # 朝向: 线段方向
            yaw = math.atan2(by - ay, bx - ax)
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            m.pose.orientation = Quaternion(*q)
            # 挡板尺寸: 长边 × 厚度 × 高30cm
            m.scale.x = board_len
            m.scale.y = board_thickness
            m.scale.z = 0.30
            m.color.r = 1.0
            m.color.g = 0.3
            m.color.b = 0.1
            m.color.a = 0.85
            m.lifetime = rospy.Duration(0.5)
            ma.markers.append(m)
        self.obs_marker_pub.publish(ma)

    def _publish_tf(self):
        now = rospy.Time.now()
        q = tf.transformations.quaternion_from_euler(0, 0, self.yaw)

        # map→odom sim_robot 始终发布 identity (仿真 ground truth)
        # AMCL 同时运行发布自己的 map→odom 估计, tf 树自动选用更新时间更近的
        t_map = TransformStamped()
        t_map.header.stamp = now
        t_map.header.frame_id = 'map'
        t_map.child_frame_id = 'odom'
        t_map.transform.translation.x = 0.0
        t_map.transform.translation.y = 0.0
        t_map.transform.translation.z = 0.0
        t_map.transform.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.tf_br.sendTransformMessage(t_map)

        # odom→base_footprint (含噪声)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(*q)
        self.tf_br.sendTransformMessage(t)

    def run(self):
        rate = rospy.Rate(20)
        tick = 0
        while not rospy.is_shutdown():
            self._publish_odom()
            self._publish_scan()
            self._publish_tf()
            # 每1s发joint_states, 驱动robot_state_publisher维持TF树
            if tick % 20 == 0:
                self._publish_joint_states()
            tick += 1
            if HAS_MARKER and tick % 10 == 0:
                self._publish_obstacle_markers()
            rate.sleep()

    def _publish_joint_states(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = []
        js.position = []
        js.velocity = []
        self.joint_pub.publish(js)


if __name__ == '__main__':
    rospy.init_node('sim_robot')
    SimRobot().run()
