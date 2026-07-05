# 地面巡航比赛系统 — 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 基于现有 ABOT 机器人代码（7 个可用包），完成地面巡航比赛的完整软件系统：语音唤醒 → 任务图像识别 → 导航到点 → 语音播报 → 终点结束，含安全保护和日志系统。

**Architecture:** 新建 `mission_manager` ROS 包承载任务状态机和安全管理，新建 `config/` 目录存放全部比赛参数（6 个 YAML），改造 `abot_vlm` 做任务图像识别，调优 `robot_slam` 导航参数适配 3.6m 小场地，复用 `robot_slam/scripts/start.py` 做语音唤醒。不改动 `abot_base` 底层驱动。

**Tech Stack:** ROS Melodic (Ubuntu 18.04 WSL), Python 2.7, C++14, catkin, move_base, Cartographer, 豆包 Vision Pro VLM

---

### Task 1: 比赛场地配置文件

**Files:**
- Create: `config/competition_field.yaml`

- [ ] **Step 1: 创建场地配置文件**

```yaml
# config/competition_field.yaml
# 比赛场地完整描述：尺寸、网格、起终点、任务点、障碍物、任务图像位置

field:
  size_m: [3.6, 3.6]
  grid_rows: 9
  grid_cols: 9
  cell_size_m: 0.4
  fence_height_m: 0.30

# 网格编号约定：PDF 图中 1 在左上角(西北)，9 在右上角(东北)，
# 73 在左下角(西南)，81 在右下角(东南)。
# map 坐标系：以场地中心为原点，x 轴向东，y 轴向北。
cell_index_convention:
  description: >
    网格 1 在左上角，按行优先排列。
    网格编号 N 对应的 map 坐标：
      row = (N - 1) // 9   (0 为最北行)
      col = (N - 1) % 9    (0 为最西列)
      x = (col - 4) * 0.4 + 0.2
      y = (4 - row) * 0.4 - 0.2

start_cells: [1, 81]
finish_cell: 9

task_cells: [31, 32, 33, 40, 41, 42, 49, 50, 51]

task_region:
  size_m: [0.38, 0.32]
  description: "任务点区域 38cm x 32cm，在对应黄色格子中居中"

obstacles:
  board_size_m: [0.40, 0.30]
  placement: "random_on_red_lines_in_rule_diagram"
  detection_required: true

task_images:
  count: 4
  center_height_m: 0.20
  placed_on_inner_fence: true
  description: "4 张任务信息图像贴在围栏内侧，中心距地面 20cm"
```

- [ ] **Step 2: 提交**

```bash
git add config/competition_field.yaml
git commit -m "feat(config): 添加比赛场地配置文件"
```

### Task 2: 任务参数配置文件

**Files:**
- Create: `config/mission.yaml`

- [ ] **Step 1: 创建任务配置文件**

```yaml
# config/mission.yaml
# 比赛任务全局参数：超时、重试、语音要求

mission:
  max_time_s: 180
  required_task_image_count: 4
  require_start_voice: true
  require_finish_voice: true
  stop_before_voice: true
  voice_static_hold_s: 0.5
  localization_required: true
  retry_on_low_confidence: true

timeouts:
  no_motion_after_start_s: 20
  no_state_change_s: 20
  perception_retry_limit: 3
  navigation_retry_limit: 2
  navigation_goal_timeout_s: 60

confidence:
  min_confidence: 0.6
  description: "低于此置信度触发重识别或人工介入"
```

- [ ] **Step 2: 提交**

```bash
git add config/mission.yaml
git commit -m "feat(config): 添加任务参数配置文件"
```

### Task 3: 机器人参数配置文件

**Files:**
- Create: `config/robot.yaml`

- [ ] **Step 1: 创建机器人配置文件**

```yaml
# config/robot.yaml
# 机器人自身参数：尺寸、footprint、传感器、驱动方式

robot:
  drive_type: "mecanum"
  max_linear_speed_mps: 1.0
  size_m: [0.35, 0.30, 0.24]
  wheel_diameter_m: 0.097
  min_front_rear_wheelbase_m: 0.24

footprint: [[-0.175, -0.15], [-0.175, 0.15], [0.175, 0.15], [0.175, -0.15]]

sensors:
  lidar:
    fov_deg: 360
    range_m: [0.15, 12.0]
    topic: "/scan_filtered"
    frame: "laser_link"
  camera:
    resolution: "1080p"
    max_fps: 120
    fov_deg: 120
    topic: "/usb_cam/image_raw"
    frame: "usb_cam"
  imu:
    axes: 9
    topic: "/imu"
  encoder:
    lines_per_rev_after_multiplier: 3960
```

- [ ] **Step 2: 提交**

```bash
git add config/robot.yaml
git commit -m "feat(config): 添加机器人参数配置文件"
```

### Task 4: 导航参数配置文件

**Files:**
- Create: `config/navigation.yaml`

- [ ] **Step 1: 创建导航配置文件**

```yaml
# config/navigation.yaml
# 导航模块参数：costmap、全局规划器、局部规划器、到点判定
# 以下参数将覆盖 robot_slam/params/carto/ 中的对应值

global_costmap:
  width_m: 4.0
  height_m: 4.0
  resolution_m: 0.02
  inflation_radius_m: 0.22
  cost_scaling_factor: 10.0
  obstacle_range_m: 1.5
  raytrace_range_m: 2.5
  update_frequency: 2.0
  publish_frequency: 1.0

local_costmap:
  width_m: 2.0
  height_m: 2.0
  resolution_m: 0.02
  rolling_window: true
  inflation_radius_m: 0.1
  cost_scaling_factor: 3.0
  update_frequency: 5.0
  publish_frequency: 2.0

global_planner:
  type: "GlobalPlanner"
  allow_unknown: true
  default_tolerance: 0.1
  use_dijkstra: true

local_planner:
  type: "DWAPlannerROS"
  max_vel_x: 0.55
  min_vel_x: -0.20
  max_vel_y: 0.20
  min_vel_y: -0.20
  max_vel_theta: 1.4
  min_vel_theta: 0.8
  acc_lim_x: 2.5
  acc_lim_y: 2.5
  acc_lim_theta: 3.0
  xy_goal_tolerance: 0.03
  yaw_goal_tolerance: 0.02
  sim_time: 1.7
  holonomic_robot: true
  oscillation_reset_dist: 0.05

fence_safety_margin_m: 0.15
# 围挡安全距离，机器人 footprint 边缘不得进入此范围
```

- [ ] **Step 2: 提交**

```bash
git add config/navigation.yaml
git commit -m "feat(config): 添加导航参数配置文件"
```

### Task 5: 图像识别参数与语音文本配置文件

**Files:**
- Create: `config/perception.yaml`
- Create: `config/voice_text.yaml`

- [ ] **Step 1: 创建识别参数配置**

```yaml
# config/perception.yaml
# 任务图像识别模块参数

perception:
  vlm_model: "doubao-1-5-vision-pro-32k-250115"
  prompt_template: >
    你是一个机器人比赛的视觉识别系统。
    请识别图中围栏上的任务信息图像，
    输出图像内容和对应的任务点编号（31-33, 40-42, 49-51 之一）。
    回复格式必须是 JSON：
    {"target_cell": <数字>, "content": "<图像内容描述>", "confidence": <0-1之间的浮点数>}
    如果无法识别，设置 confidence 为 0。

task_image_mapping:
  description: "任务图像内容到任务点编号的映射（赛前填充）"
  mappings: []
  # 示例：mappings:
  #   - image_content: "红色圆形"
  #     target_cell: 41

camera:
  capture_width: 640
  capture_height: 480
  save_images: true
  save_path: "logs/"
```

- [ ] **Step 2: 创建语音文本配置**

```yaml
# config/voice_text.yaml
# 所有播报文本集中配置

voice_text:
  start: "比赛开始"
  finish: "比赛结束"
  wakeup_detected: "唤醒成功，比赛开始"
  task_image_searching: "正在搜索第 {index} 个任务信息图像"
  task_image_recognized: "已识别第 {index} 个任务信息，目标任务点为 {target_cell} 号区域"
  task_image_failed: "第 {index} 个任务信息识别失败，正在重试"
  navigating_to_task: "正在前往 {target_cell} 号任务点"
  task_arrived: "已到达 {target_cell} 号任务点"
  navigating_to_finish: "全部任务完成，正在前往终点"
  finish_arrived: "已到达终点，比赛结束"
  abort_timeout: "任务超时，比赛结束"
  abort_collision: "检测到碰撞风险，比赛终止"
```

- [ ] **Step 3: 提交**

```bash
git add config/perception.yaml config/voice_text.yaml
git commit -m "feat(config): 添加图像识别参数和语音文本配置文件"
```

### Task 6: 公共工具包 — 配置加载与日志

**Files:**
- Create: `src/common/CMakeLists.txt`
- Create: `src/common/package.xml`
- Create: `src/common/scripts/config_loader.py`
- Create: `src/common/scripts/mission_logger.py`

- [ ] **Step 1: 创建 package.xml**

```xml
<?xml version="1.0"?>
<package format="2">
  <name>common</name>
  <version>0.1.0</version>
  <description>Common utilities: config loading, logging, data types</description>
  <maintainer email="lx_hm@example.com">lx_hm</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>rospy</exec_depend>
</package>
```

- [ ] **Step 2: 创建 CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(common)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/config_loader.py
  scripts/mission_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

- [ ] **Step 3: 创建 config_loader.py**

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""配置加载工具：从 config/ YAML 文件加载比赛参数。"""
import os
import yaml
import rospy

CONFIG_SEARCH_PATHS = [
    os.path.join(os.path.dirname(__file__), '..', '..', '..', 'config'),
    '/home/abot/abot_ws/src/config',
    os.path.expanduser('~/abot_ws/src/config'),
]


def _find_config_dir():
    for path in CONFIG_SEARCH_PATHS:
        abs_path = os.path.abspath(path)
        if os.path.isdir(abs_path):
            return abs_path
    raise IOError('Cannot find config directory. Searched: {}'.format(CONFIG_SEARCH_PATHS))


def load_config(filename):
    """加载一个 YAML 配置文件，返回 dict。"""
    config_dir = _find_config_dir()
    filepath = os.path.join(config_dir, filename)
    if not os.path.isfile(filepath):
        raise IOError('Config file not found: {}'.format(filepath))
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)
    rospy.loginfo('[config_loader] Loaded: %s', filepath)
    return data


def load_all_configs():
    """加载全部 6 个配置文件，返回合并后的 dict。"""
    files = [
        'competition_field.yaml',
        'mission.yaml',
        'robot.yaml',
        'navigation.yaml',
        'perception.yaml',
        'voice_text.yaml',
    ]
    configs = {}
    for f in files:
        configs[f.replace('.yaml', '')] = load_config(f)
    return configs


def get_cell_center_xy(cell_number, field_config):
    """
    将网格编号 (1-81) 转换为 map 坐标系的 (x, y) 中心点。
    网格 1 在左上角，9 在右上角。
    map 坐标系原点在场地中心，x 轴向东，y 轴向北。
    """
    gc = field_config['cell_index_convention']
    rows = field_config['field']['grid_rows']
    cols = field_config['field']['grid_cols']
    cell_size = field_config['field']['cell_size_m']

    n = cell_number - 1
    row = n // cols  # 0 为最北
    col = n % cols   # 0 为最西

    x = (col - cols / 2.0) * cell_size + cell_size / 2.0
    y = (rows / 2.0 - row) * cell_size - cell_size / 2.0
    return x, y
```

- [ ] **Step 4: 创建 mission_logger.py**

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""结构化日志：按 run_YYYYMMDD_HHMMSS 组织，记录状态机跳转、识别、导航、播报结果。"""
import os
import json
import time
import threading
from datetime import datetime


class MissionLogger(object):
    """比赛日志管理器。线程安全。"""

    def __init__(self, base_dir=None):
        if base_dir is None:
            base_dir = os.path.expanduser('~/abot_ws/logs')
        run_id = datetime.now().strftime('run_%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(base_dir, run_id)
        os.makedirs(self.run_dir, exist_ok=True)
        self._lock = threading.Lock()

    def _append(self, filename, record):
        record['_timestamp'] = time.time()
        with self._lock:
            filepath = os.path.join(self.run_dir, filename)
            with open(filepath, 'a') as f:
                f.write(json.dumps(record, ensure_ascii=False) + '\n')

    def log_state_transition(self, from_state, to_state, reason=''):
        self._append('mission_log.jsonl', {
            'type': 'state_transition',
            'from': from_state,
            'to': to_state,
            'reason': reason,
        })

    def log_perception(self, result):
        self._append('perception_results.jsonl', {
            'type': 'perception',
            'result': result,
        })

    def log_navigation(self, goal, arrived_pose, success):
        self._append('navigation_goals.jsonl', {
            'type': 'navigation',
            'goal': goal,
            'arrived_pose': arrived_pose,
            'success': success,
        })

    def log_voice(self, text, status):
        self._append('voice_events.jsonl', {
            'type': 'voice',
            'text': text,
            'status': status,
        })

    def log_system(self, event, detail=''):
        self._append('system_status.jsonl', {
            'type': 'system',
            'event': event,
            'detail': detail,
        })
```

- [ ] **Step 5: 编译验证并在 WSL 中测试导入**

```bash
# In WSL:
cp -r /mnt/d/StudyWorks/3.2/MachineVision_Project/AutonomousCruise_Ground/src/common /home/lx_hm/abot_ws/src/
cd /home/lx_hm/abot_ws && catkin_make --pkg common
source devel/setup.bash
python -c "from config_loader import get_cell_center_xy; print('OK')"
python -c "from mission_logger import MissionLogger; print('OK')"
```

Expected: `OK` printed twice, `catkin_make` passes.

- [ ] **Step 6: 提交**

```bash
git add src/common/
git commit -m "feat(common): 添加配置加载和结构化日志公共工具包"
```

### Task 7: Mission Manager — ROS 包骨架

**Files:**
- Create: `src/mission_manager/CMakeLists.txt`
- Create: `src/mission_manager/package.xml`
- Create: `src/mission_manager/launch/sim_mission.launch`

- [ ] **Step 1: 创建 package.xml**

```xml
<?xml version="1.0"?>
<package format="2">
  <name>mission_manager</name>
  <version>0.1.0</version>
  <description>Ground cruise mission state machine and safety monitor</description>
  <maintainer email="lx_hm@example.com">lx_hm</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>move_base_msgs</exec_depend>
  <exec_depend>actionlib_msgs</exec_depend>
  <exec_depend>common</exec_depend>
  <exec_depend>robot_slam</exec_depend>
</package>
```

- [ ] **Step 2: 创建 CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(mission_manager)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/mission_state_machine.py
  scripts/safety_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
```

- [ ] **Step 3: 创建仿真启动文件 sim_mission.launch**

```xml
<launch>
  <!-- 比赛系统仿真启动文件：加载配置 + 启动状态机 + 安全监控 -->
  <arg name="config_dir" default="$(find mission_manager)/../../config"/>

  <!-- 将所有 YAML 配置注入 ROS 参数服务器 -->
  <rosparam command="load" file="$(arg config_dir)/competition_field.yaml" ns="competition_field"/>
  <rosparam command="load" file="$(arg config_dir)/mission.yaml" ns="mission"/>
  <rosparam command="load" file="$(arg config_dir)/robot.yaml" ns="robot"/>
  <rosparam command="load" file="$(arg config_dir)/navigation.yaml" ns="navigation"/>
  <rosparam command="load" file="$(arg config_dir)/perception.yaml" ns="perception"/>
  <rosparam command="load" file="$(arg config_dir)/voice_text.yaml" ns="voice_text"/>

  <!-- 任务状态机 -->
  <node pkg="mission_manager" type="mission_state_machine.py" name="mission_state_machine" output="screen">
    <param name="sim_mode" value="true"/>
  </node>

  <!-- 安全监控 -->
  <node pkg="mission_manager" type="safety_monitor.py" name="safety_monitor" output="screen"/>
</launch>
```

- [ ] **Step 4: 提交**

```bash
git add src/mission_manager/
git commit -m "feat(mission): 添加任务管理包骨架和仿真启动文件"
```

### Task 8: 任务状态机核心流转

**Files:**
- Create: `src/mission_manager/scripts/mission_state_machine.py`

- [ ] **Step 1: 创建状态机脚本**

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""地面巡航比赛任务状态机。统一管理比赛流程。"""
import rospy
import time
from enum import Enum
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib

from config_loader import load_config, get_cell_center_xy
from mission_logger import MissionLogger


class MissionState(Enum):
    IDLE = 'IDLE'
    WAIT_FOR_WAKEUP = 'WAIT_FOR_WAKEUP'
    START_ANNOUNCE = 'START_ANNOUNCE'
    SEARCH_TASK_IMAGE = 'SEARCH_TASK_IMAGE_{}'   # format with 1-4
    RECOGNIZE_TASK_IMAGE = 'RECOGNIZE_TASK_IMAGE_{}'
    NAVIGATE_TO_TASK = 'NAVIGATE_TO_TASK_{}'
    ARRIVE_TASK = 'ARRIVE_TASK_{}'
    ANNOUNCE_TASK = 'ANNOUNCE_TASK_{}'
    NAVIGATE_TO_FINISH = 'NAVIGATE_TO_FINISH'
    ARRIVE_FINISH = 'ARRIVE_FINISH'
    FINISH_ANNOUNCE = 'FINISH_ANNOUNCE'
    DONE = 'DONE'
    # 异常状态
    ABORT_COLLISION_RISK = 'ABORT_COLLISION_RISK'
    ABORT_TIMEOUT = 'ABORT_TIMEOUT'
    ABORT_LOCALIZATION_LOST = 'ABORT_LOCALIZATION_LOST'
    ABORT_PERCEPTION_FAILED = 'ABORT_PERCEPTION_FAILED'
    ABORT_NAVIGATION_FAILED = 'ABORT_NAVIGATION_FAILED'
    MANUAL_STOP_REQUESTED = 'MANUAL_STOP_REQUESTED'

    @classmethod
    def task_image_state(cls, phase, step_name):
        return cls('{}_{}'.format(step_name, phase))


class MissionStateMachine(object):
    """比赛任务状态机。"""

    ABORT_STATES = {
        MissionState.ABORT_COLLISION_RISK,
        MissionState.ABORT_TIMEOUT,
        MissionState.ABORT_LOCALIZATION_LOST,
        MissionState.ABORT_PERCEPTION_FAILED,
        MissionState.ABORT_NAVIGATION_FAILED,
        MissionState.MANUAL_STOP_REQUESTED,
    }

    def __init__(self, sim_mode=False):
        self.sim_mode = sim_mode
        self.logger = MissionLogger()

        # 加载配置
        self.field_cfg = load_config('competition_field.yaml')
        self.mission_cfg = load_config('mission.yaml')
        self.voice_cfg = load_config('voice_text.yaml')

        # 状态机
        self.state = MissionState.IDLE
        self.task_index = 0           # 当前任务序号 (0-3，共 4 个)
        self.target_cell = None        # 当前目标任务点网格号
        self.task_cells_done = []      # 已完成的任务点列表
        self.state_start_time = None   # 当前状态开始时间
        self.mission_start_time = None

        # ROS 接口
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.voice_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # 心跳发布
        self.heartbeat_pub = rospy.Publisher('/mission_heartbeat', String, queue_size=1)

        # 订阅
        rospy.Subscriber('/start', String, self._on_wakeup)  # Snowboy 唤醒信号
        rospy.Subscriber('/vision_result', String, self._on_vision_result)
        rospy.Subscriber('/move_base/result', String, self._on_nav_result)

    def transition(self, new_state):
        rospy.loginfo('[Mission] %s -> %s', self.state.value, new_state.value)
        self.logger.log_state_transition(self.state.value, new_state.value)
        self.state = new_state
        self.state_start_time = time.time()

    # --- 状态机主循环 ---
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        self.mission_start_time = time.time()
        self.transition(MissionState.WAIT_FOR_WAKEUP)

        while not rospy.is_shutdown():
            self.heartbeat_pub.publish(String(data='alive'))
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
                rospy.loginfo('[Mission] All tasks completed.')
                break
            elif self.state in self.ABORT_STATES:
                self._handle_abort()
                break
            else:
                self._handle_task_phase()

            rate.sleep()

    # --- Wakeup Phase ---
    def _on_wakeup(self, msg):
        if self.state == MissionState.WAIT_FOR_WAKEUP:
            sim_trigger = self.sim_mode and msg.data == 'sim_wakeup'
            real_trigger = not self.sim_mode and msg.data == 'True'
            if sim_trigger or real_trigger:
                self.transition(MissionState.START_ANNOUNCE)

    def _handle_wait_for_wakeup(self):
        rospy.loginfo_throttle(5, '[Mission] Waiting for wake word...')

    # --- Start Announce ---
    def _handle_start_announce(self):
        text = self.voice_cfg['voice_text']['start']
        self._speak(text)
        self.logger.log_voice(text, 'start')
        self.task_index = 0
        next_state = MissionState.task_image_state(1, 'SEARCH_TASK_IMAGE')
        self.transition(next_state)

    # --- Task Phase (4 rounds) ---
    def _handle_task_phase(self):
        phase = self.task_index + 1
        search_state = MissionState.task_image_state(phase, 'SEARCH_TASK_IMAGE')
        recognize_state = MissionState.task_image_state(phase, 'RECOGNIZE_TASK_IMAGE')
        nav_state = MissionState.task_image_state(phase, 'NAVIGATE_TO_TASK')
        arrive_state = MissionState.task_image_state(phase, 'ARRIVE_TASK')
        announce_state = MissionState.task_image_state(phase, 'ANNOUNCE_TASK')

        if self.state == search_state:
            self._handle_search(phase)
        elif self.state == recognize_state:
            self._handle_recognize(phase)
        elif self.state == nav_state:
            self._handle_navigate_to_task(phase)
        elif self.state == arrive_state:
            self._handle_arrive_task(phase)
        elif self.state == announce_state:
            self._handle_announce_task(phase)

    def _handle_search(self, phase):
        text = self.voice_cfg['voice_text']['task_image_searching'].format(index=phase)
        self._speak(text)
        # TODO: 触发相机拍照 / 旋转搜索
        self.transition(MissionState.task_image_state(phase, 'RECOGNIZE_TASK_IMAGE'))

    def _handle_recognize(self, phase):
        # TODO: 等待 /vision_result 或模拟结果
        pass

    def _on_vision_result(self, msg):
        import json
        try:
            result = json.loads(msg.data)
            confidence = result.get('confidence', 0)
            min_conf = self.mission_cfg['confidence']['min_confidence']
            if confidence >= min_conf:
                self.target_cell = result['target_cell']
                self.logger.log_perception(result)
                phase = self.task_index + 1
                self.transition(MissionState.task_image_state(phase, 'NAVIGATE_TO_TASK'))
            else:
                rospy.logwarn('[Mission] Low confidence: %.2f, retrying...', confidence)
                self._retry_perception()
        except (ValueError, KeyError) as e:
            rospy.logerr('[Mission] Invalid vision result: %s', str(e))

    def _handle_navigate_to_task(self, phase):
        if self.target_cell is None:
            rospy.logerr('[Mission] No target cell set!')
            return
        x, y = get_cell_center_xy(self.target_cell, self.field_cfg)
        rospy.loginfo('[Mission] Navigating to cell %d (%.2f, %.2f)', self.target_cell, x, y)
        self._send_nav_goal(x, y)
        self.logger.log_navigation({'cell': self.target_cell, 'x': x, 'y': y}, None, True)
        self.transition(MissionState.task_image_state(phase, 'ARRIVE_TASK'))

    def _handle_arrive_task(self, phase):
        # TODO: 等待导航结果 + 区域内判定
        self.task_cells_done.append(self.target_cell)
        self.transition(MissionState.task_image_state(phase, 'ANNOUNCE_TASK'))

    def _handle_announce_task(self, phase):
        text = self.voice_cfg['voice_text']['task_arrived'].format(target_cell=self.target_cell)
        self._speak(text)
        self.logger.log_voice(text, 'task_arrived')

        if self.task_index >= 3:  # 4 个任务全部完成
            self.transition(MissionState.NAVIGATE_TO_FINISH)
        else:
            self.task_index += 1
            next_state = MissionState.task_image_state(self.task_index + 1, 'SEARCH_TASK_IMAGE')
            self.transition(next_state)

    # --- Finish Phase ---
    def _handle_navigate_to_finish(self):
        finish_cell = self.field_cfg['finish_cell']
        x, y = get_cell_center_xy(finish_cell, self.field_cfg)
        self._send_nav_goal(x, y)
        self.transition(MissionState.ARRIVE_FINISH)

    def _handle_arrive_finish(self):
        self.transition(MissionState.FINISH_ANNOUNCE)

    def _handle_finish_announce(self):
        text = self.voice_cfg['voice_text']['finish']
        self._speak(text)
        self.logger.log_voice(text, 'finish')
        self.transition(MissionState.DONE)

    # --- Abort ---
    def _handle_abort(self):
        rospy.logerr('[Mission] ABORT: %s', self.state.value)
        self.logger.log_system('abort', self.state.value)
        # 停止所有运动
        self.cmd_vel_pub.publish(Twist())

    # --- Helpers ---
    def _speak(self, text):
        """发送 TTS 播报指令。播报期间机器人必须停止。"""
        self.cmd_vel_pub.publish(Twist())  # 强制停止
        rospy.sleep(0.5)  # 稳定等待
        self.voice_pub.publish(String(data=text))
        rospy.sleep(2.0)  # TTS 播放时间估计，实际应等 feedback

    def _send_nav_goal(self, x, y, yaw=0.0):
        """发送 move_base 导航目标。"""
        import tf.transformations as tft
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        quat = tft.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        self.move_base_client.send_goal(goal)

    def _check_global_timeouts(self):
        """检查比赛全局超时。"""
        elapsed = time.time() - self.mission_start_time
        if elapsed > self.mission_cfg['mission']['max_time_s']:
            self.transition(MissionState.ABORT_TIMEOUT)
        state_elapsed = time.time() - (self.state_start_time or time.time())
        if state_elapsed > self.mission_cfg['timeouts']['no_state_change_s']:
            self.transition(MissionState.ABORT_TIMEOUT)

    def _retry_perception(self):
        rospy.loginfo('[Mission] Retrying perception...')


if __name__ == '__main__':
    rospy.init_node('mission_state_machine')
    sim_mode = rospy.get_param('~sim_mode', False)
    fsm = MissionStateMachine(sim_mode=sim_mode)
    fsm.run()
```

- [ ] **Step 2: 编译并检查语法错误**

```bash
# In WSL:
cp -r /mnt/d/StudyWorks/3.2/MachineVision_Project/AutonomousCruise_Ground/src/mission_manager /home/lx_hm/abot_ws/src/
cd /home/lx_hm/abot_ws && catkin_make --pkg mission_manager
python -m py_compile /home/lx_hm/abot_ws/src/mission_manager/scripts/mission_state_machine.py
echo "SYNTAX_CHECK: $?"
```

Expected: exit 0 (no syntax errors)

- [ ] **Step 3: 提交**

```bash
git add src/mission_manager/scripts/mission_state_machine.py
git commit -m "feat(mission): 实现任务状态机核心流转和全部状态处理"
```

### Task 9: Safety 安全监控

**Files:**
- Create: `src/mission_manager/scripts/safety_monitor.py`

- [ ] **Step 1: 创建安全监控脚本**

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""安全监控：watchdog、heartbeat、碰撞检测、超时检查。"""
import rospy
import time
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from mission_logger import MissionLogger


class SafetyMonitor(object):
    """安全监控器。"""

    def __init__(self):
        self.logger = MissionLogger()

        # 参数
        self.collision_range_m = rospy.get_param('~collision_range_m', 0.10)
        self.heartbeat_timeout_s = rospy.get_param('~heartbeat_timeout_s', 5.0)
        self.no_motion_timeout_s = rospy.get_param('~no_motion_timeout_s', 20.0)

        # 状态
        self.last_heartbeat_time = time.time()
        self.last_motion_time = time.time()
        self.last_odom = None
        self.motion_started = False
        self.estop_active = False

        # 订阅
        rospy.Subscriber('/scan_filtered', LaserScan, self._on_scan)
        rospy.Subscriber('/odom', Odometry, self._on_odom)
        rospy.Subscriber('/mission_heartbeat', String, self._on_heartbeat)
        rospy.Subscriber('/mission_estop', Empty, self._on_estop)

        # 发布
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/safety_status', String, queue_size=10)

    def _on_scan(self, msg):
        """检查激光数据，检测近距离障碍物（碰撞风险）。"""
        if self.estop_active:
            return
        for i, dist in enumerate(msg.ranges):
            if msg.range_min < dist < self.collision_range_m:
                rospy.logerr('[Safety] Collision risk! Range[%d]=%.3f', i, dist)
                self.logger.log_system('collision_risk', 'range[{}]={:.3f}'.format(i, dist))
                self._emergency_stop('collision_risk')
                return

    def _on_odom(self, msg):
        """检测机器人是否在运动。"""
        if self.last_odom is None:
            self.last_odom = msg
            return
        dx = msg.pose.pose.position.x - self.last_odom.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.last_odom.pose.pose.position.y
        dist = (dx * dx + dy * dy) ** 0.5
        if dist > 0.01:  # 移动超过 1cm
            self.last_motion_time = time.time()
            if not self.motion_started:
                self.motion_started = True
                self.logger.log_system('motion_started')
        self.last_odom = msg

    def _on_heartbeat(self, msg):
        """接收任务状态机心跳。"""
        self.last_heartbeat_time = time.time()

    def _on_estop(self, msg):
        """手动急停指令。"""
        rospy.logwarn('[Safety] Manual ESTOP requested!')
        self._emergency_stop('manual')

    def _emergency_stop(self, reason):
        self.estop_active = True
        self.cmd_vel_pub.publish(Twist())
        self.logger.log_system('emergency_stop', reason)
        self.status_pub.publish(String(data='ESTOP:{}'.format(reason)))

    def check_timeouts(self):
        """检查超时条件。"""
        now = time.time()
        if not self.motion_started:
            if now - self.last_heartbeat_time > self.no_motion_timeout_s:
                rospy.logerr('[Safety] No motion after start!')
                self._emergency_stop('no_motion_after_start')
        if now - self.last_heartbeat_time > self.heartbeat_timeout_s:
            rospy.logerr('[Safety] Heartbeat lost!')
            self._emergency_stop('heartbeat_lost')

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.check_timeouts()
            if not self.estop_active:
                self.status_pub.publish(String(data='OK'))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('safety_monitor')
    monitor = SafetyMonitor()
    monitor.run()
```

- [ ] **Step 2: 编译检查语法**

```bash
python -m py_compile /home/lx_hm/abot_ws/src/mission_manager/scripts/safety_monitor.py
echo "SYNTAX_CHECK: $?"
```

Expected: exit 0

- [ ] **Step 3: 提交**

```bash
git add src/mission_manager/scripts/safety_monitor.py
git commit -m "feat(safety): 实现安全监控 watchdog、碰撞检测、超时检查"
```

### Task 10: 改造 abot_vlm — 任务图像识别 Prompt

**Files:**
- Modify: `src/abot_vlm/scripts/doubao.py`

- [ ] **Step 1: 修改 VLM prompt 适配比赛任务**

定位 `doubao_vision_api()` 调用处（约第 41-60 行），将 PROMPT 参数改为从配置读取：

```python
# 原代码 (约第41行):
# def doubao_vision_api(PROMPT='解读以下图片包含的信息', img_path='...'):

# 改为:
def doubao_vision_api(PROMPT=None, img_path='/home/abot/abot_ws/src/abot_vlm/temp/vl_now.jpg'):
    if PROMPT is None:
        import rospy
        PROMPT = rospy.get_param('/perception/prompt_template',
            '你是一个机器人比赛的视觉识别系统。'
            '请识别图中围栏上的任务信息图像，'
            '输出图像内容和对应的任务点编号（31-33, 40-42, 49-51 之一）。'
            '回复格式必须是 JSON：'
            '{"target_cell": <数字>, "content": "<图像内容描述>", "confidence": <0-1之间的浮点数>}'
            '如果无法识别，设置 confidence 为 0。')
    # ... rest of function unchanged
```

- [ ] **Step 2: 提交**

```bash
git add src/abot_vlm/scripts/doubao.py
git commit -m "feat(perception): VLM prompt 适配比赛任务图像识别规格"
```

### Task 11: 导航参数调优 — 适配 3.6m 小场地

**Files:**
- Modify: `src/robot_slam/params/carto/costmap_common_params.yaml`
- Modify: `src/robot_slam/params/carto/dwa_local_planner_params.yaml`
- Modify: `src/robot_slam/params/carto/global_costmap_params.yaml`
- Modify: `src/robot_slam/params/carto/local_costmap_params.yaml`

- [ ] **Step 1: 调整 costmap 通用参数**

```yaml
# src/robot_slam/params/carto/costmap_common_params.yaml
# 改为适配 3.6m 场地的参数
obstacle_range: 1.5
raytrace_range: 2.0

footprint: [[-0.175, -0.15], [-0.175, 0.15], [0.175, 0.15], [0.175, -0.15]]

inflation_radius: 0.22
cost_scaling_factor: 10.0

map_type: costmap
observation_sources: scan
scan: {sensor_frame: laser_link, data_type: LaserScan, topic: scan_filtered, marking: true, clearing: true}
```

- [ ] **Step 2: 调整 DWA 局部规划器 — 精准到点**

```yaml
# src/robot_slam/params/carto/dwa_local_planner_params.yaml
DWAPlannerROS:
  max_vel_x: 0.55
  min_vel_x: -0.20
  max_vel_y: 0.20
  min_vel_y: -0.20
  max_vel_trans: 0.30
  min_vel_trans: 0.10
  max_vel_theta: 1.4
  min_vel_theta: 0.8
  acc_lim_x: 2.5
  acc_lim_theta: 3.0
  acc_lim_y: 2.5
  xy_goal_tolerance: 0.03       # 3cm 精准到点
  yaw_goal_tolerance: 0.02
  latch_xy_goal_tolerance: false
  sim_time: 1.7
  vx_samples: 10
  vy_samples: 0
  vth_samples: 20
  controller_frequency: 10.0
  path_distance_bias: 32.0
  goal_distance_bias: 24.0
  occdist_scale: 0.03
  forward_point_distance: 0.325
  stop_time_buffer: 0.2
  scaling_speed: 0.25
  max_scaling_factor: 0.2
  oscillation_reset_dist: 0.05
  publish_traj_pc: true
  publish_cost_grid_pc: true
  holonomic_robot: true
```

- [ ] **Step 3: 调整全局/局部 costmap 尺寸适配小场地**

```yaml
# src/robot_slam/params/carto/global_costmap_params.yaml (新建或覆盖)
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 2.0
  publish_frequency: 1.0
  static_map: true
  rolling_window: false
  width: 8.0      # 稍大于场地，确保完整覆盖
  height: 8.0
  resolution: 0.02
```

```yaml
# src/robot_slam/params/carto/local_costmap_params.yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true
  width: 2.0
  height: 2.0
  resolution: 0.02
```

- [ ] **Step 4: 编译验证参数加载**

```bash
# In WSL:
source /opt/ros/melodic/setup.bash
# 语法检查 YAML 文件
python -c "import yaml; yaml.safe_load(open('/home/lx_hm/abot_ws/src/robot_slam/params/carto/dwa_local_planner_params.yaml'))" && echo "YAML OK"
```

Expected: `YAML OK`

- [ ] **Step 5: 提交**

```bash
git add src/robot_slam/params/carto/
git commit -m "feat(navigation): 导航参数调优适配 3.6m 小场地，精准到点"
```

### Task 12: 统一启动入口与端到端仿真

**Files:**
- Modify: `src/mission_manager/launch/sim_mission.launch`
- Create: `launch/ground_cruise.launch`

- [ ] **Step 1: 创建完整比赛启动文件**

```xml
<!-- launch/ground_cruise.launch -->
<launch>
  <!-- 地面巡航比赛完整启动入口 -->
  <arg name="sim_mode" default="true"/>
  <arg name="map_name" default="competition_field"/>

  <!-- 1. 唤醒词检测 -->
  <include file="$(find robot_slam)/launch/GameStart.launch"/>

  <!-- 2. 导航栈 (move_base + AMCL + map_server) -->
  <include file="$(find robot_slam)/launch/navigation.launch">
    <arg name="map_name" value="$(arg map_name)"/>
  </include>

  <!-- 3. 任务图像识别 (VLM) -->
  <include file="$(find abot_vlm)/launch/vlm_node.launch"/>

  <!-- 4. 任务状态机 + 安全监控 + 配置加载 -->
  <include file="$(find mission_manager)/launch/sim_mission.launch">
    <arg name="sim_mode" value="$(arg sim_mode)"/>
  </include>
</launch>
```

- [ ] **Step 2: 更新 sim_mission.launch 增加 sim_mode 参数**

```xml
<!-- src/mission_manager/launch/sim_mission.launch (修改版) -->
<launch>
  <arg name="sim_mode" default="true"/>
  <arg name="config_dir" default="$(find mission_manager)/../../config"/>

  <rosparam command="load" file="$(arg config_dir)/competition_field.yaml" ns="competition_field"/>
  <rosparam command="load" file="$(arg config_dir)/mission.yaml" ns="mission"/>
  <rosparam command="load" file="$(arg config_dir)/robot.yaml" ns="robot"/>
  <rosparam command="load" file="$(arg config_dir)/navigation.yaml" ns="navigation"/>
  <rosparam command="load" file="$(arg config_dir)/perception.yaml" ns="perception"/>
  <rosparam command="load" file="$(arg config_dir)/voice_text.yaml" ns="voice_text"/>

  <node pkg="mission_manager" type="mission_state_machine.py" name="mission_state_machine" output="screen">
    <param name="sim_mode" value="$(arg sim_mode)"/>
  </node>

  <node pkg="mission_manager" type="safety_monitor.py" name="safety_monitor" output="screen"/>
</launch>
```

- [ ] **Step 3: 提交**

```bash
git add launch/ground_cruise.launch src/mission_manager/launch/sim_mission.launch
git commit -m "feat(launch): 添加比赛统一启动入口和仿真参数"
```

---

## 后续阶段规划

### Phase 2: M2 精准到点判定 (3 tasks)
- 实现 `check_footprint_in_region()` 函数，判断机器人 footprint 是否完全进入任务点区域
- 在 `_handle_arrive_task()` 中集成区域判定逻辑
- 创建 `tests/` 下的单元测试

### Phase 3: M3 任务图像识别完善 (3 tasks)
- 实现 VLM 识别失败的重试逻辑 (走 `_retry_perception()`)
- 添加相机拍照触发器（触发 `/top_view_shot_node/im_flag`）
- 实现 `perception_results.jsonl` 日志记录

### Phase 4: M4 语音播报完善 (2 tasks)
- 改造 `robot_slam/scripts/start.py` Snowboy 唤醒后发布 `/start` 消息
- 实现 TTS 播报完成回调，确保播报期间机器人静止

### Phase 5: M5 全链路联调 (4 tasks)
- 制作 3.6m 场地的先验地图（用于仿真）
- 端到端模拟运行：唤醒 → 4 次识别 → 4 次导航 → 播报 → 终点
- 添加随机挡板避障测试
- Safety 模块与其他模块联调

### Phase 6: M6-M7 鲁棒性与文档 (TBD)
- 异常场景覆盖：定位丢失、导航卡死、识别失败
- 180 秒计时压力测试
- 技术文档与参赛材料整理
