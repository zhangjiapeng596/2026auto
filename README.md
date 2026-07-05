# 自主巡航 — 地面巡航场景

[![Release](https://img.shields.io/badge/release-v1.2.0-blue)](https://github.com/LX-HMKK/AutonomousCruise_Ground/releases/tag/v1.1.0)
[![ROS](https://img.shields.io/badge/ROS-Melodic-green)](https://www.ros.org/)
[![Python](https://img.shields.io/badge/Python-2.7%20%7C%203.9-yellow)](https://www.python.org/)

第二十八届中国机器人及人工智能大赛 · 机器人任务挑战赛：自主巡航（场景一：地面巡航场景）。

## 项目目标

实现一套自主移动机器人系统，完成完整比赛链路：

```text
语音唤醒 → 启动比赛 → 自主巡航搜索任务图像 → 视觉识别任务信息
→ 解析目标任务点 → 导航到点 → 静止语音播报 → 下一任务
→ 全部任务完成 → 导航到终点 → 静止语音播报结束
```

## 场地与规则摘要

| 项目 | 说明 |
|---|---|
| 场地 | 3.6 m × 3.6 m，9×9 网格，围栏高 30 cm |
| 任务点 | 9 个（网格 31-33, 40-42, 49-51），尺寸 38 cm × 32 cm |
| 任务图像 | 4 张，围栏内侧，中心高 20 cm |
| 起点 | 网格 1 或 81 |
| 终点 | 网格 9 |
| 总时长 | 180 秒 |
| 机器人 | 350 mm × 300 mm × 240 mm，四麦克纳姆轮，全向运动 |

完整规则与需求说明见 [`docs/PROJECT_REQUIREMENTS_GROUND_CRUISE.md`](docs/PROJECT_REQUIREMENTS_GROUND_CRUISE.md)。

## 开发环境

| 项目 | 说明 |
|---|---|
| OS | Windows WSL Ubuntu 18.04 |
| ROS | Melodic (Python 2.7) |
| 工作空间 | `~/3X2KY2/` (catkin) |
| 仿真 | 先验地图 + mock 数据 |
| 远端设备 | ABOT 机器人 `172.16.26.119` |

## 目录结构

```
.
├── config/                              # 比赛参数配置（禁止硬编码）
│   ├── competition_field.yaml           # 场地/网格/起终点/任务点/视觉点/障碍物(cell+edge格式)
│   ├── mission.yaml                     # 超时/重试/置信度/地图名/等待时长
│   ├── robot.yaml                       # 机器人 footprint、传感器参数
│   ├── navigation.yaml                  # costmap、DWA、规划器参数（参考）
│   ├── perception.yaml                  # VLM prompt 模板、相机设置
│   └── voice_text.yaml                  # 播报文本模板（12 条）
├── src/                                 # ROS 功能包
│   ├── mission_manager/                 # 【核心】任务状态机 + 安全监控 + 仿真 mock
│   ├── common/                          # 配置加载、日志、网格坐标转换
│   ├── robot_slam/                      # 导航定位/建图/ASR/唤醒词
│   ├── abot_base/                       # ABOT 底盘驱动/IMU/URDF 模型/激光滤波/协方差降权
│   └── abot_vlm/                        # 豆包大模型视觉识别（任务图像）
├── tools/                               # Windows 端工具
│   ├── generate_map.py                  # YAML → PGM 地图生成（围栏+网格线）
│   └── mark_map_gui.py                  # PGM 可视化标点（含车头朝向）
├── launch/
│   └── ground_cruise.launch             # 比赛统一启动入口
├── scripts/                             # 远端 ABOT 启动脚本（参考）
├── docs/                                # 需求文档 + 实现计划
└── logs/                                # 运行日志（gitignore）
```

### 功能包详细说明

#### mission_manager（新建，Python）
比赛任务状态机，今年核心模块。

| 文件 | 职责 |
|---|---|
| `scripts/mission_state_machine.py` | 完整状态机：WAKEUP → 导航到视觉点(5/37/45/77) → 触发VLM获取任务区号 → 导航到任务区(31-51) → footprint判定 → 播报 → 循环4次 → 导航到终点(9) → DONE。6个异常状态。支持车头朝向对齐、视觉重试、导航卡死检测。 |
| `scripts/safety_monitor.py` | 安全监控：激光碰撞检测、里程计运动监控、heartbeat watchdog（5s 超时） |
| `scripts/mock_vlm.py` | Mock VLM：从 `competition_field.yaml` 读取 `vision_to_task` 映射，按序返回目标任务区号 |
| `scripts/mock_tts.py` | Mock TTS：订阅 `/voiceWords`，按字数估算时长后发布 `/tts_done` |
| `scripts/sim_robot.py` | 仿真机器人：odom + scan(含线段障碍物射线追踪) + 完整TF链(map→base_footprint→base_link→laser_link) + joint_states心跳 + 障碍物MarkerArray可视化 |

RViz 中默认显示全局地图、全局代价地图、local plan、cost cloud 和仿真挡板。`Local Costmap` 是随机器人移动的 3m rolling window，会伸出 3.6m 场地；raw 配色下未知区域会显示为黑色，因此默认关闭，需要调试局部代价地图时再手动启用。

#### common（新建，Python）
公共工具包，提供配置加载、日志、坐标变换。

| 文件 | 职责 |
|---|---|
| `scripts/config_loader.py` | YAML 配置加载（多路径搜索）、网格编号 → map 坐标转换 `get_cell_center_xy()`、footprint 区域判定 `check_footprint_in_region()`（射线法） |
| `scripts/mission_logger.py` | 线程安全 JSONL 日志，按 `run_YYYYMMDD_HHMMSS/` 组织，5 个日志流：状态跳转/感知结果/导航目标/语音事件/系统事件 |

#### robot_slam（复用，C++ + Python）
导航定位和语音交互的核心包，来自 2025 比赛代码。

| 模块 | 关键文件 | 职责 |
|---|---|---|
| 定位 | AMCL (`include/amcl.launch.xml`) + `amcl_tf_bridge.py` | 2D 激光雷达实时定位，保守参数抗轮式打滑 |
| 导航 | `include/move_base.launch.xml` + `params/carto/*.yaml` | move_base + DWA 局部规划 + GlobalPlanner 全局规划，已调优适配 3.6m 场地 |
| 唤醒词 | `scripts/start.py` + `resources/models/startGame.pmdl` | Snowboy 热词检测，触发后发布 `/start`（当前使用豆包 ASR 替代） |
| 语音识别 | `scripts/doubao_asr.py` | 火山引擎豆包 ASR 极速版中文识别，发布 `/start` |
| TF 桥接 | `scripts/amcl_tf_bridge.py` | 动态计算 `map→odom` TF，修复 AMCL 1.16.7 不发布 TF 的 bug |
| 建图 | `launch/gmapping.launch` `launch/hector_mapping.launch` | Gmapping / Hector SLAM 建图 |
| 地图 | `maps/competition_field.yaml, game.yaml, my_lab.yaml` | 先验地图（仿真用 competition_field 3.6m 场地, game 实赛场） |

#### abot_base（复用，C++）
ABOT 机器人底层硬件驱动，**不改动**。

| 子包 | 职责 |
|---|---|
| `abot_bringup/` | 串口底盘驱动 (`/dev/abot`, 921600bps)，里程计发布 `/odom`，TF 变换，订阅 `/cmd_vel` |
| `abot_imu/` | IMU AHRS 驱动，发布九轴 IMU 数据 |
| `abot_model/` | URDF 机器人模型描述 + Gazebo 仿真 |
| `lidar_filters/` | 激光雷达 BoxFilter，去除机器人自身点云（`/scan` → `/scan_filtered`） |

#### abot_vlm（复用，Python）
大模型视觉识别，订阅 `/usb_cam/image_raw`，触发后拍照发送豆包 Vision Pro API (`doubao-1-5-vision-pro-32k-250115`)，发布 JSON 识别结果到 `/vision_result`。Prompt 已改为比赛任务图像识别规格。

## 构建与运行

### 前置依赖

- WSL Ubuntu 18.04 + ROS Melodic
- Python 2.7 + 依赖包（rospy, yaml, tf, cv2 等）

### 编译

```bash
source /opt/ros/melodic/setup.bash
cd ~/3X2KY2
catkin_make
source devel/setup.bash
```

### 启动仿真（WSL，推荐）

```bash
# 一键：同步源码 → 启动 → 监控 → 报告
bash /mnt/d/edge下载/3X2KY2/scripts/sim_full_test.sh
```

自动完成：清理 → 同步 Windows 源码到 WSL → 启动 9 个节点 → 等待比赛完成 → 输出报告。

### 启动仿真（直接 roslaunch，需先同步）

```bash
# 先手动同步一次
cp /mnt/d/.../src/mission_manager/**/*.py ~/3X2KY2/src/mission_manager/
cp /mnt/d/.../config/*.yaml ~/3X2KY2/config/
cp /mnt/d/.../src/robot_slam/maps/competition_field.* ~/3X2KY2/src/robot_slam/maps/

# 再启动
source /opt/ros/melodic/setup.bash && source ~/3X2KY2/devel/setup.bash
roslaunch mission_manager sim_full_mission.launch map_name:=competition_field
```

### 修改比赛参数

编辑 `config/` 目录下的 YAML 文件，无需重新编译：

| 文件 | 关键配置 |
|---|---|
| `competition_field.yaml` | 场地尺寸、vision_positions(视觉点)、vision_to_task(视觉→任务区映射,含yaw)、obstacles(cell+edge格式) |
| `mission.yaml` | map_name(地图切换)、总时长、超时、重试、waits(到达稳定/VLM触发/导航轮询)、navigation(视觉点/终点容差) |
| `robot.yaml` | 尺寸、footprint |
| `costmap_common_params.yaml` | inflation_radius(膨胀层)、footprint |
| `dwa_local_planner_params.yaml` | xy_goal_tolerance(到达容差)、occdist_scale(避障权重)、速度/加速度 |
| `voice_text.yaml` | 各阶段播报模板 |

### 障碍物配置

```yaml
# competition_field.yaml — cell+edge 格式, 挡板放在 9×9 网格的内部边上
obstacles:
  - { cell: 20, edge: S }   # 20号南边, 水平挡板
  - { cell: 22, edge: E }   # 22号东边, 竖直挡板
  # edge: N/S/E/W, 不能指向场地外边界；yaw_deg 可覆盖自动朝向
```

## 开发进度

| 阶段 | 目标 | 状态 |
|---|---|---|
| M0 | 规则建模与仓库整理 | ✅ 完成 |
| M1 | 配置系统 + 状态机骨架 + 安全监控 | ✅ 完成 |
| M2 | 导航参数调优 + 精准到点判定 | ✅ 完成 |
| M3 | 任务图像识别完善（旋转搜索、mock VLM、仿真验证） | ✅ 完成 |
| M4 | 语音播报完善（mock TTS、播报完成回调） | ✅ 完成 |
| M5 | 完整任务链路联调（game 地图接入、坐标系验证） | ✅ 完成 |
| M6 | 鲁棒性增强（导航卡死检测、footprint 重试降级） | ✅ 完成 |
| M7 | 参赛文档与答辩准备 | ✅ 完成 |
| M8 | EKF 里程计降权 + DWA 防抽搐修复（v1.1.0 发布） | ✅ 完成 |

## 提交规范

遵循 [Angular 提交规范](https://www.conventionalcommits.org/)，使用中文编写提交信息：

```
<type>(<scope>): <中文简述>
```

示例：`feat(mission): 实现任务状态机核心流转逻辑`

详见 [CHANGELOG.md](CHANGELOG.md) 和 [CLAUDE.md](CLAUDE.md)。
