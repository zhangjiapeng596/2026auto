# Changelog

本文件遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/) 格式，使用 [Angular 提交规范](https://www.conventionalcommits.org/) 进行版本记录。

## [Unreleased]

### [1.2.0] - 2026-06-09

#### Changed
- **语音播报文案全面优化**：9 条播报文本重写，语义更清晰自然
- **关键节点 TTS 阻塞等待**：`_speak()` 新增 `wait` 参数，到达/识别完成/跳过/终点 4 处等播完再执行下一步
- **DWA 导航参数调优**：`xy_goal_tolerance` 0.12→0.06、`yaw_goal_tolerance` 0.20→0.30、`min_vel_theta` 0.05→0.01、`occdist_scale` 0.08→0.10、`inflation_radius` 0.04→0.08
- **里程计协方差拆分**：`cov_inflate` 拆出独立 `yaw_factor` 参数，平移×999/EKF 无视、旋转×35/EKF 不信任（IMU 主导）
- **任务/终点容差对齐**：`task_center_tolerance_m` 0.04→0.08、`finish_xy_tolerance_m` 0.12→0.08
- **比赛超时延长**：`max_time_s` 240→600s
- **工作空间重命名**：`abot_dev_ws` → `PN8RSR`

#### Fixed
- **GNOME 分支秒关**：`wait` 改为 `read` 阻塞，解决 `gnome-terminal` 退出后主脚本立即清理的问题
- **模式 1 无提示音**：GNOME 窗口 5 启动顺序改为 TTS→VLM→ASR，确保播报时 TTS 已就绪
- **`task_skip`/`task_image_failed` 补全播报**：原来只记日志不发声，现已激活

## [1.1.0] - 2026-06-08

### Added
- 轮式里程计协方差放大节点 `cov_inflate`，将 `/wheel_odom` 协方差×5 降权后发布到 `/wheel_odom_inflated`，robot_pose_ekf 更信任 IMU
- 开环逼近回退方法 `_openloop_approach`，move_base 重试耗尽后定时直行微调（仅用于 <0.3m 末端）

### Changed
- DWA 末端防抽搐：`min_vel_theta` 0.5→0.05，`xy_goal_tolerance` 0.03→0.06，`sim_time` 1.7→1.2
- DWA 旋转不绕路：`path_distance_bias` 32→20，`goal_distance_bias` 24→32
- AMCL 参数回归保守：`odom_alpha` 0.1→0.2，粒子 100/600→300/1500，`update_min_d` 0.06→0.12，`laser_z_rand` 0.05→0.10
- 版号更新：5 个 package 升至 1.0.0

### Fixed
- 终点定位丢失：测试终点 10s 直达，全程零重试零抽搐

## [1.0.0] - 2026-06-08

### Added
- 豆包 ASR 极速版语音唤醒（"开始比赛"语音触发，火山引擎 `volc.bigasr.auc_turbo`）
- 豆包 TTS V1 语音播报（任务点、到达、终点等场景，强制 HDMI 扬声器输出）
- 三种实车运行模式（Snowboy 唤醒 / SSH 后台 setsid / 模式3 自动开始）
- 比赛场地 8 点位视觉+任务导航流程（先拍照识别后导航执行）
- 远端导航实时监控脚本 `nav_monitor.py`
- 地图标点 GUI 工具 `mark_map_gui.py`（零依赖 tkinter，滚轮缩放+中键平移）
- `amcl_tf_bridge.py` 动态 `map→odom` TF 桥接，修复 AMCL 1.16.7 不发布 TF 的 bug
- `start_lidar_motor.py` LiDAR 电机防御启动节点
- 仿真系统：全向底盘模拟、动态障碍物、视觉点车头朝向、围墙点云
- 导航卡死检测（位移+yaw 变化监控，超时自动取消 goal）

### Changed
- 任务顺序重构为先拍照后导航执行（vision phase → task phase）
- 视觉点 `offset_m` 统一调整（0.20→0.35m，靠后防撞墙）
- 识别重试次数降低：`perception_retry_limit` 3→1
- 定位丢失监控阈值放宽：`max_pos_std_m` 0.8m，防站立起步误判
- 全局超时 180→200s（仿真围墙绕行）
- 下机流程新增远端源码清理（保留 build/devel）
- `rplidar.launch` 串口改为 `/dev/rplidar`，去掉 `respawn`

### Fixed
- LiDAR 电机不自动启动、节点同名冲突（去掉 respawn + udev 规则修复）
- 轮式里程计裸桥接 `/wheel_odom_relay` 与 `odom_ekf` 双源竞争（已移除 relay）
- AMCL 定位漂移 → 动态 TF 桥接 + 保守参数 + 协方差降权
- 终点定位丢失 → 开环回退兜底
- DWA 末端抽搐 → `min_vel_theta` 降至 0.05
- TTS 音频被蓝牙耳机劫持（强制 `alsa_output.pci-...hdmi-stereo-extra1` sink）
- VLM Py2/Py3 冲突（拆分为 Python 2 bridge + Python 3.9 worker 跨进程通信）
- 四脚本僵尸进程（trap cleanup EXIT INT TERM + 四层清理）
- SSH 多会话 ROS_MASTER_URI 不可达
- 相机 `/dev/video0→video1` 漂移（udev 规则固定 `/dev/usb_cam`）
- 实车 4 条链路 P0 崩溃（相机/位姿源/定位监控/VLM/启动时序）
- 仿真 `odom` 位姿冻结、heartbeat 刷屏、footprint 死循环
- 状态机 `<=2` 重试多跑一次导致开环触发不及时

### M7+ — 远端 LiDAR 启动问题治理 (2026-06-05)

#### Fixed
- **LiDAR 电机不自动启动**：rplidar_ros 1.7.0 构造时调 `stopMotor()`，需显式 `startScan()` 才启电机。新建 `start_lidar_motor.py` 防御节点，在 rplidarNode 就绪后自动调用 `/start_motor`
- **rplidarNode 同名冲突循环杀**：`rplidar.launch` 中 `respawn="true"` 配合竞争未清理的旧 roslaunch，导致多个同名节点互相顶替、电机状态丢失。去掉 `respawn` 与官方一致
- **udev 规则语法错误**：`/etc/udev/rules.d/rplidar.rules` 缺引号且 `ENV{ID_PATH}` 匹配阶段不可用，`/dev/rplidar` 从未创建。改用 `ATTRS{idVendor/idProduct}` 修复
- **旧进程残留**：`competition.sh` SSH 模式新增启动前 `pkill` + 端口等待清理，杜绝多 setsid 实例并存
- **VLM 识别重试过多**：`perception_retry_limit` 3→1，省 token

#### Changed
- `rplidar.launch`：串口路径改为 `/dev/rplidar`（对标参考实现）；去掉 `respawn="true"`
- `competition.sh`：SSH 模式新增进程清理段；删除冗余 `/start_motor` 重试循环
- `competition_field.yaml`：视觉点 `offset_m` 0.20→0.35m（靠后拍照）
- `mission.yaml`：`perception_retry_limit` 3→1；`max_pos_std_m` 0.8m（放宽定位阈值）

#### Added
- `start_lidar_motor.py`：等待 `/start_motor` 服务就绪后调用一次即退出，作为 rplidar.launch 子节点

### M7+ — 语音 ASR 极速版接入 (2026-06-05)

#### Changed
- `doubao_asr.py` 从旧 `/api/v1/asr` 调用切换为豆包语音识别大模型极速版 `recognize/flash`，使用 `audio.data` 传本地 WAV 的 base64 内容
- ASR 请求头改为 `X-Api-App-Key`/`X-Api-Access-Key`/`X-Api-Resource-Id=volc.bigasr.auc_turbo`，匹配极速版官方 API

#### Added
- `tools/test_doubao_asr_flash.py`：验证极速版 ASR 请求 URL、headers、body 和返回文本解析

### M7+ — 仿真系统重构 (2026-06-04)

#### Added
- **地图生成器** `tools/generate_map.py`：从 YAML 配置生成 PGM 地图，含围栏、9×9 网格线
- **障碍物系统**：YAML 配置 `{cell, edge}` 格式（edge=N/S/E/W），挡板居中放在网格线上；sim_robot 线段射线追踪模拟 LiDAR 检测；MarkerArray 发布实心薄板可视化
- **TF 可靠性**：`sim_robot` 发布 `/joint_states` 心跳驱动 `robot_state_publisher`，消除 `base_link does not exist`
- **视觉朝向**：`vision_to_task` 扩展为 `{target, yaw_rad}`，导航到视觉点时车头对墙
- **等待时长可配**：`mission.yaml` 新增 `waits` 字段（`arrival_stabilize_s`/`vision_trigger_delay_s`/`nav_poll_interval_s`）
- **代价地图可视化**：RViz 局部代价地图改用 `raw` 配色（所有格子可见），Cost Cloud 彩虹点云，障碍物 MarkerArray 显示

#### Changed
- **视觉导航流程**：`SEARCH_TASK_IMAGE` 从旋转扫描改为导航到 `vision_positions[5,37,45,77]`；`RECOGNIZE_TASK_IMAGE` 改为到达视觉点→触发VLM→获取任务区号
- **地图切换**：`mission.yaml` 新增 `map_name` 字段作为唯一地图配置源，`sim_full_test.sh` 自动读取
- **脚本**：`sim_full_test.sh` 重写（165→65 行），修复 ROS_MASTER_URI 丢失/固定 sleep/交互阻塞/`rostopic` 超时
- **默认地图**：`game` → `competition_field`
- **状态机**：移除 `_search_rotation` 旋转搜索逻辑、`rotation_attempt`、`_vision_handled_by_cb`
- **mark_map_gui**：新增两点点击标定车头朝向（yaw），箭头渲染

#### Fixed
- `sim_robot.py` 修正射线-线段求交 `u` 参数符号，避免 LiDAR 将挡板投影到线段延长线上造成假障碍
- `competition_field.yaml` 默认挡板改为内部 8×8 方框边缘；`sim_robot.py`/`generate_map.py` 对外边界 edge 和缺失 edge 直接报错，避免挡板跑到场地外
- `mission_state_machine.py` 到达任务点后的稳定等待、导航轮询间隔、footprint 修正次数改为读取 `mission.yaml`，避免 move_base 成功后继续长时间反复等待
- `mission_state_machine.py` 任务点到达判定新增 footprint 优先放行与任务中心容差降级放行，视觉点/终点新增位姿容差放行，避免 move_base 末端旋转恢复拖满 30s
- `dwa_local_planner_params.yaml` 收紧 `xy_goal_tolerance` 并降低 `min_vel_trans`，避免 5cm 容差提前宣布到达导致 footprint 反复修正
- `sim_full_mission.launch` 调快 MockTTS 播报参数，避免仿真链路被模拟语音时长吃掉终点导航时间
- `dwa_local_planner_params.yaml` 为全向底盘启用 `vy_samples` 横移采样，避免挡板附近只能前后/旋转导致卡死
- RViz 默认关闭 raw local costmap 覆盖层，避免 3m rolling window 黑块遮挡全局地图和围栏
- `sim_robot.py` 挡板 marker/LiDAR 线段中心按 `cell+edge` 精确落在内部格线边上
- `competition_field.yaml` 网格坐标说明修正，消除文档公式相对实际代码偏半格的问题
- `sim_full_test.sh` 补充同步 `robot_slam/params/carto/*.yaml` 与 RViz 配置，避免 Windows 侧导航参数修改未生效
- `sim_full_test.sh` 在 source ROS 环境时临时关闭 `nounset`，避免 `ROS_DISTRO: unbound variable` 导致同步前退出
- `sim_full_test.sh` 5 处卡死：`grep -P` locale 限制、ROS_MASTER_URI 丢失、固定 `sleep 20`、`read -r` 阻塞、`rostopic echo` 无超时
- `sim_robot.py`：`self.obstacle_xy` → `self.obstacle_segments` 变量名错误导致节点崩溃
- `sim_robot.py`：`visualization_msgs` 导入容错
- `sim_robot.py`：YAML 加载 try/except 防崩溃
- PGM 网格线 y 坐标镜像翻转（`-1.8+r*0.4` → `1.8-r*0.4`）
- PGM 像素转换 `int()` 截断改为 `int(round())`，消除 1px 漂移
- 膨胀层：`inflation_radius: 0.22 → 0.005`
- `grep -c` 零匹配时双输出导致 `NODE_COUNT='0\n0'`
- `sim_navigation.launch` 新增 `joint_state_publisher`

#### Removed
- PGM 静态障碍物绘制（改为 sim_robot LiDAR 动态注入）

### M7+ — 代码审计修复 (2026-06-04)

#### Fixed
- H1: `_handle_arrive_task` 中 footprint 硬编码改为从 `robot.yaml` 配置读取，带 fallback 默认值
- H2: `_handle_abort` 补全 6 种 abort 类型的语音播报文本映射（原来只覆盖 2 种，新增 `abort_localization_lost`/`abort_perception_failed`/`abort_navigation_failed`/`abort_manual_stop`）
- H3: 引用 `voice_text.yaml` 中 5 个已定义但未使用的模板（`wakeup_detected`/`task_image_recognized`/`task_image_failed`/`navigating_to_task`/`finish_arrived`）
- H3 附带修复: `_retry_perception` 播报顺序 — max_retries 耗尽时不再误导性播报"正在重试"
- H3 附带修复: 移除 `_handle_finish_announce` 与 `_handle_arrive_finish` 之间的"比赛结束"重复播报
- M1: `mission_logger.py` 添加 Python 2 UTF-8 编码兼容头
- M2: `_on_safety_status` 改为精确 `==` 匹配 `ESTOP:<reason>`，避免子串误匹配
- M3: `_handle_arrive_task` 导航重试前显式 `cancel_goal()`，与超限分支保持一致
- M4: `_handle_arrive_finish` 导航重试前显式 `cancel_goal()`，与超限分支保持一致
- L2: `mock_tts.py` 添加 Python 2 UTF-8 编码兼容头

#### Changed
- `_handle_start_announce` 播报模板从 `start` 改为 `wakeup_detected`

### M6 — 鲁棒性增强 (2026-06-03)

#### Added
- 导航卡死检测：`_handle_arrive_task` 改用轮询循环，每 2s 检查 robot 位移，超过 `nav_stuck_timeout_s`(10s) 无进展则取消导航并重试
- `mission.yaml` 新增 `nav_stuck_timeout_s` 参数

#### Changed
- 导航等待从阻塞 `wait_for_result()` 改为轮询循环，支持实时进度监控

### M5 — 全链路就绪 (2026-06-03)

- Game 地图（1056×992px, x∈[-15.4,11.0], y∈[-13.8,11.0]）已接入仿真
- 3.6m 比赛场地位于 map origin，task cells 均在地图可通行范围内
- 需用户在 WSL 中运行 `roslaunch mission_manager sim_full_mission.launch` 验证

### M4 — 语音播报完成回调 (2026-06-03)

#### Added
- `mock_tts.py`：Mock TTS 仿真节点，订阅 `/voiceWords`，按字数估算时长，完成后发布 `/tts_done`
- `_speak()` 改为 `tts_done_event.wait()` 阻塞等待播报完成，替代固定 `rospy.sleep`
- 播报期间机器人保持停止，超时 10s(sim)/20s(real)

#### Changed
- `sim_full_mission.launch` 节点数 9→10，新增 mock_tts

### 仓库裁剪 (2026-06-03)

#### Removed
- 功能包：user_demo, abot_find, hector_slam, imu_filter（与本次比赛无关）
- 脚本：8 个远端 ABOT 旧 sh + 18 个旧 Python（AR标签/射击/demo/旧导航）

### M3 仿真调试 — 关键修复 (2026-06-03)

#### Fixed
- rosnode list XML-RPC 调用无超时卡 134s — 改为从 log 文件读取节点进程
- 端口 11311 TIME_WAIT 60s — 改随机端口 + tcp_tw_reuse
- launch-prefix 语法不兼容 — 状态机内置 sim_mode 5s 自动唤醒
- Python 2 ASCII 编码崩溃 — 所有脚本加 `sys.setdefaultencoding('utf-8')`
- sim_robot 起点 (-1.5,1.5) 在地图外 — 改为 (0,0) + 创建 competition_field 地图
- map→odom TF 缺失 — sim_robot 发布 identity transform
- 安全监控不检测角运动 — _on_odom 增加 yaw 变化判断
- 安全监控 heartbeat 时机 — 改为 rospy.Timer(2s) 独立线程
- 导航结果订阅类型错误 — String 改为 SimpleActionClient.wait_for_result
- 感知重试计数器无限重置 — 移到任务切换时重置
- mission_logger 并发崩溃 — os.makedirs 加 try/except
- /abot/pose 仿真无发布者 — 增加 /odom 备选位姿源

#### Changed
- 仿真启动从复杂脚本简化为单条 roslaunch 命令
- 默认地图从 my_lab 改为 competition_field（3.6m×3.6m 场地）

### M3 — 任务图像识别完善 (2026-06-03)

#### Added

- `sim_robot.py` 仿真机器人节点：发布 mock odometry、laser scan、TF，订阅 cmd_vel 模拟运动，支持全链路仿真测试
- `sim_navigation.launch` 仿真导航启动文件：map_server + sim_robot + AMCL + move_base，用于 WSL 无硬件的完整导航验证
- `_search_rotation()` 旋转搜索：找不到图像时原地旋转 90 度扫描围栏四个方向，每个方向触发相机并等待识别结果
- `mock_vlm.py` Mock VLM 仿真节点：模拟豆包大模型返回预设任务序列，支持 WSL 无摄像头的完整链路测试
- `seen_image_ids` 去重逻辑：防止同一张任务图像被识别 4 次，已识别过的 image_id 自动跳过

#### Changed

- `_handle_recognize_task_image` 增加旋转重试：每方向等待 10s，4 个方向全部失败才触发 `_retry_perception`
- `_retry_perception` 改为接收 `phase` 参数，重置旋转计数后回到 SEARCH 状态从头开始
- `_on_vision_result` 增加 image_id 去重检查 + 低置信度旋转重试（而非直接重试）

#### Fixed

- 修复 3 个 map YAML 文件（my_lab.yaml, shoot.yaml, my_map.yaml）中硬编码的绝对路径（指向 `/home/abot/` 和 `/home/bcsh/`），改为相对路径，使 map_server 可正常加载地图

### M2 — 精准到点判定 (2026-05-24)

#### Added

- `check_footprint_in_region()` 精准到点判定函数（射线法判断 footprint 四顶点是否全在任务区域内）
- `_on_pose` 回调订阅 `/abot/pose` 获取机器人实时位姿
- `_get_current_pose()` 位姿读取方法
- 三种运行模式启动脚本：`mapping.sh`（8 节点）、`navigation_test.sh`（10 节点）、`competition.sh`（14 节点）
- `my_lab` 先验地图（从远端 ABOT 备份拷贝，用于仿真）
- README.md 重写：实际开发环境、9 个功能包详细职责、构建命令、开发进度

#### Changed

- `_handle_arrive_task` 增加 footprint 区域验证：导航成功后等待位姿稳定 → 检查 footprint → 不通过则发送修正目标
- CLAUDE.md 补充节点架构文档（三种模式树状图 + 数据流 + 14 个节点清单）

### M1 — 配置系统与状态机骨架 (2026-05-24)

#### Added

- `config/` 6 个 YAML 参数配置文件（competition_field / mission / robot / navigation / perception / voice_text）
- `src/common/` 公共工具包：`config_loader.py`（配置加载 + 网格坐标转换）、`mission_logger.py`（JSONL 结构化日志）
- `src/mission_manager/` 任务管理包：`mission_state_machine.py`（438 行完整状态机，6 个正常状态 + 6 个异常状态）、`safety_monitor.py`（激光碰撞检测 + 里程计运动监控 + heartbeat watchdog + 急停）
- `launch/ground_cruise.launch` 比赛统一启动入口
- Snowboy 唤醒词模型 `startGame.pmdl`
- FunASR Paraformer 中文语音识别模型（本地保留，gitignore）

#### Changed

- `abot_vlm/doubao.py` VLM prompt 改为比赛任务图像识别规格（从 ROS param `/perception/prompt_template` 读取）
- `robot_slam/params/carto/` 导航参数调优适配 3.6m 小场地（xy_goal_tolerance 0.03m, footprint 175×150mm, inflation_radius 0.22m）
- MissionState 从 Python 3 `Enum` 改为 Python 2 兼容的自定义类

#### Fixed

- `mission_logger.py` 移除 `os.makedirs(exist_ok=True)` Python 3 专有参数

### M0 — 仓库初始化 (2026-05-23)

#### Added

- `docs/PROJECT_REQUIREMENTS_GROUND_CRUISE.md` 比赛完整需求说明（来源：第二十八届中国机器人及人工智能大赛比赛规则）
- `README.md` 项目说明
- `CLAUDE.md` Claude Code 协作指引
- 远端 ABOT 设备 `abot_ws/` 源码同步（17 个功能包，保留 7 个有用包）
- 比赛场地图与机器人参数表素材
- `.gitignore` 配置（build/devel/logs/__pycache__/大模型文件/API_KEY）
