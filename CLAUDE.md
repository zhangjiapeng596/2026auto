# CLAUDE.md

## ⚠️ 铁律（不可违反）

1. **未经用户审查和明确同意，禁止提交或推送代码** — 包括本地 `git commit` 和远端推送。写完代码后必须先让用户审查，用户说"commit"才能提交
2. **禁止触碰 ABOT 原厂 `~/abot_ws/`** — 只操作 `~/3X2KY2_backup2/`
3. **禁止修改实车共享文件而不评估影响** — 这些文件原厂也在用：`navigation.launch`、`costmap_common_params.yaml`、`amcl.launch.xml`、`robot_with_imu.launch` 及其 include 链
4. **禁止未经验证就声称"修好了"** — 修改仿真相关代码后必须在 WSL 跑完整仿真；实车相关代码须远端验证
5. **最小侵入修改** — 不引入非必要大型依赖，优先最小可验证修改
6. **禁止写假代码** — 无实际功能的占位代码、永远不会执行的分支都不允许

## 快速命令

```bash
# WSL 仿真完整测试（一键，自动同步+编译前清理）
wsl bash /mnt/d/edge下载/3X2KY2_backup2/scripts/sim_full_test.sh

# 远端同步（Windows → ABOT）
tar czf - --exclude='__pycache__' --exclude='*.pyc' --exclude='.git' -C <本机仓库> src config scripts | ssh abot@<IP> 'cd ~/3X2KY2_backup2 && tar xzf -'

# 远端单文件同步
scp <本地路径> abot@<IP>:~/3X2KY2_backup2/<远端路径>

# 远端编译
ssh abot@<IP> 'source /opt/ros/melodic/setup.bash && cd ~/3X2KY2_backup2 && catkin_make'

# 远端启动比赛
ssh abot@<IP> 'bash ~/3X2KY2_backup2/scripts/competition.sh competition_field true'

# 远端优雅停止（不再需要 pkill 盲杀）
ssh abot@<IP> 'bash ~/3X2KY2_backup2/scripts/competition.sh --stop'

# 远端监控
ssh abot@<IP> 'source /opt/ros/melodic/setup.bash && source ~/3X2KY2_backup2/devel/setup.bash && rosrun robot_slam nav_monitor.py'

# ASR 语音识别测试（免 ROS 环境）
python tools/test_doubao_asr_flash.py
```

## 环境

- **本机**: Windows WSL Ubuntu 18.04 + ROS Melodic，工作空间 `~/3X2KY2_backup2/`
- **远端**: ABOT `172.16.26.119`。开发工作空间 `~/3X2KY2_backup2/`
- **远端当前 IP** 以用户告知为准。连接方式为 SSH `abot@<IP>`

### 解释器隔离

远端 ROS Melodic 需要 Python 2.7，但 anaconda 默认 Python 3。通过 PATH 控制：
- **py2 节点**（ROS 工具、状态机、底盘驱动）：`export PATH="/usr/bin:/opt/ros/melodic/bin:$PATH"`
- **py39 节点**（VLM、ASR、TTS）：`/home/abot/anaconda3/envs/py39/bin/python3.9`，通过 shim 目录暴露为 `python3`

### 音频路由

- **TTS 输出** → HDMI 扬声器（`PULSE_SINK=alsa_output.pci-...hdmi-stereo-extra1`），`doubao_tts.py` 已硬编码，不被蓝牙耳机劫持
- **ASR 输入** → 系统默认麦克风（蓝牙耳机连接时为蓝牙麦克风）
- `doubao_tts.py` 支持 `~speaker_sink` ROS param 覆盖输出设备

## 远端操作流程

### 上机
用户说"上机"/"推送"/"远端测试"时：
1. **增量同步**：仅 scp 变更的文件到 ABOT `~/3X2KY2_backup2/`（禁止全量 tar，减少耗时）
2. 在远端 `catkin_make` 编译（仅 C++ 修改时需要；纯 Python 修改跳过）
3. Source `~/3X2KY2_backup2/devel/setup.bash` 运行

### 增量同步示例
```bash
# 单文件
scp <本地路径> abot@<IP>:~/3X2KY2_backup2/<远端路径>
# 多文件
scp <本地路径1> <本地路径2> abot@<IP>:~/3X2KY2_backup2/<远端路径>/
```

### 两端同步
用户说"两端同步"/"同步"时：
1. 本机是源码唯一源头（git 仓库），远端 `~/3X2KY2_backup2/` 不是 git 仓库
2. **本机 → 远端**：scp 本机修改过的文件到 ABOT 对应路径
3. **本机 → GitHub**：`git commit` + `git push` 将改动提交到远端仓库
4. 提交完成后确认两端一致

### 下机
用户说"下机"/"远端使用时间结束"时：
1. 将远端 `~/3X2KY2_backup2/` 修改文件 `scp` 回本机
2. 确认改动已同步后告知用户
3. ABOT 公用 → 清远端源码（保留 devel/build 免重编译）：`ssh abot@<IP> 'rm -rf ~/3X2KY2_backup2/src ~/3X2KY2_backup2/config ~/3X2KY2_backup2/scripts ~/3X2KY2_backup2/launch'`

### 启动前检查
- SSH 到 ABOT，`ps aux | grep ros` 确认无人占用
- 若原厂 `~/demo/` 系统在跑，等其结束
- 若需强制清理：`ssh abot@<IP> 'bash ~/3X2KY2_backup2/scripts/competition.sh --stop'`

### 进程管理（防僵尸）
四脚本全部加了 `trap cleanup EXIT INT TERM`，四层清理（SIGTERM → sleep 3s → SIGKILL → pkill 兜底）：
- `competition.sh` SSH 模式：内层 setsid 脚本自带 trap + PID 文件，`--stop` 精准停止
- 脚本正常退出 / Ctrl+C / SSH 断连 / `--stop` 都会触发清理

## 目录结构

```
config/          # 比赛参数 YAML（禁止硬编码）
scripts/         # 启动脚本（competition/mapping/navigation_test/sim_full_test）
tools/           # 辅助工具（地图生成/标注、语音/ASR 测试）
docs/            # 项目需求文档 + superpowers 设计规范/实施计划
src/
├── mission_manager/  # 任务状态机 + 安全监控 + 仿真 mock
├── common/           # 公共工具
├── robot_slam/       # 导航定位 + 建图 + 语音（doubao_tts/asr）
├── abot_base/        # 底层驱动（尽量不改）
└── abot_vlm/         # 豆包 VLM 视觉识别
```

## Git 提交规范

Angular 格式，中文消息：`<type>(<scope>): <简述>`。type: feat/fix/docs/style/refactor/test/chore/perf。
**禁止在 commit message 中添加 `Co-Authored-By: Claude` 等 Claude 协作者信息。**

## 关键开发陷阱

| 陷阱 | 解决 |
|------|------|
| `rosnode list` XML-RPC 无超时卡死 134s | `grep "process\[" log_file` 替代 |
| Python 2 中文编码崩溃 | `reload(sys); sys.setdefaultencoding('utf-8')` |
| ROS 端口 11311 TIME_WAIT 60s | `competition.sh` 已内置 `lsof -ti:11311` 等待 |
| 仿真唤醒等待 /start | `sim_mode=true` 下状态机 5s 自动唤醒 |
| 起点在地图外 → AMCL 发散 | 检查 init_x/init_y 是否在 map 范围内 |
| heartbeat 不执行 | `rospy.Timer(2s)` 独立线程，不要靠 rospy.spin |
| AMCL 定位发散（全局地图漂移） | 用 `amcl_tf_bridge.py` 动态计算 `map→odom` TF 广播，时间戳对齐 AMCL header.stamp |
| TF 旋转方向反 → 点云漂移 | `_invert_transform` 旋转公式 `conj(q)*(-t)*q`，不是 `q*(-t)*conj(q)` |
| DWA 原地振荡不前进 | 增大 `osc_reset`（0.05→0.10），配合 `laser_max_range` 5m 减少盲区死锁 |

## 比赛约束

| 约束 | 值 |
|------|-----|
| 场地 / 网格 | 3.6m×3.6m，9×9，每格 0.4m×0.4m |
| 任务点 | 9 个（网格 31-33/40-42/49-51），38cm×32cm |
| 总时长 | 180s |
| 无状态变化上限 | 20s |
| 机器人 | 350mm×300mm×240mm，4 麦克纳姆轮 φ97mm |

## 设计原则

- **参数与逻辑分离** — 比赛参数全部在 `config/`，业务代码禁止硬编码
- **识别带置信度** — 低置信度触发重试，不得直接导航
- **语音播报期间停车** — TTS 完成后再进入下一状态
- **导航卡死 → 恢复策略** — 不无限等待，检测位移/yaw 变化
