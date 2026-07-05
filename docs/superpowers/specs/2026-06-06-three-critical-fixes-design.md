# 三项关键修复 — 设计 Spec

> 基于 Superpowers 深度审查（2026-06-06），三路并行 Agent 覆盖语音唤醒、相机画幅、初始定位全链路。

**优先级排序**：定位 > 唤醒 > 相机（按对比赛成绩的影响）

---

## Fix 1: 修复 AMCL 初始定位（P0 — 三重失效 → 激光修正定位）

### 问题

`competition.sh:368` 用 `static_transform_publisher` 硬编码 map→odom，切断了 AMCL 粒子滤波的修正输出。加上 `laser_z_rand=0.5`（50% 激光视为随机噪声）和 `recovery_alpha_fast=0.0`（永不注入随机粒子），定位系统三重失效：纯里程计导航，全程漂移可达 32cm。

### 设计

1. **移除 static TF**（`competition.sh:368`），恢复 AMCL 的 map→odom TF 发布
2. **修正观测模型参数**（`amcl.launch.xml`）：
   - `laser_z_hit`: 0.5 → 0.9（激光束大概率命中地图障碍物）
   - `laser_z_rand`: 0.5 → 0.05（极小概率随机噪声）
   - `laser_z_short`: 0.05 → 0.05（保持）
   - `laser_z_max`: 0.05 → 0.05（保持）
3. **启用量子注入恢复**：
   - `recovery_alpha_slow`: 0.0 → 0.001（缓慢注入随机粒子）
   - `recovery_alpha_fast`: 0.0 → 0.1（定位差时快速注入粒子）
4. **降低更新阈值**：`update_min_d`: 0.25 → 0.1（更频繁的粒子更新）
5. **保留现有 initialpose**：`(-1.5, 1.5)` 不动，由 AMCL 粒子滤波自动配准到精确位姿。初始摆放只需大致在 cell 1 区域，十几厘米偏差是 AMCL 正常处理范围
6. **AMCL TF 兜底**：如果 AMCL 1.16.7 确实不发 TF，用 `tf2_ros.TransformBroadcaster` 订阅 `/amcl_pose` 动态发布 map→odom，替代 static TF

### 涉及文件

| 文件 | 改动 |
|------|------|
| `scripts/competition.sh` | 去掉 static_transform_publisher 行（~368） |
| `src/robot_slam/launch/include/amcl.launch.xml` | 改 5 个参数 |
| （可选）新增 `src/robot_slam/scripts/amcl_tf_bridge.py` | 仅当 AMCL 真的不发 TF 时 |

### 验收标准

- `rosrun tf tf_echo map odom` 能看到动态变化的 TF（非 static）
- 机器人静止时 `/amcl_pose` 协方差收敛（位置 std < 0.1m）
- 导航全程无 `ABORT_LOCALIZATION_LOST`

---

## Fix 2: 修复语音唤醒（P1 — 假实现 → 真正 ASR）

### 问题

SSH 比赛模式（唯一实车模式）中，`competition.sh` 启动的是 `start.py`（播放 MP3 + 5s 倒计时盲发 /start），而非 `doubao_asr.py`（录音 + 火山引擎 ASR API + 语义匹配"开始比赛"）。`require_start_voice: true` 是死参数。

### 设计

1. **SSH 内层拆分唤醒流程**：
   - 先用 `start.py`（或直接用 ffplay）播放 MP3 提示音
   - 再用 `doubao_asr.py`（py3.9 shim）做真正语音识别
2. **`start.py` 拆分职责**：分离"播 MP3"和"发布 /start"为两个独立功能；或直接在内层脚本用 `ffplay` 播 MP3
3. **`doubao_asr.py` 改进**：
   - 添加麦克风可用性检查（`pyaudio.PyAudio().get_device_count()`）
   - 录音时长常量与实际调用一致（统一为 3s）
   - 添加 VAD 或降低轮询频率以减少 API 浪费（可选/低优先级）

### 涉及文件

| 文件 | 改动 |
|------|------|
| `scripts/competition.sh` | SSH 内层：替换 GameStart.launch 为 ffplay + doubao_asr.py |
| `src/robot_slam/scripts/start.py` | 可选：分离 MP3 播放逻辑 |
| `src/robot_slam/scripts/doubao_asr.py` | 加麦克风检查，统一录音时长 |

### 验收标准

- SSH 登录 ABOT，启动比赛后播放提示音，说"开始比赛"，机器人开始执行
- 说其他话不会触发 /start
- 麦克风未连接时有明确报错日志

---

## Fix 3: 相机开全幅（P2 — 640×480 → 1280×720）

### 问题

`vlm_node.launch:4-5` 硬编码 640×480，但 `robot.yaml:20` 声明摄像头为 1080p。三处配置互相矛盾，实际以 launch 的 640×480 为准。卡片在图像中仅占 ~70k 像素，VLM 处于识别临界。

### 设计

1. **统一分辨率为 1280×720**（1080p 可能帧率不足 + API 延迟增加）：
   - `vlm_node.launch`: image_width=1280, image_height=720
   - `perception.yaml`: capture_width=1280, capture_height=720
   - `robot.yaml`: resolution 改为 "1280x720"
2. **添加分辨率校验**（`usb_cam_node.py`）：对比 `cap.set()` 后的实际值与请求值，不一致时 warn
3. **添加回退逻辑**（可选/低优先级）：摄像头不支持 1280×720 时回退 640×480

### 涉及文件

| 文件 | 改动 |
|------|------|
| `src/abot_vlm/launch/vlm_node.launch` | 改 arg 默认值 |
| `config/perception.yaml` | 改 capture_width/height |
| `config/robot.yaml` | 改 camera.resolution |
| `src/abot_vlm/scripts/usb_cam_node.py` | 加分辨率一致性校验（~3 行） |

### 验收标准

- `rostopic echo /usb_cam/image_raw` 的 width/height 字段为 1280×720
- VLM 识别置信度不下降（预期更稳定）
- API 调用不超时（30s 超时充足）

---

## 实施顺序

```
Fix 1 (定位) → 远端验证 AMCL TF 正常 → Fix 2 (唤醒) → 远端验证 ASR 可用 → Fix 3 (相机) → 远端验证 VLM 识别
```

每个 Fix 独立可验证，互不阻塞。定位最先修因为它是整个导航系统的基础。

## 风险

| 风险 | 缓解 |
|------|------|
| AMCL 1.16.7 真的不发 TF | 用 tf2_ros TransformBroadcaster 订阅 /amcl_pose 动态发 TF |
| SSH 环境无 PyAudio/麦克风 | 先 `ssh abot@IP 'python3 -c "import pyaudio; print(pyaudio.PyAudio().get_device_count())"'` 确认 |
| ABOT 摄像头不支持 1280×720 | 先 `v4l2-ctl --list-formats-ext` 确认，不支持则回退 |
