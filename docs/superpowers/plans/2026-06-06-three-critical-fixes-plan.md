# 三项关键修复 — 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复初始定位（恢复 AMCL 配准）、语音唤醒（SSH 模式接入真正 ASR）、相机画幅（640×480 → 1280×720）

**Architecture:** 三个独立 Fix，按 P0→P1→P2 顺序执行。Fix 1 去 static TF 恢复 AMCL 粒子滤波配准能力；Fix 2 将 SSH 唤醒替换为 doubao_asr 火山引擎 ASR；Fix 3 改 3 个配置文件提升 VLM 输入分辨率。

**Tech Stack:** ROS Melodic, Python 2.7/3.9, Bash, AMCL, 火山引擎 ASR API

---

### Task 1: 移除 static TF，恢复 AMCL 自动配准

**Files:**
- Modify: `scripts/competition.sh:365-369`
- Modify: `src/robot_slam/launch/include/amcl.launch.xml:30-46`

- [ ] **Step 1: 删除 competition.sh 中 static_transform_publisher 行**

在 `scripts/competition.sh` SSH 内层脚本中，删除第 365-369 行（注释 + static_transform_publisher），替换为等待 AMCL 发布 map→odom TF 的逻辑：

```bash
# 改前 (行 365-369):
	# AMCL 1.16.7 有 bug: 收到 initialpose 后不发布 map->odom TF
	# 导致 map frame 不存在, move_base costmap 无法初始化, 车不动。
	# 用 static_transform_publisher 手动建立 map frame, 机器人完全靠 odometry 导航。
	/opt/ros/melodic/lib/tf/static_transform_publisher -1.5 1.5 0 0 0 0 1 map odom 100 &
	track $!

# 改后:
	# 等待 AMCL 发布 map->odom TF（替代 static_transform_publisher workaround）
	sleep 3
	for i in $(seq 1 20); do
	    rostopic echo -n 1 /amcl_pose > /dev/null 2>&1 && break
	    sleep 1
	done
```

- [ ] **Step 2: 修正 AMCL 观测模型参数**

在 `src/robot_slam/launch/include/amcl.launch.xml` 中修改 5 个参数：

```xml
<!-- 改前 -->
<param name="laser_z_hit" value="0.5"/>
<param name="laser_z_rand" value="0.5"/>

<!-- 改后 -->
<param name="laser_z_hit" value="0.9"/>
<param name="laser_z_rand" value="0.05"/>
```

```xml
<!-- 改前 -->
<param name="recovery_alpha_slow" value="0.0"/>
<param name="recovery_alpha_fast" value="0.0"/>

<!-- 改后 -->
<param name="recovery_alpha_slow" value="0.001"/>
<param name="recovery_alpha_fast" value="0.1"/>
```

```xml
<!-- 改前 -->
<param name="update_min_d" value="0.25"/>

<!-- 改后 -->
<param name="update_min_d" value="0.1"/>
```

- [ ] **Step 3: 同步到远端并验证 AMCL TF**

```bash
# 同步修改的文件到 ABOT
scp scripts/competition.sh abot@172.16.25.154:~/abot_dev_ws/scripts/
scp src/robot_slam/launch/include/amcl.launch.xml abot@172.16.25.154:~/abot_dev_ws/src/robot_slam/launch/include/

# 启动比赛（自动唤醒模式，快速验证）
ssh abot@172.16.25.154 'bash ~/abot_dev_ws/scripts/competition.sh competition_field true'
```

SSH 到 ABOT 另开终端验证 TF：
```bash
ssh abot@172.16.25.154
source /opt/ros/melodic/setup.bash
source ~/abot_dev_ws/devel/setup.bash
rosrun tf tf_echo map odom
```

**预期**：看到动态变化的 map→odom TF（Translation 和 Rotation 随时间微小变化），而非常量。

同时检查 AMCL 粒子收敛：
```bash
rostopic echo /amcl_pose -n 5 | grep -E "position|covariance"
```
**预期**：位置协方差逐渐减小（粒子群收敛）。

- [ ] **Step 4: 若 AMCL 不发 TF — 创建 tf2_ros 动态桥接**

仅当 Step 3 中 `tf_echo map odom` 持续无输出时，创建 `src/robot_slam/scripts/amcl_tf_bridge.py`：

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""AMCL TF 桥接：订阅 /amcl_pose 动态发布 map->odom TF。
仅当 AMCL 1.16.7 不发布 map->odom TF 时作为兜底方案。
"""
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import PoseWithCovarianceStamped

class AmclTfBridge(object):
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                                     self._on_amcl_pose)

    def _on_amcl_pose(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = tf2_ros.TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('amcl_tf_bridge')
    AmclTfBridge()
    rospy.spin()
```

然后在 `competition.sh` SSH 内层脚本中，在等待 `/amcl_pose` 就绪后添加：
```bash
rosrun robot_slam amcl_tf_bridge.py > /tmp/comp_tf_bridge.log 2>&1 &
track $!
```

- [ ] **Step 5: 远端验证完整导航流程**

停止先前进程，重新启动完整比赛：
```bash
ssh abot@172.16.25.154 'bash ~/abot_dev_ws/scripts/competition.sh --stop'
ssh abot@172.16.25.154 'bash ~/abot_dev_ws/scripts/competition.sh competition_field true'
```

监控日志确认：
```bash
ssh abot@172.16.25.154 'tail -f /tmp/comp_nav.log | grep -E "amcl|map.+odom|Received a goal"'
```

**验收**：
- [ ] 机器人能正常规划路径并移动
- [ ] 日志无 `ABORT_LOCALIZATION_LOST`
- [ ] `rostopic hz /amcl_pose` 持续有输出（非仅 1 条后停止）

- [ ] **Step 6: 提交**

```bash
git add scripts/competition.sh src/robot_slam/launch/include/amcl.launch.xml
git commit -m "fix(amcl): 移除 static TF 恢复粒子滤波配准，修正观测模型参数"
```

---

### Task 2: SSH 模式接入真正语音识别

**Files:**
- Modify: `scripts/competition.sh:379-386`（SSH 内层 [5] 语音唤醒段）
- Modify: `src/robot_slam/scripts/doubao_asr.py:33,76`（统一录音时长 + 加麦克风检查）

- [ ] **Step 1: 替换 SSH 内层唤醒逻辑**

在 `scripts/competition.sh` SSH 内层脚本中，将 GameStart.launch 替换为 ffplay 播提示音 + doubao_asr.py 做识别：

```bash
# 改前 (行 381-386):
	# [5] 语音唤醒（仅非 sim 模式，Snowboy 关键词"开始比赛"）
	if [ "${SIM_MODE}" = "false" ]; then
	    echo '[5/7] 语音唤醒 (Snowboy)...'
	    roslaunch robot_slam GameStart.launch > /tmp/comp_wakeup.log 2>&1 &
	    track $!
	    sleep 3
	fi

# 改后:
	# [5] 语音唤醒（仅非 sim 模式，豆包 ASR 检测"开始比赛"）
	if [ "${SIM_MODE}" = "false" ]; then
	    echo '[5/7] 语音唤醒 (豆包 ASR)...'
	    # 1) 播放提示音
	    ffplay -nodisp -autoexit \
	        ${WS_PATH}/src/robot_slam/scripts/start_record.mp3 \
	        > /tmp/comp_wakeup_prompt.log 2>&1 &
	    # 2) 启动豆包 ASR（需 py3.9 shim）
	    __PY39SHIM=/tmp/abot_py39_shim
	    mkdir -p "$__PY39SHIM"
	    ln -sf /home/abot/anaconda3/envs/py39/bin/python3.9 "$__PY39SHIM/python3"
	    PATH="$__PY39SHIM:$PATH" rosrun robot_slam doubao_asr.py \
	        > /tmp/comp_wakeup_asr.log 2>&1 &
	    track $!
	    sleep 3
	fi
```

- [ ] **Step 2: 修正 doubao_asr.py 录音时长常量**

`src/robot_slam/scripts/doubao_asr.py` 第 33 行 `RECORD_SECONDS = 4` 与实际调用 `duration=3` 不一致，统一为 3：

```python
# 改前 (行 33):
RECORD_SECONDS = 4

# 改后:
RECORD_SECONDS = 3
```

同时将 `run()` 中的调用改为使用常量：
```python
# 改前 (行 76):
record_audio(tmp_path, duration=3)

# 改后:
record_audio(tmp_path, duration=RECORD_SECONDS)
```

- [ ] **Step 3: 添加麦克风可用性检查**

在 `src/robot_slam/scripts/doubao_asr.py` 的 `DoubaoASR.__init__` 末尾添加：

```python
# 在 __init__ 中 self.start_pub 定义之后添加:
try:
    import pyaudio
    dev_count = pyaudio.PyAudio().get_device_count()
    if dev_count < 2:  # 至少要有输入设备（通常 ≥2：默认+麦克风）
        rospy.logwarn('[DoubaoASR] Audio devices: %d (expected >=2); ASR may fail', dev_count)
    else:
        rospy.loginfo('[DoubaoASR] Audio devices: %d OK', dev_count)
except Exception as e:
    rospy.logwarn('[DoubaoASR] Cannot enumerate audio devices: %s', e)
```

- [ ] **Step 4: 远端验证麦克风可用性**

```bash
# 先确认远端有 PyAudio 和麦克风
ssh abot@172.16.25.154 'source /opt/ros/melodic/setup.bash && source ~/abot_dev_ws/devel/setup.bash && \
    /home/abot/anaconda3/envs/py39/bin/python3.9 -c "import pyaudio; print(pyaudio.PyAudio().get_device_count())"'
```

输出 ≥2 表示有可用麦克风。如果为 0 或 1，需要连接麦克风后再继续。

- [ ] **Step 5: 远端同步 + 验证语音唤醒**

```bash
# 同步
scp scripts/competition.sh abot@172.16.25.154:~/abot_dev_ws/scripts/
scp src/robot_slam/scripts/doubao_asr.py abot@172.16.25.154:~/abot_dev_ws/src/robot_slam/scripts/

# 启动（语音唤醒模式）
ssh abot@172.16.25.154 'bash ~/abot_dev_ws/scripts/competition.sh competition_field false'

# 监控 ASR 日志
ssh abot@172.16.25.154 'tail -f /tmp/comp_wakeup_asr.log'
```

**验收**：
- [ ] 启动后先播放 `start_record.mp3` 提示音
- [ ] 对麦克风说"开始比赛"后，日志出现 `"开始比赛" detected! → /start`
- [ ] 机器人开始执行比赛流程
- [ ] 说其他话不触发开始

- [ ] **Step 6: 提交**

```bash
git add scripts/competition.sh src/robot_slam/scripts/doubao_asr.py
git commit -m "fix(wakeup): SSH 模式替换为豆包 ASR 真正语音识别，修正录音时长"
```

---

### Task 3: 相机分辨率 640×480 → 1280×720

**Files:**
- Modify: `src/abot_vlm/launch/vlm_node.launch:4-5`
- Modify: `config/perception.yaml:40-41`
- Modify: `config/robot.yaml:20`
- Modify: `src/abot_vlm/scripts/usb_cam_node.py:80-83`（加校验）

- [ ] **Step 1: 远端确认摄像头支持 1280×720**

```bash
ssh abot@172.16.25.154 'v4l2-ctl --list-formats-ext 2>/dev/null || v4l2-ctl -d /dev/video0 --list-formats-ext'
```

在输出中搜索 `1280x720`，确认摄像头支持该分辨率。如果不支持，找到支持的最高 ≥720p 分辨率。

- [ ] **Step 2: 修改 vlm_node.launch 默认分辨率**

```xml
<!-- 改前 (行 3-5) -->
<arg name="video_device" default="0"/>
<arg name="image_width"  default="640"/>
<arg name="image_height" default="480"/>

<!-- 改后 -->
<arg name="video_device" default="0"/>
<arg name="image_width"  default="1280"/>
<arg name="image_height" default="720"/>
```

- [ ] **Step 3: 修改 perception.yaml 分辨率**

```yaml
# 改前 (行 39-43):
camera:
  capture_width: 640
  capture_height: 480
  save_images: true
  save_path: "logs/"

# 改后:
camera:
  capture_width: 1280
  capture_height: 720
  save_images: true
  save_path: "logs/"
```

- [ ] **Step 4: 修改 robot.yaml 分辨率声明**

```yaml
# 改前 (行 20):
    resolution: "1920x1080"

# 改后:
    resolution: "1280x720"
```

- [ ] **Step 5: usb_cam_node.py 添加分辨率校验**

在 `src/abot_vlm/scripts/usb_cam_node.py` 第 81-83 行（actual_w/h 日志之后）添加：

```python
# 在 rospy.loginfo('[usb_cam] 已打开 ...') 之后添加:
if actual_w != width or actual_h != height:
    rospy.logwarn('[usb_cam] 请求 %dx%d, 实际 %dx%d — 摄像头不支持请求分辨率!',
                  width, height, actual_w, actual_h)
```

- [ ] **Step 6: 远端同步 + 验证**

```bash
# 同步
scp src/abot_vlm/launch/vlm_node.launch abot@172.16.25.154:~/abot_dev_ws/src/abot_vlm/launch/
scp src/abot_vlm/scripts/usb_cam_node.py abot@172.16.25.154:~/abot_dev_ws/src/abot_vlm/scripts/
scp config/perception.yaml abot@172.16.25.154:~/abot_dev_ws/config/
scp config/robot.yaml abot@172.16.25.154:~/abot_dev_ws/config/

# 启动验证
ssh abot@172.16.25.154 'bash ~/abot_dev_ws/scripts/competition.sh competition_field true'
```

检查话题中图像的实际分辨率：
```bash
ssh abot@172.16.25.154
source /opt/ros/melodic/setup.bash
source ~/abot_dev_ws/devel/setup.bash
rostopic echo /usb_cam/image_raw -n 1 | grep -E "width|height"
```

**预期**：width=1280, height=720。若摄像头不支持，warn 日志会出现，此时回退到 Step 1 中确认的支持分辨率。

- [ ] **Step 7: 触发一次 VLM 识别验证超时**

在日志中确认 VLM 调用完成且未超时：
```bash
ssh abot@172.16.25.154 'grep -E "Worker (timeout|failed|done)|Result:" /tmp/comp_vlm.log | tail -5'
```

**预期**：无 timeout，有正常 Result 输出。

- [ ] **Step 8: 提交**

```bash
git add src/abot_vlm/launch/vlm_node.launch config/perception.yaml config/robot.yaml src/abot_vlm/scripts/usb_cam_node.py
git commit -m "feat(camera): 提升采集分辨率至 1280×720，添加摄像头能力校验"
```

---

## 自检

| 检查项 | 状态 |
|--------|:--:|
| Spec 覆盖 — 3 个 Fix 全部覆盖 | ✅ |
| 无 Placeholder — 每步有确切代码和命令 | ✅ |
| 类型一致性 — 参数名/文件路径前后一致 | ✅ |
| 每步 2-5 分钟可完成 | ✅ |
