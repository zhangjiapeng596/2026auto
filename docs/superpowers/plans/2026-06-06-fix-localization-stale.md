# 修复 AMCL stale 误判 + move_base 就绪时序

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans.

**Goal:** 消除 ABORT_LOCALIZATION_LOST 误判，确保 move_base 就绪后才发导航目标。

**Architecture:** 两个独立修复：(R1) 禁用 AMCL stale 检查 + TF fallback 位姿；(R2) 状态机增加 costmap 就绪等待。

**Tech Stack:** Python 2.7, ROS Melodic

---

### 根因回顾

| # | 问题 | 证据 |
|---|------|------|
| R1 | AMCL 只发 1 条 `/amcl_pose` 然后停止 (TF 正常) | `rostopic hz /amcl_pose` 15s 零消息，诊断 `stale=True(9.8/8.0s)` |
| R2 | 状态机在 costmap 就绪前发导航目标，车不动 | Safety `no_motion_after_start` ESTOP，用户反馈"路径规划还没出来" |

---

### Task 1: 禁用 stale 检查，仅保留协方差检查 (R1)

**Files:**
- Modify: `config/mission.yaml:51`

- [ ] **Step 1: 改配置**

```yaml
# 改前:
  amcl_stale_s: 8.0
# 改后:
  amcl_stale_s: 0.0  # 0=禁用 stale 检查，AMCL 1.16.7 /amcl_pose 发布有 bug
```

- [ ] **Step 2: 在 _check_localization 中跳过 stale_s=0 的检查**

`src/mission_manager/scripts/mission_state_machine.py` 行 ~917:

```python
# 改前:
if last_time > 0 and (now - last_time) > stale_s:
    diverged = True
# 改后:
if stale_s > 0 and last_time > 0 and (now - last_time) > stale_s:
    diverged = True
```

- [ ] **Step 3: 添加 TF 位姿 fallback**

当 `/amcl_pose` 不可用时用 TF (`map→base_link`) 获取位姿，避免 `current_pose` 冻结。

在 `__init__` 中添加 TF listener:
```python
import tf as ros_tf  # 顶部 import
self.tf_listener = ros_tf.TransformListener()
```

在 `run()` 循环中定期更新位姿（当 AMCL rank < 3 时从 TF 读取）:
```python
# 在 _check_global_timeouts() 之后添加:
self._update_pose_from_tf()
```

新增方法:
```python
def _update_pose_from_tf(self):
    """当 AMCL 不可用时从 TF 获取 map→base_link 位姿。"""
    if self._pose_source_rank >= self._POSE_RANK['amcl_pose']:
        return  # AMCL 正常，不覆盖
    try:
        (trans, rot) = self.tf_listener.lookupTransform(
            'map', 'base_link', rospy.Time(0))
        x, y = trans[0], trans[1]
        _, _, yaw = tft.euler_from_quaternion(rot)
        self._update_pose('odom_tf', x, y, yaw)
    except (ros_tf.LookupException, ros_tf.ConnectivityException,
            ros_tf.ExtrapolationException):
        pass
```

并在 `_POSE_RANK` 字典中添加 `'odom_tf': 2` (低于 amcl_pose 的 3，高于 odom 的 1)。

- [ ] **Step 4: 同步 + 远端验证**

```bash
scp config/mission.yaml src/mission_manager/scripts/mission_state_machine.py abot@<IP>:~/abot_dev_ws/...
ssh abot@<IP> 'bash ~/abot_dev_ws/scripts/competition.sh --stop && bash ~/abot_dev_ws/scripts/competition.sh competition_field true'
```

验收：日志不再出现 `ABORT_LOCALIZATION_LOST`。

---

### Task 2: 状态机等待 costmap 就绪 (R2)

**Files:**
- Modify: `src/mission_manager/scripts/mission_state_machine.py` `_handle_search_task_image` 方法

- [ ] **Step 1: 发送导航目标前等待 move_base 有活跃规划**

在 `_handle_search_task_image` 中 `_send_nav_goal` 之前添加等待逻辑:

```python
# 等待 move_base 收到 goal 并开始规划 (costmap 可能还在初始化)
# 等待最多 15 秒，每次 0.5 秒轮询
wait_deadline = time.time() + 15.0
while time.time() < wait_deadline:
    if self._check_aborted():
        return
    state = self.move_base_client.get_state()
    if state in (GoalStatus.ACTIVE, GoalStatus.PENDING):
        break
    rospy.sleep(0.5)
```

然后将 `_send_nav_goal` 后的 `self.transition(RECOGNIZE_TASK_IMAGE)` 改为等待导航完成后再 transition:

```python
# 改前: 立即 transition
self._send_nav_goal(x, y, yaw)
self.transition(MissionState.task_image_state(phase, 'RECOGNIZE_TASK_IMAGE'))

# 改后: 在 SEARCH 状态内等待导航完成
self._send_nav_goal(x, y, yaw)
# 等待导航完成
timeout_s = self.mission_cfg['timeouts'].get('navigation_goal_timeout_s', 30)
deadline = time.time() + timeout_s
arrived = False
while time.time() < deadline:
    if self._check_aborted():
        self.move_base_client.cancel_goal()
        return
    state = self.move_base_client.get_state()
    if state == GoalStatus.SUCCEEDED:
        arrived = True
        break
    if state in (GoalStatus.ABORTED, GoalStatus.REJECTED, ...):
        self._retry_perception(phase)
        return
    # 提前到达检查
    if self.last_nav_goal and self._pose_near_goal(gx, gy, gyaw, ...):
        arrived = True
        self.move_base_client.cancel_goal()
        break
    rospy.sleep(0.5)

if not arrived:
    self._retry_perception(phase)
    return

self.transition(MissionState.task_image_state(phase, 'RECOGNIZE_TASK_IMAGE'))
```

- [ ] **Step 2: 同步 + 远端验证**

验收：日志出现 `Nav goal sent` 后，机器人先移动，再到视觉点后才 transition 到 RECOGNIZE。

---

### Task 3: 延长 auto-wakeup 和不运动超时 (兜底)

**Files:**
- Modify: `config/mission.yaml:18,50`

- [ ] **Step 1: 改配置**

```yaml
# no_motion_after_start_s: 20 → 40 (给 move_base costmap 初始化留时间)
# no_state_change_s: 40 → 60  (导航+识别可能较慢)
```

- [ ] **Step 2: 同步**
