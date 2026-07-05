# Fix Audit Issues (9 个剩余问题) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复代码审计发现的 3 个 High + 4 个 Medium + 2 个 Low 问题

**Architecture:** 所有修改集中在 main 分支，涉及 5 个文件（mission_state_machine.py、voice_text.yaml、mission_logger.py、safety_monitor.py 位置、mock_tts.py），按优先级分组

**Tech Stack:** Python 2 + ROS Melodic + YAML

---

### Task 1: H1 — footprint 从 robot.yaml 读取，去除硬编码

**Files:**
- Modify: `src/mission_manager/scripts/mission_state_machine.py:97,547`

- [ ] **Step 1: 在 `__init__` 中加载 robot.yaml 配置**

在 `self.voice_cfg = load_config('voice_text.yaml')` 后新增一行：

```python
self.robot_cfg = load_config('robot.yaml')
```

- [ ] **Step 2: 替换 `_handle_arrive_task` 中的硬编码 footprint (L547)**

将：
```python
footprint = [[-0.175, -0.15], [-0.175, 0.15], [0.175, 0.15], [0.175, -0.15]]
```

替换为：
```python
footprint = self.robot_cfg.get('footprint',
    [[-0.175, -0.15], [-0.175, 0.15], [0.175, 0.15], [0.175, -0.15]])
```

保留 fallback 默认值防御配置缺失情况。

- [ ] **Step 3: 确认 robot.yaml 配置文件存在且字段正确**

`config/robot.yaml` L11: `footprint: [[-0.175, -0.15], [-0.175, 0.15], [0.175, 0.15], [0.175, -0.15]]` — 格式与代码一致。

- [ ] **Step 4: Commit**

```bash
git add src/mission_manager/scripts/mission_state_machine.py
git commit -m "fix(mission): footprint 从 robot.yaml 配置读取，去除硬编码

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: H2 — 补全 abort 播报文本

**Files:**
- Modify: `config/voice_text.yaml:16-17`
- Modify: `src/mission_manager/scripts/mission_state_machine.py:702-707`

- [ ] **Step 1: 在 voice_text.yaml 补充 4 条 abort 播报**

在 `abort_collision:` 行后新增：

```yaml
  abort_localization_lost: "定位丢失，比赛终止"
  abort_perception_failed: "任务图像识别失败，比赛终止"
  abort_navigation_failed: "导航失败，比赛终止"
  abort_manual_stop: "手动停止，比赛终止"
```

- [ ] **Step 2: 在 `_handle_abort` 中补全 `abort_texts` 映射**

将：
```python
abort_texts = {
    MissionState.ABORT_TIMEOUT: self.voice_cfg['voice_text']['abort_timeout'],
    MissionState.ABORT_COLLISION_RISK: self.voice_cfg['voice_text']['abort_collision'],
}
text = abort_texts.get(self.state, '任务终止')
```

替换为：
```python
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
```

- [ ] **Step 3: Commit**

```bash
git add config/voice_text.yaml src/mission_manager/scripts/mission_state_machine.py
git commit -m "fix(mission): 补全所有 6 种 abort 类型的语音播报文本

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 3: H3 — 引用 voice_text.yaml 中未使用的模板

**Files:**
- Modify: `src/mission_manager/scripts/mission_state_machine.py:346,418,592,614,775`

- [ ] **Step 1: `_handle_start_announce` 使用 `wakeup_detected` 模板**

L337 当前使用 `start` 模板（"比赛开始"）。唤醒后应播报唤醒成功文本，原 `start` 文本改为唤醒检测后再在合适时机播报。实际上查看逻辑：`_handle_start_announce` 就是唤醒后的第一次播报，应改用 `wakeup_detected`：

```python
def _handle_start_announce(self):
    text = self.voice_cfg['voice_text']['wakeup_detected']
    self._speak(text)
```

同时把"比赛开始"播报移到一个合适的地方。但这可能改变流程——原 `start` 文本"比赛开始"内容比 `wakeup_detected`"唤醒成功，比赛开始"更短。两者语义相近。为最小侵入，保留原 `start` 不变，在 `_handle_start_announce` 中播报 `start` 后紧接着也播报 `wakeup_detected` 的消息作为确认。或者直接替换：

根据比赛规则，唤醒后应该播报确认。改用 `wakeup_detected` 更合适：

```python
def _handle_start_announce(self):
    text = self.voice_cfg['voice_text']['wakeup_detected']
    self._speak(text)
```

- [ ] **Step 2: `_handle_search_task_image` 后播报识别成功**

`_handle_recognize_task_image` 中（L454-456），识别成功后应播报 `task_image_recognized`：

在 `self.transition(MissionState.task_image_state(phase, 'NAVIGATE_TO_TASK'))` 之前：

```python
rospy.loginfo('[Mission] Phase %d: Recognition successful, target cell=%d',
              phase, self.target_cell)
text = self.voice_cfg['voice_text']['task_image_recognized'].format(
    index=phase, target_cell=self.target_cell)
self._speak(text)
if self._check_aborted():
    return
```

- [ ] **Step 3: `_retry_perception` 中播报失败重试**

在 `_retry_perception`（L775）中，重试前播报 `task_image_failed`：

```python
def _retry_perception(self, phase):
    max_retries = self.mission_cfg['timeouts']['perception_retry_limit']
    self.perception_retry_count += 1
    if self.perception_retry_count > max_retries:
        rospy.logerr(...)
        self.transition(MissionState.ABORT_PERCEPTION_FAILED)
        return
    text = self.voice_cfg['voice_text']['task_image_failed'].format(index=phase)
    self._speak(text)
    if self._check_aborted():
        return
    # ... 后续不变
```

- [ ] **Step 4: `_handle_navigate_to_task` 中播报导航中**

在 `_handle_navigate_to_task`（L467，发送导航目标前）播报 `navigating_to_task`：

```python
text = self.voice_cfg['voice_text']['navigating_to_task'].format(target_cell=self.target_cell)
self._speak(text)
if self._check_aborted():
    return
```

- [ ] **Step 5: 终点到达后播报 `finish_arrived`**

在 `_handle_arrive_finish` 成功路径（L682-683）和 `_handle_finish_announce`（L685-691）之间。当前流程是：ARRIVE_FINISH → FINISH_ANNOUNCE → DONE。`finish` 文本"比赛结束"在 FINISH_ANNOUNCE 播报。`finish_arrived`"已到达终点，比赛结束"应在 ARRIVE_FINISH 成功后播报。但 `_handle_arrive_finish` 成功后立即 transition 到 FINISH_ANNOUNCE，后者播报 `finish`。

为保持一致，在 `_handle_arrive_finish` 成功路径播报 `finish_arrived`，并将 `_handle_finish_announce` 中的 `finish` 播报移除（避免重复）：

在 `_handle_arrive_finish` L682（`self._stop_robot()` 后）：

```python
self._stop_robot()
text = self.voice_cfg['voice_text']['finish_arrived']
self._speak(text)
if self._check_aborted():
    return
self.transition(MissionState.FINISH_ANNOUNCE)
```

并在 `_handle_finish_announce` 中移除重复的 finish 播报（原来播报 `finish` "比赛结束"，现在已在 ARRIVE_FINISH 时播报 `finish_arrived`"已到达终点，比赛结束"）。但 `_handle_finish_announce` 仍然应发一个简洁的完成确认：

```python
def _handle_finish_announce(self):
    text = self.voice_cfg['voice_text']['finish']
    self._speak(text)
    if self._check_aborted():
        return
    self.logger.log_voice(text, 'finish')
    self.transition(MissionState.DONE)
```

保持不动，因为 `finish_arrived` 在 ARRIVE_FINISH 中播，`finish` 在 FINISH_ANNOUNCE 中播是合理的——先确认到达，再确认结束。

将 `finish_arrived` 播报插入 `_handle_arrive_finish` 成功路径：

```python
self._stop_robot()
# L682 之后：
text = self.voice_cfg['voice_text']['finish_arrived']
self._speak(text)
if self._check_aborted():
    return
self.transition(MissionState.FINISH_ANNOUNCE)
```

- [ ] **Step 6: Commit**

```bash
git add src/mission_manager/scripts/mission_state_machine.py
git commit -m "feat(mission): 引用 voice_text.yaml 中 5 个未使用的播报模板

wakeup_detected, task_image_recognized, task_image_failed,
navigating_to_task, finish_arrived

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 4: M1 — mission_logger.py 添加 Python 2 UTF-8 编码兼容

**Files:**
- Modify: `src/common/scripts/mission_logger.py:1-8`

- [ ] **Step 1: 在文件头部添加编码设置**

在 `# -*- coding: utf-8 -*-` 后、`import os` 前插入：

```python
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
```

- [ ] **Step 2: Commit**

```bash
git add src/common/scripts/mission_logger.py
git commit -m "fix(logger): 添加 Python 2 UTF-8 编码兼容，防止中文日志崩溃

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 5: M2+M3+M4 — 安全消息精确匹配 + 残留 goal 一致性

**Files:**
- Modify: `src/mission_manager/scripts/mission_state_machine.py:313-325,534,674`
- Modify: `src/mission_manager/scripts/safety_monitor.py:113` (不变，仅审查)

- [ ] **Step 1: 精确匹配 `_on_safety_status` 中的 reason**

将 `_on_safety_status` 中的子串匹配（L311-325）重构为精确解析。

safety_monitor 发送格式：`ESTOP:<reason>`，其中 reason ∈ {`collision_risk`, `manual`, `heartbeat_lost`, `no_motion_after_start`}。

替换为：

```python
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
```

- [ ] **Step 2: `_handle_arrive_task` 重试路径加 `cancel_goal()`**

L534 前加 `self.move_base_client.cancel_goal()`：

```python
if self.navigation_retry_count <= max_retries:
    rospy.loginfo('[Mission] Nav retry %d/%d', self.navigation_retry_count, max_retries)
    self.move_base_client.cancel_goal()
    self.transition(MissionState.task_image_state(phase, 'NAVIGATE_TO_TASK'))
    return
```

- [ ] **Step 3: `_handle_arrive_finish` 重试路径加 `cancel_goal()`**

L674 前加 `self.move_base_client.cancel_goal()`：

```python
if self.finish_nav_retry_count <= max_retries:
    rospy.loginfo('[Mission] Finish nav retry %d/%d', ...)
    self.move_base_client.cancel_goal()
    self.transition(MissionState.NAVIGATE_TO_FINISH)
    return
```

- [ ] **Step 4: Commit**

```bash
git add src/mission_manager/scripts/mission_state_machine.py
git commit -m "fix(mission): 精确匹配安全状态 reason + 导航重试前显式 cancel_goal

- _on_safety_status 改为精确解析 ESTOP:<reason>，避免子串误匹配
- _handle_arrive_task/_handle_arrive_finish 重试路径加 cancel_goal
  保持与超限分支一致

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 6: L1+L2 — mock_tts.py 编码 + sim_robot.py 运动模型改进

**Files:**
- Modify: `src/mission_manager/scripts/mock_tts.py:1-2`

- [ ] **Step 1: mock_tts.py 添加 Python 2 编码兼容**

在 `# -*- coding: utf-8 -*-` 后添加：

```python
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
```

- [ ] **Step 2: L1 sim_robot.py 运动模型暂不修改**

运动模型简化是已知限制，不影响基本仿真功能验证。如后续需要高精度仿真再改进。本次跳过。

- [ ] **Step 3: Commit**

```bash
git add src/mission_manager/scripts/mock_tts.py
git commit -m "fix(tts): mock_tts.py 添加 Python 2 UTF-8 编码兼容

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task Summary

| Task | 级别 | 描述 | 文件数 |
|------|------|------|--------|
| 1 | H1 | footprint 从 robot.yaml 读取 | 1 |
| 2 | H2 | 补全 6 种 abort 播报文本 | 2 |
| 3 | H3 | 引用 5 个未使用的语音模板 | 1 |
| 4 | M1 | mission_logger.py UTF-8 编码 | 1 |
| 5 | M2+M3+M4 | 精确安全匹配 + cancel_goal 一致性 | 1 |
| 6 | L1+L2 | mock_tts.py 编码（运动模型延期） | 1 |

**执行顺序**: 1 → 2 → 3 → 4 → 5 → 6（前 3 个 High 优先）
