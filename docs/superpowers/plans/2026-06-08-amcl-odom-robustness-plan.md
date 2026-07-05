# AMCL 里程计鲁棒性增强 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 增强 AMCL 对轮式里程计打滑的鲁棒性（参数回归保守 + 开环逼近回退），解决终点定位丢失。

**Architecture:** 两文件修改，零新依赖。Part C 调整 AMCL 9 个参数降低对轮式里程计的信任；Part A 在状态机中新增开环速度控制方法，在 move_base 重试耗尽后做最后的定时逼近。

**Spec:** `docs/superpowers/specs/2026-06-08-amcl-odom-robustness-design.md`

**Tech Stack:** ROS Melodic, Python 2.7, XML (launch)

---

### Task 1: AMCL 参数回归保守

**Files:**
- Modify: `src/robot_slam/launch/include/amcl.launch.xml`

- [ ] **Step 1: 修改 odom_alpha 噪声参数 (odom_alpha1-4: 0.1 → 0.2)**

```xml
<!-- 旧 (L26-L29) -->
<param name="odom_alpha1" value="0.1"/>
<param name="odom_alpha2" value="0.1"/>
<param name="odom_alpha3" value="0.1"/>
<param name="odom_alpha4" value="0.1"/>

<!-- 新 -->
<param name="odom_alpha1" value="0.2"/>
<param name="odom_alpha2" value="0.2"/>
<param name="odom_alpha3" value="0.2"/>
<param name="odom_alpha4" value="0.2"/>
```

使用 Edit 工具，精确替换。

- [ ] **Step 2: 修改粒子数 (min: 100→300, max: 600→1500)**

```xml
<!-- 旧 (L21-L22) -->
<param name="min_particles" value="100"/>
<param name="max_particles" value="600"/>

<!-- 新 -->
<param name="min_particles" value="300"/>
<param name="max_particles" value="1500"/>
```

- [ ] **Step 3: 修改更新阈值 (update_min_d: 0.06→0.12, update_min_a: 0.08→0.12)**

```xml
<!-- 旧 (L40-L41) -->
<param name="update_min_d" value="0.06"/>
<param name="update_min_a" value="0.08"/>

<!-- 新 -->
<param name="update_min_d" value="0.12"/>
<param name="update_min_a" value="0.12"/>
```

- [ ] **Step 4: 修改随机注入 (laser_z_rand: 0.05→0.10)**

```xml
<!-- 旧 (L33) -->
<param name="laser_z_rand" value="0.05"/>

<!-- 新 -->
<param name="laser_z_rand" value="0.10"/>
```

- [ ] **Step 5: 验证文件完整性**

运行: `grep -E "odom_alpha|particles|update_min|laser_z_rand" src/robot_slam/launch/include/amcl.launch.xml`

预期输出:
```
<param name="odom_alpha1" value="0.2"/>
<param name="odom_alpha2" value="0.2"/>
<param name="odom_alpha3" value="0.2"/>
<param name="odom_alpha4" value="0.2"/>
<param name="odom_alpha5" value="0.1"/>
<param name="min_particles" value="300"/>
<param name="max_particles" value="1500"/>
<param name="update_min_d" value="0.12"/>
<param name="update_min_a" value="0.12"/>
<param name="laser_z_rand" value="0.10"/>
```

---

### Task 2: 新增 `_openloop_approach` 方法

**Files:**
- Modify: `src/mission_manager/scripts/mission_state_machine.py`

在 `MissionStateMachine` 类中添加方法。插入位置：`_speak` 方法之前（L876 附近），与其他 helper 方法在一起。

- [ ] **Step 1: 添加 `_openloop_approach` 方法**

```python
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
        
        rospy.loginfo('[Mission] Open-loop: approaching (%.3fm, %.3f°)',
                      dist, math.degrees(math.atan2(dy, dx)))
        
        # 1) 转正车头对准目标方向
        target_heading = math.atan2(dy, dx)
        yaw_err = self._angle_diff(ryaw, target_heading)
        if yaw_err > 0.15:
            twist = Twist()
            twist.angular.z = 1.0 if math.sin(target_heading - ryaw) > 0 else -1.0
            rotate_duration = min(yaw_err / 1.0, 2.0)
            t0 = time.time()
            while time.time() - t0 < rotate_duration:
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(0.05)
            self._stop_robot()
            rospy.sleep(0.3)
        
        # 2) 直行
        twist = Twist()
        twist.linear.x = 0.10  # 0.10 m/s, 极慢速度最小化打滑
        drive_duration = max(0.5, min(dist / 0.10, 3.0))
        t0 = time.time()
        while time.time() - t0 < drive_duration:
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.05)
        self._stop_robot()
        
        # 验证
        rx2, ry2, _ = self._get_current_pose()
        if rx2 is not None:
            final_dist = math.sqrt((target_x - rx2)**2 + (target_y - ry2)**2)
            rospy.loginfo('[Mission] Open-loop done: final distance %.3fm', final_dist)
        
        return True
```

使用 Edit 工具插入在 `_speak` 方法之前（`def _speak(self, text):` 行之前）。

---

### Task 3: 集成开环回退到 `_handle_arrive_finish`

**Files:**
- Modify: `src/mission_manager/scripts/mission_state_machine.py`

- [ ] **Step 1: 修改 finish retry 耗尽后的逻辑**

当前代码（L834-L836）:
```python
            else:
                rospy.logwarn('[Mission] Max finish nav retries exceeded, proceeding anyway')
                self.move_base_client.cancel_goal()
```

替换为:
```python
            else:
                rospy.logwarn('[Mission] Max finish nav retries exceeded, trying open-loop fallback')
                self.move_base_client.cancel_goal()
                rospy.sleep(0.5)
                if self.last_nav_goal:
                    self._openloop_approach(self.last_nav_goal[0], self.last_nav_goal[1])
```

---

### Task 4: 集成开环回退到 `_handle_arrive_task` (footprint retry 耗尽)

**Files:**
- Modify: `src/mission_manager/scripts/mission_state_machine.py`

- [ ] **Step 1: 修改 footprint retry 耗尽后的逻辑**

当前代码（L717-L719）:
```python
                    else:
                        rospy.logwarn('[Mission] Phase %d: Footprint retry limit reached (%d), accepting position',
                                      phase, max_footprint_retries)
```

替换为:
```python
                    else:
                        cx, cy = detail['task_center']
                        rospy.logwarn('[Mission] Phase %d: Footprint retry limit reached (%d), trying open-loop',
                                      phase, max_footprint_retries)
                        self._openloop_approach(cx, cy)
                        rospy.sleep(0.3)
                        rospy.logwarn('[Mission] Phase %d: Accepting position after open-loop attempt', phase)
```

---

### Task 5: 本地验证（语法检查 + WSL 仿真）

**Files:** 无（仅验证）

- [ ] **Step 1: Python 语法检查**

运行:
```bash
python2 -m py_compile src/mission_manager/scripts/mission_state_machine.py
```
预期: 无输出（编译成功）

- [ ] **Step 2: AMCL launch XML 语法检查**

运行:
```bash
xmllint --noout src/robot_slam/launch/include/amcl.launch.xml 2>/dev/null || echo "xmllint not available, skipping XML check"
```
预期: 无错误

- [ ] **Step 3: WSL 仿真测试**

运行:
```bash
wsl bash /mnt/d/StudyWorks/3.2/MachineVision_Project/AutonomousCruise_Ground/scripts/sim_full_test.sh
```
预期: 全部节点启动正常，参数修改不破坏现有功能。注意仿真中里程计无滑移，开环回退不会触发（正常行为）。

---

### Task 6: 远端同步与测试

**远端:** ABOT `172.16.25.45`, 用户 `abot`

- [ ] **Step 1: 检查远端无人占用**

```bash
ssh abot@172.16.25.45 'ps aux | grep ros | grep -v grep'
```
预期: 无 ROS 进程在跑

- [ ] **Step 2: 同步两个修改文件**

```bash
scp "D:\StudyWorks\3.2\MachineVision_Project\AutonomousCruise_Ground\src\robot_slam\launch\include\amcl.launch.xml" abot@172.16.25.45:~/abot_dev_ws/src/robot_slam/launch/include/amcl.launch.xml

scp "D:\StudyWorks\3.2\MachineVision_Project\AutonomousCruise_Ground\src\mission_manager\scripts\mission_state_machine.py" abot@172.16.25.45:~/abot_dev_ws/src/mission_manager/scripts/mission_state_machine.py
```

- [ ] **Step 3: 启动竞赛**

```bash
ssh abot@172.16.25.45 'bash ~/abot_dev_ws/scripts/competition.sh competition_field true'
```

- [ ] **Step 4: 监控运行**

关注日志:
- AMCL 协方差：`grep "pos_std" /tmp/comp_mission.log`
- 开环触发：`grep "open-loop" /tmp/comp_mission.log`
- 定位丢失：`grep "LOCALIZATION_LOST" /tmp/comp_mission.log`

预期: 4 任务点完成，终点不丢定位。若终点 move_base 失败，开环回退应触发并完成最后 ~0.2m 逼近。

- [ ] **Step 5: 停止**

```bash
ssh abot@172.16.25.45 'bash ~/abot_dev_ws/scripts/competition.sh --stop'
```

---

### 回滚方案

若 AMCL 参数变化导致定位性能显著下降：

```bash
# 回滚 amcl.launch.xml
git checkout HEAD -- src/robot_slam/launch/include/amcl.launch.xml
# 重新同步并测试
```

若开环回退导致意外行为，可在 `mission_state_machine.py` 中将 `_openloop_approach` 调用注释掉，开环方法本身保留（不影响正常流程）。
