# AMCL 里程计鲁棒性增强设计

**日期**: 2026-06-08  
**状态**: 设计中  
**分支**: main (e133ba5)

## 1. 问题陈述

### 1.1 现象

- 麦克纳姆轮转弯/横移时打滑，`/wheel_odom` 报告错误位移
- AMCL 粒子基于错误 odom 传播 → 全局地图上点云整体偏移
- 扫描匹配累积误差 → **终点定位丢失**（`ABORT_LOCALIZATION_LOST`）
- 4/4 任务点到齐（配准精度尚可），但终点定位丢失

### 1.2 根因

```
wheel_odom (有滑移误差)
   → robot_pose_ekf (EKF融合, 被错误 odom 污染)
      → /odom (已带漂移)
         → AMCL 粒子传播 (粒子云整体偏移)
            → 扫描匹配失败 → 定位丢失
```

核心矛盾：当前 AMCL 参数**过度信任里程计**（`odom_alpha=0.1`，仅 100 粒子，`update_min_d=0.06`），而麦克纳姆轮滑移是物理问题无法根除。

### 1.3 去年代码验证

分析 2025 年竞赛代码（`Computer-Vision-2025-ROS-main`）发现：
- 架构完全相同（`wheel_odom + IMU → robot_pose_ekf → /odom`，`vo_used=false`）
- AMCL 参数**保守得多**：`odom_alpha=0.2`、粒子 500/2000、`update_min_d=0.25`
- 最终逼近用**开环速度控制**（`move2end()` + `move_rotate_180()`），不依赖 AMCL

→ 去年团队已知轮子打滑问题，用"粗导航(AMCL) + 精逼近(开环)"策略绕开

## 2. 架构上下文

### 2.1 当前数据流

```
abot_driver → /wheel_odom (Odometry, 有滑移)
abot_imu    → /imu/data
                   ↘
robot_pose_ekf → /robot_pose_ekf/odom_combined (PoseWithCovarianceStamped)
                   ↓
              odom_ekf.py → /odom (Odometry)
                   ↓
              AMCL (粒子滤波, 用 /odom 做运动模型)
                   ↓
              amcl_tf_bridge.py → map→odom TF (10Hz)
                   ↓
              move_base (DWA局部规划, 在 odom 系)
                   ↓
              mission_state_machine (footprint检查, 开环回退)
```

### 2.2 不改动的部分

- `robot_pose_ekf` 配置不变（`vo_used` 保持 `false`）
- `odom_ekf.py` 不变
- `amcl_tf_bridge.py` 不变
- `competition.sh` 启动顺序不变

### 2.3 改动范围

| 文件 | 改动类型 | 改动量 |
|------|---------|--------|
| `src/robot_slam/launch/include/amcl.launch.xml` | 参数修改 | ~8 行 |
| `src/mission_manager/scripts/mission_state_machine.py` | 新增方法 + 集成点 | ~50 行 |

零新依赖，零新文件。

## 3. Part C：AMCL 参数回归保守

### 3.1 修改文件

`src/robot_slam/launch/include/amcl.launch.xml`

### 3.2 参数变更表

| 参数 | 当前值 | 新值 | 理由 |
|------|--------|------|------|
| `odom_alpha1` (x平移噪声) | 0.1 | **0.2** | 翻倍：不信任轮式里程计平移分量 |
| `odom_alpha2` (y平移噪声) | 0.1 | **0.2** | 翻倍：全向轮横移打滑更严重 |
| `odom_alpha3` (x旋转噪声) | 0.1 | **0.2** | 翻倍：转弯打滑引入旋转误差 |
| `odom_alpha4` (y旋转噪声) | 0.1 | **0.2** | 翻倍 |
| `min_particles` | 100 | **300** | 3倍：更多粒子覆盖不确定性 |
| `max_particles` | 600 | **1500** | 2.5倍：恢复去年水平 |
| `update_min_d` | 0.06 | **0.12** | 翻倍：减少错误里程计触发AMCL更新频率 |
| `update_min_a` | 0.08 | **0.12** | 1.5倍：转向上也放宽更新条件 |
| `laser_z_rand` | 0.05 | **0.10** | 翻倍：稍增随机注入防止粒子塌陷到错误位姿 |

### 3.3 保持不变（去年没有的改进）

| 参数 | 值 | 理由 |
|------|-----|------|
| `recovery_alpha_fast` | 0.05 | 去年为0，快速恢复是今年的改进 |
| `recovery_alpha_slow` | 0.001 | 慢速随机注入维持多样性 |
| `laser_max_range` | 5.0 | 适配 3.6m×3.6m 场地 |
| `laser_max_beams` | 120 | 匹配 RPLidar A1 360点密度 |
| `laser_z_hit` | 0.85 | 激光匹配本身可靠，不需要降低 |

### 3.4 预期效果

- **粒子云更分散**：打滑时粒子不会全体塌陷到同一错误位姿
- **更新频率更低**：错误里程计数据更少被注入 AMCL
- **随机注入更多**：丢失的粒子更容易通过 `laser_z_rand` 和 `recovery_alpha` 恢复
- **CPU 影响**：1500 粒子 × 120 束 = 180k 评估/更新，预期 1.5-2 Hz（可接受）

## 4. Part A：开环逼近回退

### 4.1 策略

move_base 重试耗尽后，不直接放弃。若剩余距离 < 0.3m，用定时速度指令做最后逼近：

1. **转正车头** → 对准目标方向（纯旋转）
2. **直行** → 纯 x 轴运动（Mecanum 最可靠方向：四轮同向驱动）

### 4.2 新增方法

`mission_state_machine.py` 中新增 `_openloop_approach(target_x, target_y)`:

```
算法：
  1. 获取当前位姿，计算 dx, dy, dist
  2. if dist > 0.30 or dist < 0.015: 跳过（太远不可靠 / 已到位）
  3. if 车头朝向误差 > 0.15rad: 旋转对准（ω=±1.0 rad/s，最长2s）
  4. 直行 vx=0.10 m/s，duration = max(0.5, min(dist/0.10, 3.0))
  5. 停稳
  6. 返回 True/False
```

**速度参数**：
- 旋转：1.0 rad/s（慢转，不引入额外滑移）
- 直行：0.10 m/s（极慢，最小化打滑）
- 最长直行时间：3.0s（对应 0.3m）

### 4.3 集成点

**终点（优先级最高）**：`_handle_arrive_finish` retry limit 耗尽后

```python
# 旧逻辑：直接放弃
# rospy.logwarn('[Mission] Max finish nav retries exceeded, proceeding anyway')

# 新逻辑：先尝试开环
rospy.logwarn('[Mission] Max finish nav retries exceeded, trying open-loop fallback')
self.move_base_client.cancel_goal()
if self.last_nav_goal:
    ok = self._openloop_approach(self.last_nav_goal[0], self.last_nav_goal[1])
    if ok:
        rospy.loginfo('[Mission] Open-loop approach succeeded')
    else:
        rospy.logwarn('[Mission] Open-loop approach skipped/failed, proceeding anyway')
```

**任务点（可选复用）**：`_handle_arrive_task` footprint retry limit 耗尽后

```python
# 同样在"accepting position anyway"之前插入开环尝试
if self.footprint_retry_count > max_footprint_retries:
    self._openloop_approach(cx, cy)  # 尝试微调
    rospy.logwarn('[Mission] Footprint retry limit reached, accepting position')
```

### 4.4 安全边界

| 条件 | 行为 |
|------|------|
| dist > 0.30m | 跳过，距离太远开环不可靠 |
| dist < 0.015m | 跳过，已到位 |
| 无当前位姿 | 跳过，返回 False |
| 最长直行 3s | 0.30m @ 0.10m/s，防止无限执行 |
| 执行期间不检查 safety | 极短时间（<3s），且速度极低（0.1m/s），碰撞风险可忽略 |

## 5. 测试策略

### 5.1 远端验证（实车）

1. `scp` 两个修改文件到 ABOT `172.16.25.45`
2. 远端 `catkin_make`（仅 Python 修改，无需编译 C++）
3. 启动 `competition.sh competition_field true`（模式3，自动开始）
4. 监控指标：
   - AMCL 粒子收敛状态（`/amcl_pose` 协方差）
   - 4 任务点导航成功率
   - 终点定位是否丢失
   - 若终点导航失败，开环回退是否触发并成功

### 5.2 WSL 仿真（可选）

```bash
bash scripts/sim_full_test.sh
```

仿真中里程计无滑移，主要验证：
- 参数修改不破坏现有功能
- 开环回退在正常流程中不误触发

## 6. 风险评估

| 风险 | 概率 | 影响 | 缓解 |
|------|------|------|------|
| 粒子数增加导致 AMCL 更新变慢 | 中 | AMCL 频率可能降至 <2Hz | 若 <1.5Hz 可回退 max_particles 到 1000 |
| 开环逼近在电池电压低时距离不准 | 低 | 微调不到位 | 速度极低(0.1m/s)，即使差 20% 也仅 6cm |
| laser_z_rand 增大导致配准精度下降 | 低 | 任务点定位精度下降 | 从 0.05→0.10 是温和调整，不会显著影响 |
| 开环逼近期间发生碰撞 | 极低 | 轻微接触 | 速度 0.1m/s，且只在 0.3m 内执行 |

## 7. 未来扩展

本方案是**最小侵入的即时修复**。若效果仍不理想，后续可考虑：

1. **rf2o 激光里程计** — 从 `/scan` 提取帧间运动，作为 `/vo` 输入 robot_pose_ekf，实现真正三源融合。需编译 C++ 节点，RPLidar 360 点密度需实测验证
2. **轮式里程计在线协方差估计** — 根据 `/cmd_vel` 指令类型动态调整 odom 协方差（横移/转弯时增大），替代固定 `odom_alpha=0.2`
3. **AMCL 粒子重置策略** — 检测到协方差发散时自动在任务点附近重采样，而非等待 ABORT
