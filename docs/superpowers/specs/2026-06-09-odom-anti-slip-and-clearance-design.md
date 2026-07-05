# 里程计抗打滑 + 窄缝安全穿行 设计规范

**日期**: 2026-06-09
**分支**: main
**状态**: 设计中

---

## 背景

v1.1.0 提交 `10f2589` 为导航鲁棒性做了激进调参：
- 膨胀层缩到 0.05m/0.02m，footprint 缩到 0.30×0.26m（实际 0.35×0.30m）
- cov_inflate 30x 一刀切降权轮式里程计
- DWA occdist_scale 降到 0.02

副作用：
1. **车体擦障碍物** — footprint 小于实际，costmap 认为车比实物小
2. **轮子打滑里程计漂移** — EKF 纯靠 IMU 积分，位置漂移严重，AMCL 激光配准跳跃
3. **末端导航不流畅** — 目标引力过强 + 定位波动 → 末端过冲震荡

## 核心矛盾

0.4m 网格窄缝是硬约束：车宽 0.35m，两侧仅 2.5cm 间隙。膨胀层稍大就堵死，稍小就擦边。

## 设计方案

### 第一层：代价函数精细化工况（不依赖膨胀层硬隔离）

**思路**: 全局膨胀层极小（允许规划穿缝），但 local costmap 用高 cost_scaling_factor 让近身代价剧痛，DWA 自动避让

| 参数 | 旧值 | 新值 | 理由 |
|------|------|------|------|
| footprint | 0.30×0.26 | **0.34×0.29** | 只比实际缩 1cm 容忍噪声 |
| global inflation_radius | 0.05 | **0.04** | 更小，保证 DIjkstra 能找到缝 |
| local inflation_radius | 0.02 | **0.04** | 翻倍近身保护 |
| local cost_scaling_factor | 3.0 | **10.0** | 代价从障碍物表面急剧衰减 (proxemic) |
| occdist_scale | 0.02 | **0.06** | DWA 近身排斥大幅增强 |

**效果**: 全局路径规划器仍能穿过 0.4m 缝（inflation 0.04m），但 DWA 评分中贴近障碍物代价极高，只在必须穿缝时走中间线。

### 第二层：里程计各向异性降权

**思路**: 麦克纳姆轮前进方向滚动不打滑（encoder 可信），横移/旋转小辊子侧滑严重（保持降权）

| 维度 | 旧因子 | 新因子 | 理由 |
|------|--------|--------|------|
| 前进 (xx) | 30x | **5x** | 轮子滚动，恢复 6 倍信任 |
| 横移 (yy) | 30x | **20x** | 小辊子滑，保持 distrust |
| 旋转 (yawyaw) | 30x | **20x** | 小辊子滑，旋转靠陀螺仪主导 |

**代码改动**: `cov_inflate.py` 从均匀放大改为各向异性缩放。数学上等价于对角缩放矩阵 S = diag(√fwd, √lat, 1, 1, 1, √lat) 做 congruence transform: C_new = S × C × S^T，保持协方差正半定。

### 第三层：AMCL + DWA 联动

| 参数 | 旧值 | 新值 | 理由 |
|------|------|------|------|
| amcl update_min_d | 0.12m | **0.08m** | odom 更准后更频繁更新安全 |
| amcl update_min_a | 0.12rad | **0.10rad** | 同上 |
| amcl laser_z_hit | 0.85 | **0.90** | 激光配准更可信，占比提高 |
| amcl laser_z_rand | 0.10 | **0.05** | 减少随机权重，粒子更聚拢 |
| amcl recovery_alpha_fast | 0.05 | **0.03** | 降低随机粒子注入速率 |
| amcl laser_max_beams | 120 | **180** | 更精细配准 |
| dwa goal_distance_bias | 45.0 | **35.0** | 降低目标引力，减少过冲 |
| dwa path_distance_bias | 12.0 | **15.0** | 增强路径跟随 |
| dwa sim_time | 1.5 | **1.3** | 缩短模拟避免大曲线绕远 |
| dwa oscillation_reset_dist | 0.15 | **0.12** | 定位好了恢复灵敏度 |

**不变**: xy_goal_tolerance 0.12 / yaw_goal_tolerance 0.20 — 定位改善后无需再放宽。

### 链式效果

```
cov_inflate 各向异性
  → EKF 融合前进方向轮式 odom 约束
    → odom 漂移减小
      → AMCL 粒子传播更准
        → 激光配准跳跃消失
          → 导航不反复重规划
            → 末端行为流畅
```

## 改动文件

```
代码 (1 file):
  src/abot_base/abot_bringup/scripts/cov_inflate.py  — 各向异性缩放

参数 (4 files):
  config/robot.yaml                                  — footprint
  config/navigation.yaml                             — inflation/DWA/cost
  src/robot_slam/params/carto/costmap_common_params.yaml — footprint + inflation 同步
  src/robot_slam/launch/include/amcl.launch.xml       — AMCL 参数

Launch (1 file):
  src/abot_base/abot_bringup/launch/robot_with_imu.launch — factor 拆分
```

## 验证

1. 仿真全测试: `bash scripts/sim_full_test.sh`
2. 远端编译: `catkin_make`
3. 比赛模式 3 远端运行: 观察 AMCL 收敛、障碍物距离、末端行为
