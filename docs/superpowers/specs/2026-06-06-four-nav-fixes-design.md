# 四项导航修复 — 设计 Spec

> 基于 Superpowers 四路并行 Agent 分析（2026-06-06）

---

## Fix 1: AMCL 3m 配准距离限制

### 问题

`laser_max_range: 12.0`，远距离地图形变数据参与粒子权重计算，影响定位精度。

### 设计

`amcl.launch.xml` 单参数修改：`laser_max_range` 12.0 → 3.0。

3.0m 为平衡点：场地中心(1.8,1.8)四面墙全在范围内；角落丢弃远墙但近墙约束足够。`laser_likelihood_max_dist` 保持 2.0。

### 涉及文件

| 文件 | 改动 |
|------|------|
| `src/robot_slam/launch/include/amcl.launch.xml:20` | `laser_max_range` 12.0 → 3.0 |

---

## Fix 2: DWA 轨迹箭头稳定性

### 问题

轨迹模拟 0.94m，局部代价地图前方仅 1.0m，余量 6cm。`min_vel_theta: 1.0` 导致弧转至少 97°，轨迹经常从侧边穿出，DWA 评分不稳定。

### 设计

`config/navigation.yaml` 四参数修改：

| 参数 | 当前 | 改后 | 理由 |
|------|------|------|------|
| `local_costmap.width` | 2.0 | 3.0 | 前方 1.5m，余量 0.56m |
| `local_costmap.height` | 2.0 | 3.0 | 侧向同步 |
| `DWAPlannerROS.min_vel_theta` | 1.0 | 0.3 | 允许接近直线轨迹 |
| `DWAPlannerROS.oscillation_reset_dist` | 0.05 | 0.10 | 降低边界噪声敏感 |

### 涉及文件

| 文件 | 改动 |
|------|------|
| `config/navigation.yaml:44-48,61` | 4 参数 |

---

## Fix 3: 终点完全入框

### 问题

双层容差(5cm+5cm)叠加导致车在目标 5cm 外判到达；车头朝东(yaw=0)，长边正对墙仅 2.5cm 余量。

### 设计

**A. 收紧容差**：

| 参数 | 当前 | 改后 |
|------|------|------|
| `mission.finish_xy_tolerance_m` | 0.05 | 0.03 |
| `mission.finish_yaw_tolerance_rad` | 0.20 | 0.06 |
| `navigation.xy_goal_tolerance` | 0.05 | 0.03 |

**B. 终点车头朝场地中心**：`_handle_navigate_to_finish` 中 yaw 从 0 改为 `atan2(-y, -x)`。Cell 9 → 西南(-135°)，车尾靠墙角。

### 涉及文件

| 文件 | 改动 |
|------|------|
| `config/mission.yaml:37-38` | 收紧 finish 容差 |
| `config/navigation.yaml:68` | `xy_goal_tolerance` 0.05 → 0.03 |
| `src/mission_manager/scripts/mission_state_machine.py` | 终点 yaw 计算 |

---

## Fix 4: 任务点朝向规整化

### 问题

任务区 0.38m×0.32m（长边东西），车 0.35m×0.30m。默认 yaw=0 恰好入框，但 footprint 修正时可能发送错误 yaw（±π/2 放不下）。

### 设计

最小改动：
- 任务导航 yaw 显式设为 0（朝东）
- Footprint 修正时将当前 yaw 归化到最近有效值（0 或 π）

```python
ryaw_norm = math.atan2(math.sin(ryaw), math.cos(ryaw))
target_yaw = 0.0 if abs(ryaw_norm) < math.pi/2 else math.pi
self._send_nav_goal(cx, cy, target_yaw)
```

### 涉及文件

| 文件 | 改动 |
|------|------|
| `src/mission_manager/scripts/mission_state_machine.py` | yaw 显式 + 归化 |

---

## 实施顺序

Fix 1 → Fix 2 → Fix 3 → Fix 4（互不阻塞，但建议顺序验证）
