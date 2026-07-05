# Navigation Stability Fix — Design Spec

**Date**: 2026-06-09
**Status**: approved
**Branch**: main

## Problem Summary

2026-06-09 实车测试中，DWA 169 次失败 + AMCL 冷启动 24s 额外开销 → 累计超时 240s 未完成。

### Root Causes (verified via code inspection)

1. **inflation=0.14 导致 DWA 振荡** — 0.4m 网格内自由通道仅 0.12m (< 车宽 0.32m)，DWA 在所有方向看到高代价 → 无可行轨迹
2. **AMCL 初始协方差过大** — cov=0.25 (std=0.5m) 导致 1500 粒子散布半径 ≈1m，首次导航中隐式收敛吃掉 24s
3. **级联超时** — #1 + #2 累积延迟导致 Finish 只剩 7s → ABORT_TIMEOUT

## Fix Design

### Fix 1: DWA + Costmap 参数回退到稳定配置

File: `config/navigation.yaml`

| Parameter | Before | After | Rationale |
|-----------|--------|-------|-----------|
| `local_costmap.inflation_radius` | 0.14 | **0.04** | 恢复自由通道 ≥0.32m |
| `local_costmap.cost_scaling_factor` | 8.0 | **10.0** | 配合 0.04 膨胀的快速衰减 |
| `DWAPlannerROS.goal_distance_bias` | 30.0 | **35.0** | 增强目标导向，减少绕路 |
| `DWAPlannerROS.path_distance_bias` | 25.0 | **15.0** | 减少过度的路径跟随 |
| `DWAPlannerROS.occdist_scale` | 0.08 | **0.06** | 0.04 膨胀下已有足够安全边距 |

### Fix 2: AMCL 初始协方差缩小 (B1)

File: `scripts/competition.sh`

```
covariance position: 0.25 → 0.04  (std: 0.5m → 0.2m)
covariance yaw:      0.068 → 0.017 (std: 0.26rad → 0.13rad)
```

粒子初始聚集在半径 0.4m 内，KLD 可用 ~500 粒子快速收敛。

### Fix 3: 启动收敛旋转 (B2)

File: `src/mission_manager/scripts/mission_state_machine.py`

在 `_handle_start_announce()` → `SEARCH_TASK_IMAGE_1` 之间插入约 5s 原地收敛旋转：
- 以 0.8 rad/s 旋转 ~180°（正反各 90°）
- 仿真模式跳过
- 新增 `_convergence_rotate()` 方法

### Net Effect

| Metric | Before | After (expected) |
|--------|--------|-------------------|
| DWA failures | 169 | <20 |
| Vision 1 time | 38s | ~19s |
| Cell 31 nav | ❌ skip | ✅ ~15s |
| Time margin | -5s | ~50s |
| Completion | ❌ | ✅ 5/5 tasks |

## Scope

3 files changed, no new dependencies, no API changes.
