---
name: fast-lio2-changes
description: Record of all modifications made to FAST-LIO2 source code and launch configs
metadata: 
  node_type: memory
  type: reference
  originSessionId: 34eea3d4-a46a-4de0-9202-1fcb30b8b1e5
---

# FAST-LIO2 改动记录

## 改动的文件

### 1. `src/laserMapping.cpp` — 启用地图点云发布

**位置**: 第 935 行

```
改前: if(0) // If you need to see map point, change to "if(1)"
改后: if(1) // PCL需要密集地图做障碍物处理，启用ikd-tree展平
```

**原因**: 原始代码用 `if(0)` 禁用了 `featsFromMap` 的 ikd-tree 展平逻辑，导致 `/Laser_map`（实机）/ `/fastlio_map`（仿真）发空点云。改成 `if(1)` 后每帧把 ikd-tree 的累积点云展平发布。

**警告**: 每次展平全量 ikd-tree 可能在大场景下耗时增加，但赛区 ~10m 范围内没问题。

---

### 2. `launch/mapping_mid360_fastlio.launch` — 仿真专用 launch（新建）

在 `ros_libraries_ws/src/FAST_LIO/launch/` 下新建，与原始 `mapping_mid360.launch` 的区别：

| 参数               | 原始值         | 改后值                        | 原因                                   |
| ------------------ | -------------- | ----------------------------- | -------------------------------------- |
| `point_filter_num` | 3              | **1**                         | 保留全部点云，增加地图密度             |
| `filter_size_surf` | 0.5            | **0.05**                      | 更细的表面特征分辨率                   |
| `filter_size_map`  | 0.5            | **0.05**                      | 更细的 ikd-tree 地图分辨率             |
| node name          | `laserMapping` | `fastlio_mapping`             | 避免与仿真环境的 `laserMapping` 冲突   |
| 话题 remap         | 无             | `/Laser_map` → `/fastlio_map` | 避免两个 FAST-LIO2 实例抢 `/Laser_map` |
| rviz 默认启动      | true           | false                         | 节省资源                               |

---

**其中那个node要换成下面内容**
```xml
<node pkg="fast_lio" type="fastlio_mapping" name="fastlio_mapping" output="screen">
    <remap from="/Laser_map" to="/fastlio_map" />
</node>
```
## 实机部署时需注意

实机没有 `sim.launch` 冲突，因此不需要：

- ❌ 不需要改 node name（用原始 `laserMapping` 即可）


**实机只需要改两处**：

| 文件                           | 改动                                                                            |
| ------------------------------ | ------------------------------------------------------------------------------- |
| `src/laserMapping.cpp:935`     | `if(0)` → `if(1)`                                                               |
| `launch/mapping_mid360.launch` | `point_filter_num=3→1`, `filter_size_surf=0.5→0.05`, `filter_size_map=0.5→0.05` |

还有remap

引用：[[pcl-fastlio-integration]]
