# 柱子检测与PCL定点导航

## 目标
用PCL投影+模板匹配检测两个柱子的位置组合（4种情况），根据结果用PCL定点飞替代EGO导航。

## 架构

### 1. 柱子检测模块 `pcl_detection2/include/core/pillar_detect.hpp`
- 输入：ROI裁剪后的累加点云（map坐标系）
- 处理：投影到Z平面→生成二值图→与4个模板匹配
- 输出：匹配到的配置编号(0-3) + 置信度

**ROI参数**（YAML化）：
```yaml
pillar_roi:
  x_min: -3.0, x_max: -1.7
  y_min: -2.65, y_max: 0.0
  z_min: 0.3, z_max: 1.5    # 和现有roi一致
  resolution: 0.05           # m/pixel，和现有一致
```

**投影**：直接Z轴投影（点云→z=0平面，统计每格有点=255）
**模板**：4张和投影图同尺寸的二值图，在柱子位置画白点(4px半径)

### 2. 模板生成
运行时在`pillar_detect::init()`中根据YAML参数自动生成4张模板：
- 柱子1位置1(x=-2.05, y=-0.80) + 柱子2位置1(x=-2.05, y=-2.05) → 模板0
- 柱子1位置2(x=-2.65, y=-0.80) + 柱子2位置2(x=-2.65, y=-2.05) → 模板1
- 柱子1位置1(x=-2.05, y=-0.80) + 柱子2位置2(x=-2.65, y=-2.05) → 模板2
- 柱子1位置2(x=-2.65, y=-0.80) + 柱子2位置1(x=-2.05, y=-2.05) → 模板3

### 3. YAML配置 `main_control/config/pillar_nav.yaml`
```yaml
# 导航模式: "ego" 或 "pcl"
nav_mode: "pcl"

# 柱子位置（世界坐标）
pillars:
  p1_pos1: [-2.05, -0.80]
  p1_pos2: [-2.65, -0.80]
  p2_pos1: [-2.05, -2.05]
  p2_pos2: [-2.65, -2.05]

# 4种配置的航点序列（相对init_pos的偏移）
pillar_waypoints:
  case_00:  # 柱1位1, 柱2位1
    - [-2.75, 0.0, 0.0]
    - [-2.75, -2.7, 0.0]
    - [-0.45, -2.7, 0.0]
  case_01:  # 柱1位2, 柱2位2
    - [-1.95, 0.0, 0.0]
    - [-1.95, -2.7, 0.0]
    - [-0.45, -2.7, 0.0]
  case_02:  # 柱1位1, 柱2位2
    - [-2.75, 0.0, 0.0]
    - [-2.75, -1.0, 0.0]
    - [-1.45, -2.7, 0.0]
  case_03:  # 柱1位2, 柱2位1
    - [-1.95, 0.0, 0.0]
    - [-1.95, -0.8, 0.0]
    - [-2.80, -2.7, 0.0]
    - [-0.45, -2.7, 0.0]

# 返程时反向遍历航点
```

### 4. 主控状态机
新增/修改状态（注释旧代码，不要删除）：
- `PILLAR_DETECT` — 悬停等待PCL柱子检测结果
- `NAV_PILLAR_WAYPOINTS` — 按顺序飞航点（正向）
- `RETURN_PILLAR_WAYPOINTS` — 反向飞航点

状态流转：
```
... → PILLAR_DETECT → NAV_PILLAR_WAYPOINTS → ... → RETURN_PILLAR_WAYPOINTS → RETURN
```

### 5. EGO/PCL切换
在`main_control.yaml`加：
```yaml
pillar_nav_mode: "pcl"  # "ego" 或 "pcl"
```
主控读取此变量，决定走新PCL定点路线还是旧EGO路线。旧代码全部注释保留。

## 修改文件清单
| 文件 | 操作 |
|------|------|
| `pcl_detection2/include/core/pillar_detect.hpp` | **新建** |
| `pcl_detection2/src/main.cpp` | 集成pillar_detect |
| `pcl_detection2/config/pcl_detection2.yaml` | 加pillar_roi配置 |
| `main_control/config/pillar_nav.yaml` | **新建** |
| `main_control/config/main_control.yaml` | 加pillar_nav_mode |
| `main_control/include/mission_manager.h` | 加状态/成员/航点vector |
| `main_control/src/mission_manager_core.cpp` | 加载YAML航点 |
| `main_control/src/state_handlers.cpp` | 新状态+注释旧代码 |

## 实现步骤

### Step 1: pillar_detect.hpp
- `PillarDetect`类：init()生成模板，detect()返回最佳匹配配置ID
- 模板生成：根据pillar坐标在对应像素位置画白点

### Step 2: main_control配置和航点加载
- 创建pillar_nav.yaml
- 用`ros::NodeHandle::getParam`读取嵌套YAML到`std::vector<std::vector<Waypoint>>`
- 加载pillar_nav_mode

### Step 3: 状态机
- PILLAR_DETECT: 等待检测结果，确定case_id
- NAV_PILLAR_WAYPOINTS: 遍历waypoints[case_id]
- RETURN_PILLAR_WAYPOINTS: 反向遍历

### Step 4: main.cpp集成
- 在cloudCallback中调用pillar_detect（和ring_detect类似，每5帧一次）
- 发布检测结果到topic

## 验证
- 编译pcl_detection2和main_control
- 检查YAML加载是否正确
- 跑仿真验证4种配置的检测和导航
