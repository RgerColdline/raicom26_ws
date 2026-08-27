# RAICOM 无人机竞赛工作空间介绍

> 工作空间根目录：`/home/gutlord/raicom26_ws`
> 技术栈：ROS1 (Noetic) + C++ 飞行控制 + Python 视觉 + PCL 点云感知 + MAVROS + PX4 飞控

## 1. 概述

本工作空间用于 **RAICOM（睿抗）无人机竞赛**，实现从起飞到降落的完整自主任务：

```
起飞 → 穿方环 → 穿越绕柱(平滑样条轨迹) → 悬停下视识别字母(A/B) →
下降投货 → 固定映射飞射击点 → 激光射击 → 原路返程 → 降落
```

核心设计思路是 **“定点 + 平滑”**：不依赖在线避障/复杂规划，用固定航点 + 离线平滑轨迹换取可复现的稳定性。

## 2. 目录结构

```
raicom26_ws/
├── src/
│   ├── main_control/          # 主控状态机（本包，见第 6 节）
│   ├── raicom_vision_laser/   # 视觉 + 定点射击 + 穿越规划 + STM32 串口
│   ├── pcl_detection2/        # 点云感知：方环检测 + 柱子布局检测(case)
│   ├── uav_navigation/        # EGO-Planner 路径规划（影子模式 / 备选）
│   └── utils/                 # 依赖工具（decomp / pose_utils / quadrotor_msgs 等）
├── devel/                     # catkin 编译产物
└── build/                     # catkin 构建中间文件
```

## 3. 各包说明

| 包 | 作用 | 关键内容 |
|----|------|----------|
| `main_control` | **主控状态机**：起飞、穿环、穿越绕柱、投货、定点射击、返航降落 | `src/main_control.cpp` + `include/main_control.h` |
| `raicom_vision_laser` | 视觉 + 射击/投货串口 + 穿越规划 + 精简版流程 | `mission_flow` / `traverse_node` / `stm32_shooter_node` / YOLO 节点 |
| `pcl_detection2` | Livox 点云感知 | 方环 `square_ring`、柱子布局 `pillar_case_id` |
| `uav_navigation` | EGO-Planner | 仅影子模式可视化，不参与实际控制 |
| `utils` | 三方依赖 | decomp 凸分解、quadrotor_msgs、pose_utils 等 |

## 4. 完整任务流程

`main_control` 是总控，负责把下面这些阶段串起来：

1. **起飞**：发送设定点 → 切 OFFBOARD → 解锁 → 爬升到 `takeoff_height`。
2. **穿方环**：飞往环前方 → 穿越 → 记住环前/环后记忆点（返程用）。
3. **穿越绕柱**（`pillar_nav_mode = "pcl"`）：
   - 飞到悬停扫描点，触发 `pcl_detection2` 检测柱子布局 `case`；
   - 现场规划 `leg2` 绕柱平滑轨迹（自然三次样条 + 碰撞检测 + 时间参数化）；
   - 按时间插值跟踪轨迹，飞到投放区。
4. **悬停识别**：在投放区上方悬停，下视 YOLO 对字母 A/B 投票，决定射击靶标。
5. **投货**：下降到 `drop_z` → 开舱（`/servo_control`）→ 保持 → 关舱。
6. **定点射击**：按识别字母固定映射飞左/右射击点 → 稳定 → 发 `/shoot`。
7. **返航降落**：倒放 `leg2` 原路返回 → 穿环返回 → 回起飞点 → 定点下降 → `AUTO.LAND`。

> `pillar_nav_mode = "ego"` 时，绕柱段走 EGO-Planner 路径（`navTo` / `sendEgoGoal`），作为回退方案。

## 5. 关键话题 / 服务

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `/mavros/state` | `mavros_msgs/State` | 飞控连接/模式/解锁状态 |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | FAST-LIO 里程计 |
| `/yolo_down_detect` | `raicom_vision_laser/DetectionInfo` | 下视字母检测（投货投票） |
| `/yolo_front_detect` | `raicom_vision_laser/DetectionInfo` | 前视检测（仅日志/画面） |
| `/pcl_detection2/square_ring` | `pcl_detection2/SquareRing` | 方环位姿 |
| `/pcl_detection2/pillar_case_id` | `std_msgs/Int32` | 柱子布局 case |
| `/ego_controller/status` | `std_msgs/Int8` | EGO 导航状态 |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | OFFBOARD 设定点 |
| `/fsm/ego_goal` | `geometry_msgs/PoseStamped` | EGO 导航目标 |
| `/servo_control` | `std_msgs/UInt8` | 投货舵机 |
| `/shoot` | `std_msgs/Empty` | 一键激光射击 |
| `/laser_control` | `std_msgs/Bool` | 激光开关（安全保险） |
| `/pcl_detection2/start_pillar_detect` | `std_msgs/Empty` | 触发柱子检测 |

### 服务客户端

`/mavros/set_mode`（OFFBOARD / AUTO.LAND）、`/mavros/cmd/arming`、`/switch_camera`、`/reset_target`。

## 6. main_control 包结构（已重构）

`main_control` 已由原来的 `MissionManager` 类 + 多文件架构，重构为**单文件过程式**，
结构与 `raicom_vision_laser` 的 `mission_flow` 完全对齐：

```
main_control/
├── include/
│   └── main_control.h    # 全局变量 + 所有函数声明/实现（回调、控制辅助、穿越规划器等）
├── src/
│   └── main_control.cpp  # main()：起飞前 OFFBOARD/解锁 + 展开后的完整状态机
├── config/
│   ├── main_control.yaml # 主控参数
│   └── pillar_nav.yaml   # （已废弃，参数已并入 traverse_map.yaml）
├── launch/
│   └── main_control.launch
├── shell/
│   ├── main_control.sh     # 仿真一键启动
│   └── main_control_uav.sh # 实机一键启动
├── CMakeLists.txt
└── package.xml
```

- 无 `class`、无 `private`，全部改为全局变量 + 自由函数。
- 起飞前的 OFFBOARD/解锁/爬升逻辑放在 `main()` 里（对齐 `mission_flow.cpp` 的写法）。
- 后续任务流程用 `switch(current_state)` 大状态机展开在 `main()` 中。

## 7. 编译与运行

```bash
cd /home/gutlord/raicom26_ws
catkin build main_control
source devel/setup.bash
```

- **仿真**：`bash src/main_control/shell/main_control.sh`
- **实机**：`bash src/main_control/shell/main_control_uav.sh`

启动后焦点落在 `main_control` 面板，**输入 `1` 才开始任务**。

## 8. 参数修改提示

- 改 `.yaml` / `.launch` 无需重新编译；改 `.cpp` / `.h` 需 `catkin build`。
- 主控参数在 `config/main_control.yaml`，穿越地图在 `raicom_vision_laser/config/traverse_map.yaml`（由 launch 加载到 `main_control` 命名空间）。
- 坐标系约定、STM32 串口协议、case 编号等细节见 `raicom_vision_laser/CLAUDE.md`。

## 9. 查看 ROS 日志

所有节点的运行日志统一写到 `~/.ros/log/`（rosout 汇总 + 每个节点独立文件）：

- `~/.ros/log/latest/`：软链接，指向**最近一次** roscore 会话的目录
- `~/.ros/log/2026-08-19/` 等日期目录：历史会话按日期归档
- `rosout.log`：**所有节点日志的汇总**（排查问题首选）
- `<节点名>-N.log`：单个节点的独立 stdout 日志
- `roslaunch-*.log`：roslaunch 生命周期（进程死因 / RLException / 退出码）

常用命令：

```bash
# 1. 看最近一次运行的全部日志
cd ~/.ros/log/latest

# 2. 按节点 + 关键词过滤
#    日志行带 [topics: ...] 大尾巴，用 grep -o 只留消息部分
grep "/main \[" ~/.ros/log/latest/rosout.log | grep -o "\[Pillar.*\|\[数据源.*\|\[穿越.*"

# 3. 只看某个节点的独立日志
cat ~/.ros/log/latest/main-*.log

# 4. roslaunch 报错 / 节点死因
grep -E "process has died|RLException" ~/.ros/log/latest/roslaunch-*.log

# 5. 实时看（不查文件）
rostopic echo /rosout          # 原始流
rqt_console                    # 图形化过滤

# 6. 找某关键词出现在哪次运行（历史日志都保留着）
grep -rl "ROI裁剪后无点" ~/.ros/log/ 2>/dev/null
```

要点：

- 每行行首时间戳是**仿真时间**（如 `96.331000000`），不是墙钟时间；可与 rviz 同一次任务互相对时间线。
- 排查"某功能没生效"时，先确认 `rosout.log` 里该节点是否出现过——完全没有 = 节点没起来或没收到输入。
- `roslaunch-*.log` 里 `process has died [exit code N]` 是定位节点崩溃/依赖缺失的第一现场（2026-08-19 的 livox 插件旧库问题就是这么查出来的）。

## 10. 改动记录（2026-08-18）

### 10.1 main_control 架构重构（单文件过程式）

把原来的 `MissionManager` 类 + 多文件架构重构为与 `mission_flow` 一致的**单文件过程式**：

| 操作 | 文件 |
|------|------|
| 删除 | `src/main.cpp`、`callbacks.cpp`、`mission_manager_core.cpp`、`state_handlers.cpp`、`include/mission_manager.h`、`include/traverse_planner.h` |
| 新增 | `include/main_control.h`（全局变量 + 回调 + 控制辅助 + 穿越规划器） |
| 新增 | `src/main_control.cpp`（`#include "main_control.h"` + `main()`：起飞前 OFFBOARD/解锁 + 展开后的 `switch` 状态机） |
| 修改 | `CMakeLists.txt`（`add_executable(main_control src/main_control.cpp)`） |

- 无 `class`、无 `private`，全部为全局变量 + 自由函数。
- 起飞前的 OFFBOARD/解锁/爬升逻辑严格对齐 `raicom_vision_laser/src/mission_flow.cpp` 第 80~178 行。
- 功能完全一致：节点名、话题、服务、参数命名空间、状态流转、日志全部不变，`main_control.sh` 启动方式不变。

### 10.2 雷达仿真链路修复（不建图 → 正常建图）

**根因**：无人机模型引用的 Livox 激光仿真插件（`liblivox_laser_simulation.so`）本机不存在，Gazebo 加载失败 → `/livox/lidar` 无数据 → FAST-LIO 无输入 → 不建图。

| 操作 | 内容 |
|------|------|
| 新增插件源码 | `~/catkin_ws/src/Mid360_simulation_plugin/`（fork 自 fratopa/Mid360_simulation_plugin，改了几处才在本机编译通过） |
| 插件编译产物 | `liblivox_laser_simulation.so`，当前装于 `/tmp/livox_build_ws/devspace/lib/`（`main_control.sh` 的 `GAZEBO_PLUGIN_PATH` 已优先指向它） |
| 插件改动 | 发布 `livox_ros_driver2::CustomMsg`；`ros::init` 节点名修正；打空/无效点直接丢弃（不保留零点）；支持 csv 绝对路径；`zenith` 转换方向修正（`M_PI_2 - 天顶角`，原来方向反了导致雷达向下扫地面、看不到柱子和墙） |
| 修改 SDF | `tutorial_gazebo/models/livox_mid360/model.sdf` + `models/raicom_map/map.world`（内嵌无人机副本，两处都要改）：csv 指向队友的 `mid360.csv`（绝对路径）；`publish_pointcloud_type=3`（CustomMsg）；`frameName=body`；删除 `laser_livox_pcl` sensor；关掉冗余可视化；盲区 `min=0.35m`（滤机架/桨叶）；`max=50m`（打空点仍被插件丢弃） |
| 修改 map.world | 给 `laser_livox` sensor 补 `<always_on>1</always_on>`（否则雷达永远不更新） |
| 修改 sim.launch | 移除 include 旧版 `mapping_mid360.launch`（避免与 main_control.sh 窗口 2 的 `fastlio_mapping` 双实例抢数据） |
| 修改 main_control.sh | CMD_SIM 加 `LD_LIBRARY_PATH`（gazebo-11/plugins，相机插件依赖）+ `GAZEBO_PLUGIN_PATH` 优先指向新插件 |
| 地面过滤 | 插件发布点云时按**世界系 z < 0.02m** 丢弃地面点（发布坐标仍为雷达系，不破坏 FAST-LIO/pcl_detection2） |

**关键数据事实**（队友 `mid360.csv`）：
- 位置：`~/catkin_ws/src/tutorial_gazebo/models/livox_mid360/mid360.csv`（16MB，800000 行，格式 `Time/s,Azimuth/deg,Zenith/deg`）
- `zenith` 是**天顶角**（相对竖直向上 0°），范围 37.836°~97.212°（跨度 59.38° = MID360 垂直视场）
- 正确俯仰角换算：`pitch = 90° - zenith`，覆盖 **-7.2°~+52.2°**（上半球 + 微向下）

**rviz 查看要点**：
- `/livox/lidar` 是 `livox_ros_driver2/CustomMsg`，**rviz 标准 PointCloud2 显示不支持**，看原始雷达用 `/pcl_detection2/raw_livox_cloud`
- FAST-LIO 建图看 `/cloud_registered`、`/fastlio_map`，Fixed Frame 设为 `camera_init`（TF 链 `map→odom→camera_init→body` 完整）

### 10.3 已知限制 / 注意事项

- **沙箱限制**：本会话的命令被 TRAE 沙箱限制，catkin_ws / PX4 等路径不可写，插件编译只能在 `/tmp/livox_build_ws` 进行；`/tmp` 重启会被清空，**验证通过后建议把 `liblivox_laser_simulation.so` 正式拷到 `~/Libraries/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic/`**（或保留 `GAZEBO_PLUGIN_PATH` 指向）。
- 仿真**不需要**启动 `livox_ros_driver2` 节点（那是真机驱动），雷达数据全靠 Gazebo 插件。
- 贴地（未起飞）时雷达向上大角度没有物体可反射，扫不到柱子是正常物理；起飞到任务高度后视野完整。

### 10.4 柱子检测重构 + 插件旧库坑（2026-08-19）

**问题一（旧链路）**：pcl_detection2 跑的是 livox 自建图+ICP 链路，柱子检测只吃"ICP 接受帧"的配准地图，仿真里 ICP 持续回退导致柱区无点，每帧报 `[Pillar] ROI裁剪后无点`，永远超时回退 default_case。

**问题二（10.3 预言的坑成真）**：重启后 `/tmp/livox_build_ws` 被清空，gazebo 退而加载 PX4 `build_gazebo-classic/` 里的**旧版插件拷贝**（无 csv 绝对路径修复）-> csv 加载失败 -> `/livox/lidar` 无数据 -> FAST-LIO 不建图 -> 一切检测饿死。

| 操作 | 内容 |
|------|------|
| 重构 pillar_detect.hpp | 模板匹配为主 -> **坐标判定为主**（4 个候选柱位半径内点数，A/B 侧各出 1 根拼 case，连续 3 帧一致才发布），模板匹配降级为歧义兜底 |
| 切数据源 | `pcl_detection2.yaml` 加 `cloud_source: fastlio`，柱子/方环/障碍物管线统一吃 `/fastlio_map`（camera_init 与 odom 重合），废弃自建图+ICP |
| 新增 pillar_coord 参数段 | `radius=0.25 / min_points=5 / confirm_frames=3 / ambiguous_ratio=0.8`（灵敏度按仿真取宽松值，真机再调） |
| 重编译插件 | 源码本就有绝对路径修复但 .so 过期，`catkin_make` 重编到 `~/livox_plugin_ws/devel/lib/` |
| 修 main_control.sh | `GAZEBO_PLUGIN_PATH` 由易失的 `/tmp/livox_build_ws/...` 改为持久的 `~/livox_plugin_ws/devel/lib`（优先级最高，遮蔽 PX4 目录旧拷贝） |

**验证结果**（2026-08-19 仿真）：触发后 **1.14s** 检出 `case2`（A右+B左，与 map.world 摆柱一致）；空柱位点数 0、实柱 ~250，判定极干净。main_control 正常采用并完成后续绕柱。

### 10.5 任务流程提速（2026-08-19）

**目标**：缩短整个任务流程时间（悬停扫描、穿环、绕柱、返程各环节）。

**改动一：流程状态机提速（main_control）**

| 操作 | 内容 |
|------|------|
| 删除环检测流程 | 去掉 `MOVE_TO_RING_FRONT`/`SETOUT_CROSS_RING` 两个状态、`/pcl_detection2/square_ring` 订阅、环多假设追踪（`updateRingTracking`/`decayRingCandidates`/`checkRingTimeout`）及 `ring_track` 配置。起飞后直接 `TRAVERSE_TO_SCAN` 飞扫描点，途中自然穿过环 |
| 扫描到即走 | `TRAVERSE_SCAN`：`detected_case>=0` 立即采用并开始绕柱，不再等满 `scan_hover_time`（原 3.0s）；超时才回退 `default_case` |
| 射击后不回投放区 | `SIMULATE_ATTACK` 结束后直接 `TRAVERSE_RETURN_LEG2` 倒放返程，删除 `TRAVERSE_READY_RETURN`（先回投放区）状态 |
| 返程直接穿环 | `RETURN_CROSS_RING` 简化为固定航点两段：飞环后方（y=0 中心线正对环孔）→ 沿 y=0 直线垂直穿过环孔回环前方，删除记忆点分支与中间停顿 |

**改动二：速度参数 ×1.4**

| 文件 | 参数 | 原值 -> 新值 |
|------|------|------|
| `raicom_vision_laser/config/traverse_map.yaml` | `traverse/v_max` | 0.5 -> **0.7** |
| 同上 | `traverse/a_max` | 0.4 -> **0.56** |
| 同上 | `traverse/a_lat_max` | 0.6 -> **0.84** |
| `main_control/config/main_control.yaml` | `max_speed`（穿环定点控制） | 0.8 -> **1.12** |

**改动三：返程穿环高度提高**

环孔几何（map.world）：孔 z∈[0.75, 2.25]，中心 1.5。返程穿环原高度 1.0（距下沿仅 0.25m，有卡下沿风险），`wp_ring_front_z`/`wp_ring_back_z` 提至 **1.3**（距下沿 0.55m，与去程 `flight_z=1.3` 对齐）。

**改动四：相机话题（顺带）**

`raicom_vision_laser/config/mission_flow.yaml` 前视 `/camera/color/image_raw` -> `/camera_front/image_raw`，下视 `/usb_cam/image_raw` -> `/camera/image_raw`。

**待仿真验证**：提速后穿环/绕柱的过冲与磕碰、`v_max` 是否需要回调。

### 10.6 实机降速 + 实机启动评估（2026-08-19）

**改动：速度参数恢复原值**（实机上线前回退 10.5 的 1.4x）：

| 文件 | 参数 | 值 |
|------|------|-----|
| `raicom_vision_laser/config/traverse_map.yaml` | `traverse/v_max / a_max / a_lat_max` | 0.5 / 0.4 / 0.6 |
| `main_control/config/main_control.yaml` | `max_speed`（穿环定点控制） | 0.8 |

纯 yaml 改动，重启 launch 生效。（返程穿环高度 1.3 属安全项，保留。）

**实机启动评估（`main_control_uav.sh`）结论**：
- ✅ 可跑通链路：uav shell 用 `mapping_mid360_fastlio.launch` 发布 `/fastlio_map`+`/Odometry`，与 pcl_detection2 `cloud_source: fastlio` 订阅匹配；EGO `shadow_mode:=true` 不抢飞控；`cloud_extruder` 随 ego_nav 启动消费投影点云
- ⚠️ 上实机前必办：
  1. pcl_detection2 用 `CMakeLists_uav.txt`（Jetson OpenCV 路径）替换 `CMakeLists.txt` 再编译
  2. `export LIO_WS=<实机FAST-LIO工作空间>`（uav shell 默认 `~/fast_lio_ws`，目录校验不过会 exit 1）
  3. 确认实机 FAST-LIO `laserMapping.cpp:935 if(1)` 已改，否则 `/fastlio_map` 空
  4. 核对相机话题：usb_cam/astra 默认话题 vs yaml（`/camera/image_raw`、`/camera_front/image_raw`）是否有 remap
  5. 场地坐标系：`traverse_map.yaml` origin/pillar_candidates、`pcl_detection2.yaml` pillar_pos 均为官方场地系，出生点不同需全改

### 10.7 camera_init z 偏移 & FAST-LIO IMU 初始化机制（2026-08-26）

**现象**（WSL 仿真 + filter_size_map=0.2）：柱子真实位置 z∈[0,3]（map.world：radius 0.1 / length 3 / pose z=1.5），但 `/fastlio_map`（camera_init 系）里柱子点 z∈[-0.7,0.45]。pcl 的 ROI `z_min=0.3` 把柱子 90% 的点裁掉（日志 `ROI点=6~15 A左=0 A右=1 B左=3 B右=0`），导致检测失败；而 A右 0.3m 圈内实际有 16 点、B左 12 点，**xy 坐标与 pillar_pos 完全吻合**。

**根因**：camera_init（FAST-LIO 世界系）原点 = **初始化完成那一刻 IMU 的位姿**。初始化时飞机已离地（连续多次任务），camera_init z=0 ≈ 场地 2.1m 空中 → 整张地图 z 下沉 ~2m。xy 因初始化时水平位置≈起飞点而重合，所以柱子 xy 判定不受影响。

**FAST-LIO IMU 初始化机制**（`FAST_LIO/src/IMU_Processing.hpp`）：
- 做的事：对初始一小段 IMU 数据求均值——加速度均值=重力方向（定初始 roll/pitch），陀螺均值=gyro 零偏
- 完成条件：`init_iter_num > MAX_INI_COUNT(10)`，即 ~10+ 个 IMU 数据点 = 第 1~2 帧点云，毫秒级完成
- **不是性能问题，也不依赖运动**——恰恰假设 IMU 静止（运动时均值混入运动加速度 → 初始姿态不准）
- 代码**无静止检测**，运动中也能"初始化"成功但结果不可靠
- 日志 `IMU Initial Done` 延迟（如 t=57s）是节点等第一帧点云+IMU 的数据流就绪时间，非算法耗时

**影响与处理**：
- 对 2D 柱子检测**无害**：xy 吻合，把 ROI `z_min` 调宽（如 -1.0）覆盖偏移后的点带即可
- 对消费 3D z 的模块**有害**：方环反投影 z、EGO 投影平面（`plane_z=0.7`）都会带 ~2m 偏移
- **根治**：让 FAST-LIO 在飞机静止于地面时完成初始化（重启后确认未起飞再启动建图）

## 11. TODO

### 主攻方向：提速 -- 缩短整个任务流程时间

以 2026-08-19 仿真实测为基线（各段耗时）：

| 阶段 | 实测 | 备注 |
|------|------|------|
| 悬停扫描 | **3.0s（强制等满）** | 检测本身仅 **1.14s**（触发->3 帧确认->发布），`scan_hover_time=3.0` 强制等满 |
| leg2 绕柱（扫描点->投放区） | ~13.1s | `v_max=0.5, a_max=0.4, a_lat_max=0.6` 偏保守 |
| 投货+返程回扫描点 | ~26.5s | 含 `drop_stabilize_time=1.0 / hold_time=4.0 / hover_before_return=2.0` |

待办（按性价比排序）：

- [x] **扫描提前结束**：收到 case 立即走，不等满 `scan_hover_time`（已做，10.5 改动一）
- [x] **提高巡航速度/加速度**：`v_max/a_max/a_lat_max` ×1.4（已做，10.5 改动二），待仿真验证
- [ ] **压缩投货流程**：`drop_stabilize_time / hold_time / hover_before_return` 各项按舵机实际响应压到下限
- [ ] **到位判据微调**：`err_max=0.15` 收敛末期耗时可观，可试 0.2 配合过冲抑制
- [ ] **起飞/爬升提速**：爬升速率与过渡平滑参数（`main_control.yaml`）
- [ ] 复测全流程总时长，更新上表基线
