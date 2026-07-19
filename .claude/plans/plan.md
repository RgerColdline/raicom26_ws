# 融合方案：把 raicom_vision_laser 的投货/视觉/射击融合进 main_control

## 目标
把队友的 main_control 主控里的**投货、视觉识别、射击激光**三块业务逻辑，替换为我的 `raicom_vision_laser/src/mission_flow.cpp` 的实现。
**不动**起飞、穿环、柱子导航、EGO、返程、降落等控制和避障代码。

## 任务流程（用户指定）
飞到投放区 `(-0.45, -2)` → **z=1.5 悬停** → **下降到 0.3m 投货**（mission_flow 原逻辑，沙包垂直落更准）→ 升回 1.5 → 前视识别靶标 `A` → 判左/右/中 → 飞射击点 → 稳定 → 激光射击 → 等待确认 → 返程

- 左射击点 `(-0.45, -1.5)`，右射击点 `(-0.45, -2.5)`，默认/中心 `(-0.45, -2.0)`
- 投货纯定点，**不接下视 YOLO**（用户确认）

## 核心策略
1. **复用 main_control 的控制框架**：类架构、状态机骨架、控制辅助函数（`moveTo`/`navTo`/`positionControl`/`hover`）全部保留，沿用队友的速度模式 setpoint（`0b100111000111`），不引入 mission_flow 的位置模式 setpoint，不碰控制。
2. **只换业务逻辑**：把 mission_flow 状态3（投货）和状态5（识别+射击）的逻辑，移植到 main_control 对应状态处理函数。
3. **视觉换源**：订阅我的 `/yolo_front_detect`（前视 YOLO，320×240，中心160）替换队友的 `/ocr_detect`（OCR）。
4. **投货/激光接口已匹配**：`/servo_control`、`/shoot`、`/laser_control` 三话题 main_control 已发布、stm32_shooter_node 已订阅，**无需改 stm32_shooter_node 和 yolo_front_detector.py**。

## 坐标系核对（已验算，无冲突）
- main_control 航点是**相对起飞点偏移**（`moveTo` 内部 `init_pos + offset`）。用户给的"相较于无人机"坐标 = 相对起飞点，**一致**。
- 投放区 `wp_drop_area=(-0.45,-2.0)` 已与用户数据吻合，只需把 z 从 1.2 改 1.5。
- 射击点符号：`left_y=-1.5`（+Y 侧）、`right_y=-2.5`（-Y 侧）、投放区 `y=-2.0` 居中。前视相机不镜像（图像左=+Y=机体左），与 mission_flow"目标在图像左→飞左射击点(+Y)"逻辑一致。
- 前视 YOLO 图像 320×240，**中心 x=160**（main_control 现有 `IMG_CENTER_X=320` 是 OCR 的分辨率，需改用 160）。

## 改动清单（7 个文件）

### 1. `main_control/include/mission_manager.h`
- `Config` 结构增加 mission_flow 参数：
  - 投货：`cargo_drop_angle`、`cargo_reset_angle`、`cargo_hold_time`、`descent_timeout`、`drop_hover_time`、`drop_z`
  - 前视识别：`yolo_img_center_x`(=160)、`yolo_target_timeout`、`yolo_detect_timeout`、`mission_target`
  - 射击：`shoot_duration`、`shoot_z`(=1.5)
- 增加成员变量：
  - 前视：`front_target_matched_`、`matched_target_`、`matched_center_x_/y_`、`last_matched_time_`
  - 投货子状态：`drop_sub_state_`、`last_drop_pub_time_`
  - 射击：`shoot_triggered_`、`shoot_time_`

### 2. `main_control/src/mission_manager_core.cpp`
- `initROSCommunication`：`yolo_detect_sub_` 订阅话题从 `/ocr_detect` 改为 `/yolo_front_detect`
- `loadParameters`：读取上述新增参数

### 3. `main_control/src/callbacks.cpp`
- `yoloDetectCallback` 重写为 mission_flow 风格：订阅 `/yolo_front_detect`，在攻击相关状态匹配 `attack_real_target`("A")，更新 `front_target_matched_`/`matched_center_x/y_`/`last_matched_time_`。保留 `current_detection_` 更新以兼容精准降落回退逻辑。

### 4. `main_control/src/state_handlers.cpp`（核心改动）
- `handleNavToDropArea`：去掉 `callSwitchCamera`/`callResetTarget` 调用，到达投放区后进 `HOVER_RECOG_DROP`
- `handleHoverRecognizeDrop`：重写为 mission_flow 状态3 Sub0（在投放点 z=1.5 悬停 `drop_hover_time` 秒），完成进 `DROP_SUPPLY`
- `handleDropSupply`：重写为 mission_flow 状态3 Sub1+Sub2（下降到 `drop_z=0.3` 带超时兜底 → 发 `cargo_drop_angle` 触发投货 → 每 0.2s 重发保持 → `cargo_hold_time` 后发 `cargo_reset_angle`×3 复位），完成进 `MOVE_TO_ATTACK_AREA`
- `handleMoveToAttackArea`：简化为飞到攻击区 `(-0.45,-2,1.5)`（升回 1.5），到达后进 `RECOG_ATTACK_TARGET`（去掉视觉服务调用）
- `handleRecognizeAttackTarget`：重写为 mission_flow 状态5 Sub1（前视找 target A，判左/右/中/超时，设 `shoot_target_x/y_` 为相对偏移），进 `ALIGN_ATTACK_TARGET`
- `handleAlignAttackTarget`：重写为 mission_flow 状态5 Sub2/3/5（`moveTo` 飞左/右/默认射击点 z=1.5）+ 到达稳定 `shoot_stable_time`，进 `SIMULATE_ATTACK`
- `handleSimulateAttack`：重写为发 `/shoot` + 等 `shoot_duration`，进 `WAIT_HIT_CONFIRMATION`
- `handleWaitHitConfirmation`：保留（等 `hit_confirmed_` 或超时返程）

### 5. `main_control/config/main_control.yaml`
- `wp_drop_area_z: 1.5`、`wp_attack_area_z: 1.5`
- `shoot` 段更新：`left_x=-0.45 left_y=-1.5 right_x=-0.45 right_y=-2.5 default_x=-0.45 default_y=-2.0`，新增 `duration: 1.5`、`shoot_z: 1.5`
- 新增 `cargo` 段：`drop_angle=0 reset_angle=180 hold_time=4.0 descent_timeout=10.0 drop_hover_time=2.0 drop_z=0.3`
- 新增 `yolo` 段：`img_center_x=160 target_timeout=0.5 detect_timeout=60.0 mission_target="A"`
- 新增 `yolo_front_detector` 段（Python 节点私有参数：device/confidence/imgsz/image_topic 等）

### 6. `main_control/launch/main_control.launch`
- 加载 `yolo_front_detector` 私有参数
- 启动 `yolo_front_detector` 节点（pkg=raicom_vision_laser，订阅 `/camera/color/image_raw`，`weights_path` 指向 `best.engine`）
- 启动 `stm32_shooter_node` 节点（串口 `/dev/ttyCH343USB0`@115200）

### 7. `main_control/shell/main_control_uav.sh`
- 去掉 `raicom_vision_only.launch`（避免与 main_control.launch 里的 stm32_shooter 抢串口 + OCR 不再需要）
- 该 tmux 窗口改为 `rostopic echo /yolo_front_detect` 监控前视检测

## 明确不动的部分
- 状态：`INIT_TAKEOFF`、`MOVE_TO_RING_FRONT`、`SETOUT_CROSS_RING`、`RETURN_CROSS_RING`、`PILLAR_DETECT`、`NAV_PILLAR_WAYPOINTS`、`RETURN_PILLAR_WAYPOINTS`、`NAV_TO_RING_BACK`、`READY_NAV_TO_RING_BACK`、`RETURN`、`LAND`、`TASK_END`、`MOVE_TO_FRONT_OF_TARGET`（死代码）
- 控制辅助：`moveTo`/`navTo`/`positionControl`/`hover`/`reachedTarget`/`getPixPidVel`/环追踪/柱子导航等
- 外部依赖：EGO planner、pcl_detection2、FAST-LIO、`pillar_nav.yaml`、`uav_navigation`
- 我的节点源码：`stm32_shooter_node.cpp`、`yolo_front_detector.py`（接口已匹配，不改）

## 编译/验证（本机 Windows 仅编辑，不编译）
- 机载 Linux：`catkin_make --pkg main_control` → `source devel/setup.bash`
- 运行：`bash src/main_control/shell/main_control_uav.sh`（实机全套），或先手动起 mavros+相机+FAST-LIO 再 `roslaunch main_control main_control.launch`
- main_control 节点启动后按回车开始任务（main.cpp 的 `std::cin.get()`）

## 风险/注意
- main_control 现有 `RECOG_ATTACK_TARGET`/`ALIGN_ATTACK_TARGET` 存在 `shoot_target_x_ = init_pos + cfg` 后又 `moveTo(shoot_target_x_)`（moveTo 再加一次 init_pos）的双加隐患。重写时统一用**相对偏移语义**（`shoot_target_x_/y_` 存 cfg 值，`moveTo` 内部加 init_pos），消除该隐患。
- `IMG_CENTER_X` 由 320(OCR) 改 160(YOLO 320×240)，否则左右判断全偏。
- 前视 YOLO 全程开启（无下视，不发布 `/yolo_front_enable`，节点默认开）。
