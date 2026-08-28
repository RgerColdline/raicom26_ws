#!/usr/bin/env bash
# 预期存放路径: <工作空间>/src/main_control/shell/
# 兼容 bash/zsh 执行，支持环境变量覆盖

export LANG="${LANG:-zh_CN.UTF-8}"
export LC_ALL="${LC_ALL:-zh_CN.UTF-8}"

# ============================================
# 1. 动态路径推导 (兼容 source 与直接执行)
# ============================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# 向上追溯三层: shell/ -> main_control/ -> src/ -> 工作空间根目录
WS="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"

# ============================================
# 2. 环境变量配置 (固定项与可变项分离)
# ============================================
# 可变路径: 支持通过 export 覆盖，适应不同设备
SIM_WS="${SIM_WS:-$HOME/catkin_ws}"
EGO_WS="${EGO_WS:-$HOME/ego_ws}"
LIO_WS="${LIO_WS:-$HOME/ros_libraries_ws}"

# 固定路径: 按需求保留默认值，多设备位置一致
PX4_PATH="$HOME/Libraries/PX4-Autopilot"

# 识别当前终端 Shell 类型，动态匹配 setup 脚本后缀
CURRENT_SHELL="${SHELL##*/}"
[ -z "$CURRENT_SHELL" ] && CURRENT_SHELL="bash"

SESSION="mission"
tmux kill-session -t "$SESSION" 2>/dev/null
sleep 1

# 清理上次残留的Gazebo进程（不杀roscore，不影响其他ROS使用）
killall -9 gzclient gzserver gazebo 2>/dev/null || true
sleep 1

echo "======================================"
echo "  无人机竞赛任务启动中..."
echo "======================================"
echo "  主控工作空间: $WS"
echo "  仿真工作空间: $SIM_WS"
echo "  导航工作空间: $EGO_WS"
echo "  LiDAR工作空间: $LIO_WS"
echo "  PX4 路径:     $PX4_PATH"
echo "======================================"

# 关键目录存在性校验
for dir in "$WS" "$SIM_WS" "$EGO_WS" "$LIO_WS" "$PX4_PATH"; do
    if [ ! -d "$dir" ]; then
        echo "[错误] 依赖目录缺失: $dir"
        exit 1
    fi
done

# ---------------------------------------------------------
# 窗口 0：核心与仿真 (左右分屏)
# ---------------------------------------------------------
tmux new-session -d -s "$SESSION" -n "Core_Sim"
tmux send-keys -t "$SESSION:0" "roscore" C-m

tmux split-window -h -t "$SESSION:0"
CMD_SIM="sleep 3; \
source '${SIM_WS}/devel/setup.${CURRENT_SHELL}'; \
source '${PX4_PATH}/Tools/setup_gazebo.sh' '${PX4_PATH}' '${PX4_PATH}/build/px4_sitl_default'; \
export ROS_PACKAGE_PATH=\"\$ROS_PACKAGE_PATH:${PX4_PATH}:${PX4_PATH}/Tools/sitl_gazebo\"; \
roslaunch tutorial_gazebo sim.launch"
tmux send-keys -t "$SESSION:0" "$CMD_SIM" C-m

# ---------------------------------------------------------
# 窗口 1：主控、监控与视觉 (四等分 2x2)
# 注: main_control.launch 内含 状态机 + 前视YOLO + stm32_shooter(仿真已禁用)
# ---------------------------------------------------------
tmux new-window -t "$SESSION" -n "Control_Vision"

tmux send-keys -t "$SESSION:1" "sleep 18; rostopic echo /mavros/local_position/pose" C-m

tmux split-window -h -t "$SESSION:1"
# 仿真覆盖: use_stm32:=false(无串口, required节点退出会拖垮launch)
#           yolo走 best.pt + cpu(无GPU, engine是Jetson TensorRT专用)
CMD_MAIN="sleep 10; \
source '${WS}/devel/setup.${CURRENT_SHELL}'; \
roslaunch main_control main_control.launch \
use_stm32:=false \
yolo_device:=cpu \
yolo_weights:=${WS}/src/raicom_vision_laser/models/best.pt"
tmux send-keys -t "$SESSION:1" "$CMD_MAIN" C-m

tmux split-window -v -t "$SESSION:1"
tmux send-keys -t "$SESSION:1" "sleep 16; source '${WS}/devel/setup.${CURRENT_SHELL}'; rostopic echo /yolo_front_detect" C-m

tmux select-pane -L -t "$SESSION:1"
tmux split-window -v -t "$SESSION:1"
tmux send-keys -t "$SESSION:1" "sleep 18; rostopic echo /mavros/state" C-m

tmux select-layout -t "$SESSION:1" tiled

# ---------------------------------------------------------
# 窗口 2：感知与导航 (2x2 田字格)
# ---------------------------------------------------------
tmux new-window -t "$SESSION" -n "Perception_Nav"

# 左上：FAST-LIO2 (LiDAR-IMU紧耦合SLAM)
tmux send-keys -t "$SESSION:2" "sleep 10; source '${LIO_WS}/devel/setup.${CURRENT_SHELL}'; roslaunch fast_lio mapping_mid360_fastlio.launch" C-m

# 左下：PCL点云感知 (方环检测 + 柱子检测pillar_case_id + 障碍物处理)
tmux split-window -v -t "$SESSION:2"
tmux send-keys -t "$SESSION:2" "sleep 14; source '${LIO_WS}/devel/setup.${CURRENT_SHELL}'; source '${WS}/devel/setup.${CURRENT_SHELL}' --extend; roslaunch pcl_detection2 pcl_detection2.launch" C-m

# 右：EGO-Planner 路径规划与避障
# 注: EGO 影子模式：仅规划+RViz可视化，飞控由 main_control PCL/traverse 全权负责
# tmux split-window -h -t "$SESSION:2"
# CMD_NAV="sleep 16; \
# source '${WS}/devel/setup.${CURRENT_SHELL}'; \
# source '${EGO_WS}/devel/setup.${CURRENT_SHELL}' --extend; \
# roslaunch uav_navigation ego_nav.launch shadow_mode:=true"
# tmux send-keys -t "$SESSION:2" "$CMD_NAV" C-m

tmux select-layout -t "$SESSION:2" tiled

# ---------------------------------------------------------
# 窗口 3：PCL柱子检测监控
# ---------------------------------------------------------
tmux new-window -t "$SESSION" -n "Pillar_Monitor"

# PCL柱子检测结果监控 (pillar_nav_mode:=pcl 时 PILLAR_DETECT 状态等这个话题)
tmux send-keys -t "$SESSION:3" "sleep 16; rostopic echo /pcl_detection2/pillar_case_id" C-m

# ============================================
# 完成配置并附加会话
# ============================================
tmux select-window -t "$SESSION:1"
tmux attach-session -t "$SESSION"