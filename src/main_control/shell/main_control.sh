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

# 固定路径: 按需求保留默认值，多设备位置一致
PX4_PATH="$HOME/Libraries/PX4-Autopilot"

# 识别当前终端 Shell 类型，动态匹配 setup 脚本后缀
CURRENT_SHELL="${SHELL##*/}"
[ -z "$CURRENT_SHELL" ] && CURRENT_SHELL="bash"

SESSION="mission"
tmux kill-session -t "$SESSION" 2>/dev/null
sleep 1

echo "======================================"
echo "  无人机竞赛任务启动中..."
echo "======================================"
echo "  主控工作空间: $WS"
echo "  仿真工作空间: $SIM_WS"
echo "  导航工作空间: $EGO_WS"
echo "  PX4 路径:     $PX4_PATH"
echo "======================================"

# 关键目录存在性校验
for dir in "$WS" "$SIM_WS" "$EGO_WS" "$PX4_PATH"; do
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
# ---------------------------------------------------------
tmux new-window -t "$SESSION" -n "Control_Vision"

tmux send-keys -t "$SESSION:1" "sleep 18; rostopic echo /mavros/local_position/pose" C-m

tmux split-window -h -t "$SESSION:1"
tmux send-keys -t "$SESSION:1" "sleep 10; source '${WS}/devel/setup.${CURRENT_SHELL}'; roslaunch main_control main_control.launch" C-m

tmux split-window -v -t "$SESSION:1"
tmux send-keys -t "$SESSION:1" "sleep 16; source '${WS}/devel/setup.${CURRENT_SHELL}'; roslaunch raicom_vision_laser raicom_ocr_laser.launch" C-m

tmux select-pane -L -t "$SESSION:1"
tmux split-window -v -t "$SESSION:1"
tmux send-keys -t "$SESSION:1" "sleep 18; rostopic echo /ego_controller/status" C-m

tmux select-layout -t "$SESSION:1" tiled

# ---------------------------------------------------------
# 窗口 2：感知与导航 (左右分屏)
# ---------------------------------------------------------
tmux new-window -t "$SESSION" -n "Perception_Nav"

tmux send-keys -t "$SESSION:2" "sleep 12; source '${WS}/devel/setup.${CURRENT_SHELL}'; roslaunch pcl_detection2 pcl_detection2.launch" C-m

tmux split-window -h -t "$SESSION:2"
CMD_NAV="sleep 14; \
source '${WS}/devel/setup.${CURRENT_SHELL}'; \
source '${EGO_WS}/devel/setup.${CURRENT_SHELL}' --extend; \
roslaunch uav_navigation ego_nav.launch"
tmux send-keys -t "$SESSION:2" "$CMD_NAV" C-m

# ============================================
# 完成配置并附加会话
# ============================================
tmux select-window -t "$SESSION:1"
tmux attach-session -t "$SESSION"