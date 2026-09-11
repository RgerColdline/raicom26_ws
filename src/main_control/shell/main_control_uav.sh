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
LIO_WS="${LIO_WS:-$HOME/fast_lio_ws}"

# 固定路径: 按需求保留默认值，多设备位置一致
# PX4_PATH="$HOME/Libraries/PX4-Autopilot"

# 识别当前终端 Shell 类型，动态匹配 setup 脚本后缀
CURRENT_SHELL="${SHELL##*/}"
[ -z "$CURRENT_SHELL" ] && CURRENT_SHELL="bash"

SESSION="mission"
tmux kill-session -t "$SESSION" 2>/dev/null
sleep 1

# 清理上次残留的Gazebo进程
killall -9 gzclient gzserver gazebo 2>/dev/null || true
sleep 1

echo "======================================"
echo "  无人机竞赛任务启动中..."
echo "======================================"
echo "  主控工作空间: $WS"
echo "  LiDAR工作空间: $LIO_WS"
echo "======================================"

# 关键目录存在性校验
for dir in "$WS" "$LIO_WS"; do
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
# CMD_SIM="sleep 3; \
# source '${SIM_WS}/devel/setup.${CURRENT_SHELL}'; \
# source '${PX4_PATH}/Tools/setup_gazebo.sh' '${PX4_PATH}' '${PX4_PATH}/build/px4_sitl_default'; \
# export ROS_PACKAGE_PATH=\"\$ROS_PACKAGE_PATH:${PX4_PATH}:${PX4_PATH}/Tools/sitl_gazebo\"; \
# roslaunch tutorial_gazebo sim.launch"
# tmux send-keys -t "$SESSION:0" "$CMD_SIM" C-m
CMD_UAV="sleep 3; \
roslaunch abot_bringup location_accumu.launch"
tmux send-keys -t "$SESSION:0" "$CMD_UAV" C-m

tmux split-window -v -t "$SESSION:0"
tmux send-keys -t "$SESSION:0" "sleep 4; roslaunch foxglove_bridge foxglove_bridge.launch" C-m
tmux split-window -v -t "$SESSION:0"
tmux send-keys -t "$SESSION:0" "sleep 3; roslaunch usb_cam usb_cam-test.launch" C-m

# ---------------------------------------------------------
# 窗口 1：主控、监控与下视视觉 (四等分 2x2)
# ---------------------------------------------------------
tmux new-window -t "$SESSION" -n "Control_Vision"

tmux send-keys -t "$SESSION:1" "sleep 18; rostopic echo /mavros/local_position/pose" C-m

tmux split-window -h -t "$SESSION:1"
tmux send-keys -t "$SESSION:1" "sleep 10; source '${WS}/devel/setup.${CURRENT_SHELL}'; roslaunch main_control main_control.launch" C-m

tmux split-window -v -t "$SESSION:1"
tmux send-keys -t "$SESSION:1" "sleep 16; source '${WS}/devel/setup.${CURRENT_SHELL}'; rostopic echo /yolo_down_detect" C-m

tmux select-pane -L -t "$SESSION:1"
tmux split-window -v -t "$SESSION:1"
tmux send-keys -t "$SESSION:1" "sleep 18; rostopic echo /mavros/state" C-m

tmux select-layout -t "$SESSION:1" tiled

# ---------------------------------------------------------
# 窗口 2：感知与导航 (2x2 田字格)
# ---------------------------------------------------------
tmux new-window -t "$SESSION" -n "Perception_Nav"

# 左上：FAST-LIO2 (LiDAR-IMU紧耦合SLAM)
# tmux send-keys -t "$SESSION:2" "sleep 10; source '${LIO_WS}/devel/setup.${CURRENT_SHELL}'; roslaunch fast_lio mapping_mid360_fastlio.launch rviz:=false" C-m

# 左下：PCL点云感知 (方环检测 + 障碍物处理)
# tmux split-window -v -t "$SESSION:2"
tmux send-keys -t "$SESSION:2" "sleep 14; source '${WS}/devel/setup.${CURRENT_SHELL}'; roslaunch pcl_detection2 pcl_detection2.launch" C-m

tmux select-layout -t "$SESSION:2" tiled

# ============================================
# 完成配置并附加会话
# ============================================
tmux select-window -t "$SESSION:1"
tmux attach-session -t "$SESSION"

