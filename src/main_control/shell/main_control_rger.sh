#!/usr/bin/env bash
# 预期存放路径: <工作空间>/src/main_control/shell/
# 兼容 bash/zsh 执行，支持环境变量覆盖
#
# 本脚本由 main_control.sh 派生，区别仅在「窗口 0：核心与仿真」的 Gazebo 部分：
#   原脚本硬编码了 PX4 路径（$HOME/Libraries/PX4-Autopilot）、另一台机器的
#   GAZEBO_PLUGIN_PATH(/home/gutlord/...) 以及已不存在的 gazebo-classic 版
#   setup_gazebo.bash。本容器里 ~/.zshrc 已经完成全部环境 setup，故这些硬编码
#   全部删除，仿真窗口只保留 PX4_SIM_MODEL 与 roslaunch。
# 其余窗口（主控/感知/监控）与 main_control.sh 保持一致。

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
#   SIM_WS 现在只用于目录校验与提示（tutorial_gazebo 所在工作空间），
#   它的 devel/setup 已由 ~/.zshrc 加载，window 0 不再重复 source。
SIM_WS="${SIM_WS:-$HOME/catkin_ws}"
LIO_WS="${LIO_WS:-$HOME/ros_libraries_ws}"

# 注意: PX4 路径不再在此声明。
#   容器 ~/.zshrc 中已 source PX4 的 Tools/setup_gazebo.bash 并导出
#   ROS_PACKAGE_PATH / GAZEBO_MODEL_PATH / GAZEBO_PLUGIN_PATH，
#   tmux 每个 pane 都是交互式 zsh，会自动加载它们。
#   (若不依赖 ~/.zshrc，请自行 export PX4_PATH 并在下方校验循环中加入)

# 识别当前终端 Shell 类型，动态匹配 setup 脚本后缀
CURRENT_SHELL="${SHELL##*/}"
[ -z "$CURRENT_SHELL" ] && CURRENT_SHELL="bash"

SESSION="mission"
tmux kill-session -t "$SESSION" 2>/dev/null
sleep 1

# 清理上次残留的Gazebo/px4进程（不杀roscore，不影响其他ROS使用）
# 注：px4 必须清，否则残留实例会占住 PX4 instance 0，导致新 sitl 报
#    "PX4 server already running" 直接退出
killall -9 gzclient gzserver gazebo px4 2>/dev/null || true
sleep 1

echo "======================================"
echo "  无人机竞赛任务启动中..."
echo "======================================"
echo "  主控工作空间: $WS"
echo "  仿真工作空间: $SIM_WS"
echo "  LiDAR工作空间: $LIO_WS"
echo "  PX4/Gazebo 环境: 由 ~/.zshrc 提供"
echo "======================================"

# 关键目录存在性校验
for dir in "$WS" "$SIM_WS" "$LIO_WS"; do
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
# Gazebo 环境全部来自 ~/.zshrc，此处不再重复 source/setup：
#   - PX4 setup_gazebo.bash (含 ROS_PACKAGE_PATH / GAZEBO_PLUGIN_PATH / GAZEBO_MODEL_PATH)
#   - catkin_ws(devel) 与 ros_libraries_ws(devel) 的 setup
#   - Livox 插件经 LD_LIBRARY_PATH(catkin_ws/devel/lib) 由 dlopen 兜底加载
#   - gazebo-11 系统插件目录是 gazebo 编译内置搜索路径，无需导出 LD_LIBRARY_PATH
# 注意: 这里也不能再写 source '${SIM_WS}/devel/setup.${CURRENT_SHELL}'。
#   catkin 的 setup 脚本被二次 source 时会按首次记录的方式重算环境，会把
#   ~/.zshrc 追加的 PX4 条目从 ROS_PACKAGE_PATH 中抹掉（带 --extend 同样会），
#   导致 roslaunch 里的 $(find px4) 找不到包。
# 这里只做一次保险：确认 px4 包可被 rospack 找到，否则给出可读提示而不是
# roslaunch 的隐晦报错（pane 保持存活以便看到提示）。
CMD_SIM="sleep 3; \
export PX4_SIM_MODEL=iris; \
if rospack find px4 >/dev/null 2>&1; then \
    roslaunch tutorial_gazebo sim.launch; \
else \
    echo '[错误] 未找到 px4 包: 请确认容器内 ~/.zshrc 已 source PX4 的 setup_gazebo.bash'; \
fi"
tmux send-keys -t "$SESSION:0" "$CMD_SIM" C-m

# ---------------------------------------------------------
# 窗口 1：主控、监控与下视视觉 (四等分 2x2)
# 注: main_control.launch 内含 状态机 + 下视 YOLO + stm32_shooter(仿真已禁用)
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
tmux send-keys -t "$SESSION:2" "sleep 10; source '${LIO_WS}/devel/setup.${CURRENT_SHELL}'; roslaunch fast_lio mapping_mid360_fastlio.launch" C-m

# 左下：PCL点云感知 (方环检测 + 柱子检测pillar_case_id + 障碍物处理)
tmux split-window -v -t "$SESSION:2"
tmux send-keys -t "$SESSION:2" "sleep 14; source '${LIO_WS}/devel/setup.${CURRENT_SHELL}'; source '${WS}/devel/setup.${CURRENT_SHELL}' --extend; roslaunch pcl_detection2 pcl_detection2.launch" C-m

tmux select-layout -t "$SESSION:2" tiled

# ---------------------------------------------------------
# 窗口 3：PCL柱子检测监控
# ---------------------------------------------------------
tmux new-window -t "$SESSION" -n "Pillar_Monitor"

# PCL 柱子检测结果监控
tmux send-keys -t "$SESSION:3" "sleep 16; rostopic echo /pcl_detection2/pillar_case_id" C-m

# ============================================
# 完成配置并附加会话
# ============================================
tmux select-window -t "$SESSION:1"
tmux attach-session -t "$SESSION"
