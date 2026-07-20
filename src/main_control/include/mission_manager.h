#pragma once

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <limits>

// 自定义消息（需根据实际包名调整）
#include <pcl_detection2/SquareRing.h>
#include <raicom_vision_laser/DetectionInfo.h>

// 穿越赛段平滑轨迹规划器（从 raicom_vision_laser 拷入，纯计算无 ROS 依赖）
#include "traverse_planner.h"

// ==================== case -> 候选柱索引映射（全队统一，勿单独改） ====================
// 候选索引（map/pillar_candidates 顺序固定）：
//   0=A左(2.7,1.55)  1=A右(3.3,1.55)  2=B左(2.7,2.8)  3=B右(3.3,2.8)
// 与 traverse_map.yaml、pcl_detection2 模板 pillar_case_0X.png 保持一致
inline const int TRAV_CASE_PILLARS[4][2] = {{0, 2}, {0, 3}, {1, 2}, {1, 3}};
inline const char* TRAV_CASE_DESC[4] = {
    "A左+B左（两柱都在 x=2.7）",
    "A左+B右（(2.7,1.55)+(3.3,2.8)）",
    "A右+B左（(3.3,1.55)+(2.7,2.8)）",
    "A右+B右（两柱都在 x=3.3）"
};

// 穿越段轨迹跟踪用的位置控制掩码：忽略 vx/vy/vz/afx/afy/afz + IGNORE_YAW_RATE=2048
// （与 traverse_node 一致，2026-07-19 仿真新规，不带 512）
inline const uint16_t TRAV_TYPE_MASK_POSITION_ONLY = 8 + 16 + 32 + 64 + 128 + 256 + 2048;

// ============================================================================
// 状态机枚举
// ============================================================================
enum MissionState {
    INIT_TAKEOFF,  // 起飞

    MOVE_TO_RING_FRONT,
    SETOUT_CROSS_RING,

    NAV_TO_DROP_AREA,         // 导航至物资投放区
    HOVER_RECOG_DROP,         // 悬停识别投放区标识
    DROP_SUPPLY,              // 投放物资箱
    MOVE_TO_ATTACK_AREA,      // 移动至攻击目标识别区
    RECOG_ATTACK_TARGET,      // 识别正确攻击目标
    MOVE_TO_FRONT_OF_TARGET,  // 移动到目标正前方
    ALIGN_ATTACK_TARGET,      // 前视像素对准目标
    SIMULATE_ATTACK,          // 激光指示攻击
    WAIT_HIT_CONFIRMATION,    // 等待裁判确认

    NAV_TO_RING_BACK,
    READY_NAV_TO_RING_BACK,       // 返程前先飞到来时的目标点，再开始返程导航
    RETURN_CROSS_RING,

    // === 穿越赛段（traverse 融合：pillar_nav_mode="pcl" 时替换 EGO 路径） ===
    TRAVERSE_TO_SCAN,       // 穿环后飞到悬停扫描点
    TRAVERSE_SCAN,          // 悬停扫描选 case + 现场规划 leg2
    TRAVERSE_LEG2,          // leg2 绕柱段轨迹跟踪（悬停点 -> 投放区）
    TRAVERSE_READY_RETURN,  // 射击后先回投放区（轨迹终点），再开始倒放
    TRAVERSE_RETURN_LEG2,   // leg2 时间倒放原路返回（投放区 -> 悬停扫描点）

    RETURN,
    LAND,
    TASK_END
};

class MissionManager
{
  public:
    MissionManager(ros::NodeHandle &nh);
    void run();
    void initROSCommunication();
    void waitForConnection();

    struct Waypoint
    {
        float x, y, z;
        Waypoint(float x = 0.0f, float y = 0.0f, float z = 0.0f) : x(x), y(y), z(z) {}
    };

  private:
    // ---------- ROS 通信 ----------
    ros::NodeHandle nh_;
    ros::Publisher setpoint_pub_;
    ros::Publisher ego_goal_pub_;
    ros::Publisher servo_control_pub_;  // 舵机投放，/servo_control (UInt8)
    ros::Publisher shoot_pub_;
    ros::Publisher laser_control_pub_;
    ros::Subscriber state_sub_, odom_sub_;
    ros::Subscriber nav_status_sub_;
    ros::Subscriber detected_target_sub_;
    ros::Subscriber yolo_detect_sub_;
    ros::Subscriber yolo_down_detect_sub_;  // 下视 YOLO（投货悬停字母投票）
    ros::Subscriber hit_confirm_sub_;
    ros::Subscriber ring_sub_;
    ros::Subscriber pillar_sub_;
    ros::Publisher pillar_start_pub_;

    ros::ServiceClient switch_camera_client_;
    ros::ServiceClient reset_target_client_;
    ros::ServiceClient get_status_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;

    // ---------- 状态机数据 ----------
    MissionState current_state_;
    ros::Time state_start_time_;
    bool init_pos_received_ = false;
    bool mission_finished_  = false;

    // ---------- 无人机状态 ----------
    mavros_msgs::State current_mav_state_;
    nav_msgs::Odometry local_odom_;
    double current_yaw_;
    double current_roll_  = 0.0;
    double current_pitch_ = 0.0;
    float init_pos_x_, init_pos_y_, init_pos_z_;
    double init_yaw_;

    // ---------- 导航状态 ----------
    int8_t nav_status_            = 0;
    bool nav_goal_sent_           = false;
    bool nav_seen_executing_      = false;  // 防止残留status=2被误判


    // ---------- 视觉识别数据 ----------
    std::string target_class_name = "";
    std::string confirmed_target_;
    bool target_confirmed_    = false;
    bool front_camera_active_ = false;

    struct DetectionData
    {
        bool detected    = false;
        float center_x   = 0.0f;
        float center_y   = 0.0f;
        float confidence = 0.0f;
        ros::Time last_update;
    };

    Eigen::Vector3f attack_target_world_;
    bool hit_confirmed_ = false;

    // 定点射击目标（vision_laser 状态6）
    // 注意：以下两个变量存【相对init_pos的偏移】(=cfg.shoot_*_x/y)，传给 moveTo() 由其内部加 init_pos
    float shoot_target_x_ = 0.0f;
    float shoot_target_y_ = 0.0f;

    // === mission_flow 融合：前视YOLO检测状态 ===
    bool        front_target_matched_ = false;
    std::string matched_target_;
    float       matched_center_x_     = 0.0f;
    float       matched_center_y_     = 0.0f;
    ros::Time   last_matched_time_;

    // === 下视字母识别（投货悬停投票，2026-07-20 固定映射射击流程） ===
    // 投货悬停时对下视检测到的字母 A/B 投票（只会是 A 或 B），悬停结束按多数票
    // 定 shoot_letter_；射击按固定映射飞对应射击点：左=A 靶、右=B 靶（a_side 可配）。
    std::string shoot_letter_ = "A";   // 射击字母：悬停结束时投票决定，初始=兜底字母
    bool        down_voting_  = false; // true=正在投票窗口（HOVER_RECOG_DROP 期间）
    int         down_vote_a_  = 0;
    int         down_vote_b_  = 0;
    bool        a_on_left_    = true;  // 由 cfg_.shoot_a_side 换算

    // === mission_flow 融合：投货子状态（对应 mission_flow 状态3 Sub1/Sub2） ===
    int       drop_sub_state_   = 0;  // 1=下降到投货高度 2=保持重发投货指令
    ros::Time last_drop_pub_time_;
    ros::Time drop_hover_start_;

    // === mission_flow 融合：射击状态 ===
    bool      shoot_triggered_ = false;
    ros::Time shoot_time_;

    // -----pcl识别环
    bool ensure_ring_   = false;
    DetectionData ring_detection;

    // 记住第一次穿环后的位置（世界坐标），返程导航直接用
    Eigen::Vector3f ring_back_memorized_ = Eigen::Vector3f::Zero();
    bool ring_back_memorized_valid_ = false;

    // === 多假设环追踪（类 PCL 点强度机制） ===
    struct RingCandidate
    {
        Eigen::Vector3f center   = Eigen::Vector3f::Zero();
        Eigen::Vector3f front_pt = Eigen::Vector3f::Zero();
        Eigen::Vector3f back_pt  = Eigen::Vector3f::Zero();
        float confidence         = 0.0f;
        ros::Time last_update;
        bool locked = false;
    };
    std::vector<RingCandidate> ring_candidates_;

    // 当前激活的环位姿（最高置信度候选，世界坐标系）
    Eigen::Vector3f ring_center_       = Eigen::Vector3f::Zero();
    Eigen::Vector3f ring_front_pt_     = Eigen::Vector3f::Zero();
    Eigen::Vector3f ring_back_pt_      = Eigen::Vector3f::Zero();
    bool ring_pose_valid_              = false;

    // 全局环置信度累加器（只增不减，不随候选清除而重置）
    float ring_accumulated_confidence_ = 0.0f;

    // 环追踪更新（回调触发匹配/置信度提升 + 主循环触发衰减/清理）
    void updateRingTracking(const pcl_detection2::SquareRing::ConstPtr &msg);
    void decayRingCandidates();

    // 根据检测位姿动态计算穿越点，失败返回 false（用硬编码回退）
    bool computeRingApproachWP(Waypoint &front_wp, Waypoint &back_wp) const;

    // === 穿越赛段（traverse_node 融合：悬停扫描选 case + leg2 平滑轨迹 + 时间倒放返程） ===
    std::string pillar_nav_mode_;       // "ego"=EGO路径  "pcl"=穿越平滑轨迹（沿用旧参数名）
    int detected_case_   = -1;          // pcl_detection2 检测结果(0~3)，-1=未收到
    int active_case_     = -1;          // 实际采用的 case（规划成功后锁定）
    int scan_sub_state_  = 0;           // TRAVERSE_SCAN 子状态：0入口 1等结果 9失败悬停
    ros::Time scan_entry_time_;         // 进入扫描子状态的时刻
    ros::Time leg_start_time_;          // 当前航段轨迹跟踪的起始时刻
    bool traverse_cfg_ok_ = false;      // 地图读取+启动预检是否通过（不通过回退 EGO 路径）

    TraversePlanner planner_leg2_;      // leg2 绕柱段（悬停扫描拿到 case 后现场规划）
    double hover_ox_ = 0, hover_oy_ = 0; // 悬停扫描点（odom 系绝对坐标）
    double end_x_ = 0, end_y_ = 0;      // 轨迹终点=投放区（odom 系绝对坐标）

    // 地图数据（yaml 里全是场地系官方坐标，规划器内部转 odom）
    double origin_fx_ = 0.65, origin_fy_ = 0.75;  // 出生点（场地系）= odom 原点
    double scan_hover_fx_ = 3.00, scan_hover_fy_ = 0.75;  // 悬停扫描点（场地系）
    double pillar_radius_ = 0.1;                  // 圆柱实际半径
    std::vector<Vec2f> pillar_cand_;              // 4 个候选柱位（索引见 TRAV_CASE_PILLARS）
    std::vector<Vec2f> via_leg2_[4];              // 4 套 leg2 途经点（按 case）
    std::vector<SegObs> walls_;                   // 线段障碍（墙 + 场地边界）

    void loadTraverseConfig();                      // 读 traverse_map.yaml + 4 case 启动预检
    std::vector<CircleObs> caseCircles(int cid) const; // 组装某 case 的两根圆柱障碍
    bool planLeg2ForCase(int cid);                  // 按 case 规划 leg2，返回净距是否达标
    bool tryPlanLeg2(int cid);                      // 规划入口（不达标回退 default_case/force_fly）
    void printLeg2Report(int cid) const;            // leg2 规划报告打印
    bool trackLeg(bool reverse, double goal_x, double goal_y, const char* label); // 轨迹跟踪
    bool moveToAbs(double x, double y, double z);   // 绝对 odom 坐标定点（不走 init_pos 偏移）

    void handleTraverseToScan();                    // TRAVERSE_TO_SCAN 状态
    void handleTraverseScan();                      // TRAVERSE_SCAN 状态
    void handleTraverseLeg2();                      // TRAVERSE_LEG2 状态
    void handleTraverseReadyReturn();               // TRAVERSE_READY_RETURN 状态
    void handleTraverseReturnLeg2();                // TRAVERSE_RETURN_LEG2 状态

    // PID控制相关
    ros::Time last_pid_control_time_;
    float pix_integral_x_ = 0.0f;
    float pix_integral_y_ = 0.0f;
    float last_pix_err_x_ = 0.0f;
    float last_pix_err_y_ = 0.0f;

    ros::Time drop_alignment_hold_start_;

    mavros_msgs::PositionTarget current_setpoint_;

    // ---------- 参数配置 ----------
    struct Config
    {
        float takeoff_height;
        float max_speed;
        float max_yaw_rate;
        float err_max;
        float hover_vert_tolerance;
        float p_xy, p_z;
        float hover_time_needed;
        float target_front_offset_x;
        float target_front_offset_y;
        float nav_goal_timeout;
        float align_pixel_threshold;
        float shoot_delay;

        float PIX_VEL_P, PIX_VEL_I, PIX_VEL_D;
        float PIX_VEL_MAX;
        float PIX_INTEGRAL_MAX;
        float PIX_FAR_NORM_DIST;

        float detection_min_confidence;
        std::string attack_real_target;  // 兜底字母 "A" 或 "B"：投货悬停下视识别失败时按此字母射击
        bool wait_for_vision_services = true;  // 是否等待视觉服务(/switch_camera等)；false跳过，方便单独测试PCL

        float drop_arrive_threshold;
        float drop_detect_timeout;
        float drop_align_hold_time;
        float drop_camera_bias_x_px;
        float drop_camera_bias_y_px;
        float drop_release_bias_x_px;
        float drop_release_bias_y_px;
        float drop_fine_pixel_radius;
        float drop_fine_vel_scale;
        float drop_descend_distance;

        float land_descend_speed         = 0.3f;     // 定点下降速度 (m/s)

        bool use_ego_planner_for_drop_area;

        // 环穿越动态参数
        float ring_front_approach_offset = 0.8f;  // 环前方悬停距离 (m)
        float ring_back_approach_offset  = 2.5f;  // 环后方穿越目标距离 (m)

        // 定点射击参数（vision_laser 状态6，抛弃视觉PID，使用位置判断+定点射击）
        float shoot_left_x      = -0.60f;   // 左射击点X（相对attack_area）
        float shoot_left_y      = -2.5f;    // 左射击点Y
        float shoot_right_x     = -0.60f;   // 右射击点X
        float shoot_right_y     = -1.8f;    // 右射击点Y
        float shoot_default_x   = -0.60f;   // 默认射击点X
        float shoot_default_y   = -2.15f;   // 默认射击点Y
        float shoot_left_right_threshold = 20.0f;  // 左右判断像素阈值
        float shoot_detect_timeout       = 60.0f;  // 目标检测超时 (s)
        float shoot_stable_time          = 0.5f;   // 射击前稳定时间 (s)
        float shoot_duration             = 1.5f;   // 激光射击后等待时间(s)（stm32_shooter 开->1s->关 + 余量）
        float shoot_z                    = 1.5f;   // 射击高度(相对init_pos)

        // === mission_flow 融合：投货参数（/servo_control -> stm32_shooter） ===
        // 固件: 0°=货舱开(投货), 160°=货舱关(复位); stm32_shooter: 发<135->0x03(开), 发>=135->0x04(关)
        int   cargo_drop_angle           = 0;      // 投货(开舱): 发0(<135) -> 0x03
        int   cargo_reset_angle          = 180;    // 复位(关舱): 发180(>=135) -> 0x04
        float cargo_hold_time            = 4.0f;   // 货舱保持打开时间(s)
        float descent_timeout            = 10.0f;  // 下降到投货高度超时(s); 超时未到也在当前位置投货
        float drop_hover_time            = 2.0f;   // 投货前在投放点悬停时间(s)
        float drop_z                     = 0.3f;   // 投货下降高度(相对init_pos)

        // === mission_flow 融合：前视YOLO识别参数（/yolo_front_detect, 320x240） ===
        float yolo_img_center_x          = 160.0f; // 前视YOLO图像中心X(图像320x240)
        float yolo_target_timeout        = 0.5f;   // 检测结果有效期(s), 超过算目标丢失
        float yolo_detect_timeout        = 60.0f;  // 前视识别总超时(s), 超时走默认射击点

        // === 下视字母识别 + 固定映射射击（2026-07-20） ===
        int   down_min_votes             = 3;      // 悬停期间某字母得票>=该值才采用，否则回退兜底字母
        std::string shoot_a_side         = "left"; // A 靶固定在哪侧射击点("left"=左A右B, "right"=镜像)

        // 环多假设追踪参数（类 PCL 强度）
        float track_match_distance       = 0.3f;   // 匹配距离阈值 (m)
        float track_confidence_boost     = 0.15f;  // 每次匹配成功增量
        float track_confidence_decay     = 0.97f;  // 每帧衰减系数 (20Hz)
        float track_confirm_threshold    = 0.7f;   // 锁定确认阈值
        float track_min_confidence       = 0.08f;  // 最低保留置信度
        int track_max_candidates         = 3;      // 最大候选数
        float track_ema_alpha            = 0.3f;   // EMA 平滑系数

        // === 穿越赛段参数（traverse_map.yaml 的 traverse/ 段，pillar_nav_mode="pcl" 时生效） ===
        float trav_flight_z        = 1.3f;   // 穿越段定高（规则锁死）
        float trav_err_max         = 0.15f;  // 轨迹端点到位容差（通道窄，比主流程紧）
        double trav_v_max          = 0.5;    // 最大线速度
        double trav_a_max          = 0.4;    // 最大线加速度
        double trav_a_lat_max      = 0.6;    // 最大向心加速度（弯道限速）
        double trav_inflation      = 0.3;    // 障碍膨胀半径（验收底线）
        double trav_sample_ds      = 0.01;   // 轨迹采样步长
        int trav_force_fly         = 0;      // 1=碰撞检测不通过也强行飞（仅调试用）
        float trav_timeout_margin  = 15.0f;  // 轨迹跟踪超时余量(s)，超时强制进下一状态
        float scan_hover_time      = 3.0f;   // 悬停扫描时长(s)：收到 case 且满该时长即走
        float scan_timeout         = 4.0f;   // 扫描超时(s)：超时未收到 case 用 default_case
        int default_case           = 1;      // 检测失败/超时回退 case
        int force_case             = -1;     // >=0：跳过检测与悬停，直接用该 case（仿真/应急）
    } cfg_;

    Waypoint wp_ring_front_;                       // 出发穿环前悬停点
    Waypoint wp_ring_back_;                        // 返回穿环前悬停点
    Waypoint wp_come_mid_;
    Waypoint wp_back_mid_;
    Waypoint wp_pillar_center_;                    // 两柱中心点
    Waypoint wp_drop_area_;
    Waypoint wp_attack_area_;

    // ---------- 初始化函数 ----------
    void loadParameters();

    // ---------- 回调函数 ----------
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void navStatusCallback(const std_msgs::Int8::ConstPtr &msg);
    void detectedTargetCallback(const std_msgs::String::ConstPtr &msg);
    void yoloDetectCallback(const raicom_vision_laser::DetectionInfo::ConstPtr &msg);
    void yoloDownDetectCallback(const raicom_vision_laser::DetectionInfo::ConstPtr &msg);
    void hitConfirmCallback(const std_msgs::Bool::ConstPtr &msg);
    void ringDetectCallback(const pcl_detection2::SquareRing::ConstPtr &msg);
    void pillarDetectCallback(const std_msgs::Int32::ConstPtr &msg);

    // ---------- 控制辅助函数 ----------
    void sendSetpoint(const mavros_msgs::PositionTarget &sp);
    void sendEgoGoal(float x, float y, float z, float yaw = NAN);
    bool waitForNavArrival();
    void positionControl(const Eigen::Vector3f &target_pos, mavros_msgs::PositionTarget &sp);
    bool reachedTarget(const Eigen::Vector3f &target, float dist_thresh);
    bool isHoveringStable(float vert_tolerance);
    bool navTo(const float x, const float y, const float z);
    inline bool navTo(const Waypoint wp) { return navTo(wp.x, wp.y, wp.z); }
    bool moveTo(const float x, const float y, const float z);
    inline bool moveTo(const Waypoint wp) { return moveTo(wp.x, wp.y, wp.z); }
    void hover();
    float getHorizontalSpeed() const;

    void getPixPidVel(float err_x, float err_y, float dt, float &vel_x, float &vel_y);
    float satfunc(float value, float limit);

    bool callSwitchCamera();
    bool callResetTarget();

    bool timeout(const float timeout_limit) const noexcept;

    // ring detection timeout - 超时未收到检测则清除
    void checkRingTimeout();

    // ---------- 状态处理函数 ----------
    void handleInitTakeoff();
    void handleMoveToRingFront();
    void handleSetoutCrossRing();
    void handleNavToDropArea();
    void handleHoverRecognizeDrop();
    void handleDropSupply();
    void handleMoveToAttackArea();
    void handleRecognizeAttackTarget();
    void handleMoveToFrontOfTarget();
    void handleAlignAttackTarget();
    void handleSimulateAttack();
    void handleWaitHitConfirmation();
    void handleNavToRingBack();
    void handleReadyNavToRingBack();
    void handleReturnCrossRing();
    void handleReturn();
    void handleLand();
    void handleTaskEnd();
};