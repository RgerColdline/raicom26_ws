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

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <limits>

// 自定义消息（需根据实际包名调整）
#include <pcl_detection2/SquareRing.h>
#include <raicom_vision_laser/DetectionInfo.h>

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

    // === 柱子检测导航（PCL模式） ===
    PILLAR_DETECT,              // 悬停等待PCL检测柱子配置
    NAV_PILLAR_WAYPOINTS,       // 按顺序飞航点（正向）
    RETURN_PILLAR_WAYPOINTS,    // 反向飞航点（返程）

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

    // === 柱子检测导航 ===
    std::string pillar_nav_mode_;                               // "ego" 或 "pcl"
    int pillar_case_id_                  = -1;                  // 检测到的配置ID (0-3), -1=未检测
    std::vector<std::vector<Waypoint>> pillar_waypoints_;      // 4种配置的航点序列
    size_t pillar_wp_index_              = 0;                   // 当前航点索引
    size_t pillar_wp_total_              = 0;                   // 当前配置总航点数
    float pillar_detect_timeout_         = 5.0f;                // 检测超时
    float pillar_waypoint_tolerance_     = 0.3f;                // 航点到达判定距离

    void loadPillarConfig();                                    // 加载柱子导航YAML
    void handlePillarDetect();                                  // PILLAR_DETECT 状态
    void handleNavPillarWaypoints();                            // 正向飞航点
    void handleReturnPillarWaypoints();                         // 反向飞航点

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
        std::string attack_real_target;  // 真实目标字母 "A" 或 "B"，前视只攻击此目标
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

        // 环多假设追踪参数（类 PCL 强度）
        float track_match_distance       = 0.3f;   // 匹配距离阈值 (m)
        float track_confidence_boost     = 0.15f;  // 每次匹配成功增量
        float track_confidence_decay     = 0.97f;  // 每帧衰减系数 (20Hz)
        float track_confirm_threshold    = 0.7f;   // 锁定确认阈值
        float track_min_confidence       = 0.08f;  // 最低保留置信度
        int track_max_candidates         = 3;      // 最大候选数
        float track_ema_alpha            = 0.3f;   // EMA 平滑系数
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