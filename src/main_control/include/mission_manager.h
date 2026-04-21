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
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <limits>

// 自定义消息（需根据实际包名调整）
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
    RETURN_CROSS_RING,

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
    ros::Publisher drop_trigger_pub_;
    ros::Publisher laser_trigger_pub_;
    ros::Subscriber state_sub_, odom_sub_;
    ros::Subscriber nav_status_sub_;
    ros::Subscriber detected_target_sub_;
    ros::Subscriber yolo_detect_sub_;
    ros::Subscriber hit_confirm_sub_;

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
    int8_t nav_status_  = 0;
    bool nav_goal_sent_ = false;


    // ---------- 视觉识别数据 ----------
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
    DetectionData current_detection_;

    Eigen::Vector3f attack_target_world_;
    bool hit_confirmed_ = false;

    // -----pcl识别环
    bool ensure_ring_   = false;
    DetectionData ring_detection;

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
        float target_front_offset;
        float nav_goal_timeout;
        float align_pixel_threshold;
        float shoot_delay;

        float PIX_VEL_P, PIX_VEL_I, PIX_VEL_D;
        float PIX_VEL_MAX;
        float PIX_INTEGRAL_MAX;
        float PIX_FAR_NORM_DIST;

        float detection_min_confidence;

        float drop_arrive_threshold;
        float drop_detect_timeout;
        float drop_align_hold_time;
        float drop_release_max_horiz_speed;
        float drop_release_max_vert_speed;
        float drop_max_tilt;
        float drop_camera_bias_x_px;
        float drop_camera_bias_y_px;
        float drop_release_bias_x_px;
        float drop_release_bias_y_px;
        float drop_fine_pixel_radius;
        float drop_fine_vel_scale;
        float drop_descend_distance;

        float land_kp                    = 0.005f;   // 水平对准 P 增益
        float land_ki                    = 0.0005f;  // 水平对准 I 增益
        float land_kd                    = 0.0001f;  // 水平对准 D 增益
        float land_max_align_speed       = 0.5f;     // 最大对准速度 (m/s)
        float land_descend_speed         = 0.3f;     // 基础下降速度 (m/s)
        float land_align_pixel_threshold = 15.0f;    // 对准像素阈值
        float land_fine_pixel_radius     = 30.0f;    // 精细调整半径
        float land_fine_vel_scale        = 0.4f;     // 精细速度缩放
        float land_final_height          = 0.2f;     // 最终判定高度 (m)
        float land_final_hold_time       = 1.5f;     // 最终稳定保持时间 (s)

        bool use_ego_planner_for_drop_area;
    } cfg_;

    Waypoint wp_ring_front_;  // 出发穿环前悬停点
    Waypoint wp_ring_back_;   // 返回穿环前悬停点
    Waypoint wp_drop_area_;
    Waypoint wp_attack_area_;

    const float IMG_CENTER_X = 320.0f;
    const float IMG_CENTER_Y = 240.0f;

    // ---------- 初始化函数 ----------
    void loadParameters();

    // ---------- 回调函数 ----------
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void navStatusCallback(const std_msgs::Int8::ConstPtr &msg);
    void detectedTargetCallback(const std_msgs::String::ConstPtr &msg);
    void yoloDetectCallback(const raicom_vision_laser::DetectionInfo::ConstPtr &msg);
    void hitConfirmCallback(const std_msgs::Bool::ConstPtr &msg);
    // void ringDetectCallback(const pcl_detection2::RingDetectionInfo::ConstPtr &msg);

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
    float getHorizontalSpeed() const;
    bool isDropWindowStable(float target_z) const;

    void getPixPidVel(float err_x, float err_y, float dt, float &vel_x, float &vel_y);
    float satfunc(float value, float limit);

    bool callSwitchCamera();
    bool callResetTarget();

    bool timeout(const float timeout_limit) const noexcept;

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
    void handleReturnCrossRing();
    void handleReturn();
    void handleLand();
    void handleTaskEnd();
};