#pragma once

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>

// 自定义消息（需根据实际包名调整）
#include <raicom_vision_laser/DetectionInfo.h>

// ============================================================================
// 状态机枚举
// ============================================================================
enum MissionState
{
    INIT_TAKEOFF,            // 起飞
    NAV_TO_RING_FRONT,
    SETOUT_CROSS_RING,
    // NAV_TO_RECOG_AREA,       // 导航至目标识别区
    // HOVER_RECOG_TARGET,      // 悬停识别目标指示牌
    NAV_TO_DROP_AREA,        // 导航至物资投放区
    HOVER_RECOG_DROP,        // 悬停识别投放区标识

    DROP_SUPPLY,             // 投放物资箱

    MOVE_TO_ATTACK_AREA,     // 移动至攻击目标识别区
    
    RECOG_ATTACK_TARGET,     // 识别正确攻击目标
    MOVE_TO_FRONT_OF_TARGET, // 移动到目标正前方
    ALIGN_ATTACK_TARGET,     // 前视像素对准目标
    SIMULATE_ATTACK,         // 激光指示攻击
    WAIT_HIT_CONFIRMATION,   // 等待裁判确认
    NAV_TO_RING_BACK,
    RETURN_CROSS_RING,
    RETURN_LAND,             // 返回起飞点并降落
    TASK_END
};

class MissionManager
{
public:
    MissionManager(ros::NodeHandle &nh);
    void run();
    void initROSCommunication();
    void waitForConnection();

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
    ros::Time last_request_time_;
    bool init_pos_received_ = false;
    bool mission_finished_ = false;

    // ---------- 无人机状态 ----------
    mavros_msgs::State current_mav_state_;
    nav_msgs::Odometry local_odom_;
    double current_yaw_;
    double current_roll_ = 0.0;
    double current_pitch_ = 0.0;
    float init_pos_x_, init_pos_y_, init_pos_z_;
    double init_yaw_;

    // ---------- 导航状态 ----------
    int8_t nav_status_ = 0;
    bool nav_goal_sent_ = false;

    // ---------- 视觉识别数据 ----------
    std::string confirmed_target_;
    bool target_confirmed_ = false;
    bool front_camera_active_ = false;

    struct DetectionData
    {
        bool detected = false;
        float center_x = 0.0f;
        float center_y = 0.0f;
        float confidence = 0.0f;
        ros::Time last_update;
    };
    DetectionData current_detection_;

    Eigen::Vector3f attack_target_world_;
    bool hit_confirmed_ = false;

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

        bool use_ego_planner_for_drop_area;
    } cfg_;

    struct Waypoint
    {
        float x, y, z;
    };
    Waypoint wp_target_area_;
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

    // ---------- 控制辅助函数 ----------
    void sendSetpoint(const mavros_msgs::PositionTarget &sp);
    void sendEgoGoal(float x, float y, float z, float yaw = NAN);
    bool waitForNavArrival();
    void positionControl(const Eigen::Vector3f &target_pos, mavros_msgs::PositionTarget &sp);
    bool reachedTarget(const Eigen::Vector3f &target, float dist_thresh);
    bool isHoveringStable(float vert_tolerance);

    float getHorizontalSpeed() const;
    bool isDropWindowStable(float target_z) const;

    void getPixPidVel(float err_x, float err_y, float dt, float &vel_x, float &vel_y);
    float satfunc(float value, float limit);

    bool callSwitchCamera();
    bool callResetTarget();

    // ---------- 状态处理函数 ----------
            void            handleInitTakeoff();   
               void    handleNavToRingFront();     
                  void handleSetoutCrossRing();    
     void              handleNavToRecogArea();     
        void            handleHoverRecognizeDrop();
           void              handleDropSupply();   
              void   handleMoveToAttackArea();     
void                 handleRecognizeAttackTarget();
   void          handleMoveToFrontOfTarget();    
      void           handleAlignAttackTarget();    
         void            handleSimulateAttack();   
            void   handleWaitHitConfirmation();    
               void     handleNavToRingBack();     
                  void handleReturnCrossRing();    
                     void    handleReturnLand();   
                        void    handleTaskEnd();   
};