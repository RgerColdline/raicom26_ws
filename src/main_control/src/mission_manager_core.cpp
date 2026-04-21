#include "mission_manager.h"

MissionManager::MissionManager(ros::NodeHandle &nh) : nh_(nh), current_state_(INIT_TAKEOFF) {
    loadParameters();
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b100111000111;
    current_setpoint_.velocity.x = 0.0f;
    current_setpoint_.velocity.y = 0.0f;
    current_setpoint_.velocity.z = 0.0f;
    current_setpoint_.yaw = 0.0f;
}

void MissionManager::loadParameters() {
    nh_.param<float>("takeoff_height", cfg_.takeoff_height, 1.2f);
    nh_.param<float>("max_speed", cfg_.max_speed, 0.8f);
    nh_.param<float>("max_yaw_rate", cfg_.max_yaw_rate, 0.8f);
    nh_.param<float>("err_max", cfg_.err_max, 0.25f);
    nh_.param<float>("hover_vert_tolerance", cfg_.hover_vert_tolerance, 0.03f);
    nh_.param<float>("p_xy", cfg_.p_xy, 0.4f);
    nh_.param<float>("p_z", cfg_.p_z, 0.3f);
    nh_.param<float>("hover_time_needed", cfg_.hover_time_needed, 3.0f);
    nh_.param<float>("target_front_offset", cfg_.target_front_offset, 1.0f);
    nh_.param<float>("nav_goal_timeout", cfg_.nav_goal_timeout, 60.0f);
    nh_.param<float>("align_pixel_threshold", cfg_.align_pixel_threshold, 15.0f);
    nh_.param<float>("shoot_delay", cfg_.shoot_delay, 2.0f);

    nh_.param<float>("PIX_VEL_P", cfg_.PIX_VEL_P, 0.003f);
    nh_.param<float>("PIX_VEL_I", cfg_.PIX_VEL_I, 0.0001f);
    nh_.param<float>("PIX_VEL_D", cfg_.PIX_VEL_D, 0.001f);
    nh_.param<float>("PIX_VEL_MAX", cfg_.PIX_VEL_MAX, 0.4f);
    nh_.param<float>("PIX_FAR_NORM_DIST", cfg_.PIX_FAR_NORM_DIST, 150.0f);
    nh_.param<float>("PIX_INTEGRAL_MAX", cfg_.PIX_INTEGRAL_MAX, 100.0f); // 补充缺失参数

    nh_.param<float>("wp_target_area_x", wp_target_area_.x, 6.0f);
    nh_.param<float>("wp_target_area_y", wp_target_area_.y, 0.0f);
    nh_.param<float>("wp_target_area_z", wp_target_area_.z, cfg_.takeoff_height);
    nh_.param<float>("wp_drop_area_x", wp_drop_area_.x, 10.0f);
    nh_.param<float>("wp_drop_area_y", wp_drop_area_.y, 0.0f);
    nh_.param<float>("wp_drop_area_z", wp_drop_area_.z, cfg_.takeoff_height);
    nh_.param<float>("wp_attack_area_x", wp_attack_area_.x, 12.0f);
    nh_.param<float>("wp_attack_area_y", wp_attack_area_.y, 0.0f);
    nh_.param<float>("wp_attack_area_z", wp_attack_area_.z, cfg_.takeoff_height);

    nh_.param<float>("drop_arrive_threshold", cfg_.drop_arrive_threshold, 0.35f);
    nh_.param<float>("drop_detect_timeout", cfg_.drop_detect_timeout, 0.30f);
    nh_.param<float>("drop_align_hold_time", cfg_.drop_align_hold_time, 0.35f);
    nh_.param<float>("drop_release_max_horiz_speed", cfg_.drop_release_max_horiz_speed, 0.12f);
    nh_.param<float>("drop_release_max_vert_speed", cfg_.drop_release_max_vert_speed, 0.06f);
    nh_.param<float>("drop_max_tilt", cfg_.drop_max_tilt, 0.08f);
    nh_.param<float>("drop_camera_bias_x_px", cfg_.drop_camera_bias_x_px, 0.0f);
    nh_.param<float>("drop_camera_bias_y_px", cfg_.drop_camera_bias_y_px, 0.0f);
    nh_.param<float>("drop_release_bias_x_px", cfg_.drop_release_bias_x_px, 0.0f);
    nh_.param<float>("drop_release_bias_y_px", cfg_.drop_release_bias_y_px, 0.0f);
    nh_.param<float>("drop_fine_pixel_radius", cfg_.drop_fine_pixel_radius, 35.0f);
    nh_.param<float>("drop_fine_vel_scale", cfg_.drop_fine_vel_scale, 0.45f);
    nh_.param<float>("drop_descend_distance", cfg_.drop_descend_distance, 0.0f);
    nh_.param<bool>("use_ego_planner_for_drop_area", cfg_.use_ego_planner_for_drop_area, true);

    ROS_INFO("参数加载完成。");
}

void MissionManager::initROSCommunication() {
    setpoint_pub_      = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ego_goal_pub_      = nh_.advertise<geometry_msgs::PoseStamped>("/fsm/ego_goal", 1);
    drop_trigger_pub_  = nh_.advertise<std_msgs::Bool>("/uav/drop_trigger", 1);
    laser_trigger_pub_ = nh_.advertise<std_msgs::Bool>("/uav/laser_trigger", 1);

    state_sub_         = nh_.subscribe("/mavros/state", 10, &MissionManager::stateCallback, this);
    odom_sub_          = nh_.subscribe("/mavros/local_position/odom", 10, &MissionManager::odomCallback, this);
    nav_status_sub_    = nh_.subscribe("/ego_controller/status", 10, &MissionManager::navStatusCallback, this);
    detected_target_sub_ = nh_.subscribe("/detected_target", 10, &MissionManager::detectedTargetCallback, this);
    yolo_detect_sub_   = nh_.subscribe("/ocr_detect", 10, &MissionManager::yoloDetectCallback, this);
    hit_confirm_sub_   = nh_.subscribe("/referee/hit_confirmed", 10, &MissionManager::hitConfirmCallback, this);

    switch_camera_client_ = nh_.serviceClient<std_srvs::Empty>("/switch_camera");
    reset_target_client_  = nh_.serviceClient<std_srvs::Empty>("/reset_target");
    get_status_client_    = nh_.serviceClient<std_srvs::Empty>("/get_system_status");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client_   = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
}

void MissionManager::waitForConnection() {
    ros::Rate rate(20);
    while (ros::ok() && !current_mav_state_.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("MAVROS已连接");

    while (ros::ok() && !init_pos_received_) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("初始位姿已接收");

    ROS_INFO("等待YOLO服务...");
    switch_camera_client_.waitForExistence();
    reset_target_client_.waitForExistence();
    ROS_INFO("YOLO服务已就绪");
}

// --- 控制辅助函数实现 ---

void MissionManager::sendSetpoint(const mavros_msgs::PositionTarget &sp) {
    setpoint_pub_.publish(sp);
}

void MissionManager::sendEgoGoal(float x, float y, float z, float yaw) {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "world";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;
    if (!std::isnan(yaw)) {
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
        goal.pose.orientation.x = q.x();
        goal.pose.orientation.y = q.y();
        goal.pose.orientation.z = q.z();
        goal.pose.orientation.w = q.w();
    } else {
        goal.pose.orientation.w = 1.0;
    }
    ego_goal_pub_.publish(goal);
    nav_goal_sent_ = true;
    ROS_INFO("导航目标点: (%.2f, %.2f, %.2f)", x, y, z);
}

bool MissionManager::waitForNavArrival() {
    if ((ros::Time::now() - state_start_time_).toSec() > cfg_.nav_goal_timeout) {
        ROS_WARN("等待导航到达超时！");
        current_state_ = TASK_END;
        return false;
    }
    return (nav_status_ == 2);
}

void MissionManager::positionControl(const Eigen::Vector3f &target_pos, mavros_msgs::PositionTarget &sp) {
    Eigen::Vector3f err = target_pos - Eigen::Vector3f(local_odom_.pose.pose.position.x,
                                                        local_odom_.pose.pose.position.y,
                                                        local_odom_.pose.pose.position.z);
    float vx = err.x() * cfg_.p_xy;
    float vy = err.y() * cfg_.p_xy;
    float vz = err.z() * cfg_.p_z;
    vx = std::clamp(vx, -cfg_.max_speed, cfg_.max_speed);
    vy = std::clamp(vy, -cfg_.max_speed, cfg_.max_speed);
    vz = std::clamp(vz, -cfg_.max_speed, cfg_.max_speed);

    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    sp.type_mask = 0b100111000011; 
    sp.velocity.x = vx;
    sp.velocity.y = vy;
    sp.velocity.z = vz;
    sp.yaw = current_yaw_;
}

bool MissionManager::reachedTarget(const Eigen::Vector3f &target, float dist_thresh) {
    float dx = target.x() - local_odom_.pose.pose.position.x;
    float dy = target.y() - local_odom_.pose.pose.position.y;
    float dz = target.z() - local_odom_.pose.pose.position.z;
    return (dx*dx + dy*dy + dz*dz) < (dist_thresh * dist_thresh);
}

bool MissionManager::isHoveringStable(float vert_tolerance) {
    if (std::abs(local_odom_.twist.twist.linear.z) > 0.15) return false;
    if (std::abs(current_roll_) > 0.15 || std::abs(current_pitch_) > 0.15) return false;
    return true;
}

float MissionManager::getHorizontalSpeed() const {
    return std::hypot(local_odom_.twist.twist.linear.x, local_odom_.twist.twist.linear.y);
}

bool MissionManager::isDropWindowStable(float target_z) const {
    return getHorizontalSpeed() < cfg_.drop_release_max_horiz_speed &&
           std::abs(local_odom_.twist.twist.linear.z) < cfg_.drop_release_max_vert_speed &&
           std::abs(local_odom_.pose.pose.position.z - target_z) < cfg_.hover_vert_tolerance &&
           std::abs(current_roll_) < cfg_.drop_max_tilt &&
           std::abs(current_pitch_) < cfg_.drop_max_tilt;
}

float MissionManager::satfunc(float value, float limit) {
    return std::clamp(value, -limit, limit);
}

void MissionManager::getPixPidVel(float err_x, float err_y, float dt, float &vel_x, float &vel_y) {
    dt = std::clamp(dt, 0.02f, 1.0f);

    float norm_factor = 1.0f;
    float pixel_err = std::sqrt(err_x * err_x + err_y * err_y);
    if (pixel_err > cfg_.PIX_FAR_NORM_DIST) {
        norm_factor = cfg_.PIX_FAR_NORM_DIST / pixel_err;
    }
    float norm_err_x = err_x * norm_factor;
    float norm_err_y = err_y * norm_factor;

    if (last_pix_err_x_ == 0.0f && last_pix_err_y_ == 0.0f) {
        last_pix_err_x_ = err_x;
        last_pix_err_y_ = err_y;
    }

    if (pixel_err < 80.0f) {
        pix_integral_x_ += norm_err_x * dt;
        pix_integral_y_ += norm_err_y * dt;
    } else {
        pix_integral_x_ = 0.0f;
        pix_integral_y_ = 0.0f;
    }

    pix_integral_x_ = std::clamp(pix_integral_x_, -cfg_.PIX_INTEGRAL_MAX, cfg_.PIX_INTEGRAL_MAX);
    pix_integral_y_ = std::clamp(pix_integral_y_, -cfg_.PIX_INTEGRAL_MAX, cfg_.PIX_INTEGRAL_MAX);

    float deriv_x = (norm_err_x - last_pix_err_x_) / dt;
    float deriv_y = (norm_err_y - last_pix_err_y_) / dt;

    float pid_x = cfg_.PIX_VEL_P * norm_err_x + cfg_.PIX_VEL_I * pix_integral_x_ + cfg_.PIX_VEL_D * deriv_x;
    float pid_y = cfg_.PIX_VEL_P * norm_err_y + cfg_.PIX_VEL_I * pix_integral_y_ + cfg_.PIX_VEL_D * deriv_y;

    vel_x = satfunc(pid_x, cfg_.PIX_VEL_MAX);
    vel_y = satfunc(pid_y, cfg_.PIX_VEL_MAX);

    last_pix_err_x_ = norm_err_x;
    last_pix_err_y_ = norm_err_y;
}

bool MissionManager::callSwitchCamera() {
    std_srvs::Empty srv;
    if (switch_camera_client_.call(srv)) {
        front_camera_active_ = !front_camera_active_;
        ROS_INFO("摄像头切换成功，当前模式: %s", front_camera_active_ ? "前视" : "下视");
        return true;
    }
    ROS_WARN("切换摄像头服务调用失败");
    return false;
}

bool MissionManager::callResetTarget() {
    std_srvs::Empty srv;
    if (reset_target_client_.call(srv)) {
        target_confirmed_ = false;
        confirmed_target_.clear();
        current_detection_.detected = false;
        ROS_INFO("目标记忆已重置");
        return true;
    }
    ROS_WARN("重置目标服务调用失败");
    return false;
}

// --- 主循环 ---

void MissionManager::run() {
    ros::Rate rate(20);
    while (ros::ok() && !mission_finished_) {
        // 1. 默认安全设定点
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask = 0b100111000111;
        current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
        current_setpoint_.yaw = current_yaw_;

        // 2. 执行状态逻辑
        switch (current_state_) {
            case INIT_TAKEOFF:            handleInitTakeoff();            break;
            case NAV_TO_RING_FRONT:       handleNavToRingFront();         break;
            case SETOUT_CROSS_RING:       handleSetoutCrossRing();              break;
            case NAV_TO_DROP_AREA:       handleNavToRecogArea();         break;
            case HOVER_RECOG_DROP:        handleHoverRecognizeDrop();     break;
            case DROP_SUPPLY:             handleDropSupply();             break;
            case MOVE_TO_ATTACK_AREA:     handleMoveToAttackArea();       break;
            case RECOG_ATTACK_TARGET:     handleRecognizeAttackTarget();  break;
            case MOVE_TO_FRONT_OF_TARGET: handleMoveToFrontOfTarget();    break;
            case ALIGN_ATTACK_TARGET:     handleAlignAttackTarget();      break;
            case SIMULATE_ATTACK:         handleSimulateAttack();         break;
            case WAIT_HIT_CONFIRMATION:   handleWaitHitConfirmation();    break;
            case NAV_TO_RING_BACK:        handleNavToRingBack();         break;
            case RETURN_CROSS_RING:       handleReturnCrossRing();              break;
            case RETURN_LAND:             handleReturnLand();             break;
            case TASK_END:                handleTaskEnd();                break;
            default: break;
        }

        // 3. 发布设定点
        sendSetpoint(current_setpoint_);

        ros::spinOnce();
        rate.sleep();
    }
}