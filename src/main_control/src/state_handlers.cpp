#include "mission_manager.h"

// 8.1 起飞
void MissionManager::handleInitTakeoff() {
    static int sub_state               = 0;
    static int setpoint_count          = 0;

    float target_z                     = init_pos_z_ + cfg_.takeoff_height;
    float current_z                    = local_odom_.pose.pose.position.z;

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask        = 0b101111111000;
    current_setpoint_.position.x       = init_pos_x_;
    current_setpoint_.position.y       = init_pos_y_;
    current_setpoint_.position.z       = target_z;
    current_setpoint_.yaw              = init_yaw_;

    switch (sub_state) {
    case 0:
        if (setpoint_count < 100) {
            setpoint_count++;
            if (setpoint_count == 100) {
                ROS_INFO("已发送100个设定点，准备切换OFFBOARD模式");
                sub_state         = 1;
                state_start_time_ = ros::Time::now();
            }
        }
        break;

    case 1:
        if (current_mav_state_.mode != "OFFBOARD" && timeout(3.0)) {
            mavros_msgs::SetMode srv;
            srv.request.custom_mode = "OFFBOARD";
            if (set_mode_client_.call(srv) && srv.response.mode_sent) {
                ROS_INFO("OFFBOARD模式请求成功");
                sub_state         = 2;
                state_start_time_ = ros::Time::now();
            }
            else {
                ROS_WARN("切换OFFBOARD失败，重试中...");
                state_start_time_ = ros::Time::now();
            }
        }
        else if (current_mav_state_.mode == "OFFBOARD") {
            sub_state         = 2;
            state_start_time_ = ros::Time::now();
        }
        break;

    case 2:
        if (!current_mav_state_.armed && timeout(3.0f)) {
            mavros_msgs::CommandBool srv;
            srv.request.value = true;
            if (arming_client_.call(srv) && srv.response.success) {
                ROS_INFO("无人机解锁成功");
                sub_state         = 3;
                state_start_time_ = ros::Time::now();
            }
            else {
                ROS_WARN("解锁失败，重试中...");
                state_start_time_ = ros::Time::now();
            }
        }
        else if (current_mav_state_.armed) {
            sub_state         = 3;
            state_start_time_ = ros::Time::now();
        }
        break;

    case 3:
        if (fabs(current_z - target_z) < 0.2) {
            if (timeout(1.0f)) {
                ROS_INFO("起飞完成，稳定悬停，进入导航至目标识别区阶段");
                current_state_    = MOVE_TO_RING_FRONT;
                nav_goal_sent_    = false;
                state_start_time_ = ros::Time::now();
                sub_state         = 0;
                setpoint_count    = 0;
            }
        }
        else {
            state_start_time_ = ros::Time::now();
        }
        break;
    }

    if (sub_state == 3) {
        ROS_INFO_THROTTLE(1.0, "爬升中... 当前高度: %.2f / %.2f", current_z, target_z);
    }
}

void MissionManager::handleMoveToRingFront() {
    Waypoint target_wp;
    static bool should_move = false;
    if (!should_move || !timeout(10.0f)) {
        if (!ring_detection.detected) {
            ROS_INFO_STREAM_THROTTLE(1, "等待pcl确认环位置");
            return;
        }
        else {
            ROS_INFO_STREAM("pcl确认成功");
            should_move = true;
            target_wp   = wp_ring_front_;
        }
    }
    if (!should_move) {
        ROS_WARN_STREAM("pcl确认环超时，改换定点");
        should_move = true;
        target_wp   = wp_ring_front_;
    }
    if (moveTo(target_wp)) {
        current_state_    = SETOUT_CROSS_RING;
        state_start_time_ = ros::Time::now();
        ROS_INFO_STREAM("准备穿环");
    }
}
void MissionManager::handleSetoutCrossRing() {
    if (moveTo(wp_ring_back_)) {
        current_state_    = NAV_TO_DROP_AREA;
        state_start_time_ = ros::Time::now();
        ROS_INFO_STREAM("准备穿随机障碍物");
    }
}

// 8.4 导航至物资投放区
void MissionManager::handleNavToDropArea() {
    Waypoint drop_target(init_pos_x_ + wp_drop_area_.x, init_pos_y_ + wp_drop_area_.y,
                         init_pos_z_ + wp_drop_area_.z);
    if (navTo(drop_target) && isDropWindowStable(drop_target.z)) {
        current_state_    = HOVER_RECOG_DROP;
        nav_goal_sent_    = false;
        state_start_time_ = ros::Time::now();
        if (front_camera_active_) callSwitchCamera();
        callResetTarget();
        last_pid_control_time_     = ros::Time(0);
        drop_alignment_hold_start_ = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
        ROS_INFO("到达投放区 (ego_planner)，开始下视识别投放标识");
    }
}

// 8.5 悬停识别投放区标识
void MissionManager::handleHoverRecognizeDrop() {
    auto holdDropHover = [this]() {
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask        = 0b100111000111;
        current_setpoint_.velocity.x = current_setpoint_.velocity.y = 0.0f;
        current_setpoint_.velocity.z =
            (init_pos_z_ + cfg_.takeoff_height - local_odom_.pose.pose.position.z) * cfg_.p_z;
        current_setpoint_.yaw = init_yaw_;
    };

    if (!target_confirmed_) {
        holdDropHover();
        drop_alignment_hold_start_ = ros::Time(0);
        last_pid_control_time_     = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
        ROS_INFO_THROTTLE(1.0, "等待投放标识确认...");
        return;
    }

    const double detect_age = current_detection_.last_update.isZero()
                                  ? std::numeric_limits<double>::infinity()
                                  : (ros::Time::now() - current_detection_.last_update).toSec();
    if (!current_detection_.detected || detect_age > cfg_.drop_detect_timeout) {
        holdDropHover();
        drop_alignment_hold_start_ = ros::Time(0);
        last_pid_control_time_     = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
        ROS_WARN_THROTTLE(1.0, "投放标识检测丢失或超时，保持悬停");
        return;
    }

    const float aim_center_x =
        IMG_CENTER_X + cfg_.drop_camera_bias_x_px + cfg_.drop_release_bias_x_px;
    const float aim_center_y =
        IMG_CENTER_Y + cfg_.drop_camera_bias_y_px + cfg_.drop_release_bias_y_px;

    float err_x      = aim_center_x - current_detection_.center_x;
    float err_y      = aim_center_y - current_detection_.center_y;
    float pixel_dist = std::sqrt(err_x * err_x + err_y * err_y);

    ros::Time now    = ros::Time::now();
    float dt         = (now - last_pid_control_time_).toSec();
    if (last_pid_control_time_.isZero()) dt = 0.05f;
    last_pid_control_time_ = now;

    float vel_x, vel_y;
    getPixPidVel(err_x, err_y, dt, vel_x, vel_y);

    if (pixel_dist < cfg_.drop_fine_pixel_radius) {
        vel_x *= cfg_.drop_fine_vel_scale;
        vel_y *= cfg_.drop_fine_vel_scale;
    }

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    current_setpoint_.type_mask        = 0b100111000111;
    current_setpoint_.velocity.x       = vel_y;
    current_setpoint_.velocity.y       = vel_x;
    current_setpoint_.velocity.z =
        (init_pos_z_ + cfg_.takeoff_height - local_odom_.pose.pose.position.z) * cfg_.p_z;
    current_setpoint_.yaw = init_yaw_ - 1.57;

    ROS_INFO_THROTTLE(0.5,
                      "[投放区对准] 像素误差: %.1f px, 有效中心:(%.1f, %.1f), 机体速度: %.2f m/s",
                      pixel_dist, aim_center_x, aim_center_y, getHorizontalSpeed());

    const bool ready_to_drop = pixel_dist < cfg_.align_pixel_threshold &&
                               isDropWindowStable(init_pos_z_ + wp_drop_area_.z);
    if (!ready_to_drop) {
        drop_alignment_hold_start_ = ros::Time(0);
        return;
    }

    if (drop_alignment_hold_start_.isZero()) {
        drop_alignment_hold_start_ = now;
        return;
    }

    if ((now - drop_alignment_hold_start_).toSec() >= cfg_.drop_align_hold_time) {
        ROS_INFO("投放区对准完成，进入投放");
        current_state_             = DROP_SUPPLY;
        state_start_time_          = ros::Time::now();
        drop_alignment_hold_start_ = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
    }
}

// 8.6 投放物资
void MissionManager::handleDropSupply() {
    static bool dropped                  = false;
    static bool drop_profile_initialized = false;
    static int drop_phase                = 0;
    static float hold_x                  = 0.0f;
    static float hold_y                  = 0.0f;
    static float cruise_z                = 0.0f;
    static float release_z               = 0.0f;

    const ros::Time now                  = ros::Time::now();

    if (!drop_profile_initialized) {
        hold_x    = local_odom_.pose.pose.position.x;
        hold_y    = local_odom_.pose.pose.position.y;
        cruise_z  = local_odom_.pose.pose.position.z;
        release_z = cruise_z;

        if (cfg_.drop_descend_distance > 0.0f) {
            release_z = std::max(init_pos_z_ + 0.20f, cruise_z - cfg_.drop_descend_distance);
        }

        drop_phase = (cfg_.drop_descend_distance > 0.0f && (cruise_z - release_z) > 1e-3f) ? 0 : 1;
        dropped    = false;
        drop_profile_initialized = true;
        state_start_time_        = now;
        ROS_INFO("投放剖面初始化: 巡航高度 %.2f m, 释放高度 %.2f m, 起始阶段 %d", cruise_z,
                 release_z, drop_phase);
    }

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask        = 0b101111111000;
    current_setpoint_.position.x       = hold_x;
    current_setpoint_.position.y       = hold_y;
    current_setpoint_.yaw              = init_yaw_;

    if (drop_phase == 0) {
        current_setpoint_.position.z = release_z;

        const bool reached_descend_height =
            std::abs(local_odom_.pose.pose.position.z - release_z) < cfg_.hover_vert_tolerance;
        const bool stable_at_release_height =
            getHorizontalSpeed() < cfg_.drop_release_max_horiz_speed &&
            std::abs(local_odom_.twist.twist.linear.z) < cfg_.drop_release_max_vert_speed &&
            std::abs(current_roll_) < cfg_.drop_max_tilt &&
            std::abs(current_pitch_) < cfg_.drop_max_tilt;

        if (reached_descend_height && stable_at_release_height) {
            ROS_INFO("已下降至释放高度并稳定，进入释放阶段");
            drop_phase        = 1;
            state_start_time_ = now;
        }
        return;
    }

    if (drop_phase == 1) {
        current_setpoint_.position.z = release_z;

        if (!isDropWindowStable(release_z)) {
            ROS_WARN_THROTTLE(1.0, "投放窗口不稳定，继续等待速度和姿态收敛");
            return;
        }

        if (!dropped) {
            std_msgs::Bool trigger;
            trigger.data = true;
            drop_trigger_pub_.publish(trigger);
            dropped           = true;
            state_start_time_ = now;
            ROS_INFO("物资投放指令已发送");
            return;
        }

        if ((now - state_start_time_).toSec() > 1.0) {
            ROS_INFO("释放完成，开始回升");
            drop_phase        = 2;
            state_start_time_ = now;
        }
        return;
    }

    current_setpoint_.position.z = cruise_z;

    const bool reached_cruise_height =
        std::abs(local_odom_.pose.pose.position.z - cruise_z) < cfg_.hover_vert_tolerance;
    const bool stable_after_ascend =
        getHorizontalSpeed() < cfg_.drop_release_max_horiz_speed &&
        std::abs(local_odom_.twist.twist.linear.z) < cfg_.drop_release_max_vert_speed &&
        std::abs(current_roll_) < cfg_.drop_max_tilt &&
        std::abs(current_pitch_) < cfg_.drop_max_tilt;

    if (reached_cruise_height && stable_after_ascend) {
        ROS_INFO("已回升至巡航高度并稳定，投放任务完成，前往攻击目标识别区");
        dropped                  = false;
        drop_profile_initialized = false;
        drop_phase               = 0;
        current_state_    = MOVE_TO_ATTACK_AREA;  // 注意：原代码此处跳转至 RETURN_LAND
        nav_goal_sent_    = false;
        state_start_time_ = now;
    }
}

// 8.7 移动至攻击目标识别区
void MissionManager::handleMoveToAttackArea() {
    if (moveTo(wp_attack_area_)) {
        current_state_    = RECOG_ATTACK_TARGET;
        nav_goal_sent_    = false;
        state_start_time_ = ros::Time::now();
        if (!front_camera_active_) callSwitchCamera();
        callResetTarget();
        ROS_INFO("到达攻击目标识别区，开始前视识别正确目标");
    }
}

// 8.8 识别攻击目标
void MissionManager::handleRecognizeAttackTarget() {
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask        = 0b100111000111;
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z =
        0.0f;
    current_setpoint_.yaw = current_yaw_;

    if (!target_confirmed_) {
        ROS_INFO_THROTTLE(1.0, "等待前视目标确认...");
        return;
    }

    if (current_detection_.detected) {
        float fx      = 500.0f;
        float fy      = 500.0f;
        float z       = cfg_.takeoff_height;
        float world_x = local_odom_.pose.pose.position.x +
                        (current_detection_.center_x - IMG_CENTER_X) * z / fx;
        float world_y = local_odom_.pose.pose.position.y +
                        (current_detection_.center_y - IMG_CENTER_Y) * z / fy;
        attack_target_world_ = Eigen::Vector3f(world_x, world_y, init_pos_z_);
        ROS_INFO("攻击目标世界坐标估算: (%.2f, %.2f)", world_x, world_y);
    }

    if ((ros::Time::now() - state_start_time_).toSec() > 1.0) {
        ROS_INFO("正确攻击目标已确认，移动到目标正前方");
        current_state_    = MOVE_TO_FRONT_OF_TARGET;
        nav_goal_sent_    = false;
        state_start_time_ = ros::Time::now();
    }
}

// 8.9 移动到目标正前方
void MissionManager::handleMoveToFrontOfTarget() {
    Eigen::Vector3f front_pos =
        attack_target_world_ + Eigen::Vector3f(cfg_.target_front_offset * cos(init_yaw_),
                                               cfg_.target_front_offset * sin(init_yaw_), 0.0f);
    if (moveTo(front_pos.x(), front_pos.y(), front_pos.z())) {
        current_state_         = ALIGN_ATTACK_TARGET;
        nav_goal_sent_         = false;
        state_start_time_      = ros::Time::now();
        last_pid_control_time_ = ros::Time(0);
        ROS_INFO("已到达攻击位置，开始前视像素对准");
    }
}

// 8.10 前视像素对准目标
void MissionManager::handleAlignAttackTarget() {
    if (!current_detection_.detected) {
        ROS_WARN_THROTTLE(1.0, "前视未检测到目标，保持悬停");
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask        = 0b100111000111;
        current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z =
            0.0f;
        current_setpoint_.yaw = current_yaw_;
        return;
    }

    float err_x      = IMG_CENTER_X - current_detection_.center_x;
    float err_y      = IMG_CENTER_Y - current_detection_.center_y;
    float pixel_dist = sqrt(err_x * err_x + err_y * err_y);

    ros::Time now    = ros::Time::now();
    float dt         = (now - last_pid_control_time_).toSec();
    if (last_pid_control_time_.isZero()) dt = 0.05f;
    last_pid_control_time_ = now;

    float vel_x, vel_y;
    getPixPidVel(err_x, err_y, dt, vel_x, vel_y);

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    current_setpoint_.type_mask        = 0b100111000111;
    current_setpoint_.velocity.x       = vel_y;
    current_setpoint_.velocity.y       = -vel_x;
    current_setpoint_.velocity.z       = 0.0f;
    current_setpoint_.yaw              = current_yaw_;

    ROS_INFO_THROTTLE(0.5, "[前视攻击对准] 像素误差: %.1f px", pixel_dist);

    if (pixel_dist < cfg_.align_pixel_threshold) {
        ROS_INFO("前视对准完成，准备攻击");
        current_state_    = SIMULATE_ATTACK;
        state_start_time_ = ros::Time::now();
        pix_integral_x_ = pix_integral_y_ = 0.0f;
    }
}

// 8.11 模拟攻击
void MissionManager::handleSimulateAttack() {
    static bool laser_fired            = false;

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask        = 0b100111000111;
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z =
        0.0f;
    current_setpoint_.yaw = current_yaw_;

    if (!laser_fired) {
        std_msgs::Bool trigger;
        trigger.data = true;
        laser_trigger_pub_.publish(trigger);
        ROS_INFO("激光指示装置已触发，等待裁判确认...");
        laser_fired       = true;
        state_start_time_ = ros::Time::now();
    }

    if (hit_confirmed_) {
        current_state_    = WAIT_HIT_CONFIRMATION;
        state_start_time_ = ros::Time::now();
        ROS_INFO("击中确认，返回起飞点");
    }
    else if ((ros::Time::now() - state_start_time_).toSec() > 5.0) {
        current_state_    = WAIT_HIT_CONFIRMATION;
        state_start_time_ = ros::Time::now();
        ROS_WARN("未收到确认，进入等待状态");
    }
}

// 8.12 等待裁判确认
void MissionManager::handleWaitHitConfirmation() {
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask        = 0b100111000111;
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z =
        0.0f;
    current_setpoint_.yaw = current_yaw_;

    if (hit_confirmed_) {
        current_state_    = NAV_TO_RING_BACK;
        state_start_time_ = ros::Time::now();
        ROS_INFO("裁判确认击中，返回");
    }
}

void MissionManager::handleNavToRingBack() {
    if (navTo(wp_ring_back_) ||
        (local_odom_.pose.pose.position.y <= wp_ring_back_.y + 0.3 && ring_detection.detected))
    {
        current_state_    = RETURN_CROSS_RING;
        nav_goal_sent_    = false;
        state_start_time_ = ros::Time::now();
        ROS_INFO_STREAM("已通过ego_planner穿过随机障碍物，准备返回穿环");
    }
}

void MissionManager::handleReturnCrossRing() {
    enum class SubState { MOVE_TO_RING_FRONT, CROSS_RING };
    static SubState sub_state = SubState::MOVE_TO_RING_FRONT;
    switch (sub_state) {
    case SubState::MOVE_TO_RING_FRONT: {

        Waypoint target_wp;
        static bool should_move = false;
        if (!should_move || !timeout(10.0f)) {
            if (!ring_detection.detected) {
                ROS_INFO_STREAM_THROTTLE(1, "等待pcl确认环位置");
                return;
            }
            else {
                ROS_INFO_STREAM("pcl确认成功");
                should_move = true;
                target_wp   = wp_ring_back_;
            }
        }
        if (!should_move) {
            ROS_WARN_STREAM("pcl确认环超时，改换定点");
            should_move = true;
            target_wp   = wp_ring_back_;
        }
        if (moveTo(target_wp)) {
            current_state_    = RETURN;
            state_start_time_ = ros::Time::now();
            ROS_INFO_STREAM("准备穿环");
        }
        break;
    }
    case SubState::CROSS_RING:
    default:
        if (moveTo(wp_ring_front_)) {
            current_state_    = RETURN;
            state_start_time_ = ros::Time::now();
            ROS_INFO_STREAM("准备穿环");
        }
        break;
    }
}


// 8.13 返回起飞点并降落
void MissionManager::handleReturn() {
    if (moveTo(init_pos_x_, init_pos_y_, init_pos_z_ + cfg_.takeoff_height)) {
        nav_goal_sent_    = false;
        state_start_time_ = ros::Time::now();
        ROS_INFO("已返回起飞点上方，开始降落");
    }
}

void MissionManager::handleLand() {
    static bool land_profile_initialized = false;
    static int land_phase                = 0;  // 0:下降对准, 1:触地判定, 2:完成
    static float hold_x                  = 0.0f;
    static float hold_y                  = 0.0f;
    static float start_z                 = 0.0f;
    static ros::Time align_hold_start    = ros::Time(0);
    static ros::Time last_pid_time       = ros::Time(0);
    static float pix_integral_x          = 0.0f;
    static float pix_integral_y          = 0.0f;
    static float last_pix_err_x          = 0.0f;
    static float last_pix_err_y          = 0.0f;

    const ros::Time now                  = ros::Time::now();
    const float current_z                = local_odom_.pose.pose.position.z;
    const float ground_z                 = init_pos_z_;
    const float target_hover_z           = ground_z + cfg_.takeoff_height;

    // === 初始化 ===
    if (!land_profile_initialized) {
        hold_x           = local_odom_.pose.pose.position.x;
        hold_y           = local_odom_.pose.pose.position.y;
        start_z          = current_z;
        land_phase       = 0;
        align_hold_start = ros::Time(0);
        last_pid_time    = ros::Time(0);
        pix_integral_x = pix_integral_y = 0.0f;
        last_pix_err_x = last_pix_err_y = 0.0f;
        land_profile_initialized        = true;
        ROS_INFO("精准降落初始化: 起始高度 %.2f m, 地面高度 %.2f m", start_z, ground_z);
    }

    // === 阶段0: 下降 + 视觉对准 ===
    if (land_phase == 0) {
        // 检查下视检测有效性
        const double detect_age = current_detection_.last_update.isZero()
                                      ? std::numeric_limits<double>::infinity()
                                      : (now - current_detection_.last_update).toSec();

        if (!current_detection_.detected || detect_age > cfg_.drop_detect_timeout) {
            // 检测丢失：保持当前位置缓慢下降
            current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            current_setpoint_.type_mask        = 0b100111000111;
            current_setpoint_.velocity.x       = 0.0f;
            current_setpoint_.velocity.y       = 0.0f;
            current_setpoint_.velocity.z       = -cfg_.land_descend_speed;
            current_setpoint_.yaw              = init_yaw_;

            align_hold_start                   = ros::Time(0);
            pix_integral_x = pix_integral_y = 0.0f;
            ROS_WARN_THROTTLE(1.0, "[精准降落] 下视检测丢失，盲降模式，高度: %.2f m", current_z);

            // 高度足够低则进入触地判定
            if (current_z <= ground_z + cfg_.land_final_height) {
                ROS_INFO("[精准降落] 到达最终高度，进入触地判定");
                land_phase       = 1;
                align_hold_start = now;
            }
            return;
        }

        // 计算像素误差
        const float aim_center_x = IMG_CENTER_X + cfg_.drop_camera_bias_x_px;
        const float aim_center_y = IMG_CENTER_Y + cfg_.drop_camera_bias_y_px;
        float err_x              = aim_center_x - current_detection_.center_x;
        float err_y              = aim_center_y - current_detection_.center_y;
        float pixel_dist         = std::sqrt(err_x * err_x + err_y * err_y);

        // PID 计算
        float dt                 = last_pid_time.isZero() ? 0.05f : (now - last_pid_time).toSec();
        last_pid_time            = now;

        // 抗饱和限幅
        const float max_integral = 50.0f;
        pix_integral_x += err_x * dt;
        pix_integral_y += err_y * dt;
        pix_integral_x = std::clamp(pix_integral_x, -max_integral, max_integral);
        pix_integral_y = std::clamp(pix_integral_y, -max_integral, max_integral);

        float diff_x   = (err_x - last_pix_err_x) / dt;
        float diff_y   = (err_y - last_pix_err_y) / dt;
        last_pix_err_x = err_x;
        last_pix_err_y = err_y;

        float vel_x = cfg_.land_kp * err_x + cfg_.land_ki * pix_integral_x + cfg_.land_kd * diff_x;
        float vel_y = cfg_.land_kp * err_y + cfg_.land_ki * pix_integral_y + cfg_.land_kd * diff_y;

        // 速度限幅
        const float max_vel = cfg_.land_max_align_speed;
        float vel_norm      = std::sqrt(vel_x * vel_x + vel_y * vel_y);
        if (vel_norm > max_vel) {
            vel_x = vel_x * max_vel / vel_norm;
            vel_y = vel_y * max_vel / vel_norm;
        }

        // 近距离精细调整
        if (pixel_dist < cfg_.land_fine_pixel_radius) {
            vel_x *= cfg_.land_fine_vel_scale;
            vel_y *= cfg_.land_fine_vel_scale;
        }

        // 计算下降速度（随高度降低而减小）
        float height_ratio                 = (current_z - ground_z) / (start_z - ground_z);
        height_ratio                       = std::clamp(height_ratio, 0.0f, 1.0f);
        float descend_speed                = cfg_.land_descend_speed * (0.3f + 0.7f * height_ratio);

        // 对准良好时允许下降，否则悬停调整
        const bool aligned                 = pixel_dist < cfg_.land_align_pixel_threshold;
        float vel_z                        = aligned ? -descend_speed : 0.0f;

        // 发布控制指令
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        current_setpoint_.type_mask        = 0b100111000111;
        current_setpoint_.velocity.x       = vel_y;  // BODY_NED: x=前, y=右
        current_setpoint_.velocity.y       = vel_x;  // 图像x对应机体y
        current_setpoint_.velocity.z       = vel_z;
        current_setpoint_.yaw              = init_yaw_;

        // 对准计时
        if (aligned) {
            if (align_hold_start.isZero()) {
                align_hold_start = now;
            }
        }
        else {
            align_hold_start = ros::Time(0);
        }

        ROS_INFO_THROTTLE(
            0.5, "[精准降落] 像素误差: %.1f px, 高度: %.2f m, 速度: (%.2f, %.2f, %.2f) m/s",
            pixel_dist, current_z, vel_x, vel_y, vel_z);

        // 阶段切换条件
        if (current_z <= ground_z + cfg_.land_final_height) {
            ROS_INFO("[精准降落] 到达最终高度，进入触地判定");
            land_phase       = 1;
            align_hold_start = now;
        }
        return;
    }

    // === 阶段1: 触地判定 ===
    if (land_phase == 1) {
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask        = 0b101111111000;
        current_setpoint_.position.x       = hold_x;
        current_setpoint_.position.y       = hold_y;
        current_setpoint_.position.z       = ground_z + 0.1f;
        current_setpoint_.yaw              = init_yaw_;

        // 稳定判定
        const bool stable =
            getHorizontalSpeed() < cfg_.drop_release_max_horiz_speed &&
            std::abs(local_odom_.twist.twist.linear.z) < cfg_.drop_release_max_vert_speed &&
            std::abs(current_roll_) < cfg_.drop_max_tilt &&
            std::abs(current_pitch_) < cfg_.drop_max_tilt;

        if (stable && (now - align_hold_start).toSec() >= cfg_.land_final_hold_time) {
            ROS_INFO("精准降落完成，任务结束");
            land_phase               = 2;
            current_state_           = TASK_END;
            state_start_time_        = now;

            // 重置静态变量
            land_profile_initialized = false;
            align_hold_start         = ros::Time(0);
            pix_integral_x = pix_integral_y = 0.0f;
        }
        return;
    }
}

// 8.14 任务结束
void MissionManager::handleTaskEnd() {
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask        = 0b101111111000;
    current_setpoint_.position.x       = init_pos_x_;
    current_setpoint_.position.y       = init_pos_y_;
    current_setpoint_.position.z       = init_pos_z_;
    current_setpoint_.yaw              = init_yaw_;
    mission_finished_                  = true;
    ROS_INFO("任务完成，节点退出");
}