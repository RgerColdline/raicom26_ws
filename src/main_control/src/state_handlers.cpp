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
    static Waypoint target_wp;
    static bool should_move = false;
    if (!should_move && !timeout(2.0f)) {
        if (!ring_detection.detected) {
            hover();
            ROS_INFO_STREAM_THROTTLE(1, "等待pcl确认环位置");
            return;
        }
        else {
            ROS_INFO_STREAM("pcl确认成功");
            should_move = true;
            // 优先用检测位姿动态计算，失败回退硬编码
            Waypoint dyn_front, dyn_back;
            if (computeRingApproachWP(dyn_front, dyn_back)) {
                target_wp = dyn_front;
                ROS_INFO("[Ring] 动态前方悬停点: (%.2f, %.2f, %.2f)", dyn_front.x, dyn_front.y,
                         dyn_front.z);
            }
            else {
                target_wp = wp_ring_front_;
                ROS_WARN("[Ring] 动态位姿无效，使用硬编码前方点");
            }
        }
    }
    if (!should_move) {
        ROS_WARN_STREAM("pcl确认环超时，改换定点");
        should_move = true;
        target_wp   = wp_ring_front_;
    }
    ROS_INFO_STREAM_THROTTLE(1, target_wp.x << target_wp.y << target_wp.z);
    if (moveTo(target_wp)) {
        current_state_    = SETOUT_CROSS_RING;
        state_start_time_ = ros::Time::now();
        ROS_INFO_STREAM("准备穿环");
    }
}
void MissionManager::handleSetoutCrossRing() {
    // 进入状态时计算一次穿越点，冻结不再变动
    static Waypoint cross_target = wp_ring_back_;
    static bool cross_wp_frozen  = false;
    if (!cross_wp_frozen) {
        Waypoint dyn_front, dyn_back;
        if (computeRingApproachWP(dyn_front, dyn_back)) {
            cross_target = dyn_back;
            ROS_INFO("[Ring] 去程穿越点已冻结: (%.2f, %.2f, %.2f)", dyn_back.x, dyn_back.y,
                     dyn_back.z);
        }
        else {
            ROS_WARN("[Ring] 动态穿越点无效，使用硬编码");
        }
        cross_wp_frozen = true;
    }

    if (moveTo(cross_target)) {
        // 记住穿越后的位置（世界坐标），返程直接导航到此处
        ring_back_memorized_.x()   = init_pos_x_ + cross_target.x;
        ring_back_memorized_.y()   = init_pos_y_ + cross_target.y;
        ring_back_memorized_.z()   = init_pos_z_ + cross_target.z;
        ring_back_memorized_valid_ = true;
        ROS_INFO("[Ring] 记住环后方位置: (%.2f, %.2f, %.2f)", ring_back_memorized_.x(),
                 ring_back_memorized_.y(), ring_back_memorized_.z());

        current_state_    = (pillar_nav_mode_ == "pcl") ? PILLAR_DETECT : NAV_TO_DROP_AREA;
        // NAV_TO_DROP_AREA  // [注释] 原有EGO路径：直接去投放区
        state_start_time_ = ros::Time::now();
        cross_wp_frozen   = false;
        ROS_INFO_STREAM((pillar_nav_mode_ == "pcl") ? "进入PCL柱子检测" : "准备穿随机障碍物");
    }
}

// 8.4 导航至物资投放区
void MissionManager::handleNavToDropArea() {
    Waypoint mid_target(init_pos_x_ + wp_come_mid_.x, init_pos_y_ + wp_come_mid_.y,
                        init_pos_z_ + wp_come_mid_.z);
    Waypoint drop_target(init_pos_x_ + wp_drop_area_.x, init_pos_y_ + wp_drop_area_.y,
                         init_pos_z_ + wp_drop_area_.z);
    static bool come_mid_reached = false;

    // PCL柱子导航模式：航点已定位，跳过中途点直接去投放区
    if (pillar_nav_mode_ == "pcl") {
        come_mid_reached = true;
    }

    if (!come_mid_reached) {
        if (navTo(mid_target)) {
            come_mid_reached = true;
            ROS_INFO_STREAM("中途点到达，继续前往投放区");
            nav_goal_sent_ = false;
            nav_status_    = 0;
        }
        return;
    }
    if (navTo(drop_target) && moveTo(drop_target) && isDropWindowStable(drop_target.z)) {
        // 跳过投放，直入前视攻击
        current_state_    = MOVE_TO_ATTACK_AREA;
        nav_goal_sent_    = false;
        nav_status_       = 0;
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
    // auto holdDropHover = [this]() {
    //     current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    //     current_setpoint_.type_mask        = 0b100111000111;
    //     current_setpoint_.velocity.x = current_setpoint_.velocity.y = 0.0f;
    //     current_setpoint_.velocity.z =
    //         (init_pos_z_ + cfg_.takeoff_height - local_odom_.pose.pose.position.z) * cfg_.p_z;
    //     current_setpoint_.yaw = init_yaw_;
    // };

    if (!target_confirmed_) {
        // holdDropHover();
        hover();
        drop_alignment_hold_start_ = ros::Time(0);
        last_pid_control_time_     = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
        ROS_INFO_THROTTLE(1.0, "等待投放标识确认...");
        return;
    }

    const double detect_age = current_detection_.last_update.isZero()
                                  ? 0.0  // 从未检测到 → 不超时，等待 OCR
                                  : (ros::Time::now() - current_detection_.last_update).toSec();
    if (!current_detection_.detected || detect_age > cfg_.drop_detect_timeout) {
        // holdDropHover();
        hover();
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

    if (drop_phase == 2) {
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
    static bool reset_target_called = false;
    static bool logged_once         = false;
    if (!logged_once) {
        ROS_WARN("[DEBUG-ATTACK] 进入 MOVE_TO_ATTACK_AREA, front_camera_active_=%d",
                 front_camera_active_);
        logged_once = true;
    }
    if (!reset_target_called) {
        ROS_WARN("[DEBUG-ATTACK] 执行 callResetTarget + callSwitchCamera");
        callResetTarget();
        if (!front_camera_active_) {
            bool ok = callSwitchCamera();
            ROS_WARN("[DEBUG-ATTACK] callSwitchCamera 返回=%d, front_camera_active_=%d", ok,
                     front_camera_active_);
        }
        reset_target_called = true;
    }
    if (target_confirmed_ || moveTo(wp_attack_area_)) {
        ROS_WARN("[DEBUG-ATTACK] 转换到 RECOG_ATTACK_TARGET, target_confirmed_=%d",
                 target_confirmed_);
        current_state_    = RECOG_ATTACK_TARGET;
        nav_goal_sent_    = false;
        state_start_time_ = ros::Time::now();
        ROS_INFO("到达攻击目标识别区，开始前视识别正确目标");
    }
}

// 8.8 识别攻击目标（vision_laser 定点模式：确认目标后判断左右→设置射击点）
void MissionManager::handleRecognizeAttackTarget() {
    // 悬停
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask        = 0b100111000111;
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z =
        0.0f;
    current_setpoint_.yaw = current_yaw_;

    if (!target_confirmed_) {
        hover();
        ROS_WARN_THROTTLE(2.0,
                          "[DEBUG-ATTACK] RECOG等待: target_confirmed_=%d, confirmed_target_='%s', "
                          "detected=%d, front_camera_active_=%d",
                          target_confirmed_, confirmed_target_.c_str(), current_detection_.detected,
                          front_camera_active_);
        return;
    }

    // 目标已确认，但需要检测到目标才能判断左右
    if (current_detection_.detected) {
        float target_cx = current_detection_.center_x;
        float threshold = cfg_.shoot_left_right_threshold;

        // 判断目标在图像中的位置（图像中心 = IMG_CENTER_X = 320）
        if (target_cx < (IMG_CENTER_X - threshold)) {
            shoot_target_x_ = init_pos_x_ + cfg_.shoot_left_x;
            shoot_target_y_ = init_pos_y_ + cfg_.shoot_left_y;
            ROS_INFO("[攻击] 目标偏左 (cx=%.1f)，前往左射击点 (%.2f, %.2f)", target_cx,
                     shoot_target_x_, shoot_target_y_);
        }
        else if (target_cx > (IMG_CENTER_X + threshold)) {
            shoot_target_x_ = init_pos_x_ + cfg_.shoot_right_x;
            shoot_target_y_ = init_pos_y_ + cfg_.shoot_right_y;
            ROS_INFO("[攻击] 目标偏右 (cx=%.1f)，前往右射击点 (%.2f, %.2f)", target_cx,
                     shoot_target_x_, shoot_target_y_);
        }
        else {
            shoot_target_x_ = init_pos_x_ + cfg_.shoot_default_x;
            shoot_target_y_ = init_pos_y_ + cfg_.shoot_default_y;
            ROS_INFO("[攻击] 目标在中心 (cx=%.1f)，前往默认射击点 (%.2f, %.2f)", target_cx,
                     shoot_target_x_, shoot_target_y_);
        }

        ROS_INFO("[攻击] 定点射击目标已确定，开始移动");
        current_state_    = ALIGN_ATTACK_TARGET;
        state_start_time_ = ros::Time::now();
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
    }
    else {
        // 目标已确认但当前视野未检测到
        hover();
        ROS_WARN_THROTTLE(1.0, "目标已确认但未检测到，等待检测...");

        // 60s超时使用默认射击点
        if (timeout(cfg_.shoot_detect_timeout)) {
            ROS_WARN("[攻击] 检测超时(%.0fs)，使用默认射击点", cfg_.shoot_detect_timeout);
            shoot_target_x_   = init_pos_x_ + cfg_.shoot_default_x;
            shoot_target_y_   = init_pos_y_ + cfg_.shoot_default_y;
            current_state_    = ALIGN_ATTACK_TARGET;
            state_start_time_ = ros::Time::now();
            pix_integral_x_ = pix_integral_y_ = 0.0f;
            last_pix_err_x_ = last_pix_err_y_ = 0.0f;
        }
    }
}

// 8.9 移动到目标正前方
void MissionManager::handleMoveToFrontOfTarget() {
    Eigen::Vector3f front_pos =
        attack_target_world_ +
        Eigen::Vector3f(cfg_.target_front_offset_x, cfg_.target_front_offset_y, 0.0f);
    if (moveTo(front_pos.x(), front_pos.y(), front_pos.z())) {
        current_state_         = ALIGN_ATTACK_TARGET;
        nav_goal_sent_         = false;
        state_start_time_      = ros::Time::now();
        last_pid_control_time_ = ros::Time(0);
        ROS_INFO("已到达攻击位置，开始前视像素对准");
    }
}

// 8.10 定点飞行到射击点（vision_laser 定点模式，抛弃视觉PID）
void MissionManager::handleAlignAttackTarget() {
    static bool arrived = false;

    // 使用位置控制飞到射击点
    if (moveTo(shoot_target_x_, shoot_target_y_, wp_attack_area_.z)) {
        if (!arrived) {
            arrived           = true;
            state_start_time_ = ros::Time::now();
            ROS_INFO("[攻击] 到达射击点 (%.2f, %.2f)，稳定 %.1fs 后射击", shoot_target_x_,
                     shoot_target_y_, cfg_.shoot_stable_time);
        }

        // 稳定后射击（与 vision_laser Sub 4 一致）
        if (timeout(cfg_.shoot_stable_time)) {
            ROS_INFO("\n");
            ROS_INFO("╔════════════════════════════════════════╗");
            ROS_INFO("║          ★★★ 射击！ ★★★            ║");
            ROS_INFO("╠════════════════════════════════════════╣");
            ROS_INFO("║  射击坐标: (%.3f, %.3f, %.3f)     ", local_odom_.pose.pose.position.x,
                     local_odom_.pose.pose.position.y, local_odom_.pose.pose.position.z);
            ROS_INFO("║  识别目标: %s", confirmed_target_.c_str());
            ROS_INFO("╚════════════════════════════════════════╝");
            ROS_INFO("\n");

            arrived           = false;
            current_state_    = SIMULATE_ATTACK;
            state_start_time_ = ros::Time::now();
        }
    }
    else {
        arrived = false;
        ROS_INFO_THROTTLE(0.5, "[攻击] 飞行到射击点 (%.2f, %.2f)...", init_pos_x_ + shoot_target_x_,
                          init_pos_y_ + shoot_target_y_);
    }
}

// 8.11 模拟攻击
void MissionManager::handleSimulateAttack() {
    static bool laser_fired = false;
    hover();
    if (!laser_fired) {
        std_msgs::Bool trigger;
        trigger.data = true;
        laser_trigger_pub_.publish(trigger);
        ROS_INFO("激光指示装置已触发，等待裁判确认...");
        laser_fired       = true;
        state_start_time_ = ros::Time::now();
    }
    current_state_    = WAIT_HIT_CONFIRMATION;
    state_start_time_ = ros::Time::now();
    ROS_WARN("进入等待状态");
}

// 8.12 等待裁判确认
void MissionManager::handleWaitHitConfirmation() {
    hover();
    if (timeout(5) || hit_confirmed_) {
        // PCL模式下去反向柱子航点，EGO模式去环后方
        current_state_ = (pillar_nav_mode_ == "pcl") ? RETURN_PILLAR_WAYPOINTS : NAV_TO_RING_BACK;
        // NAV_TO_RING_BACK  // [注释] 原有EGO路径
        state_start_time_ = ros::Time::now();
        if (hit_confirmed_)
            ROS_INFO("裁判确认击中，返回");
        else
            ROS_WARN("等待击中确认超时，返回");
    }
}

void MissionManager::handleNavToRingBack() {
    Waypoint mid_target(init_pos_x_ + wp_come_mid_.x, init_pos_y_ + wp_come_mid_.y,
                        init_pos_z_ + wp_come_mid_.z);

    // 优先用记住的第一次穿环位置（绝对坐标），其次动态计算
    Waypoint ring_back(init_pos_x_ + wp_ring_back_.x, init_pos_y_ + wp_ring_back_.y,
                       init_pos_z_ + wp_ring_back_.z);
    if (ring_back_memorized_valid_) {
        ring_back.x = ring_back_memorized_.x();
        ring_back.y = ring_back_memorized_.y();
        ring_back.z = ring_back_memorized_.z();
        ROS_INFO_THROTTLE(5.0, "[Ring] 返程导航到记住的位置: (%.2f, %.2f, %.2f)", ring_back.x,
                          ring_back.y, ring_back.z);
    }
    else {
        Waypoint dyn_front, dyn_back;
        if (computeRingApproachWP(dyn_front, dyn_back)) {
            ring_back.x = init_pos_x_ + dyn_back.x;
            ring_back.y = init_pos_y_ + dyn_back.y;
            ring_back.z = init_pos_z_ + dyn_back.z;
        }
    }

    static bool back_mid_reached = false;
    if (!back_mid_reached) {
        if (navTo(mid_target)) {
            back_mid_reached  = true;
            nav_goal_sent_    = false;
            state_start_time_ = ros::Time::now();
            nav_status_       = 0;
            ROS_INFO_STREAM("中途点到达，继续前往投放区");
        }
        return;
    }
    static bool back_mid_hovered = false;
    if (!back_mid_hovered) {
        hover();
        if (timeout(3)) {
            back_mid_hovered = true;
        }
        return;
    }
    if (navTo(ring_back) || (local_odom_.pose.pose.position.y <= ring_back_memorized_.y() + 0.2 &&
                             ring_detection.detected))
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
        static bool should_move = false;
    case SubState::MOVE_TO_RING_FRONT: {

        static Waypoint target_wp;
        if (!should_move && !timeout(2.0f)) {
            if (!ring_detection.detected) {
                ROS_INFO_STREAM_THROTTLE(1, "等待pcl确认环位置");
                return;
            }
            else {
                ROS_INFO_STREAM("pcl确认成功");
                should_move = true;
                // 此时 UAV 在环后方，front_wp = 指向UAV = 环后方悬停
                Waypoint dyn_front, dyn_back;
                if (computeRingApproachWP(dyn_front, dyn_back)) {
                    target_wp = dyn_front;
                    ROS_INFO("[Ring] 动态返回悬停点: (%.2f, %.2f, %.2f)", dyn_front.x, dyn_front.y,
                             dyn_front.z);
                }
                else {
                    target_wp = wp_ring_back_;
                }
            }
        }
        if (!should_move) {
            ROS_WARN_STREAM("pcl确认环超时，改换定点");
            should_move = true;
            target_wp   = wp_ring_back_;
        }
        if (moveTo(target_wp)) {
            sub_state         = SubState::CROSS_RING;
            state_start_time_ = ros::Time::now();
            ROS_INFO_STREAM("准备穿环");
        }
        break;
    }
    case SubState::CROSS_RING:
    default                  : {
        // 穿越回前方 → back_wp = 远离UAV = 环前方目标点
        static Waypoint return_cross_target = wp_ring_front_;
        static bool return_cross_frozen     = false;
        if (!return_cross_frozen) {
            Waypoint dyn_front, dyn_back;
            if (computeRingApproachWP(dyn_front, dyn_back)) {
                return_cross_target = dyn_back;
                ROS_INFO("[Ring] 返程穿越点已冻结: (%.2f, %.2f, %.2f)", dyn_back.x, dyn_back.y,
                         dyn_back.z);
            }
            else {
                ROS_WARN("[Ring] 动态返程穿越点无效，使用硬编码");
            }
            return_cross_frozen = true;
        }
        if (moveTo(return_cross_target)) {
            current_state_      = RETURN;
            state_start_time_   = ros::Time::now();
            return_cross_frozen = false;  // 重置供下次使用
            ROS_INFO_STREAM("已穿环，正在返回起飞点上方");
        }
        break;
    }
    }
}


// 8.13 返回起飞点并降落
void MissionManager::handleReturn() {
    if (moveTo(init_pos_x_, init_pos_y_, init_pos_z_ + cfg_.takeoff_height)) {
        nav_goal_sent_    = false;
        state_start_time_ = ros::Time::now();
        current_state_    = LAND;
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

    // === 阶段0: 下降 + 视觉对准（优先使用圆检测） ===
    if (land_phase == 0) {
        // === 圆检测对准（vision_laser precise_land 状态4，优先尝试） ===
        const double circle_age = circle_detect_time_.isZero()
                                      ? std::numeric_limits<double>::infinity()
                                      : (now - circle_detect_time_).toSec();

        bool circle_valid       = circle_detected_ && (circle_age < cfg_.circle_detect_timeout_s);

        if (circle_valid) {
            // 使用圆检测进行像素环PID对准
            float err_x        = IMG_CENTER_X - circle_center_x_;
            float err_y        = IMG_CENTER_Y - circle_center_y_;

            // 检查是否在阈值内
            bool in_threshold  = (std::abs(err_x) < cfg_.circle_align_threshold &&
                                 std::abs(err_y) < cfg_.circle_align_threshold);

            // 检查坐标是否有变化（防重复帧）
            float coord_change = std::sqrt(std::pow(circle_center_x_ - last_circle_x_, 2) +
                                           std::pow(circle_center_y_ - last_circle_y_, 2));
            bool coord_changed = (coord_change > 0.01f);

            if (in_threshold) {
                // 阈值内且坐标变化才计数
                if (coord_changed) {
                    circle_in_threshold_count_++;
                    last_circle_x_ = circle_center_x_;
                    last_circle_y_ = circle_center_y_;
                    ROS_INFO_THROTTLE(1.0, "[精准降落-圆] 靶标在中心，计数: %d/%d",
                                      circle_in_threshold_count_, cfg_.circle_confirm_count);
                }

                // 连续N次确认通过 → 锁定降落坐标
                if (circle_in_threshold_count_ >= cfg_.circle_confirm_count) {
                    land_target_x_     = local_odom_.pose.pose.position.x;
                    land_target_y_     = local_odom_.pose.pose.position.y;
                    target_pos_locked_ = true;
                    ROS_INFO("[精准降落-圆] 连续%d次确认通过，锁定坐标: (%.3f, %.3f)",
                             cfg_.circle_confirm_count, land_target_x_, land_target_y_);
                    land_phase       = 1;
                    align_hold_start = now;
                    return;
                }

                // 对准良好时保持悬停
                current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                current_setpoint_.type_mask        = 0b100111000111;
                current_setpoint_.velocity.x       = 0.0f;
                current_setpoint_.velocity.y       = 0.0f;
                current_setpoint_.velocity.z       = 0.0f;
                current_setpoint_.yaw              = init_yaw_;
            }
            else {
                // 不在阈值内，计数清零，执行PID微调
                if (circle_in_threshold_count_ > 0) {
                    ROS_INFO("[精准降落-圆] 靶标偏离中心，计数重置");
                    circle_in_threshold_count_ = 0;
                }

                float dt =
                    land_last_pid_time_.isZero() ? 0.05f : (now - land_last_pid_time_).toSec();
                land_last_pid_time_ = now;

                float vel_x, vel_y;
                getLandPixPidVel(err_x, err_y, dt, vel_x, vel_y);

                // BODY_NED: X=前, Y=右. 图像Y误差→X速度(前后), 图像X误差→Y速度(左右)
                current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
                current_setpoint_.type_mask        = 0b100111000111;
                current_setpoint_.velocity.x       = vel_x;
                current_setpoint_.velocity.y       = vel_y;
                current_setpoint_.velocity.z       = 0.0f;
                current_setpoint_.yaw              = init_yaw_;

                ROS_INFO_THROTTLE(0.5, "[精准降落-圆] 像素误差: (%.1f, %.1f), 速度: (%.3f, %.3f)",
                                  err_x, err_y, vel_x, vel_y);
            }

            align_hold_start = ros::Time(0);
            pix_integral_x = pix_integral_y = 0.0f;
            return;
        }

        // === 原有的YOLO检测对准（圆检测不可用时回退） ===
        if (!current_detection_.detected) {
            if (timeout(cfg_.drop_detect_timeout)) {
                ROS_WARN_THROTTLE(1.0, "[精准降落] 下视检测超时，直接采用自动降落模式");
                land_phase = -1;
                hover();
                state_start_time_ = now;
            }
            else {

                // 检测丢失：保持当前位置缓慢下降
                current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                current_setpoint_.type_mask        = 0b100111000111;
                current_setpoint_.velocity.x       = 0.0f;
                current_setpoint_.velocity.y       = 0.0f;
                current_setpoint_.velocity.z       = -cfg_.land_descend_speed;
                current_setpoint_.yaw              = init_yaw_;

                align_hold_start                   = ros::Time(0);
                pix_integral_x = pix_integral_y = 0.0f;
                ROS_WARN_THROTTLE(1.0, "[精准降落] 下视检测丢失，盲降模式，高度: %.2f m",
                                  current_z);

                // 高度足够低则进入触地判定
                if (current_z <= ground_z + cfg_.land_final_height) {
                    ROS_INFO("[精准降落] 到达最终高度，进入触地判定");
                    land_phase       = 1;
                    align_hold_start = now;
                }
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

    // === 阶段1: 触地判定 / 锁定坐标下降 ===
    if (land_phase == 1) {
        // 如果坐标已由圆检测锁定，使用锁定的XY下降
        if (target_pos_locked_) {
            // 锁定坐标垂直下降（vision_laser precise_land 状态5）
            current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            current_setpoint_.type_mask        = 0b101111111000;
            current_setpoint_.position.x       = land_target_x_;
            current_setpoint_.position.y       = land_target_y_;
            current_setpoint_.position.z       = current_z - 0.3f;  // 持续下降
            current_setpoint_.yaw              = init_yaw_;

            ROS_INFO_THROTTLE(0.5, "[精准降落-锁定] 保持坐标 (%.3f, %.3f) 下降, 高度: %.2f",
                              land_target_x_, land_target_y_, current_z);

            // 高度低于0.3m → AUTO.LAND（同 vision_laser）
            if (current_z <= ground_z + 0.3f) {
                ROS_INFO("[精准降落-锁定] 高度 < 0.3m，切换 AUTO.LAND");
                land_phase         = -1;
                target_pos_locked_ = false;
                align_hold_start   = ros::Time(0);
                state_start_time_  = now;
            }
            return;
        }

        // 原有的触地判定逻辑
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

    if (land_phase == -1) {
        mavros_msgs::SetMode srv;
        srv.request.custom_mode       = "AUTO.LAND";
        static bool auto_land_success = false;
        if (!auto_land_success) {
            if (set_mode_client_.call(srv) && srv.response.mode_sent) {
                ROS_INFO("AUTOLAND模式请求成功");
                auto_land_success = true;
                state_start_time_ = ros::Time::now();
            }
            else {
                ROS_WARN("切换AUTOLAND失败，重试中...");
                state_start_time_ = ros::Time::now();
            }
            return;
        }
        current_state_ = TASK_END;
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

// ============================================================
//  PCL 柱子检测导航状态
// ============================================================

void MissionManager::handlePillarDetect() {
    // 进入状态时向PCL发送启动信号
    static bool start_sent = false;
    if (!start_sent) {
        pillar_case_id_ = -1;  // 重置，等待新结果
        pillar_start_pub_.publish(std_msgs::Empty());
        start_sent = true;
        ROS_INFO("[Pillar] 已发送启动信号给PCL，等待检测结果...");
    }

    hover();

    if (pillar_case_id_ >= 0) {
        ROS_INFO("[Pillar] 收到PCL检测结果: case #%d", pillar_case_id_);

        // 设置航点序列
        pillar_wp_index_ = 0;
        if (pillar_case_id_ < static_cast<int>(pillar_waypoints_.size())) {
            pillar_wp_total_ = pillar_waypoints_[pillar_case_id_].size();
        }
        else {
            pillar_wp_total_ = 0;
        }

        if (pillar_wp_total_ > 0) {
            current_state_    = NAV_PILLAR_WAYPOINTS;
            state_start_time_ = ros::Time::now();
            start_sent        = false;
            ROS_INFO("[Pillar] 开始导航，共 %zu 个航点", pillar_wp_total_);
        }
        else {
            ROS_WARN("[Pillar] 无有效航点，回退EGO");
            current_state_    = NAV_TO_DROP_AREA;
            state_start_time_ = ros::Time::now();
        }
        return;
    }

    // 超时回退 (20s)
    if (timeout(20.0f)) {
        ROS_WARN("[Pillar] 检测超时(20s)，使用默认配置0");
        pillar_case_id_ = 0;  // 触发上面逻辑
    }
}

void MissionManager::handleNavPillarWaypoints() {
    if (pillar_case_id_ < 0 || pillar_case_id_ >= static_cast<int>(pillar_waypoints_.size())) {
        ROS_WARN("[Pillar] 无效配置ID，回退EGO");
        current_state_    = NAV_TO_DROP_AREA;
        state_start_time_ = ros::Time::now();
        return;
    }

    const auto &waypoints = pillar_waypoints_[pillar_case_id_];

    if (pillar_wp_index_ >= waypoints.size()) {
        // 所有正向航点完成 → 去投放区
        ROS_INFO("[Pillar] 正向航点全部完成，进入投放区");
        current_state_    = NAV_TO_DROP_AREA;
        state_start_time_ = ros::Time::now();
        return;
    }

    Waypoint wp = waypoints[pillar_wp_index_];
    Waypoint abs_wp(init_pos_x_ + wp.x, init_pos_y_ + wp.y, init_pos_z_ + wp.z);

    if (moveTo(abs_wp)) {
        ROS_INFO("[Pillar] 航点 #%zu/%zu 到达: (%.2f,%.2f,%.2f)", pillar_wp_index_ + 1,
                 waypoints.size(), abs_wp.x, abs_wp.y, abs_wp.z);
        ++pillar_wp_index_;
    }
    else {
        ROS_INFO_THROTTLE(1.0, "[Pillar] 飞向航点 #%zu/%zu: (%.2f,%.2f,%.2f)", pillar_wp_index_ + 1,
                          waypoints.size(), abs_wp.x, abs_wp.y, abs_wp.z);
    }
}

void MissionManager::handleReturnPillarWaypoints() {
    if (pillar_case_id_ < 0 || pillar_case_id_ >= static_cast<int>(pillar_waypoints_.size())) {
        ROS_WARN("[Pillar] 无效配置ID，回退EGO");
        current_state_    = NAV_TO_RING_BACK;
        state_start_time_ = ros::Time::now();
        return;
    }

    const auto &waypoints    = pillar_waypoints_[pillar_case_id_];
    static bool init_reverse = true;

    if (init_reverse) {
        pillar_wp_index_ = waypoints.size();
        init_reverse     = false;
        ROS_INFO("[Pillar] 开始反向导航，共 %zu 个航点", waypoints.size());
    }

    if (pillar_wp_index_ == 0) {
        // 反向航点全部完成 → 去穿环返回
        init_reverse      = true;
        current_state_    = RETURN_CROSS_RING;
        state_start_time_ = ros::Time::now();
        ROS_INFO("[Pillar] 反向航点全部完成，准备穿环返回");
        return;
    }

    size_t cur_idx = pillar_wp_index_ - 1;
    Waypoint wp    = waypoints[cur_idx];
    Waypoint abs_wp(init_pos_x_ + wp.x, init_pos_y_ + wp.y, init_pos_z_ + wp.z);

    if (moveTo(abs_wp)) {
        ROS_INFO("[Pillar] 反向航点 #%zu/%zu 到达: (%.2f,%.2f,%.2f)", waypoints.size() - cur_idx,
                 waypoints.size(), abs_wp.x, abs_wp.y, abs_wp.z);
        --pillar_wp_index_;  // 到达后才递减
    }
    else {
        ROS_INFO_THROTTLE(1.0, "[Pillar] 飞向反向航点 #%zu/%zu: (%.2f,%.2f,%.2f)",
                          waypoints.size() - cur_idx, waypoints.size(), abs_wp.x, abs_wp.y,
                          abs_wp.z);
    }
}