#include "mission_manager.h"

// 8.1 起飞
void MissionManager::handleInitTakeoff()
{
    static int sub_state = 0;
    static ros::Time last_request;
    static int setpoint_count = 0;

    ros::Time now = ros::Time::now();
    float target_z = init_pos_z_ + cfg_.takeoff_height;
    float current_z = local_odom_.pose.pose.position.z;

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b101111111000;
    current_setpoint_.position.x = init_pos_x_;
    current_setpoint_.position.y = init_pos_y_;
    current_setpoint_.position.z = target_z;
    current_setpoint_.yaw = init_yaw_;

    switch (sub_state)
    {
    case 0:
        if (setpoint_count < 100)
        {
            setpoint_count++;
            if (setpoint_count == 100)
            {
                ROS_INFO("已发送100个设定点，准备切换OFFBOARD模式");
                sub_state = 1;
                last_request = now;
            }
        }
        break;

    case 1:
        if (current_mav_state_.mode != "OFFBOARD" &&
            (now - last_request) > ros::Duration(3.0))
        {
            mavros_msgs::SetMode srv;
            srv.request.custom_mode = "OFFBOARD";
            if (set_mode_client_.call(srv) && srv.response.mode_sent)
            {
                ROS_INFO("OFFBOARD模式请求成功");
                sub_state = 2;
                last_request = now;
            }
            else
            {
                ROS_WARN("切换OFFBOARD失败，重试中...");
                last_request = now;
            }
        }
        else if (current_mav_state_.mode == "OFFBOARD")
        {
            sub_state = 2;
            last_request = now;
        }
        break;

    case 2:
        if (!current_mav_state_.armed &&
            (now - last_request) > ros::Duration(3.0))
        {
            mavros_msgs::CommandBool srv;
            srv.request.value = true;
            if (arming_client_.call(srv) && srv.response.success)
            {
                ROS_INFO("无人机解锁成功");
                sub_state = 3;
                last_request = now;
            }
            else
            {
                ROS_WARN("解锁失败，重试中...");
                last_request = now;
            }
        }
        else if (current_mav_state_.armed)
        {
            sub_state = 3;
            last_request = now;
        }
        break;

    case 3:
        if (fabs(current_z - target_z) < 0.2)
        {
            if ((now - last_request).toSec() > 3.0)
            {
                ROS_INFO("起飞完成，稳定悬停，进入导航至目标识别区阶段");
                current_state_ = NAV_TO_RECOG_AREA;
                nav_goal_sent_ = false;
                state_start_time_ = now;
                sub_state = 0;
                setpoint_count = 0;
            }
        }
        else
        {
            last_request = now;
        }
        break;
    }

    if (sub_state == 3)
    {
        ROS_INFO_THROTTLE(1.0, "爬升中... 当前高度: %.2f / %.2f", current_z, target_z);
    }
}

// 8.2 导航至目标识别区
void MissionManager::handleNavToRecogArea()
{
    if (!nav_goal_sent_)
    {
        float target_x = init_pos_x_ + wp_target_area_.x;
        float target_y = init_pos_y_ + wp_target_area_.y;
        float target_z = init_pos_z_ + wp_target_area_.z;
        sendEgoGoal(target_x, target_y, target_z);
    }

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b100111000111;
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
    current_setpoint_.yaw = current_yaw_;

    if (waitForNavArrival())
    {
        current_state_ = HOVER_RECOG_TARGET;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
        last_request_time_ = ros::Time::now();
        if (front_camera_active_)
            callSwitchCamera();
        callResetTarget();
        ROS_INFO("到达目标识别区，开始下视识别");
    }
}

// 8.3 悬停识别目标指示牌
void MissionManager::handleHoverRecognizeTarget()
{
    if (!target_confirmed_)
    {
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask = 0b100111000111;
        current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
        current_setpoint_.yaw = current_yaw_;
        ROS_INFO_THROTTLE(1.0, "等待下视目标确认...");
        return;
    }

    const double detect_age = current_detection_.last_update.isZero()
                                  ? std::numeric_limits<double>::infinity()
                                  : (ros::Time::now() - current_detection_.last_update).toSec();
    if (!current_detection_.detected || detect_age > 0.5)
    {
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask = 0b100111000111;
        current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
        current_setpoint_.yaw = current_yaw_;
        ROS_WARN_THROTTLE(1.0, "目标检测丢失或超时，保持悬停");
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pid_control_time_ = ros::Time(0);
        return;
    }

    float err_x = IMG_CENTER_X - current_detection_.center_x;
    float err_y = IMG_CENTER_Y - current_detection_.center_y;
    float pixel_dist = sqrt(err_x * err_x + err_y * err_y);

    ros::Time now = ros::Time::now();
    float dt = (now - last_pid_control_time_).toSec();
    if (last_pid_control_time_.isZero())
        dt = 0.05f;
    last_pid_control_time_ = now;

    float vel_x, vel_y;
    getPixPidVel(err_x, err_y, dt, vel_x, vel_y);

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    current_setpoint_.type_mask = 0b100111000111;
    current_setpoint_.velocity.x = vel_y;
    current_setpoint_.velocity.y = vel_x;
    current_setpoint_.velocity.z = 0.0f;
    current_setpoint_.yaw = init_yaw_ - 1.57;

    ROS_INFO_THROTTLE(0.5, "[下视对准] err_x=%.1f, err_y=%.1f | vel_x_cmd=%.2f, vel_y_cmd=%.2f",
                      err_x, err_y, current_setpoint_.velocity.x, current_setpoint_.velocity.y);

    if (pixel_dist < cfg_.align_pixel_threshold && isHoveringStable(cfg_.hover_vert_tolerance))
    {
        ROS_INFO("下视对准完成，进入导航至物资投放区");
        current_state_ = NAV_TO_DROP_AREA;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
        pix_integral_x_ = pix_integral_y_ = 0.0f;
    }
}

// 8.4 导航至物资投放区
void MissionManager::handleNavToDropArea()
{
    Eigen::Vector3f drop_target(init_pos_x_ + wp_drop_area_.x,
                                init_pos_y_ + wp_drop_area_.y,
                                init_pos_z_ + wp_drop_area_.z);

    if (cfg_.use_ego_planner_for_drop_area)
    {
        if (!nav_goal_sent_)
        {
            sendEgoGoal(drop_target.x(), drop_target.y(), drop_target.z());
        }
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask = 0b100111000111;
        current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
        current_setpoint_.yaw = current_yaw_;

        if (waitForNavArrival() && isDropWindowStable(drop_target.z()))
        {
            current_state_ = HOVER_RECOG_DROP;
            nav_goal_sent_ = false;
            state_start_time_ = ros::Time::now();
            if (front_camera_active_)
                callSwitchCamera();
            callResetTarget();
            last_pid_control_time_ = ros::Time(0);
            drop_alignment_hold_start_ = ros::Time(0);
            pix_integral_x_ = pix_integral_y_ = 0.0f;
            last_pix_err_x_ = last_pix_err_y_ = 0.0f;
            ROS_INFO("到达投放区 (ego_planner)，开始下视识别投放标识");
        }
        return;
    }

    mavros_msgs::PositionTarget sp;
    positionControl(drop_target, sp);
    sp.yaw = init_yaw_;
    current_setpoint_ = sp;

    const float dx = drop_target.x() - local_odom_.pose.pose.position.x;
    const float dy = drop_target.y() - local_odom_.pose.pose.position.y;
    const float dz = drop_target.z() - local_odom_.pose.pose.position.z;
    ROS_INFO_THROTTLE(0.5, "[投放区直飞] 位置误差: xy=%.2f m, z=%.2f m",
                      std::hypot(dx, dy), dz);

    if (reachedTarget(drop_target, cfg_.drop_arrive_threshold) && isDropWindowStable(drop_target.z()))
    {
        current_state_ = HOVER_RECOG_DROP;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
        if (front_camera_active_)
            callSwitchCamera();
        callResetTarget();
        last_pid_control_time_ = ros::Time(0);
        drop_alignment_hold_start_ = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
        ROS_INFO("到达投放区 (直飞模式)，开始下视识别投放标识");
    }
}

// 8.5 悬停识别投放区标识
void MissionManager::handleHoverRecognizeDrop()
{
    auto holdDropHover = [this]()
    {
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask = 0b100111000111;
        current_setpoint_.velocity.x = current_setpoint_.velocity.y = 0.0f;
        current_setpoint_.velocity.z = (init_pos_z_ + cfg_.takeoff_height - local_odom_.pose.pose.position.z) * cfg_.p_z;
        current_setpoint_.yaw = init_yaw_;
    };

    if (!target_confirmed_)
    {
        holdDropHover();
        drop_alignment_hold_start_ = ros::Time(0);
        last_pid_control_time_ = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
        ROS_INFO_THROTTLE(1.0, "等待投放标识确认...");
        return;
    }

    const double detect_age = current_detection_.last_update.isZero()
                                  ? std::numeric_limits<double>::infinity()
                                  : (ros::Time::now() - current_detection_.last_update).toSec();
    if (!current_detection_.detected || detect_age > cfg_.drop_detect_timeout)
    {
        holdDropHover();
        drop_alignment_hold_start_ = ros::Time(0);
        last_pid_control_time_ = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
        ROS_WARN_THROTTLE(1.0, "投放标识检测丢失或超时，保持悬停");
        return;
    }

    const float aim_center_x = IMG_CENTER_X + cfg_.drop_camera_bias_x_px + cfg_.drop_release_bias_x_px;
    const float aim_center_y = IMG_CENTER_Y + cfg_.drop_camera_bias_y_px + cfg_.drop_release_bias_y_px;

    float err_x = aim_center_x - current_detection_.center_x;
    float err_y = aim_center_y - current_detection_.center_y;
    float pixel_dist = std::sqrt(err_x * err_x + err_y * err_y);

    ros::Time now = ros::Time::now();
    float dt = (now - last_pid_control_time_).toSec();
    if (last_pid_control_time_.isZero())
        dt = 0.05f;
    last_pid_control_time_ = now;

    float vel_x, vel_y;
    getPixPidVel(err_x, err_y, dt, vel_x, vel_y);

    if (pixel_dist < cfg_.drop_fine_pixel_radius)
    {
        vel_x *= cfg_.drop_fine_vel_scale;
        vel_y *= cfg_.drop_fine_vel_scale;
    }

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    current_setpoint_.type_mask = 0b100111000111;
    current_setpoint_.velocity.x = vel_y;
    current_setpoint_.velocity.y = vel_x;
    current_setpoint_.velocity.z = (init_pos_z_ + cfg_.takeoff_height - local_odom_.pose.pose.position.z) * cfg_.p_z;
    current_setpoint_.yaw = init_yaw_ - 1.57;

    ROS_INFO_THROTTLE(0.5,
                      "[投放区对准] 像素误差: %.1f px, 有效中心:(%.1f, %.1f), 机体速度: %.2f m/s",
                      pixel_dist, aim_center_x, aim_center_y, getHorizontalSpeed());

    const bool ready_to_drop = pixel_dist < cfg_.align_pixel_threshold &&
                               isDropWindowStable(init_pos_z_ + wp_drop_area_.z);
    if (!ready_to_drop)
    {
        drop_alignment_hold_start_ = ros::Time(0);
        return;
    }

    if (drop_alignment_hold_start_.isZero())
    {
        drop_alignment_hold_start_ = now;
        return;
    }

    if ((now - drop_alignment_hold_start_).toSec() >= cfg_.drop_align_hold_time)
    {
        ROS_INFO("投放区对准完成，进入投放");
        current_state_ = DROP_SUPPLY;
        state_start_time_ = ros::Time::now();
        drop_alignment_hold_start_ = ros::Time(0);
        pix_integral_x_ = pix_integral_y_ = 0.0f;
        last_pix_err_x_ = last_pix_err_y_ = 0.0f;
    }
}

// 8.6 投放物资
void MissionManager::handleDropSupply()
{
    static bool dropped = false;
    static bool drop_profile_initialized = false;
    static int drop_phase = 0;
    static float hold_x = 0.0f;
    static float hold_y = 0.0f;
    static float cruise_z = 0.0f;
    static float release_z = 0.0f;

    const ros::Time now = ros::Time::now();

    if (!drop_profile_initialized)
    {
        hold_x = local_odom_.pose.pose.position.x;
        hold_y = local_odom_.pose.pose.position.y;
        cruise_z = local_odom_.pose.pose.position.z;
        release_z = cruise_z;

        if (cfg_.drop_descend_distance > 0.0f)
        {
            release_z = std::max(init_pos_z_ + 0.20f, cruise_z - cfg_.drop_descend_distance);
        }

        drop_phase = (cfg_.drop_descend_distance > 0.0f && (cruise_z - release_z) > 1e-3f) ? 0 : 1;
        dropped = false;
        drop_profile_initialized = true;
        state_start_time_ = now;
        ROS_INFO("投放剖面初始化: 巡航高度 %.2f m, 释放高度 %.2f m, 起始阶段 %d",
                 cruise_z, release_z, drop_phase);
    }

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b101111111000;
    current_setpoint_.position.x = hold_x;
    current_setpoint_.position.y = hold_y;
    current_setpoint_.yaw = init_yaw_;

    if (drop_phase == 0)
    {
        current_setpoint_.position.z = release_z;

        const bool reached_descend_height =
            std::abs(local_odom_.pose.pose.position.z - release_z) < cfg_.hover_vert_tolerance;
        const bool stable_at_release_height =
            getHorizontalSpeed() < cfg_.drop_release_max_horiz_speed &&
            std::abs(local_odom_.twist.twist.linear.z) < cfg_.drop_release_max_vert_speed &&
            std::abs(current_roll_) < cfg_.drop_max_tilt &&
            std::abs(current_pitch_) < cfg_.drop_max_tilt;

        if (reached_descend_height && stable_at_release_height)
        {
            ROS_INFO("已下降至释放高度并稳定，进入释放阶段");
            drop_phase = 1;
            state_start_time_ = now;
        }
        return;
    }

    if (drop_phase == 1)
    {
        current_setpoint_.position.z = release_z;

        if (!isDropWindowStable(release_z))
        {
            ROS_WARN_THROTTLE(1.0, "投放窗口不稳定，继续等待速度和姿态收敛");
            return;
        }

        if (!dropped)
        {
            std_msgs::Bool trigger;
            trigger.data = true;
            drop_trigger_pub_.publish(trigger);
            dropped = true;
            state_start_time_ = now;
            ROS_INFO("物资投放指令已发送");
            return;
        }

        if ((now - state_start_time_).toSec() > 1.0)
        {
            ROS_INFO("释放完成，开始回升");
            drop_phase = 2;
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

    if (reached_cruise_height && stable_after_ascend)
    {
        ROS_INFO("已回升至巡航高度并稳定，投放任务完成，前往攻击目标识别区");
        dropped = false;
        drop_profile_initialized = false;
        drop_phase = 0;
        current_state_ = RETURN_LAND; // 注意：原代码此处跳转至 RETURN_LAND
        nav_goal_sent_ = false;
        state_start_time_ = now;
    }
}

// 8.7 移动至攻击目标识别区
void MissionManager::handleMoveToAttackArea()
{
    if (!nav_goal_sent_)
    {
        sendEgoGoal(init_pos_x_ + wp_attack_area_.x, init_pos_y_ + wp_attack_area_.y, init_pos_z_ + wp_attack_area_.z);
    }

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b100111000111;
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
    current_setpoint_.yaw = current_yaw_;

    if (waitForNavArrival())
    {
        current_state_ = RECOG_ATTACK_TARGET;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
        last_request_time_ = ros::Time::now();
        if (!front_camera_active_)
            callSwitchCamera();
        callResetTarget();
        ROS_INFO("到达攻击目标识别区，开始前视识别正确目标");
    }
}

// 8.8 识别攻击目标
void MissionManager::handleRecognizeAttackTarget()
{
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b100111000111;
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
    current_setpoint_.yaw = current_yaw_;

    if (!target_confirmed_)
    {
        ROS_INFO_THROTTLE(1.0, "等待前视目标确认...");
        return;
    }

    if (current_detection_.detected)
    {
        float fx = 500.0f;
        float fy = 500.0f;
        float z = cfg_.takeoff_height;
        float world_x = local_odom_.pose.pose.position.x + (current_detection_.center_x - IMG_CENTER_X) * z / fx;
        float world_y = local_odom_.pose.pose.position.y + (current_detection_.center_y - IMG_CENTER_Y) * z / fy;
        attack_target_world_ = Eigen::Vector3f(world_x, world_y, init_pos_z_);
        ROS_INFO("攻击目标世界坐标估算: (%.2f, %.2f)", world_x, world_y);
    }

    if ((ros::Time::now() - state_start_time_).toSec() > 1.0)
    {
        ROS_INFO("正确攻击目标已确认，移动到目标正前方");
        current_state_ = MOVE_TO_FRONT_OF_TARGET;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
    }
}

// 8.9 移动到目标正前方
void MissionManager::handleMoveToFrontOfTarget()
{
    if (!nav_goal_sent_)
    {
        Eigen::Vector3f front_pos = attack_target_world_ + Eigen::Vector3f(cfg_.target_front_offset * cos(init_yaw_),
                                                                           cfg_.target_front_offset * sin(init_yaw_),
                                                                           0.0f);
        front_pos.z() = init_pos_z_ + cfg_.takeoff_height;
        sendEgoGoal(front_pos.x(), front_pos.y(), front_pos.z());
    }

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b100111000111;
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
    current_setpoint_.yaw = current_yaw_;

    if (waitForNavArrival())
    {
        current_state_ = ALIGN_ATTACK_TARGET;
        nav_goal_sent_ = false;
        state_start_time_ = ros::Time::now();
        last_pid_control_time_ = ros::Time(0);
        ROS_INFO("已到达攻击位置，开始前视像素对准");
    }
}

// 8.10 前视像素对准目标
void MissionManager::handleAlignAttackTarget()
{
    if (!current_detection_.detected)
    {
        ROS_WARN_THROTTLE(1.0, "前视未检测到目标，保持悬停");
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask = 0b100111000111;
        current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
        current_setpoint_.yaw = current_yaw_;
        return;
    }

    float err_x = IMG_CENTER_X - current_detection_.center_x;
    float err_y = IMG_CENTER_Y - current_detection_.center_y;
    float pixel_dist = sqrt(err_x * err_x + err_y * err_y);

    ros::Time now = ros::Time::now();
    float dt = (now - last_pid_control_time_).toSec();
    if (last_pid_control_time_.isZero())
        dt = 0.05f;
    last_pid_control_time_ = now;

    float vel_x, vel_y;
    getPixPidVel(err_x, err_y, dt, vel_x, vel_y);

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    current_setpoint_.type_mask = 0b100111000111;
    current_setpoint_.velocity.x = vel_y;
    current_setpoint_.velocity.y = -vel_x;
    current_setpoint_.velocity.z = 0.0f;
    current_setpoint_.yaw = current_yaw_;

    ROS_INFO_THROTTLE(0.5, "[前视攻击对准] 像素误差: %.1f px", pixel_dist);

    if (pixel_dist < cfg_.align_pixel_threshold)
    {
        ROS_INFO("前视对准完成，准备攻击");
        current_state_ = SIMULATE_ATTACK;
        state_start_time_ = ros::Time::now();
        pix_integral_x_ = pix_integral_y_ = 0.0f;
    }
}

// 8.11 模拟攻击
void MissionManager::handleSimulateAttack()
{
    static bool laser_fired = false;

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b100111000111;
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
    current_setpoint_.yaw = current_yaw_;

    if (!laser_fired)
    {
        std_msgs::Bool trigger;
        trigger.data = true;
        laser_trigger_pub_.publish(trigger);
        ROS_INFO("激光指示装置已触发，等待裁判确认...");
        laser_fired = true;
        state_start_time_ = ros::Time::now();
    }

    if (hit_confirmed_)
    {
        current_state_ = RETURN_LAND;
        state_start_time_ = ros::Time::now();
        ROS_INFO("击中确认，返回起飞点");
    }
    else if ((ros::Time::now() - state_start_time_).toSec() > 5.0)
    {
        current_state_ = WAIT_HIT_CONFIRMATION;
        state_start_time_ = ros::Time::now();
        ROS_WARN("未收到确认，进入等待状态");
    }
}

// 8.12 等待裁判确认
void MissionManager::handleWaitHitConfirmation()
{
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b100111000111;
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
    current_setpoint_.yaw = current_yaw_;

    if (hit_confirmed_)
    {
        current_state_ = RETURN_LAND;
        state_start_time_ = ros::Time::now();
        ROS_INFO("裁判确认击中，返回");
    }
}

// 8.13 返回起飞点并降落
void MissionManager::handleReturnLand()
{
    static bool returning = false;
    static float landing_target_z = init_pos_z_ + cfg_.takeoff_height;

    if (!returning)
    {
        if (!nav_goal_sent_)
        {
            float target_x = init_pos_x_;
            float target_y = init_pos_y_;
            float target_z = init_pos_z_ + cfg_.takeoff_height;
            sendEgoGoal(target_x, target_y, target_z);
        }

        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask = 0b100111000111;
        current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z = 0.0f;
        current_setpoint_.yaw = current_yaw_;

        if (waitForNavArrival())
        {
            returning = true;
            nav_goal_sent_ = false;
            state_start_time_ = ros::Time::now();
            ROS_INFO("已通过ego_planner返回起飞点上方，开始降落");
        }
    }
    else
    {
        landing_target_z -= 0.05f;
        if (landing_target_z < init_pos_z_ + 0.1f)
            landing_target_z = init_pos_z_ + 0.1f;

        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask = 0b101111111000;
        current_setpoint_.position.x = init_pos_x_;
        current_setpoint_.position.y = init_pos_y_;
        current_setpoint_.position.z = landing_target_z;
        current_setpoint_.yaw = init_yaw_;

        if (landing_target_z <= init_pos_z_ + 0.15f &&
            std::abs(local_odom_.pose.pose.position.z - (init_pos_z_ + 0.15f)) < 0.1f)
        {
            current_state_ = TASK_END;
            ROS_INFO("降落完成，任务结束");
        }
    }
}

// 8.14 任务结束
void MissionManager::handleTaskEnd()
{
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = 0b101111111000;
    current_setpoint_.position.x = init_pos_x_;
    current_setpoint_.position.y = init_pos_y_;
    current_setpoint_.position.z = init_pos_z_;
    current_setpoint_.yaw = init_yaw_;
    mission_finished_ = true;
    ROS_INFO("任务完成，节点退出");
}