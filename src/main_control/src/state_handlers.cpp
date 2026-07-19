#include "mission_manager.h"

// 8.1 起飞
void MissionManager::handleInitTakeoff() {
    static int sub_state               = 0;
    static int setpoint_count          = 0;
    static bool laser_safety_checked   = false;

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
            if (setpoint_count == 99) {
                // 起飞前确保激光关闭（遵守规则：全程不能一直开着激光）
                if (!laser_safety_checked) {
                    std_msgs::Bool laser_off;
                    laser_off.data = false;
                    laser_control_pub_.publish(laser_off);
                    ROS_INFO("【安全】激光已关闭");
                    laser_safety_checked = true;
                }
            }
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
        else { state_start_time_ = ros::Time::now(); }
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
        else { ROS_WARN("[Ring] 动态穿越点无效，使用硬编码"); }
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
        state_start_time_ = ros::Time::now();
        cross_wp_frozen   = false;
        ROS_INFO("%s", (pillar_nav_mode_ == "pcl") ? "进入PCL柱子检测" : "准备穿随机障碍物");
    }
}

// 8.4 导航至物资投放区
void MissionManager::handleNavToDropArea() {
    Waypoint mid_target(init_pos_x_ + wp_come_mid_.x, init_pos_y_ + wp_come_mid_.y,
                        init_pos_z_ + wp_come_mid_.z);
    Waypoint drop_target(init_pos_x_ + wp_drop_area_.x, init_pos_y_ + wp_drop_area_.y,
                         init_pos_z_ + wp_drop_area_.z);
    static bool come_mid_reached = false;

    if (!come_mid_reached) {
        if (navTo(mid_target)) {
            come_mid_reached = true;
            ROS_INFO_STREAM("到达中途点，继续前往投放区");
            nav_goal_sent_ = false;
            nav_status_    = 0;
        }
        return;
    }
    // PCL模式用moveTo（不依赖EGO），EGO模式用navTo
    bool drop_arrived = (pillar_nav_mode_ == "pcl") ? moveTo(drop_target) : navTo(drop_target);
    if (drop_arrived) {
        // mission_flow 融合：到达投放区，进入悬停投货流程（不再做下视识别投放标识）
        current_state_      = HOVER_RECOG_DROP;
        drop_sub_state_     = 0;
        drop_hover_start_   = ros::Time(0);
        last_drop_pub_time_ = ros::Time(0);
        nav_goal_sent_      = false;
        nav_status_         = 0;
        state_start_time_   = ros::Time::now();
        ROS_INFO("到达投放区，开始 mission_flow 悬停投货流程");
    }
}

// 8.5 悬停投货（mission_flow 状态3 Sub0：在投放点 z=1.5 悬停 drop_hover_time 秒）
void MissionManager::handleHoverRecognizeDrop() {
    // 保持位置在投放区上方（wp_drop_area_.z = 1.5），moveTo 内部加 init_pos
    moveTo(wp_drop_area_);

    if (drop_hover_start_.isZero()) {
        drop_hover_start_ = ros::Time::now();
    }

    ROS_INFO_THROTTLE(0.5, "[投货-悬停] 当前(%.2f,%.2f,%.2f) 保持投放区, 剩余 %.1fs",
                      local_odom_.pose.pose.position.x, local_odom_.pose.pose.position.y,
                      local_odom_.pose.pose.position.z,
                      cfg_.drop_hover_time - (ros::Time::now() - drop_hover_start_).toSec());

    if ((ros::Time::now() - drop_hover_start_).toSec() > cfg_.drop_hover_time) {
        ROS_INFO("[投货] 悬停 %.1fs 完成，开始下降投货", cfg_.drop_hover_time);
        drop_sub_state_   = 1;  // 进入下降阶段
        state_start_time_ = ros::Time::now();
        current_state_    = DROP_SUPPLY;
    }
}

// 8.6 投放物资（mission_flow 状态3 Sub1+Sub2：下降->触发投货->保持重发->复位）
void MissionManager::handleDropSupply() {
    const ros::Time now = ros::Time::now();

    // Sub 1: 下降到投货高度 drop_z（带超时兜底：到不了也在当前位置投货，避免漏投）
    if (drop_sub_state_ == 1) {
        ROS_INFO_THROTTLE(0.5, "[投货-下降] 当前(%.2f,%.2f,%.2f) -> 目标z=%.2f",
                          local_odom_.pose.pose.position.x, local_odom_.pose.pose.position.y,
                          local_odom_.pose.pose.position.z, cfg_.drop_z);

        bool reached_drop        = moveTo(wp_drop_area_.x, wp_drop_area_.y, cfg_.drop_z);
        bool descent_timeout_hit = (now - state_start_time_).toSec() > cfg_.descent_timeout;

        if (reached_drop || descent_timeout_hit) {
            if (descent_timeout_hit && !reached_drop)
                ROS_WARN("[投货] 下降超时(%.1fs)，当前 z=%.2f 未到投货高度，在当前位置触发投货",
                         cfg_.descent_timeout, local_odom_.pose.pose.position.z);
            else
                ROS_INFO("[投货] 到达投货高度 %.2f，触发投货", cfg_.drop_z);

            // 触发投货：发 cargo_drop_angle(<135) -> stm32_shooter 发 0x03 -> 货舱打开
            std_msgs::UInt8 servo_msg;
            servo_msg.data = cfg_.cargo_drop_angle;
            servo_control_pub_.publish(servo_msg);
            ROS_INFO("[投货] 已发送投货指令(角度 %d -> 0x03 -> 货舱打开)", cfg_.cargo_drop_angle);

            drop_sub_state_     = 2;
            state_start_time_   = now;
            last_drop_pub_time_ = now;
        }
        return;
    }

    // Sub 2: 保持位置 + 每 0.2s 重发投货指令保持货舱打开，cargo_hold_time 后复位
    if (drop_sub_state_ == 2) {
        // 保持位置在投货点（drop_z 高度）
        moveTo(wp_drop_area_.x, wp_drop_area_.y, cfg_.drop_z);

        // 每 0.2s 重发投货指令，确保 stm32_shooter 收到并保持货舱打开
        if ((now - last_drop_pub_time_).toSec() > 0.2) {
            std_msgs::UInt8 servo_msg;
            servo_msg.data = cfg_.cargo_drop_angle;
            servo_control_pub_.publish(servo_msg);
            ROS_INFO_THROTTLE(0.5, "[投货] 持续发送投货指令(角度 %d -> 0x03 -> 货舱打开)",
                              cfg_.cargo_drop_angle);
            last_drop_pub_time_ = now;
        }

        if ((now - state_start_time_).toSec() > cfg_.cargo_hold_time) {
            // 复位：发 cargo_reset_angle(>=135) -> stm32_shooter 发 0x04 -> 货舱关闭。连发 3 次确保收到
            std_msgs::UInt8 reset_msg;
            reset_msg.data = cfg_.cargo_reset_angle;
            for (int i = 0; i < 3; ++i)
                servo_control_pub_.publish(reset_msg);
            ROS_INFO("[投货] 投货完成，货舱复位(角度 %d -> 0x04 -> 货舱关闭) x3",
                     cfg_.cargo_reset_angle);

            // 投货完成，前往攻击目标识别区（升回 1.5）
            drop_sub_state_   = 0;
            current_state_    = MOVE_TO_ATTACK_AREA;
            nav_goal_sent_    = false;
            state_start_time_ = now;
        }
    }
}

// 8.7 升回攻击区高度（mission_flow：投放区=攻击区，仅升回 z=1.5 后前视识别）
void MissionManager::handleMoveToAttackArea() {
    // 投放区与攻击区为同一点(-0.45,-2.0)，投货后只需升回攻击区高度(1.5)即可识别射击
    if (moveTo(wp_attack_area_)) {
        current_state_    = RECOG_ATTACK_TARGET;
        nav_goal_sent_    = false;
        state_start_time_ = ros::Time::now();
        // 重置前视匹配状态，等待 /yolo_front_detect 回调填充
        front_target_matched_ = false;
        matched_target_.clear();
        matched_center_x_   = 0.0f;
        matched_center_y_   = 0.0f;
        last_matched_time_  = ros::Time(0);
        ROS_INFO("升回攻击区高度(%.2f)，开始前视识别目标 %s", cfg_.shoot_z,
                 cfg_.attack_real_target.c_str());
    }
    else {
        ROS_INFO_THROTTLE(0.5, "[攻击] 升回攻击区高度中... 当前 z=%.2f",
                          local_odom_.pose.pose.position.z);
    }
}

// 8.8 识别攻击目标（mission_flow 状态5 Sub1：前视找 target，判左/右/中，设射击点偏移）
void MissionManager::handleRecognizeAttackTarget() {
    // 悬停在攻击区
    moveTo(wp_attack_area_);

    // 检查目标是否在有效期内（mission_flow 的 target_timeout 机制）
    bool target_valid = front_target_matched_ &&
                        ((ros::Time::now() - last_matched_time_).toSec() < cfg_.yolo_target_timeout);

    if (target_valid) {
        ROS_INFO("[识别] 检测到目标 %s, 像素(%.1f, %.1f)",
                 matched_target_.c_str(), matched_center_x_, matched_center_y_);

        // 判断目标在图像左/右/中（前视不镜像：图像左=机体左=+Y，图像右=-Y）
        if (matched_center_x_ < (cfg_.yolo_img_center_x - cfg_.shoot_left_right_threshold)) {
            // 目标在图像左 -> 左射击点(+Y 侧)
            shoot_target_x_ = cfg_.shoot_left_x;
            shoot_target_y_ = cfg_.shoot_left_y;
            ROS_INFO("[识别] 目标偏左(像素 %.1f)，前往左射击点(%.2f, %.2f)",
                     matched_center_x_, shoot_target_x_, shoot_target_y_);
            current_state_    = ALIGN_ATTACK_TARGET;
            state_start_time_ = ros::Time::now();
        }
        else if (matched_center_x_ > (cfg_.yolo_img_center_x + cfg_.shoot_left_right_threshold)) {
            // 目标在图像右 -> 右射击点(-Y 侧)
            shoot_target_x_ = cfg_.shoot_right_x;
            shoot_target_y_ = cfg_.shoot_right_y;
            ROS_INFO("[识别] 目标偏右(像素 %.1f)，前往右射击点(%.2f, %.2f)",
                     matched_center_x_, shoot_target_x_, shoot_target_y_);
            current_state_    = ALIGN_ATTACK_TARGET;
            state_start_time_ = ros::Time::now();
        }
        else {
            // 目标在中心附近 -> 默认射击点
            shoot_target_x_ = cfg_.shoot_default_x;
            shoot_target_y_ = cfg_.shoot_default_y;
            ROS_INFO("[识别] 目标在中心(像素 %.1f)，前往默认射击点(%.2f, %.2f)",
                     matched_center_x_, shoot_target_x_, shoot_target_y_);
            current_state_    = ALIGN_ATTACK_TARGET;
            state_start_time_ = ros::Time::now();
        }
        return;
    }

    // 未检测到有效目标，继续等待
    ROS_WARN_THROTTLE(1.0, "[识别] 等待目标 %s 出现...", cfg_.attack_real_target.c_str());

    // 超时保护：超时走默认射击点
    if ((ros::Time::now() - state_start_time_).toSec() > cfg_.yolo_detect_timeout) {
        ROS_WARN("[识别] 目标检测超时(%.0fs)，使用默认射击点(%.2f, %.2f)",
                 cfg_.yolo_detect_timeout, cfg_.shoot_default_x, cfg_.shoot_default_y);
        shoot_target_x_   = cfg_.shoot_default_x;
        shoot_target_y_   = cfg_.shoot_default_y;
        current_state_    = ALIGN_ATTACK_TARGET;
        state_start_time_ = ros::Time::now();
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

// 8.10 飞到射击点并稳定（mission_flow 状态5 Sub2/3/5 + Sub4 前半）
void MissionManager::handleAlignAttackTarget() {
    static bool arrived = false;

    // 飞到射击点（shoot_target_x/y_ 为相对偏移，shoot_z 为射击高度，moveTo 内部加 init_pos）
    if (moveTo(shoot_target_x_, shoot_target_y_, cfg_.shoot_z)) {
        if (!arrived) {
            arrived           = true;
            state_start_time_ = ros::Time::now();
            ROS_INFO("[射击] 到达射击点(%.2f, %.2f, %.2f)，稳定 %.1fs 后射击",
                     shoot_target_x_, shoot_target_y_, cfg_.shoot_z, cfg_.shoot_stable_time);
        }

        // 稳定后进入射击
        if (timeout(cfg_.shoot_stable_time)) {
            arrived           = false;
            shoot_triggered_  = false;
            current_state_    = SIMULATE_ATTACK;
            state_start_time_ = ros::Time::now();
        }
    }
    else {
        arrived = false;
        ROS_INFO_THROTTLE(0.5, "[射击] 飞向射击点(%.2f, %.2f, %.2f)...",
                          shoot_target_x_, shoot_target_y_, cfg_.shoot_z);
    }
}

// 8.11 激光射击（mission_flow 状态5 Sub4：发 /shoot + 等 shoot_duration）
void MissionManager::handleSimulateAttack() {
    hover();  // 保持当前位置稳定

    if (!shoot_triggered_) {
        // 安全确认：先关闭激光（竞赛规则：起飞时激光不能开，否则罚时60s）
        std_msgs::Bool laser_off;
        laser_off.data = false;
        laser_control_pub_.publish(laser_off);

        // 通过 /shoot 触发 stm32_shooter_node 执行完整射击序列（开 -> 照1s -> 关）
        std_msgs::Empty shoot_msg;
        shoot_pub_.publish(shoot_msg);

        ROS_INFO("╔════════════════════════════════════════╗");
        ROS_INFO("║          ★★★ 射击！ ★★★            ║");
        ROS_INFO("║  射击坐标: (%.3f, %.3f, %.3f)",
                 local_odom_.pose.pose.position.x, local_odom_.pose.pose.position.y,
                 local_odom_.pose.pose.position.z);
        ROS_INFO("║  识别目标: %s", cfg_.attack_real_target.c_str());
        ROS_INFO("╚════════════════════════════════════════╝");

        shoot_triggered_ = true;
        shoot_time_      = ros::Time::now();
    }

    // 等 shoot_duration 后进入等待确认（stm32_shooter 开->1s->关序列完成）
    if ((ros::Time::now() - shoot_time_).toSec() > cfg_.shoot_duration) {
        ROS_INFO("[射击] 射击完成，等待裁判确认");
        current_state_    = WAIT_HIT_CONFIRMATION;
        state_start_time_ = ros::Time::now();
    }
}

// 8.12 等待裁判确认
void MissionManager::handleWaitHitConfirmation() {
    hover();
    if (timeout(5) || hit_confirmed_) {
        // PCL模式下去反向柱子航点，EGO模式去环后方
        current_state_ = (pillar_nav_mode_ == "pcl") ? RETURN_PILLAR_WAYPOINTS : READY_NAV_TO_RING_BACK;
        // READY_NAV_TO_RING_BACK  // [注释] 原有EGO路径
        state_start_time_ = ros::Time::now();
        if (hit_confirmed_)
            ROS_INFO("裁判确认击中，返回");
        else
            ROS_WARN("等待击中确认超时，返回");
    }
}

void MissionManager::handleReadyNavToRingBack() {
    // 先到来时的目标点（投放区），对齐后再开始返程导航
    Waypoint forward_target(init_pos_x_ + wp_drop_area_.x, init_pos_y_ + wp_drop_area_.y,
                            init_pos_z_ + wp_drop_area_.z);
    if (moveTo(forward_target.x, forward_target.y, forward_target.z)) {
        current_state_    = NAV_TO_RING_BACK;
        nav_goal_sent_    = false;
        state_start_time_ = ros::Time::now();
        ROS_INFO("到达投放区，开始返程");
    }
}

void MissionManager::handleNavToRingBack() {
    Waypoint mid_target(init_pos_x_ + wp_back_mid_.x, init_pos_y_ + wp_back_mid_.y,
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

    // 回程：先到中途点，再到穿环点
    static bool back_mid_reached = false;
    if (!back_mid_reached) {
        if (navTo(mid_target)) {
            back_mid_reached  = true;
            nav_goal_sent_    = false;
            state_start_time_ = ros::Time::now();
            nav_status_       = 0;
            ROS_INFO_STREAM("到达中途点，继续返程");
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
                else { target_wp = wp_ring_back_; }
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
            else { ROS_WARN("[Ring] 动态返程穿越点无效，使用硬编码"); }
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
    const float current_z = local_odom_.pose.pose.position.z;
    const float ground_z  = init_pos_z_;

    // 阶段0: 定点下降——水平P控制保持在起飞点上方，垂直限速下降
    if (current_z > ground_z + 0.3f) {
        positionControl(Eigen::Vector3f(init_pos_x_, init_pos_y_, current_z), current_setpoint_);
        current_setpoint_.velocity.z = -cfg_.land_descend_speed;
        current_setpoint_.yaw        = init_yaw_;
        ROS_INFO_THROTTLE(0.5, "[降落] 定点下降中, 离地高度: %.2f m", current_z - ground_z);
        return;
    }

    // 阶段1: 低于0.3m → 切 AUTO.LAND 触地
    static bool auto_land_sent = false;
    if (!auto_land_sent) {
        mavros_msgs::SetMode srv;
        srv.request.custom_mode = "AUTO.LAND";
        if (set_mode_client_.call(srv) && srv.response.mode_sent) {
            ROS_INFO("[降落] 高度 < 0.3m，AUTO.LAND 请求成功");
            auto_land_sent    = true;
            state_start_time_ = ros::Time::now();
        }
        else {
            ROS_WARN_THROTTLE(1.0, "[降落] 切换 AUTO.LAND 失败，重试中...");
        }
        return;
    }

    ROS_INFO("[降落] 降落完成，任务结束");
    auto_land_sent    = false;  // 复位供下次任务使用
    current_state_    = TASK_END;
    state_start_time_ = ros::Time::now();
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
        else { pillar_wp_total_ = 0; }

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
        // 正向航点完成 → 进入投货射击流程（mission_flow 融合：PCL柱子导航走完不再直接返程）
        ROS_INFO("[Pillar] 正向航点全部完成，进入 mission_flow 投货射击流程");
        current_state_      = HOVER_RECOG_DROP;
        drop_sub_state_     = 0;
        drop_hover_start_   = ros::Time(0);
        last_drop_pub_time_ = ros::Time(0);
        state_start_time_   = ros::Time::now();
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