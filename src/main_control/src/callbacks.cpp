#include "mission_manager.h"

void MissionManager::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    current_mav_state_ = *msg;
}

void MissionManager::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    local_odom_ = *msg;
    tf::Quaternion q;
    tf::quaternionMsgToTF(local_odom_.pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(current_roll_, current_pitch_, current_yaw_);

    if (!init_pos_received_ && local_odom_.pose.pose.position.z > -0.5) {
        init_pos_x_        = local_odom_.pose.pose.position.x;
        init_pos_y_        = local_odom_.pose.pose.position.y;
        init_pos_z_        = local_odom_.pose.pose.position.z;
        init_yaw_          = current_yaw_;
        init_pos_received_ = true;
        ROS_INFO("初始位置记录: (%.2f, %.2f, %.2f), 偏航: %.2f°", init_pos_x_, init_pos_y_,
                 init_pos_z_, init_yaw_ * 180 / M_PI);
    }
}

void MissionManager::navStatusCallback(const std_msgs::Int8::ConstPtr &msg) {
    nav_status_ = msg->data;
    if (msg->data == 1) nav_seen_executing_ = true;
}

void MissionManager::detectedTargetCallback(const std_msgs::String::ConstPtr &msg) {
    confirmed_target_ = msg->data;
    target_confirmed_ = true;
    ROS_INFO("★★★ 目标确认: %s ★★★", confirmed_target_.c_str());
}

void MissionManager::yoloDetectCallback(const raicom_vision_laser::DetectionInfo::ConstPtr &msg) {
    bool found            = false;
    float best_center_x   = 0, best_center_y = 0;
    float best_confidence = 0;
    std::string best_class;

    // 判断当前是否为前视攻击状态
    bool is_attack_state = (current_state_ == MOVE_TO_ATTACK_AREA ||
                            current_state_ == RECOG_ATTACK_TARGET ||
                            current_state_ == MOVE_TO_FRONT_OF_TARGET ||
                            current_state_ == ALIGN_ATTACK_TARGET ||
                            current_state_ == SIMULATE_ATTACK);

    // 确定匹配目标文字
    std::string match_target;
    switch (current_state_) {
    case HOVER_RECOG_DROP:
    case DROP_SUPPLY            : match_target = cfg_.detection_drop_target_class; break;
    case RETURN                 :
    case LAND                   : match_target = cfg_.detection_land_target_class; break;
    default                     : match_target = ""; break;
    }

    // 前视攻击：用确认文字匹配（vision_laser 方式），不依赖类名
    if (is_attack_state) {
        match_target = confirmed_target_;  // 下视确认的文字，或空
    }

    ROS_WARN_THROTTLE(2.0,
        "[DEBUG-YOLO] 收到 %d 个检测, state=%d, is_attack=%d, match_target='%s', "
        "confirmed_target_='%s', target_confirmed_=%d",
        msg->num_detections, (int)current_state_, is_attack_state,
        match_target.c_str(), confirmed_target_.c_str(), target_confirmed_);

    for (int i = 0; i < msg->num_detections; ++i) {
        std::string ocr_text = msg->class_names[i];
        float conf = msg->confidences[i];

        bool is_target = false;
        if (is_attack_state) {
            // 前视攻击：匹配确认文字；若无确认文字则取首个检测
            if (match_target.empty()) {
                is_target = true;  // 自动接受
            } else {
                is_target = (ocr_text == match_target);
            }
        } else if (!match_target.empty()) {
            is_target = (ocr_text == match_target);
        }

        if (is_target) {
            if (!found || conf > best_confidence) {
                best_center_x   = msg->center_x[i];
                best_center_y   = msg->center_y[i];
                best_confidence = conf;
                best_class      = ocr_text;
                found           = true;
            }
        }
    }

    if (found && best_confidence >= cfg_.detection_min_confidence) {
        current_detection_.detected    = true;
        current_detection_.center_x    = best_center_x;
        current_detection_.center_y    = best_center_y;
        current_detection_.confidence  = best_confidence;
        current_detection_.last_update = ros::Time::now();

        ROS_WARN("[DEBUG-YOLO] 检测命中! text='%s' conf=%.2f pos=(%.1f,%.1f)",
                 best_class.c_str(), best_confidence, best_center_x, best_center_y);

        // 前视攻击下若无确认目标，自动确认（跳过下视时的自救）
        if (is_attack_state && confirmed_target_.empty()) {
            confirmed_target_  = best_class;
            target_confirmed_  = true;
            ROS_WARN("★★★ [DEBUG-YOLO] 自动确认攻击目标: %s (置信度: %.2f) ★★★",
                     best_class.c_str(), best_confidence);
        }
    }
    else if (!current_detection_.last_update.isZero() &&
             ros::Time::now() - current_detection_.last_update > ros::Duration(2.0))
    {
        current_detection_.detected = false;
    }
}

void MissionManager::hitConfirmCallback(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data) {
        hit_confirmed_ = true;
        ROS_INFO("裁判确认击中目标！");
    }
}

void MissionManager::ringDetectCallback(const pcl_detection2::SquareRing::ConstPtr &msg) {
    if (msg->corners.size() >= 4) {
        ring_detection.detected    = true;
        ring_detection.last_update = ros::Time::now();

        // 多假设追踪：匹配/创建/置信度叠加/锁定
        updateRingTracking(msg);

        ROS_DEBUG_THROTTLE(2.0, "[Ring] 检测到方环, 中心(%.2f,%.2f,%.2f), 宽%.2f 高%.2f",
                         msg->center_point.x, msg->center_point.y, msg->center_point.z,
                         msg->width, msg->height);
    }
}

void MissionManager::pillarDetectCallback(const std_msgs::Int32::ConstPtr &msg) {
    pillar_case_id_ = msg->data;
    ROS_INFO_THROTTLE(2.0, "[Pillar] 收到柱子配置 #%d", pillar_case_id_);
}

void MissionManager::circleDetectCallback(const raicom_vision_laser::CircleDetectResult::ConstPtr &msg) {
    circle_detected_  = msg->detected;
    circle_center_x_  = msg->center_x;
    circle_center_y_  = msg->center_y;
    circle_radius_    = msg->radius;

    if (circle_detected_) {
        circle_detect_time_ = ros::Time::now();
        ROS_DEBUG_THROTTLE(1.0, "[Circle] 圆检测: (%.1f, %.1f) r=%.1f",
                           circle_center_x_, circle_center_y_, circle_radius_);
    }
}