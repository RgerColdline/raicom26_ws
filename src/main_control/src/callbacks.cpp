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
    // mission_flow 融合：前视 YOLO 检测回调（订阅 /yolo_front_detect，320x240，中心 160）
    // 2026-07-20 固定映射射击后：射击点不再靠前视判决，本回调仅做日志/记录，
    // 匹配目标 = 投货悬停时识别出的 shoot_letter_（未识别则兜底 cfg_.attack_real_target）

    bool is_attack_state = (current_state_ == MOVE_TO_ATTACK_AREA ||
                            current_state_ == RECOG_ATTACK_TARGET ||
                            current_state_ == ALIGN_ATTACK_TARGET ||
                            current_state_ == SIMULATE_ATTACK);
    if (!is_attack_state) return;

    const std::string &match_target =
        shoot_letter_.empty() ? cfg_.attack_real_target : shoot_letter_;
    if (match_target.empty()) return;

    // 在检测结果里找匹配目标(取置信度最高的)
    bool  found          = false;
    float best_center_x   = 0.0f;
    float best_center_y   = 0.0f;
    float best_confidence = 0.0f;
    std::string best_class;

    for (int i = 0; i < msg->num_detections; ++i) {
        if (msg->class_names[i] == match_target) {
            if (!found || msg->confidences[i] > best_confidence) {
                best_center_x   = msg->center_x[i];
                best_center_y   = msg->center_y[i];
                best_confidence = msg->confidences[i];
                best_class      = msg->class_names[i];
                found           = true;
            }
        }
    }

    if (found && best_confidence >= cfg_.detection_min_confidence) {
        front_target_matched_ = true;
        matched_target_       = best_class;
        matched_center_x_     = best_center_x;
        matched_center_y_     = best_center_y;
        last_matched_time_    = ros::Time::now();
        ROS_INFO_THROTTLE(1.0, "★★★ [前视] 找到目标 %s, 像素(%.1f, %.1f) conf=%.2f ★★★",
                          matched_target_.c_str(), matched_center_x_, matched_center_y_,
                          best_confidence);
    }
}

void MissionManager::yoloDownDetectCallback(const raicom_vision_laser::DetectionInfo::ConstPtr &msg) {
    // 下视 YOLO 检测回调（订阅 /yolo_down_detect）：投货悬停字母投票。
    // 只在投票窗口（HOVER_RECOG_DROP）内计票；每帧最多投一票——取本帧置信度
    // 最高的 A/B 检测，防止单帧多个假框刷票。字母只会是 A 或 B。
    if (!down_voting_) return;

    float best_conf = 0.0f;
    std::string best_cls;
    for (int i = 0; i < msg->num_detections; ++i) {
        if ((msg->class_names[i] == "A" || msg->class_names[i] == "B") &&
            msg->confidences[i] > best_conf) {
            best_conf = msg->confidences[i];
            best_cls  = msg->class_names[i];
        }
    }
    if (best_cls.empty()) return;

    if (best_cls == "A") ++down_vote_a_;
    else                 ++down_vote_b_;

    ROS_INFO_THROTTLE(0.5, "[下视] 字母投票 %s(conf=%.2f)，累计 A=%d 票 B=%d 票",
                      best_cls.c_str(), best_conf, down_vote_a_, down_vote_b_);
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
    // pcl_detection2 模板匹配输出的柱子布局（0~3，编号与 traverse_map.yaml/模板一致）
    if (msg->data >= 0 && msg->data < 4) {
        if (detected_case_ != msg->data)
            ROS_INFO("[穿越] 收到柱子布局检测结果 case%d（%s）", msg->data,
                     TRAV_CASE_DESC[msg->data]);
        detected_case_ = msg->data;
    }
    else {
        ROS_WARN("[穿越] 收到非法 case_id=%d，忽略", msg->data);
    }
}