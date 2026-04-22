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
}

void MissionManager::detectedTargetCallback(const std_msgs::String::ConstPtr &msg) {
    confirmed_target_ = msg->data;
    if (confirmed_target_ == target_class_name) {
        target_confirmed_ = true;
        ROS_INFO("★★★ 目标确认: %s ★★★", confirmed_target_.c_str());
    }
    else
        ROS_INFO("★★★ 目标确认: %s ★★★，但是预期目标：%s", confirmed_target_.c_str(),
                 target_class_name.c_str());
}

void MissionManager::yoloDetectCallback(const raicom_vision_laser::DetectionInfo::ConstPtr &msg) {
    bool found          = false;
    float best_center_x = 0, best_center_y = 0;
    float best_confidence = 0;
    switch (current_state_) {
    case HOVER_RECOG_DROP:
    case DROP_SUPPLY            : target_class_name = cfg_.detection_drop_target_class; break;

    case MOVE_TO_ATTACK_AREA    :
    case RECOG_ATTACK_TARGET    :
    case MOVE_TO_FRONT_OF_TARGET:
    case ALIGN_ATTACK_TARGET    :
    case SIMULATE_ATTACK        : target_class_name = cfg_.detection_attack_target_class; break;

    case RETURN                 :
    case LAND                   : target_class_name = cfg_.detection_land_target_class; break;

    default                     : target_class_name = ""; break;
    }
    for (int i = 0; i < msg->num_detections; ++i) {
        std::string class_name = msg->class_names[i];
        bool is_target         = false;
        if (!target_class_name.empty()) {
            is_target = (class_name == target_class_name);
        }
        if (is_target) {
            ROS_INFO_STREAM_THROTTLE(1.0, "收到目标: " << class_name
                                                       << " 置信度: " << msg->confidences[i]);
            if (!found || msg->confidences[i] > best_confidence) {
                if () best_center_x = msg->center_x[i];
                best_center_y   = msg->center_y[i];
                best_confidence = msg->confidences[i];
                found           = true;
                break;
            }
        }
        else {
            ROS_INFO_STREAM_THROTTLE(1.0, "收到非目标: " << class_name
                                                         << " 置信度: " << msg->confidences[i]
                                                         << "\n预期目标： " << target_class_name);
        }
    }
    if (found && current_detection_.confidence > cfg_.detection_min_confidence) {
        current_detection_.detected    = true;
        current_detection_.center_x    = best_center_x;
        current_detection_.center_y    = best_center_y;
        current_detection_.confidence  = best_confidence;
        current_detection_.last_update = ros::Time::now();
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

// void MissionManager::ringDetectCallback(const pcl_detection2::RingDetectionInfo::ConstPtr &msg){
//     if(msg->data){
//         //todo
//     }
// }