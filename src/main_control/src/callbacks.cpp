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
    target_confirmed_ = true;
    ROS_INFO("★★★ 目标确认: %s ★★★", confirmed_target_.c_str());
}

void MissionManager::yoloDetectCallback(const raicom_vision_laser::DetectionInfo::ConstPtr &msg) {
    bool found = false;
    for (int i = 0; i < msg->num_detections; ++i) {
        if (confirmed_target_.empty() || msg->class_names[i] == confirmed_target_) {
            current_detection_.center_x   = msg->center_x[i];
            current_detection_.center_y   = msg->center_y[i];
            current_detection_.confidence = msg->confidences[i];
            found                         = true;
            break;
        }
    }
    if (found && current_detection_.confidence > cfg_.detection_min_confidence) {
        current_detection_.detected    = true;
        current_detection_.last_update = ros::Time::now();
    }
    if (current_detection_.last_update - ros::Time::now() > ros::Duration(2.0)) {
        current_detection_.detected = false;
    }
}

void MissionManager::hitConfirmCallback(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data) {
        hit_confirmed_ = true;
        ROS_INFO("裁判确认击中目标！");
    }
}