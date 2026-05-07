#include "mission_manager.h"

MissionManager::MissionManager(ros::NodeHandle &nh) : nh_(nh), current_state_(INIT_TAKEOFF) {
    loadParameters();
    loadPillarConfig();
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask        = 0b100111000111;
    current_setpoint_.velocity.x       = 0.0f;
    current_setpoint_.velocity.y       = 0.0f;
    current_setpoint_.velocity.z       = 0.0f;
    current_setpoint_.yaw              = 0.0f;
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
    nh_.param<float>("target_front_offset_x", cfg_.target_front_offset_x, -1.0f);
    nh_.param<float>("target_front_offset_y", cfg_.target_front_offset_y, 0.0f);
    nh_.param<float>("nav_goal_timeout", cfg_.nav_goal_timeout, 60.0f);
    nh_.param<float>("align_pixel_threshold", cfg_.align_pixel_threshold, 15.0f);
    nh_.param<float>("shoot_delay", cfg_.shoot_delay, 2.0f);

    nh_.param<float>("PIX_VEL_P", cfg_.PIX_VEL_P, 0.003f);
    nh_.param<float>("PIX_VEL_I", cfg_.PIX_VEL_I, 0.0001f);
    nh_.param<float>("PIX_VEL_D", cfg_.PIX_VEL_D, 0.001f);
    nh_.param<float>("PIX_VEL_MAX", cfg_.PIX_VEL_MAX, 0.4f);
    nh_.param<float>("PIX_FAR_NORM_DIST", cfg_.PIX_FAR_NORM_DIST, 150.0f);
    nh_.param<float>("PIX_INTEGRAL_MAX", cfg_.PIX_INTEGRAL_MAX, 100.0f);  // 补充缺失参数

    nh_.param<float>("wp_ring_front_x", wp_ring_front_.x, 0.65f);
    nh_.param<float>("wp_ring_front_y", wp_ring_front_.y, 0.0f);
    nh_.param<float>("wp_ring_front_z", wp_ring_front_.z, cfg_.takeoff_height);
    nh_.param<float>("wp_ring_back_x", wp_ring_back_.x, 2.05f);
    nh_.param<float>("wp_ring_back_y", wp_ring_back_.y, 0.0f);
    nh_.param<float>("wp_ring_back_z", wp_ring_back_.z, cfg_.takeoff_height);
    nh_.param<float>("wp_come_mid_x", wp_come_mid_.x, -2.35f);
    nh_.param<float>("wp_come_mid_y", wp_come_mid_.y, -2.48f);
    nh_.param<float>("wp_come_mid_z", wp_come_mid_.z, cfg_.takeoff_height);
    nh_.param<float>("wp_back_mid_x", wp_back_mid_.x, -2.35f);
    nh_.param<float>("wp_back_mid_y", wp_back_mid_.y, -2.48f);
    nh_.param<float>("wp_back_mid_z", wp_back_mid_.z, cfg_.takeoff_height);
    nh_.param<float>("wp_drop_area_x", wp_drop_area_.x, 0.45f);
    nh_.param<float>("wp_drop_area_y", wp_drop_area_.y, 2.0f);
    nh_.param<float>("wp_drop_area_z", wp_drop_area_.z, cfg_.takeoff_height);
    nh_.param<float>("wp_attack_area_x", wp_attack_area_.x, 0.45f);
    nh_.param<float>("wp_attack_area_y", wp_attack_area_.y, 2.0f);
    nh_.param<float>("wp_attack_area_z", wp_attack_area_.z, cfg_.takeoff_height);

    nh_.param<float>("detection/min_confidence", cfg_.detection_min_confidence, 0.5f);
    nh_.param<std::string>("detection/drop_target_class", cfg_.detection_drop_target_class,
                           "drop_target");
    nh_.param<std::string>("detection/attack_target_class", cfg_.detection_attack_target_class,
                           "attack_target");
    nh_.param<std::string>("detection/land_target_class", cfg_.detection_land_target_class,
                           "land_target");

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


    nh_.param<float>("land/kp", cfg_.land_kp, 0.005f);
    nh_.param<float>("land/ki", cfg_.land_ki, 0.0005f);
    nh_.param<float>("land/kd", cfg_.land_kd, 0.0001f);
    nh_.param<float>("land/max_align_speed", cfg_.land_max_align_speed, 0.5f);
    nh_.param<float>("land/descend_speed", cfg_.land_descend_speed, 0.3f);
    nh_.param<float>("land/align_pixel_threshold", cfg_.land_align_pixel_threshold, 15.0f);
    nh_.param<float>("land/fine_pixel_radius", cfg_.land_fine_pixel_radius, 30.0f);
    nh_.param<float>("land/fine_vel_scale", cfg_.land_fine_vel_scale, 0.4f);
    nh_.param<float>("land/final_height", cfg_.land_final_height, 0.2f);
    nh_.param<float>("land/final_hold_time", cfg_.land_final_hold_time, 1.5f);

    nh_.param<bool>("use_ego_planner_for_drop_area", cfg_.use_ego_planner_for_drop_area, true);

    // 环穿越动态参数
    nh_.param<float>("ring_front_approach_offset", cfg_.ring_front_approach_offset, 0.8f);
    nh_.param<float>("ring_back_approach_offset", cfg_.ring_back_approach_offset, 2.5f);

    // 环多假设追踪参数
    nh_.param<float>("ring_track/match_distance", cfg_.track_match_distance, 0.3f);
    nh_.param<float>("ring_track/confidence_boost", cfg_.track_confidence_boost, 0.15f);
    nh_.param<float>("ring_track/confidence_decay", cfg_.track_confidence_decay, 0.97f);
    nh_.param<float>("ring_track/confirm_threshold", cfg_.track_confirm_threshold, 0.7f);
    nh_.param<float>("ring_track/min_confidence", cfg_.track_min_confidence, 0.08f);
    nh_.param<int>("ring_track/max_candidates", cfg_.track_max_candidates, 3);
    nh_.param<float>("ring_track/ema_alpha", cfg_.track_ema_alpha, 0.3f);

    ROS_INFO("参数加载完成。");
}

void MissionManager::initROSCommunication() {
    setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ego_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/fsm/ego_goal", 1);
    drop_trigger_pub_  = nh_.advertise<std_msgs::Bool>("/uav/drop_trigger", 1);
    laser_trigger_pub_ = nh_.advertise<std_msgs::Bool>("/uav/laser_trigger", 1);

    state_sub_         = nh_.subscribe("/mavros/state", 10, &MissionManager::stateCallback, this);
    odom_sub_ =
        nh_.subscribe("/mavros/local_position/odom", 10, &MissionManager::odomCallback, this);
    nav_status_sub_ =
        nh_.subscribe("/ego_controller/status", 10, &MissionManager::navStatusCallback, this);
    detected_target_sub_ =
        nh_.subscribe("/detected_target", 10, &MissionManager::detectedTargetCallback, this);
    yolo_detect_sub_ = nh_.subscribe("/ocr_detect", 10, &MissionManager::yoloDetectCallback, this);
    hit_confirm_sub_ =
        nh_.subscribe("/referee/hit_confirmed", 10, &MissionManager::hitConfirmCallback, this);
    ring_sub_ =
        nh_.subscribe("/pcl_detection2/square_ring", 10, &MissionManager::ringDetectCallback, this);
    pillar_sub_       = nh_.subscribe("/pcl_detection2/pillar_case_id", 10,
                                      &MissionManager::pillarDetectCallback, this);
    pillar_start_pub_ = nh_.advertise<std_msgs::Empty>("/pcl_detection2/start_pillar_detect", 1);

    switch_camera_client_ = nh_.serviceClient<std_srvs::Empty>("/switch_camera");
    reset_target_client_  = nh_.serviceClient<std_srvs::Empty>("/reset_target");
    get_status_client_    = nh_.serviceClient<std_srvs::Empty>("/get_system_status");
    set_mode_client_      = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client_        = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
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
    goal.header.stamp    = ros::Time::now();
    goal.header.frame_id = "world";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;
    if (!std::isnan(yaw)) {
        tf::Quaternion q        = tf::createQuaternionFromYaw(yaw);
        goal.pose.orientation.x = q.x();
        goal.pose.orientation.y = q.y();
        goal.pose.orientation.z = q.z();
        goal.pose.orientation.w = q.w();
    }
    else {
        goal.pose.orientation.w = 1.0;
    }
    ego_goal_pub_.publish(goal);
    nav_goal_sent_      = true;
    nav_seen_executing_ = false;
    ROS_INFO("导航目标点: (%.2f, %.2f, %.2f)", x, y, z);
}

bool MissionManager::waitForNavArrival() {
    if ((ros::Time::now() - state_start_time_).toSec() > cfg_.nav_goal_timeout) {
        ROS_WARN("等待导航到达超时！");
        current_state_ = TASK_END;
        return false;
    }
    return nav_seen_executing_ && (nav_status_ == 2);
}

void MissionManager::positionControl(const Eigen::Vector3f &target_pos,
                                     mavros_msgs::PositionTarget &sp) {
    Eigen::Vector3f err = target_pos - Eigen::Vector3f(local_odom_.pose.pose.position.x,
                                                       local_odom_.pose.pose.position.y,
                                                       local_odom_.pose.pose.position.z);
    float vx            = err.x() * cfg_.p_xy;
    float vy            = err.y() * cfg_.p_xy;
    float vz            = err.z() * cfg_.p_z;
    vx                  = std::clamp(vx, -cfg_.max_speed, cfg_.max_speed);
    vy                  = std::clamp(vy, -cfg_.max_speed, cfg_.max_speed);
    vz                  = std::clamp(vz, -cfg_.max_speed, cfg_.max_speed);

    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    sp.type_mask        = 0b100111000111;
    sp.velocity.x       = vx;
    sp.velocity.y       = vy;
    sp.velocity.z       = vz;
    sp.yaw              = current_yaw_;
}

bool MissionManager::reachedTarget(const Eigen::Vector3f &target, float dist_thresh) {
    float dx = target.x() - local_odom_.pose.pose.position.x;
    float dy = target.y() - local_odom_.pose.pose.position.y;
    float dz = target.z() - local_odom_.pose.pose.position.z;
    return (dx * dx + dy * dy + dz * dz) < (dist_thresh * dist_thresh);
}

bool MissionManager::isHoveringStable(float vert_tolerance) {
    if (std::abs(local_odom_.twist.twist.linear.z) > 0.15) return false;
    if (std::abs(current_roll_) > 0.15 || std::abs(current_pitch_) > 0.15) return false;
    return true;
}

bool MissionManager::navTo(const float x, const float y, const float z) {
    if (!nav_goal_sent_) {
        float target_x = init_pos_x_ + x;
        float target_y = init_pos_y_ + y;
        float target_z = init_pos_z_ + z;
        sendEgoGoal(target_x, target_y, target_z);
    }

    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask        = 0b100111000111;
    current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z =
        0.0f;
    current_setpoint_.yaw = init_yaw_;

    return waitForNavArrival();
}

bool MissionManager::moveTo(const float x, const float y, const float z) {
    float target_x = init_pos_x_ + x;
    float target_y = init_pos_y_ + y;
    float target_z = init_pos_z_ + z;

    positionControl(Eigen::Vector3f(target_x, target_y, target_z), current_setpoint_);
    current_setpoint_.yaw = init_yaw_;

    return reachedTarget(Eigen::Vector3f(target_x, target_y, target_z), cfg_.err_max);
}

// 根据检测到的环 3D 位姿 + 无人机当前位置，动态计算穿越点（相对 init_pos 的偏移）
// front_wp = 无人机当前侧悬停点（靠近无人机这一侧）
// back_wp  = 环另一侧穿越点（远离无人机那一侧）
bool MissionManager::computeRingApproachWP(Waypoint &front_wp, Waypoint &back_wp) const {
    if (!ring_pose_valid_) return false;

    // 方向向量：环中心 → 当前无人机位置（动态，每次调用重新计算）
    Eigen::Vector3f uav_pos(local_odom_.pose.pose.position.x, local_odom_.pose.pose.position.y,
                            local_odom_.pose.pose.position.z);
    Eigen::Vector3f to_uav = (uav_pos - ring_center_).normalized();

    // 无人机当前侧悬停点：环中心 → 指向无人机
    Eigen::Vector3f front  = ring_center_ + to_uav * cfg_.ring_front_approach_offset;
    // 环另一侧穿越点：环中心 → 远离无人机
    Eigen::Vector3f back   = ring_center_ - to_uav * cfg_.ring_back_approach_offset;

    // 转为相对 init_pos 的偏移（与硬编码 WP 坐标系一致）
    front_wp.x             = front.x() - init_pos_x_;
    front_wp.y             = front.y() - init_pos_y_;
    front_wp.z             = front.z() - init_pos_z_;

    back_wp.x              = back.x() - init_pos_x_;
    back_wp.y              = back.y() - init_pos_y_;
    back_wp.z              = back.z() - init_pos_z_;

    return true;
}

// 环多假设追踪：匹配/创建/置信度叠加/锁定
void MissionManager::updateRingTracking(const pcl_detection2::SquareRing::ConstPtr &msg) {
    const Eigen::Vector3f new_center(msg->center_point.x, msg->center_point.y, msg->center_point.z);
    const Eigen::Vector3f new_front(msg->front_point.x, msg->front_point.y, msg->front_point.z);
    const Eigen::Vector3f new_back(msg->back_point.x, msg->back_point.y, msg->back_point.z);
    const ros::Time now = ros::Time::now();

    // 1. 找最匹配的未锁定候选
    int best_idx        = -1;
    float best_dist     = cfg_.track_match_distance;
    for (size_t i = 0; i < ring_candidates_.size(); ++i) {
        if (ring_candidates_[i].locked) continue;
        float dist = (ring_candidates_[i].center - new_center).norm();
        if (dist < best_dist) {
            best_dist = dist;
            best_idx  = static_cast<int>(i);
        }
    }

    if (best_idx >= 0) {
        // 2a. 命中：EMA 平滑位置 + 提升置信度
        auto &cand       = ring_candidates_[best_idx];
        float alpha      = cfg_.track_ema_alpha;
        cand.center      = alpha * new_center + (1.0f - alpha) * cand.center;
        cand.front_pt    = alpha * new_front + (1.0f - alpha) * cand.front_pt;
        cand.back_pt     = alpha * new_back + (1.0f - alpha) * cand.back_pt;
        cand.confidence  = std::min(1.0f, cand.confidence + cfg_.track_confidence_boost);
        cand.last_update = now;

        // 全局置信度累加（只增不减）
        ring_accumulated_confidence_ =
            std::min(1.0f, ring_accumulated_confidence_ + cfg_.track_confidence_boost);

        // 达标则锁定
        if (cand.confidence >= cfg_.track_confirm_threshold && !cand.locked) {
            cand.locked      = true;
            ring_center_     = cand.center;
            ring_front_pt_   = cand.front_pt;
            ring_back_pt_    = cand.back_pt;
            ring_pose_valid_ = true;
            ROS_INFO("[Ring] 环位姿已锁定确认! 置信度=%.2f 中心(%.2f,%.2f,%.2f)", cand.confidence,
                     ring_center_.x(), ring_center_.y(), ring_center_.z());
        }
    }
    else {
        // 2b. 未命中：新建候选
        // 初始置信度取全局累加值（不重置），确保多次检测后新建候选也有高置信度
        if (static_cast<int>(ring_candidates_.size()) < cfg_.track_max_candidates) {
            RingCandidate cand;
            cand.center      = new_center;
            cand.front_pt    = new_front;
            cand.back_pt     = new_back;
            cand.confidence  = std::max(cfg_.track_confidence_boost, ring_accumulated_confidence_);
            cand.last_update = now;

            // 全局置信度累加
            ring_accumulated_confidence_ =
                std::min(1.0f, ring_accumulated_confidence_ + cfg_.track_confidence_boost);

            ring_candidates_.push_back(cand);
            ROS_DEBUG("[Ring] 新候选 #%zu 初始置信度=%.2f (全局=%.2f)", ring_candidates_.size(),
                      cand.confidence, ring_accumulated_confidence_);
        }
    }

    // 3. 若没有锁定候选，选最高置信度的作为激活位姿
    if (!ring_pose_valid_ && !ring_candidates_.empty()) {
        size_t best = 0;
        for (size_t i = 1; i < ring_candidates_.size(); ++i) {
            if (ring_candidates_[i].confidence > ring_candidates_[best].confidence) best = i;
        }
        if (ring_candidates_[best].confidence > 0.1f) {
            ring_center_     = ring_candidates_[best].center;
            ring_front_pt_   = ring_candidates_[best].front_pt;
            ring_back_pt_    = ring_candidates_[best].back_pt;
            ring_pose_valid_ = true;
        }
    }
}

// 主循环衰减：每帧降低未命中候选的置信度 + 清理低置信度候选
void MissionManager::decayRingCandidates() {
    if (ring_candidates_.empty()) return;

    for (auto it = ring_candidates_.begin(); it != ring_candidates_.end();) {
        if (!it->locked) {
            it->confidence *= cfg_.track_confidence_decay;
        }
        // 清理极低置信度（锁定或未锁定但够低）
        if (it->confidence < cfg_.track_min_confidence && !it->locked) {
            ROS_DEBUG("[Ring] 清除低置信度候选 (%.3f)", it->confidence);
            it = ring_candidates_.erase(it);
        }
        else {
            ++it;
        }
    }

    // 所有候选都被清理后，重置激活位姿
    if (ring_candidates_.empty()) {
        ring_pose_valid_ = false;
    }
}

void MissionManager::checkRingTimeout() {
    // 先衰减候选
    decayRingCandidates();

    // 超时未收到检测 → 清除检测标记
    // 但如果有锁定候选（已确认环位姿），不超时清除——穿越过程中
    // LiDAR 可能暂时看不到环，但环位姿已锁定不应丢失
    bool has_locked = false;
    for (const auto &c : ring_candidates_) {
        if (c.locked) {
            has_locked = true;
            break;
        }
    }
    if (!has_locked && ring_detection.detected && !ring_detection.last_update.isZero() &&
        (ros::Time::now() - ring_detection.last_update) > ros::Duration(3.0))
    {
        ring_detection.detected = false;
        ROS_WARN_THROTTLE(2.0, "[Ring] 检测超时 (3s)，清除标记");
    }
}

void MissionManager::hover() {
    static float local_x             = local_odom_.pose.pose.position.x,
                 local_y             = local_odom_.pose.pose.position.y,
                 local_z             = local_odom_.pose.pose.position.z;
    static ros::Time last_hover_time = ros::Time::now();
    if (ros::Time::now() - last_hover_time > ros::Duration(3.0)) {
        local_x = local_odom_.pose.pose.position.x;
        local_y = local_odom_.pose.pose.position.y;
        local_z = local_odom_.pose.pose.position.z;
    }
    moveTo(local_x, local_y, local_z);
    last_hover_time = ros::Time::now();
}

float MissionManager::getHorizontalSpeed() const {
    return std::hypot(local_odom_.twist.twist.linear.x, local_odom_.twist.twist.linear.y);
}

bool MissionManager::isDropWindowStable(float target_z) const {
    int i1   = getHorizontalSpeed() < cfg_.drop_release_max_horiz_speed;
    int i2   = std::abs(local_odom_.twist.twist.linear.z) < cfg_.drop_release_max_vert_speed;
    int i3   = std::abs(local_odom_.pose.pose.position.z - target_z) < cfg_.hover_vert_tolerance;
    int i4   = std::abs(current_roll_) < cfg_.drop_max_tilt;
    int i5   = std::abs(current_pitch_) < cfg_.drop_max_tilt;
    int full = i1 << 4 | i2 << 3 | i3 << 2 | i4 << 1 | i5;
    ROS_INFO_STREAM_THROTTLE(0.5, "Drop窗口稳定性检查 - 二进制状态: "
                                      << std::bitset<5>(full) << " (" << i1 << ", " << i2 << ", "
                                      << i3 << ", " << i4 << ", " << i5 << ")");
    bool window_stable =
        getHorizontalSpeed() < cfg_.drop_release_max_horiz_speed &&
        std::abs(local_odom_.twist.twist.linear.z) < cfg_.drop_release_max_vert_speed &&
        std::abs(local_odom_.pose.pose.position.z - target_z) < cfg_.hover_vert_tolerance &&
        std::abs(current_roll_) < cfg_.drop_max_tilt &&
        std::abs(current_pitch_) < cfg_.drop_max_tilt;
    ROS_INFO_STREAM_THROTTLE(
        0.5, "水平速度: " << getHorizontalSpeed()
                          << " m/s, 垂直速度: " << local_odom_.twist.twist.linear.z
                          << " m/s, 离地高度: " << local_odom_.pose.pose.position.z
                          << " m, roll: " << current_roll_ << " rad, pitch: " << current_pitch_
                          << " rad, 窗口稳定: " << (window_stable ? "是" : "否"));
    return window_stable;
}

float MissionManager::satfunc(float value, float limit) {
    return std::clamp(value, -limit, limit);
}

void MissionManager::getPixPidVel(float err_x, float err_y, float dt, float &vel_x, float &vel_y) {
    dt                = std::clamp(dt, 0.02f, 1.0f);

    float norm_factor = 1.0f;
    float pixel_err   = std::sqrt(err_x * err_x + err_y * err_y);
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
    }
    else {
        pix_integral_x_ = 0.0f;
        pix_integral_y_ = 0.0f;
    }

    pix_integral_x_ = std::clamp(pix_integral_x_, -cfg_.PIX_INTEGRAL_MAX, cfg_.PIX_INTEGRAL_MAX);
    pix_integral_y_ = std::clamp(pix_integral_y_, -cfg_.PIX_INTEGRAL_MAX, cfg_.PIX_INTEGRAL_MAX);

    float deriv_x   = (norm_err_x - last_pix_err_x_) / dt;
    float deriv_y   = (norm_err_y - last_pix_err_y_) / dt;

    float pid_x =
        cfg_.PIX_VEL_P * norm_err_x + cfg_.PIX_VEL_I * pix_integral_x_ + cfg_.PIX_VEL_D * deriv_x;
    float pid_y =
        cfg_.PIX_VEL_P * norm_err_y + cfg_.PIX_VEL_I * pix_integral_y_ + cfg_.PIX_VEL_D * deriv_y;

    vel_x           = satfunc(pid_x, cfg_.PIX_VEL_MAX);
    vel_y           = satfunc(pid_y, cfg_.PIX_VEL_MAX);

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

bool MissionManager::timeout(const float timeout_limit) const noexcept {
    ros::Duration delta = ros::Time::now() - state_start_time_;
    // ROS_INFO_STREAM_THROTTLE(0.3, delta);
    return delta > ros::Duration(timeout_limit);
}

// --- 主循环 ---

void MissionManager::run() {
    ros::Rate rate(20);
    state_start_time_ = ros::Time::now();
    while (ros::ok() && !mission_finished_) {
        // 1. 默认安全设定点
        current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint_.type_mask        = 0b100111000111;
        current_setpoint_.velocity.x = current_setpoint_.velocity.y = current_setpoint_.velocity.z =
            0.0f;
        current_setpoint_.yaw = current_yaw_;

        // 2. 执行状态逻辑
        switch (current_state_) {
        case INIT_TAKEOFF           : handleInitTakeoff(); break;
        case MOVE_TO_RING_FRONT     : handleMoveToRingFront(); break;
        case SETOUT_CROSS_RING      : handleSetoutCrossRing(); break;
        case NAV_TO_DROP_AREA       : handleNavToDropArea(); break;
        case HOVER_RECOG_DROP       : handleHoverRecognizeDrop(); break;
        case DROP_SUPPLY            : handleDropSupply(); break;
        case MOVE_TO_ATTACK_AREA    : handleMoveToAttackArea(); break;
        case RECOG_ATTACK_TARGET    : handleRecognizeAttackTarget(); break;
        case MOVE_TO_FRONT_OF_TARGET: handleMoveToFrontOfTarget(); break;
        case ALIGN_ATTACK_TARGET    : handleAlignAttackTarget(); break;
        case SIMULATE_ATTACK        : handleSimulateAttack(); break;
        case WAIT_HIT_CONFIRMATION  : handleWaitHitConfirmation(); break;
        case NAV_TO_RING_BACK       : handleNavToRingBack(); break;
        case RETURN_CROSS_RING      : handleReturnCrossRing(); break;
        // --- PCL柱子导航（pillar_nav_mode="pcl"时替换EGO路径） ---
        case PILLAR_DETECT          : handlePillarDetect(); break;
        case NAV_PILLAR_WAYPOINTS   : handleNavPillarWaypoints(); break;
        case RETURN_PILLAR_WAYPOINTS: handleReturnPillarWaypoints(); break;
        case RETURN                 : handleReturn(); break;
        case LAND                   : handleLand(); break;
        case TASK_END               : handleTaskEnd(); break;
        default                     : break;
        }

        // 3. 发布设定点
        sendSetpoint(current_setpoint_);

        ros::spinOnce();

        // 4. 环追踪衰减 & 超时检测（20Hz 每帧执行）
        checkRingTimeout();

        rate.sleep();
    }
}

// === 柱子导航配置加载 ===
void MissionManager::loadPillarConfig() {
    // 导航模式
    nh_.param<std::string>("pillar_nav_mode", pillar_nav_mode_, "ego");
    nh_.param<float>("pillar_detect_timeout", pillar_detect_timeout_, 5.0f);
    nh_.param<float>("pillar_waypoint_tolerance", pillar_waypoint_tolerance_, 0.3f);

    // 读取4组航点 (嵌套YAML: pillar_waypoints/case_00, case_01, ...)
    pillar_waypoints_.clear();
    std::string case_keys[4] = {"case_00", "case_01", "case_02", "case_03"};

    for (int c = 0; c < 4; ++c) {
        std::string key = "pillar_waypoints/" + case_keys[c];
        XmlRpc::XmlRpcValue wp_list;
        if (!nh_.getParam(key, wp_list) || wp_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("[Pillar] 无法读取航点: %s", key.c_str());
            continue;
        }

        std::vector<Waypoint> waypoints;
        for (int i = 0; i < wp_list.size(); ++i) {
            if (wp_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray && wp_list[i].size() >= 2) {
                float x = static_cast<float>(static_cast<double>(wp_list[i][0]));
                float y = static_cast<float>(static_cast<double>(wp_list[i][1]));
                float z = cfg_.takeoff_height;
                if (wp_list[i].size() >= 3) {
                    z = static_cast<float>(static_cast<double>(wp_list[i][2]));
                }
                waypoints.push_back(Waypoint(x, y, z));
            }
        }
        pillar_waypoints_.push_back(waypoints);
        ROS_INFO("[Pillar] case_%02d: %zu 个航点", c, waypoints.size());
    }

    ROS_INFO("[Pillar] 导航模式=%s, 加载 %zu 组航点", pillar_nav_mode_.c_str(),
             pillar_waypoints_.size());
}