#include "mission_manager.h"

MissionManager::MissionManager(ros::NodeHandle &nh) : nh_(nh), current_state_(INIT_TAKEOFF) {
    loadParameters();
    loadTraverseConfig();
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
    nh_.param<float>("wp_pillar_center_x", wp_pillar_center_.x, -2.35f);
    nh_.param<float>("wp_pillar_center_y", wp_pillar_center_.y, -1.43f);
    nh_.param<float>("wp_pillar_center_z", wp_pillar_center_.z, cfg_.takeoff_height);
    nh_.param<float>("wp_drop_area_x", wp_drop_area_.x, 0.45f);
    nh_.param<float>("wp_drop_area_y", wp_drop_area_.y, 2.0f);
    nh_.param<float>("wp_drop_area_z", wp_drop_area_.z, cfg_.takeoff_height);
    nh_.param<float>("wp_attack_area_x", wp_attack_area_.x, 0.45f);
    nh_.param<float>("wp_attack_area_y", wp_attack_area_.y, 2.0f);
    nh_.param<float>("wp_attack_area_z", wp_attack_area_.z, cfg_.takeoff_height);

    nh_.param<float>("detection/min_confidence", cfg_.detection_min_confidence, 0.5f);
    nh_.param<std::string>("detection/attack_real_target", cfg_.attack_real_target, "A");
    nh_.param<bool>("wait_for_vision_services", cfg_.wait_for_vision_services, true);

    nh_.param<float>("drop_arrive_threshold", cfg_.drop_arrive_threshold, 0.35f);
    nh_.param<float>("drop/detect_timeout", cfg_.drop_detect_timeout, 5.0f);
    nh_.param<float>("drop_align_hold_time", cfg_.drop_align_hold_time, 0.35f);
    nh_.param<float>("drop_camera_bias_x_px", cfg_.drop_camera_bias_x_px, 0.0f);
    nh_.param<float>("drop_camera_bias_y_px", cfg_.drop_camera_bias_y_px, 0.0f);
    nh_.param<float>("drop_release_bias_x_px", cfg_.drop_release_bias_x_px, 0.0f);
    nh_.param<float>("drop_release_bias_y_px", cfg_.drop_release_bias_y_px, 0.0f);
    nh_.param<float>("drop_fine_pixel_radius", cfg_.drop_fine_pixel_radius, 35.0f);
    nh_.param<float>("drop_fine_vel_scale", cfg_.drop_fine_vel_scale, 0.45f);
    nh_.param<float>("drop_descend_distance", cfg_.drop_descend_distance, 0.0f);


    nh_.param<float>("land/descend_speed", cfg_.land_descend_speed, 0.3f);

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

    // 定点射击参数（vision_laser 状态6）
    nh_.param<float>("shoot/left_x", cfg_.shoot_left_x, -0.60f);
    nh_.param<float>("shoot/left_y", cfg_.shoot_left_y, -2.5f);
    nh_.param<float>("shoot/right_x", cfg_.shoot_right_x, -0.60f);
    nh_.param<float>("shoot/right_y", cfg_.shoot_right_y, -1.8f);
    nh_.param<float>("shoot/default_x", cfg_.shoot_default_x, -0.60f);
    nh_.param<float>("shoot/default_y", cfg_.shoot_default_y, -2.15f);
    nh_.param<float>("shoot/left_right_threshold", cfg_.shoot_left_right_threshold, 20.0f);
    nh_.param<float>("shoot/detect_timeout", cfg_.shoot_detect_timeout, 60.0f);
    nh_.param<float>("shoot/stable_time", cfg_.shoot_stable_time, 0.5f);
    nh_.param<float>("shoot/duration", cfg_.shoot_duration, 1.5f);
    nh_.param<float>("shoot/z", cfg_.shoot_z, 1.0f);

    // === mission_flow 融合：投货参数（/servo_control -> stm32_shooter） ===
    nh_.param<int>("cargo/drop_angle", cfg_.cargo_drop_angle, 0);
    nh_.param<int>("cargo/reset_angle", cfg_.cargo_reset_angle, 180);
    nh_.param<float>("cargo/hold_time", cfg_.cargo_hold_time, 2.0f);
    nh_.param<float>("cargo/descent_timeout", cfg_.descent_timeout, 10.0f);
    nh_.param<float>("cargo/drop_hover_time", cfg_.drop_hover_time, 2.0f);
    nh_.param<float>("cargo/drop_z", cfg_.drop_z, 0.8f);

    // === mission_flow 融合：前视YOLO识别参数（/yolo_front_detect, 320x240） ===
    nh_.param<float>("yolo/img_center_x", cfg_.yolo_img_center_x, 160.0f);
    nh_.param<float>("yolo/target_timeout", cfg_.yolo_target_timeout, 0.5f);
    nh_.param<float>("yolo/detect_timeout", cfg_.yolo_detect_timeout, 60.0f);

    // === 下视字母识别 + 固定映射射击（2026-07-20） ===
    nh_.param<int>("down/min_votes", cfg_.down_min_votes, 3);
    nh_.param<std::string>("shoot/a_side", cfg_.shoot_a_side, "left");
    a_on_left_ = (cfg_.shoot_a_side != "right");
    shoot_letter_ = cfg_.attack_real_target;   // 初始=兜底字母，投货悬停时投票覆盖

    // === 穿越赛段参数（traverse_map.yaml，pillar_nav_mode="pcl" 时生效） ===
    nh_.param<std::string>("pillar_nav_mode", pillar_nav_mode_, "ego");
    nh_.param<float>("traverse/flight_z", cfg_.trav_flight_z, 1.3f);
    nh_.param<float>("traverse/err_max", cfg_.trav_err_max, 0.15f);
    nh_.param<double>("traverse/v_max", cfg_.trav_v_max, 0.5);
    nh_.param<double>("traverse/a_max", cfg_.trav_a_max, 0.4);
    nh_.param<double>("traverse/a_lat_max", cfg_.trav_a_lat_max, 0.6);
    nh_.param<double>("traverse/inflation", cfg_.trav_inflation, 0.3);
    nh_.param<double>("traverse/sample_ds", cfg_.trav_sample_ds, 0.01);
    nh_.param<int>("traverse/force_fly", cfg_.trav_force_fly, 0);
    nh_.param<float>("traverse/traj_timeout_margin", cfg_.trav_timeout_margin, 15.0f);
    nh_.param<float>("traverse/scan_hover_time", cfg_.scan_hover_time, 3.0f);
    nh_.param<float>("traverse/scan_timeout", cfg_.scan_timeout, 4.0f);
    nh_.param<int>("traverse/default_case", cfg_.default_case, 1);
    nh_.param<int>("traverse/force_case", cfg_.force_case, -1);
    nh_.param<double>("map/origin_x", origin_fx_, 0.65);
    nh_.param<double>("map/origin_y", origin_fy_, 0.75);
    nh_.param<double>("map/pillar_radius", pillar_radius_, 0.1);

    ROS_INFO("参数加载完成。");
}

void MissionManager::initROSCommunication() {
    setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ego_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/fsm/ego_goal", 1);
    servo_control_pub_ = nh_.advertise<std_msgs::UInt8>("/servo_control", 1);
    shoot_pub_ = nh_.advertise<std_msgs::Empty>("/shoot", 1);
    laser_control_pub_ = nh_.advertise<std_msgs::Bool>("/laser_control", 1);

    state_sub_         = nh_.subscribe("/mavros/state", 10, &MissionManager::stateCallback, this);
    odom_sub_ =
        nh_.subscribe("/mavros/local_position/odom", 10, &MissionManager::odomCallback, this);
    nav_status_sub_ =
        nh_.subscribe("/ego_controller/status", 10, &MissionManager::navStatusCallback, this);
    detected_target_sub_ =
        nh_.subscribe("/detected_target", 10, &MissionManager::detectedTargetCallback, this);
    yolo_detect_sub_ = nh_.subscribe("/yolo_front_detect", 10, &MissionManager::yoloDetectCallback, this);
    yolo_down_detect_sub_ = nh_.subscribe("/yolo_down_detect", 10, &MissionManager::yoloDownDetectCallback, this);
    ring_sub_ =
        nh_.subscribe("/pcl_detection2/square_ring", 10, &MissionManager::ringDetectCallback, this);
    pillar_sub_       = nh_.subscribe("/pcl_detection2/pillar_case_id", 10,
                                      &MissionManager::pillarDetectCallback, this);
    pillar_start_pub_ = nh_.advertise<std_msgs::Empty>("/pcl_detection2/start_pillar_detect", 1);

    switch_camera_client_    = nh_.serviceClient<std_srvs::Empty>("/switch_camera");
    reset_target_client_     = nh_.serviceClient<std_srvs::Empty>("/reset_target");
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

    if (cfg_.wait_for_vision_services) {
        ROS_INFO("等待YOLO服务...");
        switch_camera_client_.waitForExistence();
        reset_target_client_.waitForExistence();
        ROS_INFO("YOLO服务已就绪");
    } else {
        ROS_WARN("跳过视觉服务等待（wait_for_vision_services=false），注意无视觉检测时状态机可能异常");
    }
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

    // EGO导航期间由ego_controller_node发setpoint，main_control不参与

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
        case NAV_TO_RING_BACK       : handleNavToRingBack(); break;
        case READY_NAV_TO_RING_BACK : handleReadyNavToRingBack(); break;
        case RETURN_CROSS_RING      : handleReturnCrossRing(); break;
        // --- 穿越赛段（pillar_nav_mode="pcl" 时替换EGO路径，traverse 平滑轨迹） ---
        case TRAVERSE_TO_SCAN      : handleTraverseToScan(); break;
        case TRAVERSE_SCAN         : handleTraverseScan(); break;
        case TRAVERSE_LEG2         : handleTraverseLeg2(); break;
        case TRAVERSE_READY_RETURN : handleTraverseReadyReturn(); break;
        case TRAVERSE_RETURN_LEG2  : handleTraverseReturnLeg2(); break;
        case RETURN                 : handleReturn(); break;
        case LAND                   : handleLand(); break;
        case TASK_END               : handleTaskEnd(); break;
        default                     : break;
        }

        // 3. 发布设定点（EGO正常模式由ego_controller_node接管；PCL/影子模式继续发）
        if (!nav_goal_sent_ || pillar_nav_mode_ == "pcl") {
            sendSetpoint(current_setpoint_);
        }

        ros::spinOnce();

        // 4. 环追踪衰减 & 超时检测（20Hz 每帧执行）
        checkRingTimeout();

        rate.sleep();
    }
}

// ============================================================
//  穿越赛段：地图读取 + leg2 规划 + 轨迹跟踪（traverse_node 融合）
//  地图与途经点全部是【场地系】官方坐标，规划器内部转 odom 系；
//  trackLeg/moveToAbs 用的是绝对 odom 坐标（起飞点=出生点=odom 原点）
// ============================================================

// XmlRpc 数值解析（rosparam 可能把 0.65 读成 double、把 2 读成 int）
static double xmlNum(const XmlRpc::XmlRpcValue &v) {
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) return (int)v;
    return (double)v;
}

// 读二维点列表（途经点 / 候选柱位）
static bool loadPointList(ros::NodeHandle &nh, const std::string &key, std::vector<Vec2f> &out) {
    XmlRpc::XmlRpcValue lst;
    if (!nh.getParam(key, lst)) return false;
    for (int i = 0; i < lst.size(); i++)
        if (lst[i].size() >= 2) {
            Vec2f p;
            p.x = xmlNum(lst[i][0]);
            p.y = xmlNum(lst[i][1]);
            out.push_back(p);
        }
    return true;
}

void MissionManager::loadTraverseConfig() {
    if (pillar_nav_mode_ != "pcl") {
        ROS_INFO("[穿越] pillar_nav_mode=%s，不加载穿越地图", pillar_nav_mode_.c_str());
        return;
    }

    // 参数合法性检查
    if (cfg_.default_case < 0 || cfg_.default_case > 3) {
        ROS_ERROR("[穿越] traverse/default_case=%d 非法（必须 0~3），回退 EGO 路径", cfg_.default_case);
        return;
    }
    if (cfg_.force_case < -1 || cfg_.force_case > 3) {
        ROS_ERROR("[穿越] traverse/force_case=%d 非法（必须 -1 或 0~3），回退 EGO 路径", cfg_.force_case);
        return;
    }

    // ---- 墙 + 场地边界 ----
    XmlRpc::XmlRpcValue wl;
    if (nh_.getParam("map/walls", wl)) {
        for (int i = 0; i < wl.size(); i++)
            if (wl[i].size() >= 4) {
                SegObs sg;
                sg.x1 = xmlNum(wl[i][0]);
                sg.y1 = xmlNum(wl[i][1]);
                sg.x2 = xmlNum(wl[i][2]);
                sg.y2 = xmlNum(wl[i][3]);
                walls_.push_back(sg);
            }
    }

    // ---- 4 套 leg2 绕柱段途经点 + 4 个候选柱位 ----
    for (int cid = 0; cid < 4; cid++) {
        char key[64];
        snprintf(key, sizeof(key), "map/via_points_leg2_case%d", cid);
        loadPointList(nh_, key, via_leg2_[cid]);
    }
    loadPointList(nh_, "map/pillar_candidates", pillar_cand_);

    // ---- 悬停扫描点（yaml 里是 map/scan_hover: [x, y] 列表） ----
    {
        XmlRpc::XmlRpcValue sh;
        if (nh_.getParam("map/scan_hover", sh) && sh.size() >= 2) {
            scan_hover_fx_ = xmlNum(sh[0]);
            scan_hover_fy_ = xmlNum(sh[1]);
        }
        else {
            ROS_WARN("[穿越] map/scan_hover 读取失败，用默认值 (%.2f, %.2f)",
                     scan_hover_fx_, scan_hover_fy_);
        }
    }

    // ---- 硬性检查 ----
    for (int cid = 0; cid < 4; cid++) {
        if (via_leg2_[cid].size() < 2) {
            ROS_ERROR("[穿越] map/via_points_leg2_case%d 为空或点数不足，回退 EGO 路径", cid);
            return;
        }
    }
    if (pillar_cand_.size() != 4) {
        ROS_ERROR("[穿越] map/pillar_candidates 必须是 4 个候选柱位（当前 %zu 个），回退 EGO 路径",
                  pillar_cand_.size());
        return;
    }

    // ---- 一致性检查（只告警，不阻断）----
    for (int cid = 0; cid < 4; cid++) {
        const Vec2f &p0 = via_leg2_[cid].front();
        if (fabs(p0.x - scan_hover_fx_) > 1e-6 || fabs(p0.y - scan_hover_fy_) > 1e-6)
            ROS_WARN("[穿越] leg2_case%d 首点(%.2f,%.2f) != scan_hover(%.2f,%.2f)，轨迹将从其他点起画！",
                     cid, p0.x, p0.y, scan_hover_fx_, scan_hover_fy_);
        const Vec2f &p1  = via_leg2_[cid].back();
        const Vec2f &ref = via_leg2_[0].back();
        if (fabs(p1.x - ref.x) > 1e-6 || fabs(p1.y - ref.y) > 1e-6)
            ROS_WARN("[穿越] leg2_case%d 末点(%.2f,%.2f) 与 case0 末点(%.2f,%.2f) 不一致，"
                     "投放区以 case0 末点为准！", cid, p1.x, p1.y, ref.x, ref.y);
    }

    // ---- 关键点位（场地系 -> odom 系绝对坐标）----
    Vec2f ho  = field_to_odom(scan_hover_fx_, scan_hover_fy_, origin_fx_, origin_fy_);
    hover_ox_ = ho.x;
    hover_oy_ = ho.y;
    Vec2f ep  = field_to_odom(via_leg2_[0].back().x, via_leg2_[0].back().y, origin_fx_, origin_fy_);
    end_x_    = ep.x;   // 投放区（各 case 末点一致性已在上面检查）
    end_y_    = ep.y;

    // ---- 启动预检：4 套 leg2 净距一览（正式规划在悬停扫描时按 case 做）----
    {
        bool case_ok[4];
        ROS_INFO("[穿越] 启动预检：4 套 leg2 绕柱段净距一览");
        for (int cid = 0; cid < 4; cid++) {
            TraversePlanner tp;
            tp.plan(via_leg2_[cid], origin_fx_, origin_fy_, walls_, caseCircles(cid),
                    cfg_.trav_v_max, cfg_.trav_a_max, cfg_.trav_a_lat_max, cfg_.trav_sample_ds);
            case_ok[cid] = (tp.min_clearance() >= cfg_.trav_inflation);
            ROS_INFO("[穿越]   case%d（%s）：总长 %.2f m，单程 %.1f s，最小净距 %.3f m %s",
                     cid, TRAV_CASE_DESC[cid], tp.total_length(), tp.duration(),
                     tp.min_clearance(), case_ok[cid] ? "✓" : "✗ 不达标！");
        }
        // default_case 是兜底，它挂了整个回退链就没了
        if (!case_ok[cfg_.default_case] && cfg_.trav_force_fly != 1) {
            ROS_ERROR("[穿越] default_case=%d 净距不达标，回退链失效，回退 EGO 路径！"
                      "请调整 map/via_points_leg2_case%d", cfg_.default_case, cfg_.default_case);
            return;
        }
        if (cfg_.force_case >= 0 && !case_ok[cfg_.force_case] && cfg_.trav_force_fly != 1) {
            ROS_ERROR("[穿越] force_case=%d 净距不达标，回退 EGO 路径！"
                      "请调整对应 via_points 或改 force_case", cfg_.force_case);
            return;
        }
    }

    traverse_cfg_ok_ = true;
    ROS_INFO("[穿越] 地图加载完成：悬停扫描点 odom(%.2f, %.2f)，投放区 odom(%.2f, %.2f)，定高 %.2f",
             hover_ox_, hover_oy_, end_x_, end_y_, cfg_.trav_flight_z);
}

// 组装某个 case 的圆形障碍（两根实际存在的柱子）
std::vector<CircleObs> MissionManager::caseCircles(int cid) const {
    std::vector<CircleObs> circles;
    for (int k = 0; k < 2; k++) {
        const Vec2f &c = pillar_cand_[TRAV_CASE_PILLARS[cid][k]];
        CircleObs co;
        co.x = c.x;
        co.y = c.y;
        co.r = pillar_radius_;
        circles.push_back(co);
    }
    return circles;
}

// leg2 规划报告打印
void MissionManager::printLeg2Report(int cid) const {
    ROS_INFO("╔══════════════════════════════════════════════════╗");
    ROS_INFO("║  leg2 绕柱段规划报告 case%d（%s）", cid, TRAV_CASE_DESC[cid]);
    ROS_INFO("╚══════════════════════════════════════════════════╝");
    const std::vector<Vec2f> &via = via_leg2_[cid];
    for (size_t i = 0; i < via.size(); i++) {
        Vec2f o = field_to_odom(via[i].x, via[i].y, origin_fx_, origin_fy_);
        ROS_INFO("  [%2zu] 场地(%5.2f, %5.2f) -> odom(%6.2f, %6.2f)", i, via[i].x, via[i].y, o.x, o.y);
    }
    ROS_INFO("  轨迹总长 %.2f m，单程时长 %.1f s，采样点 %zu 个",
             planner_leg2_.total_length(), planner_leg2_.duration(), planner_leg2_.point_count());
    ROS_INFO("  碰撞检测：最小净距 %.3f m @ 场地(%.2f, %.2f)，最近障碍：%s",
             planner_leg2_.min_clearance(), planner_leg2_.min_clearance_pos().x,
             planner_leg2_.min_clearance_pos().y, planner_leg2_.min_clearance_what().c_str());
    ROS_INFO("  膨胀要求：%.2f m -> %s", cfg_.trav_inflation,
             planner_leg2_.min_clearance() >= cfg_.trav_inflation ? "✓ 通过" : "✗ 不通过！");
}

// 按 case 规划 leg2；返回 true = 规划成功且净距达标
// （不达标时 planner_leg2_ 里仍持有该 case 的轨迹，供 force_fly 用）
bool MissionManager::planLeg2ForCase(int cid) {
    if (!planner_leg2_.plan(via_leg2_[cid], origin_fx_, origin_fy_, walls_, caseCircles(cid),
                            cfg_.trav_v_max, cfg_.trav_a_max, cfg_.trav_a_lat_max,
                            cfg_.trav_sample_ds)) {
        ROS_ERROR("[穿越] case%d 轨迹规划失败（途经点异常）！", cid);
        return false;
    }
    printLeg2Report(cid);
    return planner_leg2_.min_clearance() >= cfg_.trav_inflation;
}

// 悬停扫描拿到 case 后的规划入口（含净距回退）
// 先按 cid 规划，净距不达标回退 default_case；仍不达标看 force_fly。
// 成功返回 true 并把实际采用的 case 写入 active_case_；false = 全部失败（悬停等人工接管）
bool MissionManager::tryPlanLeg2(int cid) {
    if (planLeg2ForCase(cid)) {
        active_case_ = cid;
        return true;
    }
    if (cid != cfg_.default_case) {
        ROS_WARN("[穿越] case%d 净距不达标，回退 default_case=%d 重试", cid, cfg_.default_case);
        if (planLeg2ForCase(cfg_.default_case)) {
            active_case_ = cfg_.default_case;
            return true;
        }
    }
    if (cfg_.trav_force_fly == 1) {
        planLeg2ForCase(cid);   // 把 cid 的（不达标）轨迹重新装入 planner_leg2_
        active_case_ = cid;
        ROS_WARN("[穿越] force_fly=1，强行按 case%d 飞行（净距 %.3f < %.3f，危险！）",
                 cid, planner_leg2_.min_clearance(), cfg_.trav_inflation);
        return true;
    }
    return false;
}

// 轨迹跟踪（核心）：按航段经过的时间取轨迹点发位置设定点
//   reverse  false=正向(悬停点->投放区)，true=返程(时间倒放=原路返回)
//   goal_x/goal_y 到位判定目标（odom 系绝对坐标）
//   返回 true = 跟踪完成且已到达端点；超时也会返回 true 并告警
bool MissionManager::trackLeg(bool reverse, double goal_x, double goal_y, const char *label) {
    double t  = (ros::Time::now() - leg_start_time_).toSec();
    double T  = planner_leg2_.duration();
    double qt = reverse ? (T - t) : t;   // 返程 = 时间倒放，实现"原路返回"
    if (qt < 0.0) qt = 0.0;

    double sx, sy;
    planner_leg2_.sample(qt, sx, sy);

    // 位置控制掩码（忽略速度/加速度/yaw_rate，与 traverse_node 一致）
    current_setpoint_.type_mask        = TRAV_TYPE_MASK_POSITION_ONLY;
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.position.x       = sx;
    current_setpoint_.position.y       = sy;
    current_setpoint_.position.z       = cfg_.trav_flight_z;
    current_setpoint_.yaw              = init_yaw_;

    double cx = local_odom_.pose.pose.position.x;
    double cy = local_odom_.pose.pose.position.y;
    double cz = local_odom_.pose.pose.position.z;

    bool arrived = (fabs(cx - goal_x) < cfg_.trav_err_max &&
                    fabs(cy - goal_y) < cfg_.trav_err_max &&
                    fabs(cz - cfg_.trav_flight_z) < cfg_.trav_err_max);

    ROS_INFO_THROTTLE(0.5, "[跟踪%s] t=%.1f/%.1fs 设定(%.2f,%.2f) 当前(%.2f,%.2f,%.2f)",
                      label, t, T, sx, sy, cx, cy, cz);

    if (t >= T && arrived) return true;

    // 超时兜底：避免卡死（轨迹时长 + 余量仍未到位就进入下一状态）
    if (t > T + cfg_.trav_timeout_margin) {
        ROS_WARN("[跟踪%s] 超时(t=%.1f > %.1f+%.1f)，当前(%.2f,%.2f) 强制进入下一状态",
                 label, t, T, cfg_.trav_timeout_margin, cx, cy);
        return true;
    }
    return false;
}

// 绝对 odom 坐标定点（穿越段用，不走 init_pos 偏移；速度P控制，与 moveTo 同款）
bool MissionManager::moveToAbs(double x, double y, double z) {
    positionControl(Eigen::Vector3f(x, y, z), current_setpoint_);
    current_setpoint_.yaw = init_yaw_;
    return reachedTarget(Eigen::Vector3f(x, y, z), cfg_.err_max);
}