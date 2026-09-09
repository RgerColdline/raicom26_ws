#include "main_control.h"

// RETURN_CROSS_RING 返程穿环子状态
enum RingReturnSubState { RR_MOVE_TO_RING_FRONT, RR_CROSS_RING };

// ==================== 主函数 ====================
int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "mission_state_machine");
    ros::NodeHandle nh("~");

    // ========== 参数 + 穿越地图 ==========
    loadParameters(nh);
    loadTraverseConfig(nh);

    // 初始化设定点（速度控制，正式任务用）
    current_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint.type_mask        = TYPE_MASK_VELOCITY_ONLY;
    current_setpoint.velocity.x       = 0.0f;
    current_setpoint.velocity.y       = 0.0f;
    current_setpoint.velocity.z       = 0.0f;
    current_setpoint.yaw              = 0.0f;

    // ========== 订阅/发布话题 ==========
    initROSCommunication(nh);

    ros::Rate rate(20);

    // ========== 确认启动 ==========
    int choice = 0;
    std::cout << "\n====================" << std::endl;
    std::cout << "主控状态机节点（穿环 + 穿越绕柱 + 投货 + 定点射击）" << std::endl;
    std::cout << "====================" << std::endl;
    std::cout << "1: 开始任务" << std::endl;
    std::cout << "其他: 退出" << std::endl;
    std::cout << "====================\n" << std::endl;
    std::cin >> choice;
    if (choice != 1)
    {
        ROS_INFO("用户取消任务");
        return 0;
    }
    ros::spinOnce();
    rate.sleep();

    // ========== 等待飞控连接 ==========
    while (ros::ok() && !current_mav_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO_THROTTLE(1.0, "等待连接到飞控...");
    }
    ROS_INFO("✓ 已连接到飞控");

    // ========== 等待初始位置 ==========
    while (ros::ok() && !init_pos_received)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("✓ 位置数据已就绪");

    // ========== 等待视觉服务（可选） ==========
    if (cfg.wait_for_vision_services)
    {
        ROS_INFO("等待YOLO服务...");
        switch_camera_client.waitForExistence();
        reset_target_client.waitForExistence();
        ROS_INFO("YOLO服务已就绪");
    }
    else
    {
        ROS_WARN("跳过视觉服务等待（wait_for_vision_services=false）");
    }

    // ========== 初始化设定点（起飞位置控制） ==========
    current_setpoint.type_mask        = TYPE_MASK_TAKEOFF_POS;
    current_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint.position.x       = init_pos_x;
    current_setpoint.position.y       = init_pos_y;
    current_setpoint.position.z       = init_pos_z + cfg.takeoff_height;
    current_setpoint.yaw              = init_yaw;

    // ========== 发送设定点 ==========
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        setpoint_pub.publish(current_setpoint);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("✓ 设定点发送完成");

    // ========== 切换 OFFBOARD 模式并解锁 ==========
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // 安全保险：起飞前确保激光关闭
    std_msgs::Bool laser_off;
    laser_off.data = false;
    laser_control_pub.publish(laser_off);

    state_start_time = ros::Time::now();

    // ========== 等待进入任务（OFFBOARD + 解锁 + 起飞到 takeoff_height 悬停） ==========
    float target_z = init_pos_z + cfg.takeoff_height;
    while (ros::ok())
    {
        if (current_mav_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("✓ Offboard 模式已启用");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_mav_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
            {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("✓ 无人机已解锁");
                    mission_start_time = ros::Time::now();   // 任务计时起点（arm 成功）
                }
                last_request = ros::Time::now();
            }
        }

        // 稳定悬停在 (init_pos_x, init_pos_y, init_pos_z+takeoff_height) 后进入任务
        if (fabs(local_odom.pose.pose.position.z - target_z) < 0.2)
        {
            if (ros::Time::now() - last_request > ros::Duration(1.0))
            {
                if (pillar_nav_mode == "pcl" && traverse_cfg_ok) {
                    current_state = TRAVERSE_TO_SCAN;
                    ROS_INFO("进入穿越赛段：直接飞向悬停扫描点（途中穿环）");
                }
                else {
                    current_state = NAV_TO_DROP_AREA;
                    ROS_INFO("进入 EGO 回退路径（穿随机障碍物）");
                }
                nav_goal_sent    = false;
                state_start_time = ros::Time::now();
                ROS_INFO("\n========================================");
                ROS_INFO("=== 任务流程正式开始 ===");
                ROS_INFO("========================================\n");
                break;
            }
        }

        // 每帧发起飞设定点，防止 OFFBOARD 掉线
        current_setpoint.type_mask        = TYPE_MASK_TAKEOFF_POS;
        current_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint.position.x       = init_pos_x;
        current_setpoint.position.y       = init_pos_y;
        current_setpoint.position.z       = target_z;
        current_setpoint.yaw              = init_yaw;
        setpoint_pub.publish(current_setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    // ==================== 任务状态机 ====================
    while (ros::ok() && !mission_finished)
    {
        // 1. 默认安全设定点（速度控制）
        current_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_setpoint.type_mask        = TYPE_MASK_VELOCITY_ONLY;
        current_setpoint.velocity.x = current_setpoint.velocity.y = current_setpoint.velocity.z =
            0.0f;
        current_setpoint.yaw = current_yaw;

        // 2. 执行状态逻辑
        switch (current_state)
        {

        // ========== 状态: 穿环后飞到悬停扫描点 ==========
        case TRAVERSE_TO_SCAN:
        {
            // 【边飞边扫】起飞进入本状态即触发柱子检测（只发一次，多发会反复 reset 连续确认计数），
            // 无人机边穿环边扫，无需到悬停点才开始
            if (cfg.trav_early_scan == 1 && !scan_trigger_sent && cfg.force_case < 0)
            {
                std_msgs::Empty trig;
                pillar_start_pub.publish(trig);
                scan_trigger_sent = true;
                ROS_INFO("[穿越] 前移触发柱子检测：边飞边扫，扫到即切（不再等悬停点）");
            }

            // 【扫到即切】已检出 case 且已穿环（场地 x>2.05）：
            // 用当前位置替换 leg2 首点重规划，直接切入绕柱段，跳过悬停点
            if (cfg.trav_early_scan == 1 && detected_case >= 0)
            {
                double cur_fx = origin_fx - local_odom.pose.pose.position.x;
                double cur_fy = origin_fy - local_odom.pose.pose.position.y;
                if (cur_fx > 2.05)
                {
                    Vec2f cur_field{cur_fx, cur_fy};
                    if (tryPlanLeg2FromCurrent(detected_case, cur_field))
                    {
                        ROS_INFO("[穿越] ✓ 边飞边扫命中 case%d（%s），当前位置切入 leg2，跳过悬停点",
                                 active_case, TRAV_CASE_DESC[active_case]);
                        current_state    = TRAVERSE_LEG2;
                        nav_goal_sent    = false;
                        leg_start_time   = ros::Time::now();
                        state_start_time = ros::Time::now();
                        break;
                    }
                    ROS_WARN_THROTTLE(1.0, "[穿越] 当前位置切入 case%d 净距不达标，继续飞悬停点用完整途经点兜底",
                                      detected_case);
                }
            }

            if (!nav_goal_sent)
            {
                sendEgoGoal(hover_ox, hover_oy, cfg.trav_flight_z);
            }
            if (moveToAbs(hover_ox, hover_oy, cfg.trav_flight_z))
            {
                ROS_INFO("[穿越] ✓ 到达悬停扫描点 odom(%.2f, %.2f, %.2f)",
                         hover_ox, hover_oy, cfg.trav_flight_z);
                current_state    = TRAVERSE_SCAN;
                scan_sub_state   = 0;
                nav_goal_sent    = false;
                state_start_time = ros::Time::now();
            }
            else
            {
                ROS_INFO_THROTTLE(0.5, "[穿越] 飞向悬停扫描点 (%.2f, %.2f, %.2f)... 当前(%.2f,%.2f,%.2f)",
                                  hover_ox, hover_oy, cfg.trav_flight_z,
                                  local_odom.pose.pose.position.x, local_odom.pose.pose.position.y,
                                  local_odom.pose.pose.position.z);
            }
        }
        break;

        // ========== 状态: 悬停扫描选 case + 现场规划 leg2 ==========
        case TRAVERSE_SCAN:
        {
            moveToAbs(hover_ox, hover_oy, cfg.trav_flight_z);

            if (scan_sub_state == 0)
            {
                if (cfg.force_case >= 0)
                {
                    ROS_WARN("[穿越] force_case=%d，跳过检测与悬停，直接规划 leg2", cfg.force_case);
                    if (tryPlanLeg2(cfg.force_case))
                    {
                        ROS_INFO("[穿越] ✓ leg2 规划完成 case%d，开始绕柱段", active_case);
                        current_state    = TRAVERSE_LEG2;
                        scan_sub_state   = 0;
                        leg_start_time   = ros::Time::now();
                        state_start_time = ros::Time::now();
                    }
                    else
                    {
                        scan_sub_state = 9;
                    }
                }
                else
                {
                    std_msgs::Empty trig;
                    pillar_start_pub.publish(trig);
                    detected_case   = -1;
                    scan_entry_time = ros::Time::now();
                    scan_sub_state  = 1;
                    ROS_INFO("[穿越] 悬停扫描开始（%.1fs，超时 %.1fs 回退 case%d），已触发 pcl_detection2 模板匹配",
                             cfg.scan_hover_time, cfg.scan_timeout, cfg.default_case);
                }
            }
            else if (scan_sub_state == 1)
            {
                double elapsed = (ros::Time::now() - scan_entry_time).toSec();
                int decided = -1;
                if (detected_case >= 0)
                    decided = detected_case;  // 扫描到 case 立即穿越，不等满 scan_hover_time
                else if (elapsed >= cfg.scan_timeout)
                    decided = cfg.default_case;  // 超时回退

                if (decided >= 0)
                {
                    if (detected_case >= 0)
                        ROS_INFO("[穿越] ✓ 采用检测结果 case%d（扫描耗时 %.1fs）", decided, elapsed);
                    else
                        ROS_WARN("[穿越] 扫描超时(%.1fs)未检测到，回退 default_case=%d", elapsed, decided);

                    if (tryPlanLeg2(decided))
                    {
                        ROS_INFO("[穿越] ✓ leg2 规划完成 case%d（%s），开始绕柱段",
                                 active_case, TRAV_CASE_DESC[active_case]);
                        current_state    = TRAVERSE_LEG2;
                        scan_sub_state   = 0;
                        leg_start_time   = ros::Time::now();
                        state_start_time = ros::Time::now();
                    }
                    else
                    {
                        scan_sub_state = 9;
                    }
                }
                else
                {
                    ROS_INFO_THROTTLE(0.5, "[穿越] 悬停扫描中 %.1f/%.1fs（超时 %.1fs），detected=%d",
                                      elapsed, cfg.scan_hover_time, cfg.scan_timeout, detected_case);
                }
            }
            else
            {
                ROS_ERROR_THROTTLE(2.0, "[穿越] leg2 规划净距不达标且未开 force_fly，"
                                        "原地悬停，请遥控器接管或检查 via_points！");
            }
        }
        break;

        // ========== 状态: leg2 绕柱段轨迹跟踪（悬停点 -> 投放区） ==========
        case TRAVERSE_LEG2:
        {
            if (!nav_goal_sent)
            {
                sendEgoGoal(end_x, end_y, cfg.trav_flight_z);
            }
            if (trackLeg(false, end_x, end_y, "去程leg2"))
            {
                ROS_INFO("[穿越] ✓ 到达投放区 (%.2f, %.2f, %.2f)，进入悬停投货流程",
                         end_x, end_y, cfg.trav_flight_z);
                current_state      = HOVER_RECOG_DROP;
                drop_sub_state     = 0;
                drop_hover_start   = ros::Time(0);
                last_drop_pub_time = ros::Time(0);
                nav_goal_sent      = false;
                nav_status         = 0;
                state_start_time   = ros::Time::now();

                down_vote_a = 0;
                down_vote_b = 0;
                down_voting = true;
            }
        }
        break;

        // ========== 状态: 导航至物资投放区（EGO 回退路径） ==========
        case NAV_TO_DROP_AREA:
        {
            static bool come_mid_reached = false;

            Waypoint mid_target(init_pos_x + wp_come_mid.x, init_pos_y + wp_come_mid.y,
                                init_pos_z + wp_come_mid.z);
            Waypoint drop_target(init_pos_x + wp_drop_area.x, init_pos_y + wp_drop_area.y,
                                 init_pos_z + wp_drop_area.z);

            if (!come_mid_reached)
            {
                bool mid_arrived = (pillar_nav_mode == "pcl") ? moveTo(mid_target) : navTo(mid_target);
                if (mid_arrived)
                {
                    come_mid_reached = true;
                    ROS_INFO_STREAM("到达中途点，继续前往投放区");
                    nav_goal_sent = false;
                    nav_status    = 0;
                }
                break;
            }
            bool drop_arrived = (pillar_nav_mode == "pcl") ? moveTo(drop_target) : navTo(drop_target);
            if (drop_arrived)
            {
                current_state      = HOVER_RECOG_DROP;
                drop_sub_state     = 0;
                drop_hover_start   = ros::Time(0);
                last_drop_pub_time = ros::Time(0);
                nav_goal_sent      = false;
                nav_status         = 0;
                state_start_time   = ros::Time::now();

                down_vote_a = 0;
                down_vote_b = 0;
                down_voting = true;
                ROS_INFO("到达投放区，开始悬停投货流程 + 下视字母识别投票");
            }
        }
        break;

        // ========== 状态: 悬停识别投放区标识（下视字母投票） ==========
        case HOVER_RECOG_DROP:
        {
            moveToPositionVelocity(wp_drop_area);

            if (drop_hover_start.isZero())
            {
                drop_hover_start = ros::Time::now();
            }

            ROS_INFO_THROTTLE(0.5, "[投货-悬停] 当前(%.2f,%.2f,%.2f) 保持投放区, 剩余 %.1fs",
                              local_odom.pose.pose.position.x, local_odom.pose.pose.position.y,
                              local_odom.pose.pose.position.z,
                              cfg.drop_hover_time - (ros::Time::now() - drop_hover_start).toSec());

            if ((ros::Time::now() - drop_hover_start).toSec() > cfg.drop_hover_time)
            {
                down_voting = false;
                if (down_vote_a >= cfg.down_min_votes || down_vote_b >= cfg.down_min_votes)
                {
                    shoot_letter = (down_vote_a >= down_vote_b) ? "A" : "B";
                    ROS_INFO("[投货] ✓ 下视字母识别完成：%s（A=%d 票 B=%d 票）-> 射击 %s 靶",
                             shoot_letter.c_str(), down_vote_a, down_vote_b, shoot_letter.c_str());
                }
                else
                {
                    shoot_letter = cfg.attack_real_target;
                    ROS_WARN("[投货] ⚠ 下视字母票数不足（A=%d B=%d，需≥%d），回退兜底字母 %s",
                             down_vote_a, down_vote_b, cfg.down_min_votes, shoot_letter.c_str());
                }

                ROS_INFO("[投货] 悬停 %.1fs 完成，开始下降投货", cfg.drop_hover_time);
                drop_sub_state   = 1;
                state_start_time = ros::Time::now();
                current_state    = DROP_SUPPLY;
            }
        }
        break;

        // ========== 状态: 投放物资箱 ==========
        case DROP_SUPPLY:
        {
            const ros::Time now = ros::Time::now();

            if (drop_sub_state == 1)
            {
                // 【改 2026-09】悬停识别完成后直接在悬停高度(投放区 1.3m)投货，
                // 不再下降到 cfg.drop_z。
                bool at_drop = moveToPositionVelocity(wp_drop_area);

                if (at_drop)
                {
                    ROS_INFO("[投货] 在悬停高度(%.2f)直接触发投货", local_odom.pose.pose.position.z);

                    // 开舱指令第 1/3 次（t=0s），后续 0.2s/0.4s 各补发一次
                    std_msgs::UInt8 servo_msg;
                    servo_msg.data = cfg.cargo_drop_angle;
                    servo_control_pub.publish(servo_msg);
                    drop_pub_count = 1;
                    ROS_INFO("[投货] 开舱指令 1/3 (角度 %d -> 0x03 -> 货舱打开)",
                             cfg.cargo_drop_angle);

                    drop_sub_state     = 2;
                    state_start_time   = now;
                    last_drop_pub_time = now;
                }
                break;
            }

            if (drop_sub_state == 2)
            {
                moveToPositionVelocity(wp_drop_area);

                // 开舱指令第 2/3 次（t=0.2s）、第 3/3 次（t=0.4s）：固定节奏共发 3 次
                double t_drop = (now - state_start_time).toSec();
                if (drop_pub_count < 3 && t_drop >= 0.2 * drop_pub_count)
                {
                    std_msgs::UInt8 servo_msg;
                    servo_msg.data = cfg.cargo_drop_angle;
                    servo_control_pub.publish(servo_msg);
                    ++drop_pub_count;
                    ROS_INFO("[投货] 开舱指令 %d/3 (t=%.1fs, 角度 %d -> 0x03)",
                             drop_pub_count, t_drop, cfg.cargo_drop_angle);
                    last_drop_pub_time = now;
                }

                // 3 次开舱发完即关舱，直接进入射击流程（不再等 cargo_hold_time）
                if (drop_pub_count >= 3 && t_drop >= 0.5)
                {
                    std_msgs::UInt8 reset_msg;
                    reset_msg.data = cfg.cargo_reset_angle;
                    for (int i = 0; i < 3; ++i)
                        servo_control_pub.publish(reset_msg);
                    ROS_INFO("[投货] 3 次开舱完成(0/0.2/0.4s)，货舱复位(角度 %d -> 0x04 -> 关闭) x3，"
                             "直接进入射击流程",
                             cfg.cargo_reset_angle);

                    drop_sub_state   = 0;
                    current_state    = RECOG_ATTACK_TARGET;
                    nav_goal_sent    = false;
                    state_start_time = now;
                }
            }
        }
        break;

        // ========== 状态: 识别正确攻击目标（固定映射选射击点） ==========
        case RECOG_ATTACK_TARGET:
        {
            bool letter_is_a = (shoot_letter != "B");
            bool go_left     = (letter_is_a == a_on_left);

            shoot_target_x = go_left ? cfg.shoot_left_x : cfg.shoot_right_x;
            shoot_target_y = go_left ? cfg.shoot_left_y : cfg.shoot_right_y;

            ROS_INFO("[识别] 投货识别字母 = %s -> 射击 %s 靶（%s射击点 %.2f, %.2f）",
                     shoot_letter.c_str(), letter_is_a ? "A" : "B",
                     go_left ? "左" : "右", shoot_target_x, shoot_target_y);

            current_state    = ALIGN_ATTACK_TARGET;
            state_start_time = ros::Time::now();
        }
        break;


        // ========== 状态: 前视像素对准目标 ==========
        case ALIGN_ATTACK_TARGET:
        {
            static bool arrived = false;

            if (moveToPositionVelocity(shoot_target_x, shoot_target_y, cfg.shoot_z))
            {
                if (!arrived)
                {
                    arrived           = true;
                    state_start_time  = ros::Time::now();
                    ROS_INFO("[射击] 到达射击点(%.2f, %.2f, %.2f)，稳定 %.1fs 后射击",
                             shoot_target_x, shoot_target_y, cfg.shoot_z, cfg.shoot_stable_time);
                }

                if (timeout(cfg.shoot_stable_time))
                {
                    arrived           = false;
                    shoot_triggered   = false;
                    current_state     = SIMULATE_ATTACK;
                    state_start_time  = ros::Time::now();
                }
            }
            else
            {
                arrived = false;
                ROS_INFO_THROTTLE(0.5, "[射击] 飞向射击点(%.2f, %.2f, %.2f)...",
                                  shoot_target_x, shoot_target_y, cfg.shoot_z);
            }
        }
        break;

        // ========== 状态: 激光指示攻击 ==========
        case SIMULATE_ATTACK:
        {
            moveToPositionVelocity(shoot_target_x, shoot_target_y, cfg.shoot_z);

            if (!shoot_triggered)
            {
                std_msgs::Bool laser_off;
                laser_off.data = false;
                laser_control_pub.publish(laser_off);

                std_msgs::Empty shoot_msg;
                shoot_pub.publish(shoot_msg);

                ROS_INFO("╔════════════════════════════════════════╗");
                ROS_INFO("║          ★★★ 射击！ ★★★            ║");
                ROS_INFO("║  射击坐标: (%.3f, %.3f, %.3f)",
                         local_odom.pose.pose.position.x, local_odom.pose.pose.position.y,
                         local_odom.pose.pose.position.z);
                ROS_INFO("║  投货识别字母: %s -> 射击 %s 靶", shoot_letter.c_str(),
                         (shoot_letter != "B") ? "A" : "B");
                ROS_INFO("╚════════════════════════════════════════╝");

                shoot_triggered = true;
                shoot_time      = ros::Time::now();
            }

            if ((ros::Time::now() - shoot_time).toSec() > cfg.shoot_duration)
            {
                if (pillar_nav_mode == "pcl" && traverse_cfg_ok && cfg.trav_return_smooth == 1 &&
                    planReturnFromCurrent())
                {
                    // 返程直通：射击点 -> 倒放绕柱 -> 穿环 -> 起飞点，单条平滑轨迹中途不停顿
                    current_state  = TRAVERSE_RETURN_HOME;
                    leg_start_time = ros::Time::now();
                    ROS_INFO("[射击] 射击完成，返程直通轨迹已规划（不回悬停点、穿环不停顿）");
                }
                else if (pillar_nav_mode == "pcl" && traverse_cfg_ok) {
                    current_state  = TRAVERSE_RETURN_LEG2;  // 回退：分段返程（倒放leg2停悬停点再穿环）
                    leg_start_time = ros::Time::now();
                } else {
                    current_state = READY_NAV_TO_RING_BACK;
                }
                state_start_time = ros::Time::now();
                ROS_INFO("[射击] 射击完成，开始返程");
            }
        }
        break;

        // ========== 状态: 返程直通（当前位置 -> 倒放绕柱 -> 穿环 -> 起飞点，单条轨迹不停顿） ==========
        case TRAVERSE_RETURN_HOME:
        {
            if (!nav_goal_sent)
            {
                sendEgoGoal(init_pos_x, init_pos_y, init_pos_z + cfg.takeoff_height);
            }
            // z_end=return_land_z：过环后边飞边降到低高度，到起飞点时已接近落地高度
            if (trackPlan(planner_return, false, init_pos_x, init_pos_y, "返程直通",
                          cfg.trav_return_land_z))
            {
                ROS_INFO("[穿越] ✓ 返程直通完成（穿环不停顿+末段降高），已到起飞点，直接降落");
                current_state    = LAND;   // 轨迹终点即起飞点（无需 RETURN 精修），直接进降落
                nav_goal_sent    = false;
                state_start_time = ros::Time::now();
            }
        }
        break;

        // ========== 状态: 返程 leg2 时间倒放（投放区 -> 悬停扫描点）【旧回退路径】 ==========
        case TRAVERSE_RETURN_LEG2:
        {
            if (!nav_goal_sent)
            {
                sendEgoGoal(hover_ox, hover_oy, cfg.trav_flight_z);
            }
            if (trackLeg(true, hover_ox, hover_oy, "返程leg2"))
            {
                ROS_INFO("[穿越] ✓ 回到悬停扫描点，准备穿环返回");
                current_state    = RETURN_CROSS_RING;
                nav_goal_sent    = false;
                state_start_time = ros::Time::now();
            }
        }
        break;

        // ========== 状态: 返程前先回到投放区 ==========
        case READY_NAV_TO_RING_BACK:
        {
            Waypoint forward_target(init_pos_x + wp_drop_area.x, init_pos_y + wp_drop_area.y,
                                    init_pos_z + wp_drop_area.z);
            if (moveTo(forward_target.x, forward_target.y, forward_target.z))
            {
                current_state    = NAV_TO_RING_BACK;
                nav_goal_sent    = false;
                state_start_time = ros::Time::now();
                ROS_INFO("到达投放区，开始返程");
            }
        }
        break;

        // ========== 状态: 返程导航（EGO 回退路径） ==========
        case NAV_TO_RING_BACK:
        {
            static bool back_mid_reached = false;

            Waypoint mid_target(init_pos_x + wp_back_mid.x, init_pos_y + wp_back_mid.y,
                                init_pos_z + wp_come_mid.z);

            Waypoint ring_back(init_pos_x + wp_ring_back.x, init_pos_y + wp_ring_back.y,
                               init_pos_z + wp_ring_back.z);

            if (!back_mid_reached)
            {
                if (navTo(mid_target))
                {
                    back_mid_reached  = true;
                    nav_goal_sent     = false;
                    state_start_time  = ros::Time::now();
                    nav_status        = 0;
                    ROS_INFO_STREAM("到达中途点，继续返程");
                }
                break;
            }
            if (navTo(ring_back))
            {
                current_state    = RETURN_CROSS_RING;
                nav_goal_sent    = false;
                state_start_time = ros::Time::now();
                ROS_INFO_STREAM("已通过ego_planner穿过随机障碍物，准备返回穿环");
            }
        }
        break;

        // ========== 状态: 返回穿环（固定航点，垂直穿过环） ==========
        case RETURN_CROSS_RING:
        {
            static RingReturnSubState sub_state = RR_MOVE_TO_RING_FRONT;

            switch (sub_state)
            {
            case RR_MOVE_TO_RING_FRONT:
            {
                // Phase1: 先飞到环后方（y=0 中心线，正对环孔，给垂直穿环留出对位余量）
                if (!nav_goal_sent)
                {
                    sendEgoGoal(init_pos_x + wp_ring_back.x, init_pos_y + wp_ring_back.y,
                                init_pos_z + wp_ring_back.z);
                }
                if (moveTo(wp_ring_back))
                {
                    sub_state         = RR_CROSS_RING;
                    nav_goal_sent     = false;
                    state_start_time  = ros::Time::now();
                    ROS_INFO_STREAM("到达环后方，准备垂直穿环");
                }
                break;
            }
            case RR_CROSS_RING:
            default:
            {
                // Phase2: 沿 y=0 直线垂直穿过环孔，到环前方
                if (!nav_goal_sent)
                {
                    sendEgoGoal(init_pos_x + wp_ring_front.x, init_pos_y + wp_ring_front.y,
                                init_pos_z + wp_ring_front.z);
                }
                if (moveTo(wp_ring_front))
                {
                    current_state      = RETURN;
                    nav_goal_sent      = false;
                    state_start_time   = ros::Time::now();
                    ROS_INFO_STREAM("已垂直穿环，正在返回起飞点上方");
                }
                break;
            }
            }
        }
        break;

        // ========== 状态: 返回起飞点 ==========
        case RETURN:
        {
            if (!nav_goal_sent)
            {
                sendEgoGoal(init_pos_x, init_pos_y, init_pos_z + cfg.takeoff_height);
            }
            if (moveTo(init_pos_x, init_pos_y, init_pos_z + cfg.takeoff_height))
            {
                nav_goal_sent    = false;
                state_start_time = ros::Time::now();
                current_state    = LAND;
                ROS_INFO("已返回起飞点上方，开始降落");
            }
        }
        break;

        // ========== 状态: 降落 ==========
        case LAND:
        {
            const float current_z = local_odom.pose.pose.position.z;
            const float ground_z  = init_pos_z;

            if (current_z > ground_z + 0.3f)
            {
                positionControl(Eigen::Vector3f(init_pos_x, init_pos_y, current_z), current_setpoint);
                current_setpoint.velocity.z = -cfg.land_descend_speed;
                current_setpoint.yaw        = init_yaw;
                ROS_INFO_THROTTLE(0.5, "[降落] 定点下降中, 离地高度: %.2f m", current_z - ground_z);
                break;
            }

            static bool auto_land_sent = false;
            if (!auto_land_sent)
            {
                mavros_msgs::SetMode srv;
                srv.request.custom_mode = "AUTO.LAND";
                if (set_mode_client.call(srv) && srv.response.mode_sent)
                {
                    ROS_INFO("[降落] 高度 < 0.3m，AUTO.LAND 请求成功");
                    auto_land_sent    = true;
                    state_start_time  = ros::Time::now();
                }
                else
                {
                    ROS_WARN_THROTTLE(1.0, "[降落] 切换 AUTO.LAND 失败，重试中...");
                }
                break;
            }

            ROS_INFO("[降落] 降落完成，任务结束");
            auto_land_sent    = false;
            current_state     = TASK_END;
            state_start_time  = ros::Time::now();
        }
        break;

        // ========== 状态: 任务结束 ==========
        case TASK_END:
        {
            current_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            current_setpoint.type_mask        = TYPE_MASK_TAKEOFF_POS;
            current_setpoint.position.x       = init_pos_x;
            current_setpoint.position.y       = init_pos_y;
            current_setpoint.position.z       = init_pos_z;
            current_setpoint.yaw              = init_yaw;
            mission_finished                  = true;

            // 任务计时：arm 解锁成功 -> 降落完毕，目标 50s 内
            double t_mission = (ros::Time::now() - mission_start_time).toSec();
            ROS_INFO("╔══════════════════════════════════════╗");
            ROS_INFO("║          ★ 任务计时 ★");
            ROS_INFO("║  任务时长（arm→降落完成）: %.1f s", t_mission);
            ROS_INFO("║  50s 目标: %s", (t_mission <= 50.0) ? "✓ 达标" : "✗ 未达标");
            ROS_INFO("╚══════════════════════════════════════╝");
            ROS_INFO("任务完成，节点退出");
        }
        break;

        default:
            break;
        }

        // 3. 发布设定点（EGO 正常模式由 ego_controller_node 接管；PCL/影子模式继续发）
        if (!nav_goal_sent || pillar_nav_mode == "pcl")
        {
            sendSetpoint(current_setpoint);
        }

        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}
