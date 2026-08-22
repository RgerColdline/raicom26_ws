// ============================================================
// 文件: main_control.h
// 作者: 肚腩特大哥
// 日期: 2026-08
// 功能: RAICOM 无人机竞赛主控节点 —— 头文件（全局变量 + 函数声明/实现）
//   架构说明：与 raicom_vision_laser/include/mission_flow.h 结构一致，
//   只放全局变量、回调、控制辅助函数、穿越规划器等，不含 main 与状态机。
//   状态机全部展开在 src/main_control.cpp 的 main() 里。
// ============================================================

#ifndef MAIN_CONTROL_H
#define MAIN_CONTROL_H

// ==================== 依赖头文件 ====================
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <locale.h>
#include <sstream>
#include <string>
#include <vector>

// 自定义消息
#include <raicom_vision_laser/DetectionInfo.h>

using namespace std;

// ==================== case -> 候选柱索引映射（全队统一，勿单独改） ====================
const int TRAV_CASE_PILLARS[4][2] = {{0, 2}, {0, 3}, {1, 2}, {1, 3}};
const char* TRAV_CASE_DESC[4] = {
    "A左+B左（两柱都在 x=2.7）",
    "A左+B右（(2.7,1.55)+(3.3,2.8)）",
    "A右+B左（(3.3,1.55)+(2.7,2.8)）",
    "A右+B右（两柱都在 x=3.3）"
};

// ==================== 控制掩码常量 ====================
// 速度控制掩码：忽略 PX/PY/PZ + 加速度 + yaw_rate，用 vx/vy/vz + yaw（positionControl 用）
const uint16_t TYPE_MASK_VELOCITY_ONLY = 0b100111000111;
// 起飞位置控制掩码：忽略 vx/vy/vz + 加速度 + FORCE + yaw_rate，用 x/y/z + yaw
const uint16_t TYPE_MASK_TAKEOFF_POS = 0b101111111000;
// 穿越段轨迹跟踪位置控制掩码：忽略 vx/vy/vz/afx/afy/afz + IGNORE_YAW_RATE=2048（不带 512）
const uint16_t TRAV_TYPE_MASK_POSITION_ONLY = 8 + 16 + 32 + 64 + 128 + 256 + 2048;

// ==================== 状态机枚举 ====================
enum MissionState {
    NAV_TO_DROP_AREA,
    HOVER_RECOG_DROP,
    DROP_SUPPLY,
    MOVE_TO_ATTACK_AREA,
    RECOG_ATTACK_TARGET,
    MOVE_TO_FRONT_OF_TARGET,
    ALIGN_ATTACK_TARGET,
    SIMULATE_ATTACK,

    NAV_TO_RING_BACK,
    READY_NAV_TO_RING_BACK,
    RETURN_CROSS_RING,

    // === 穿越赛段（pillar_nav_mode="pcl" 时替换 EGO 路径） ===
    TRAVERSE_TO_SCAN,
    TRAVERSE_SCAN,
    TRAVERSE_LEG2,
    TRAVERSE_RETURN_LEG2,

    RETURN,
    LAND,
    TASK_END
};

// ==================== 基础数据结构 ====================
struct Vec2f { double x, y; };                 // 二维点
struct CircleObs { double x, y, r; };          // 圆形障碍（圆柱，r=实际半径，未膨胀）
struct SegObs { double x1, y1, x2, y2; };      // 线段障碍（墙 / 场地边界）
struct TrajPoint { double t, x, y; };          // 时间参数化轨迹点（odom 系）

struct Waypoint
{
    float x, y, z;
    Waypoint(float x = 0.0f, float y = 0.0f, float z = 0.0f) : x(x), y(y), z(z) {}
};

// ==================== 参数配置 ====================
struct Config
{
    float takeoff_height;
    float max_speed;
    float max_yaw_rate;
    float err_max;
    float hover_vert_tolerance;
    float p_xy, p_z;
    float hover_time_needed;
    float target_front_offset_x;
    float target_front_offset_y;
    float nav_goal_timeout;
    float align_pixel_threshold;
    float shoot_delay;

    float PIX_VEL_P, PIX_VEL_I, PIX_VEL_D;
    float PIX_VEL_MAX;
    float PIX_INTEGRAL_MAX;
    float PIX_FAR_NORM_DIST;

    float detection_min_confidence;
    std::string attack_real_target;
    bool wait_for_vision_services = true;

    float drop_arrive_threshold;
    float drop_detect_timeout;
    float drop_align_hold_time;
    float drop_camera_bias_x_px;
    float drop_camera_bias_y_px;
    float drop_release_bias_x_px;
    float drop_release_bias_y_px;
    float drop_fine_pixel_radius;
    float drop_fine_vel_scale;
    float drop_descend_distance;

    float land_descend_speed         = 0.3f;
    bool use_ego_planner_for_drop_area;

    float shoot_left_x      = -0.60f;
    float shoot_left_y      = -2.5f;
    float shoot_right_x     = -0.60f;
    float shoot_right_y     = -1.8f;
    float shoot_default_x   = -0.60f;
    float shoot_default_y   = -2.15f;
    float shoot_left_right_threshold = 20.0f;
    float shoot_detect_timeout       = 60.0f;
    float shoot_stable_time          = 0.5f;
    float shoot_duration             = 1.5f;
    float shoot_z                    = 1.0f;

    int   cargo_drop_angle           = 0;
    int   cargo_reset_angle          = 180;
    float cargo_hold_time            = 2.0f;
    float descent_timeout            = 10.0f;
    float drop_hover_time            = 2.0f;
    float drop_z                     = 0.8f;

    float yolo_img_center_x          = 160.0f;
    float yolo_target_timeout        = 0.5f;
    float yolo_detect_timeout        = 60.0f;

    int   down_min_votes             = 3;
    std::string shoot_a_side         = "left";

    float trav_flight_z        = 1.3f;
    float trav_err_max         = 0.15f;
    double trav_v_max          = 0.5;
    double trav_a_max          = 0.4;
    double trav_a_lat_max      = 0.6;
    double trav_inflation      = 0.3;
    double trav_sample_ds      = 0.01;
    int trav_force_fly         = 0;
    float trav_timeout_margin  = 15.0f;
    float scan_hover_time      = 3.0f;
    float scan_timeout         = 4.0f;
    int default_case           = 1;
    int force_case             = -1;
} cfg;

// ==================== ROS 通信 ====================
ros::Publisher setpoint_pub;
ros::Publisher ego_goal_pub;
ros::Publisher servo_control_pub;
ros::Publisher shoot_pub;
ros::Publisher laser_control_pub;
ros::Subscriber state_sub, odom_sub;
ros::Subscriber nav_status_sub;
ros::Subscriber detected_target_sub;
ros::Subscriber yolo_detect_sub;
ros::Subscriber yolo_down_detect_sub;
ros::Subscriber pillar_sub;
ros::Publisher pillar_start_pub;

ros::ServiceClient switch_camera_client;
ros::ServiceClient reset_target_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient arming_client;

// ==================== 状态机数据 ====================
MissionState current_state = TRAVERSE_TO_SCAN;
ros::Time state_start_time;
bool init_pos_received = false;
bool mission_finished  = false;

// ==================== 无人机状态 ====================
mavros_msgs::State current_mav_state;
nav_msgs::Odometry local_odom;
double current_yaw   = 0.0;
double current_roll  = 0.0;
double current_pitch = 0.0;
float init_pos_x = 0.0f, init_pos_y = 0.0f, init_pos_z = 0.0f;
double init_yaw = 0.0;

// ==================== 导航状态 ====================
int8_t nav_status            = 0;
bool nav_goal_sent           = false;
bool nav_seen_executing      = false;

// ==================== 视觉识别数据 ====================
std::string confirmed_target;
bool target_confirmed    = false;
bool front_target_matched = false;
std::string matched_target;
float matched_center_x = 0.0f;
float matched_center_y = 0.0f;
ros::Time last_matched_time;

// ==================== 下视字母识别（投货悬停投票） ====================
std::string shoot_letter = "A";
bool down_voting  = false;
int  down_vote_a  = 0;
int  down_vote_b  = 0;
bool a_on_left    = true;

// ==================== 投货子状态 ====================
int       drop_sub_state   = 0;
ros::Time last_drop_pub_time;
ros::Time drop_hover_start;

// ==================== 射击状态 ====================
bool      shoot_triggered = false;
ros::Time shoot_time;
float     shoot_target_x = 0.0f;
float     shoot_target_y = 0.0f;

// ==================== 穿越赛段 ====================
std::string pillar_nav_mode;
int detected_case   = -1;
int active_case     = -1;
int scan_sub_state  = 0;
ros::Time scan_entry_time;
ros::Time leg_start_time;
bool traverse_cfg_ok = false;

double hover_ox = 0, hover_oy = 0;
double end_x = 0, end_y = 0;
double origin_fx = 0.65, origin_fy = 0.75;
double scan_hover_fx = 3.00, scan_hover_fy = 0.75;
double pillar_radius = 0.1;
std::vector<Vec2f> pillar_cand;
std::vector<Vec2f> via_leg2[4];
std::vector<SegObs> walls;

// ==================== PID / 控制 ====================
ros::Time last_pid_control_time;
Eigen::Vector3f attack_target_world;
mavros_msgs::PositionTarget current_setpoint;

Waypoint wp_ring_front;
Waypoint wp_ring_back;
Waypoint wp_come_mid;
Waypoint wp_back_mid;
Waypoint wp_pillar_center;
Waypoint wp_drop_area;
Waypoint wp_attack_area;

// ==================== 自然三次样条（二维，参数=累积弦长） ====================
struct Spline2D
{
    std::vector<double> u;
    std::vector<double> x, y;
    std::vector<double> mx, my;
};

// ==================== 穿越轨迹规划结果 ====================
struct TraversePlanResult
{
    std::vector<TrajPoint> traj;
    double total_length = 0.0;
    double min_clearance = 1e9;
    Vec2f min_clear_pos{0.0, 0.0};
    std::string min_clear_what;
};

TraversePlanResult planner_leg2;

// ==================== 函数声明 ====================
// 初始化
void loadParameters(ros::NodeHandle &nh);
void loadTraverseConfig(ros::NodeHandle &nh);
void initROSCommunication(ros::NodeHandle &nh);

// 回调
void stateCallback(const mavros_msgs::State::ConstPtr &msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
void navStatusCallback(const std_msgs::Int8::ConstPtr &msg);
void detectedTargetCallback(const std_msgs::String::ConstPtr &msg);
void yoloDetectCallback(const raicom_vision_laser::DetectionInfo::ConstPtr &msg);
void yoloDownDetectCallback(const raicom_vision_laser::DetectionInfo::ConstPtr &msg);
void pillarDetectCallback(const std_msgs::Int32::ConstPtr &msg);

// 控制辅助
void sendSetpoint(const mavros_msgs::PositionTarget &sp);
void sendEgoGoal(float x, float y, float z, float yaw = NAN);
bool waitForNavArrival();
void positionControl(const Eigen::Vector3f &target_pos, mavros_msgs::PositionTarget &sp);
bool reachedTarget(const Eigen::Vector3f &target, float dist_thresh);
bool navTo(const float x, const float y, const float z);
bool navTo(const Waypoint wp);
bool moveTo(const float x, const float y, const float z);
bool moveTo(const Waypoint wp);
bool moveToAbs(double x, double y, double z);
void hover();
bool timeout(const float timeout_limit);

// 穿越赛段
std::vector<CircleObs> caseCircles(int cid);
bool planLeg2ForCase(int cid);
bool tryPlanLeg2(int cid);
void printLeg2Report(int cid);
bool trackLeg(bool reverse, double goal_x, double goal_y, const char *label);

// ==================== 样条 / 规划器辅助函数实现 ====================
inline Vec2f field_to_odom(double fx, double fy, double origin_fx, double origin_fy)
{
    Vec2f p;
    p.x = origin_fx - fx;
    p.y = origin_fy - fy;
    return p;
}

inline double dist_point_seg(double px, double py, const SegObs& sg)
{
    double vx = sg.x2 - sg.x1, vy = sg.y2 - sg.y1;
    double wx = px - sg.x1, wy = py - sg.y1;
    double len2 = vx * vx + vy * vy;
    double t = (len2 > 1e-12) ? (wx * vx + wy * vy) / len2 : 0.0;
    t = std::max(0.0, std::min(1.0, t));
    double cx = sg.x1 + t * vx, cy = sg.y1 + t * vy;
    return std::hypot(px - cx, py - cy);
}

static std::vector<double> spline_solve_second_deriv(const std::vector<double>& u,
                                                     const std::vector<double>& v)
{
    int N = (int)u.size();
    std::vector<double> M(N, 0.0);
    if (N < 3) return M;

    int m = N - 2;
    std::vector<double> a(m), b(m), c(m), d(m);
    for (int i = 1; i <= N - 2; i++)
    {
        double h_prev = u[i] - u[i - 1];
        double h_next = u[i + 1] - u[i];
        int k = i - 1;
        a[k] = h_prev;
        b[k] = 2.0 * (h_prev + h_next);
        c[k] = h_next;
        d[k] = 6.0 * ((v[i + 1] - v[i]) / h_next - (v[i] - v[i - 1]) / h_prev);
    }
    for (int k = 1; k < m; k++)
    {
        double w = a[k] / b[k - 1];
        b[k] -= w * c[k - 1];
        d[k] -= w * d[k - 1];
    }
    std::vector<double> x(m);
    x[m - 1] = d[m - 1] / b[m - 1];
    for (int k = m - 2; k >= 0; k--)
        x[k] = (d[k] - c[k] * x[k + 1]) / b[k];
    for (int k = 0; k < m; k++)
        M[k + 1] = x[k];
    return M;
}

static int spline_find_seg(const Spline2D& sp, double u)
{
    int N = (int)sp.u.size();
    if (u <= sp.u.front()) return 0;
    if (u >= sp.u.back()) return N - 2;
    int lo = 0, hi = N - 1;
    while (lo + 1 < hi)
    {
        int mid = (lo + hi) / 2;
        if (sp.u[mid] <= u) lo = mid; else hi = mid;
    }
    return lo;
}

bool spline_build(Spline2D& sp, const std::vector<Vec2f>& pts)
{
    if (pts.size() < 2) return false;
    int N = (int)pts.size();
    sp.u.assign(N, 0.0);
    sp.x.resize(N);
    sp.y.resize(N);
    for (int i = 0; i < N; i++)
    {
        sp.x[i] = pts[i].x;
        sp.y[i] = pts[i].y;
    }
    for (int i = 1; i < N; i++)
        sp.u[i] = sp.u[i - 1] + std::hypot(sp.x[i] - sp.x[i - 1], sp.y[i] - sp.y[i - 1]);
    sp.mx = spline_solve_second_deriv(sp.u, sp.x);
    sp.my = spline_solve_second_deriv(sp.u, sp.y);
    return true;
}

Vec2f spline_eval(const Spline2D& sp, double u)
{
    int i = spline_find_seg(sp, u);
    double h = sp.u[i + 1] - sp.u[i];
    double A = (sp.u[i + 1] - u) / h;
    double B = (u - sp.u[i]) / h;
    Vec2f p;
    p.x = A * sp.x[i] + B * sp.x[i + 1]
        + ((A * A * A - A) * sp.mx[i] + (B * B * B - B) * sp.mx[i + 1]) * h * h / 6.0;
    p.y = A * sp.y[i] + B * sp.y[i + 1]
        + ((A * A * A - A) * sp.my[i] + (B * B * B - B) * sp.my[i + 1]) * h * h / 6.0;
    return p;
}

void spline_eval_deriv(const Spline2D& sp, double u, double& dx, double& dy, double& ddx, double& ddy)
{
    int i = spline_find_seg(sp, u);
    double h = sp.u[i + 1] - sp.u[i];
    double A = (sp.u[i + 1] - u) / h;
    double B = (u - sp.u[i]) / h;
    dx = (sp.x[i + 1] - sp.x[i]) / h
       + ((-3.0 * A * A + 1.0) * sp.mx[i] + (3.0 * B * B - 1.0) * sp.mx[i + 1]) * h / 6.0;
    dy = (sp.y[i + 1] - sp.y[i]) / h
       + ((-3.0 * A * A + 1.0) * sp.my[i] + (3.0 * B * B - 1.0) * sp.my[i + 1]) * h / 6.0;
    ddx = A * sp.mx[i] + B * sp.mx[i + 1];
    ddy = A * sp.my[i] + B * sp.my[i + 1];
}

double spline_length_param(const Spline2D& sp) { return sp.u.empty() ? 0.0 : sp.u.back(); }

static double clearance_at(const Vec2f& pt, const std::vector<SegObs>& walls,
                           const std::vector<CircleObs>& circles, std::string& what)
{
    double best = 1e9;
    what = "无";
    for (size_t i = 0; i < walls.size(); i++)
    {
        double d = dist_point_seg(pt.x, pt.y, walls[i]);
        if (d < best)
        {
            best = d;
            std::ostringstream os;
            os << "墙#" << i << " (" << walls[i].x1 << "," << walls[i].y1
               << ")-(" << walls[i].x2 << "," << walls[i].y2 << ")";
            what = os.str();
        }
    }
    for (size_t i = 0; i < circles.size(); i++)
    {
        double d = std::hypot(pt.x - circles[i].x, pt.y - circles[i].y) - circles[i].r;
        if (d < best)
        {
            best = d;
            std::ostringstream os;
            os << "圆柱#" << i << " (" << circles[i].x << "," << circles[i].y << ")";
            what = os.str();
        }
    }
    return best;
}

bool traverse_plan(TraversePlanResult& out,
                   const std::vector<Vec2f>& field_pts,
                   double origin_fx, double origin_fy,
                   const std::vector<SegObs>& walls,
                   const std::vector<CircleObs>& circles,
                   double v_max, double a_max, double a_lat_max, double ds)
{
    if (field_pts.size() < 2) return false;

    Spline2D spline;
    if (!spline_build(spline, field_pts)) return false;

    double U = spline_length_param(spline);
    int N = std::max(2, (int)std::ceil(U / ds));
    double du = U / N;

    std::vector<Vec2f> p(N + 1);
    std::vector<double> s(N + 1, 0.0), kap(N + 1, 0.0);
    for (int k = 0; k <= N; k++)
        p[k] = spline_eval(spline, k * du);
    for (int k = 1; k <= N; k++)
        s[k] = s[k - 1] + std::hypot(p[k].x - p[k - 1].x, p[k].y - p[k - 1].y);
    for (int k = 0; k <= N; k++)
    {
        double dx, dy, ddx, ddy;
        spline_eval_deriv(spline, k * du, dx, dy, ddx, ddy);
        double denom = std::pow(dx * dx + dy * dy, 1.5);
        kap[k] = (denom > 1e-9) ? std::fabs(dx * ddy - dy * ddx) / denom : 0.0;
    }

    out.min_clearance = 1e9;
    for (int k = 0; k <= N; k++)
    {
        std::string what;
        double c = clearance_at(p[k], walls, circles, what);
        if (c < out.min_clearance)
        {
            out.min_clearance = c;
            out.min_clear_pos = p[k];
            out.min_clear_what = what;
        }
    }

    std::vector<double> v(N + 1);
    for (int k = 0; k <= N; k++)
        v[k] = std::min(v_max, std::sqrt(a_lat_max / std::max(kap[k], 1e-6)));
    v[0] = 0.0;
    v[N] = 0.0;
    for (int k = 1; k <= N; k++)
        v[k] = std::min(v[k], std::sqrt(v[k - 1] * v[k - 1] + 2.0 * a_max * (s[k] - s[k - 1])));
    for (int k = N - 1; k >= 0; k--)
        v[k] = std::min(v[k], std::sqrt(v[k + 1] * v[k + 1] + 2.0 * a_max * (s[k + 1] - s[k])));

    out.traj.assign(N + 1, TrajPoint{0.0, 0.0, 0.0});
    for (int k = 1; k <= N; k++)
    {
        double ds_k = s[k] - s[k - 1];
        double v_sum = v[k] + v[k - 1];
        double dt = (v_sum > 1e-6) ? 2.0 * ds_k / v_sum : 2.0 * std::sqrt(ds_k / a_max);
        out.traj[k].t = out.traj[k - 1].t + dt;
    }
    out.total_length = s.back();
    for (int k = 0; k <= N; k++)
    {
        Vec2f o = field_to_odom(p[k].x, p[k].y, origin_fx, origin_fy);
        out.traj[k].x = o.x;
        out.traj[k].y = o.y;
    }
    return true;
}

void traverse_sample(const TraversePlanResult& plan, double t, double& x, double& y)
{
    if (plan.traj.empty()) { x = 0.0; y = 0.0; return; }
    if (t <= 0.0) { x = plan.traj.front().x; y = plan.traj.front().y; return; }
    if (t >= plan.traj.back().t) { x = plan.traj.back().x; y = plan.traj.back().y; return; }
    int lo = 0, hi = (int)plan.traj.size() - 1;
    while (lo + 1 < hi)
    {
        int mid = (lo + hi) / 2;
        if (plan.traj[mid].t <= t) lo = mid; else hi = mid;
    }
    double span = plan.traj[hi].t - plan.traj[lo].t;
    double r = (span > 1e-9) ? (t - plan.traj[lo].t) / span : 0.0;
    x = plan.traj[lo].x + r * (plan.traj[hi].x - plan.traj[lo].x);
    y = plan.traj[lo].y + r * (plan.traj[hi].y - plan.traj[lo].y);
}

double traverse_duration(const TraversePlanResult& plan)
{
    return plan.traj.empty() ? 0.0 : plan.traj.back().t;
}

// ==================== 回调函数实现 ====================
void stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    current_mav_state = *msg;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    local_odom = *msg;
    tf::Quaternion q;
    tf::quaternionMsgToTF(local_odom.pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(current_roll, current_pitch, current_yaw);

    if (!init_pos_received && local_odom.pose.pose.position.z > -0.5) {
        init_pos_x        = local_odom.pose.pose.position.x;
        init_pos_y        = local_odom.pose.pose.position.y;
        init_pos_z        = local_odom.pose.pose.position.z;
        init_yaw          = current_yaw;
        init_pos_received = true;
        ROS_INFO("初始位置记录: (%.2f, %.2f, %.2f), 偏航: %.2f°", init_pos_x, init_pos_y,
                 init_pos_z, init_yaw * 180 / M_PI);
    }
}

void navStatusCallback(const std_msgs::Int8::ConstPtr &msg) {
    nav_status = msg->data;
    if (msg->data == 1) nav_seen_executing = true;
}

void detectedTargetCallback(const std_msgs::String::ConstPtr &msg) {
    confirmed_target = msg->data;
    target_confirmed = true;
    ROS_INFO("★★★ 目标确认: %s ★★★", confirmed_target.c_str());
}

void yoloDetectCallback(const raicom_vision_laser::DetectionInfo::ConstPtr &msg) {
    bool is_attack_state = (current_state == MOVE_TO_ATTACK_AREA ||
                            current_state == RECOG_ATTACK_TARGET ||
                            current_state == ALIGN_ATTACK_TARGET ||
                            current_state == SIMULATE_ATTACK);
    if (!is_attack_state) return;

    const std::string &match_target =
        shoot_letter.empty() ? cfg.attack_real_target : shoot_letter;
    if (match_target.empty()) return;

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

    if (found && best_confidence >= cfg.detection_min_confidence) {
        front_target_matched = true;
        matched_target       = best_class;
        matched_center_x     = best_center_x;
        matched_center_y     = best_center_y;
        last_matched_time    = ros::Time::now();
        ROS_INFO_THROTTLE(1.0, "★★★ [前视] 找到目标 %s, 像素(%.1f, %.1f) conf=%.2f ★★★",
                          matched_target.c_str(), matched_center_x, matched_center_y,
                          best_confidence);
    }
}

void yoloDownDetectCallback(const raicom_vision_laser::DetectionInfo::ConstPtr &msg) {
    if (!down_voting) return;

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

    if (best_cls == "A") ++down_vote_a;
    else                 ++down_vote_b;

    ROS_INFO_THROTTLE(0.5, "[下视] 字母投票 %s(conf=%.2f)，累计 A=%d 票 B=%d 票",
                      best_cls.c_str(), best_conf, down_vote_a, down_vote_b);
}

void pillarDetectCallback(const std_msgs::Int32::ConstPtr &msg) {
    if (msg->data >= 0 && msg->data < 4) {
        if (detected_case != msg->data)
            ROS_INFO("[穿越] 收到柱子布局检测结果 case%d（%s）", msg->data,
                     TRAV_CASE_DESC[msg->data]);
        detected_case = msg->data;
    }
    else {
        ROS_WARN("[穿越] 收到非法 case_id=%d，忽略", msg->data);
    }
}

// ==================== 控制辅助函数实现 ====================
void sendSetpoint(const mavros_msgs::PositionTarget &sp) {
    setpoint_pub.publish(sp);
}

void sendEgoGoal(float x, float y, float z, float yaw) {
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
    ego_goal_pub.publish(goal);
    nav_goal_sent      = true;
    nav_seen_executing = false;
    ROS_INFO("导航目标点: (%.2f, %.2f, %.2f)", x, y, z);
}

bool waitForNavArrival() {
    if ((ros::Time::now() - state_start_time).toSec() > cfg.nav_goal_timeout) {
        ROS_WARN("等待导航到达超时！");
        current_state = TASK_END;
        return false;
    }
    return nav_seen_executing && (nav_status == 2);
}

void positionControl(const Eigen::Vector3f &target_pos,
                     mavros_msgs::PositionTarget &sp) {
    Eigen::Vector3f err = target_pos - Eigen::Vector3f(local_odom.pose.pose.position.x,
                                                       local_odom.pose.pose.position.y,
                                                       local_odom.pose.pose.position.z);
    float vx            = err.x() * cfg.p_xy;
    float vy            = err.y() * cfg.p_xy;
    float vz            = err.z() * cfg.p_z;
    vx                  = std::clamp(vx, -cfg.max_speed, cfg.max_speed);
    vy                  = std::clamp(vy, -cfg.max_speed, cfg.max_speed);
    vz                  = std::clamp(vz, -cfg.max_speed, cfg.max_speed);

    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    sp.type_mask        = TYPE_MASK_VELOCITY_ONLY;
    sp.velocity.x       = vx;
    sp.velocity.y       = vy;
    sp.velocity.z       = vz;
    sp.yaw              = current_yaw;
}

bool reachedTarget(const Eigen::Vector3f &target, float dist_thresh) {
    float dx = target.x() - local_odom.pose.pose.position.x;
    float dy = target.y() - local_odom.pose.pose.position.y;
    float dz = target.z() - local_odom.pose.pose.position.z;
    return (dx * dx + dy * dy + dz * dz) < (dist_thresh * dist_thresh);
}

bool navTo(const float x, const float y, const float z) {
    if (!nav_goal_sent) {
        float target_x = init_pos_x + x;
        float target_y = init_pos_y + y;
        float target_z = init_pos_z + z;
        sendEgoGoal(target_x, target_y, target_z);
    }
    return waitForNavArrival();
}

bool navTo(const Waypoint wp) { return navTo(wp.x, wp.y, wp.z); }

bool moveTo(const float x, const float y, const float z) {
    float target_x = init_pos_x + x;
    float target_y = init_pos_y + y;
    float target_z = init_pos_z + z;

    positionControl(Eigen::Vector3f(target_x, target_y, target_z), current_setpoint);
    current_setpoint.yaw = init_yaw;

    return reachedTarget(Eigen::Vector3f(target_x, target_y, target_z), cfg.err_max);
}

bool moveTo(const Waypoint wp) { return moveTo(wp.x, wp.y, wp.z); }

bool moveToAbs(double x, double y, double z) {
    positionControl(Eigen::Vector3f(x, y, z), current_setpoint);
    current_setpoint.yaw = init_yaw;
    return reachedTarget(Eigen::Vector3f(x, y, z), cfg.err_max);
}

void hover() {
    static float local_x             = local_odom.pose.pose.position.x;
    static float local_y             = local_odom.pose.pose.position.y;
    static float local_z             = local_odom.pose.pose.position.z;
    static ros::Time last_hover_time = ros::Time::now();
    if (ros::Time::now() - last_hover_time > ros::Duration(3.0)) {
        local_x = local_odom.pose.pose.position.x;
        local_y = local_odom.pose.pose.position.y;
        local_z = local_odom.pose.pose.position.z;
    }
    moveTo(local_x, local_y, local_z);
    last_hover_time = ros::Time::now();
}

bool timeout(const float timeout_limit) {
    ros::Duration delta = ros::Time::now() - state_start_time;
    return delta > ros::Duration(timeout_limit);
}

// ==================== 参数加载 ====================
void loadParameters(ros::NodeHandle &nh) {
    nh.param<float>("takeoff_height", cfg.takeoff_height, 1.2f);
    nh.param<float>("max_speed", cfg.max_speed, 0.8f);
    nh.param<float>("max_yaw_rate", cfg.max_yaw_rate, 0.8f);
    nh.param<float>("err_max", cfg.err_max, 0.25f);
    nh.param<float>("hover_vert_tolerance", cfg.hover_vert_tolerance, 0.03f);
    nh.param<float>("p_xy", cfg.p_xy, 0.4f);
    nh.param<float>("p_z", cfg.p_z, 0.3f);
    nh.param<float>("hover_time_needed", cfg.hover_time_needed, 3.0f);
    nh.param<float>("target_front_offset_x", cfg.target_front_offset_x, -1.0f);
    nh.param<float>("target_front_offset_y", cfg.target_front_offset_y, 0.0f);
    nh.param<float>("nav_goal_timeout", cfg.nav_goal_timeout, 60.0f);
    nh.param<float>("align_pixel_threshold", cfg.align_pixel_threshold, 15.0f);
    nh.param<float>("shoot_delay", cfg.shoot_delay, 2.0f);

    nh.param<float>("PIX_VEL_P", cfg.PIX_VEL_P, 0.003f);
    nh.param<float>("PIX_VEL_I", cfg.PIX_VEL_I, 0.0001f);
    nh.param<float>("PIX_VEL_D", cfg.PIX_VEL_D, 0.001f);
    nh.param<float>("PIX_VEL_MAX", cfg.PIX_VEL_MAX, 0.4f);
    nh.param<float>("PIX_FAR_NORM_DIST", cfg.PIX_FAR_NORM_DIST, 150.0f);
    nh.param<float>("PIX_INTEGRAL_MAX", cfg.PIX_INTEGRAL_MAX, 100.0f);

    nh.param<float>("wp_ring_front_x", wp_ring_front.x, 0.65f);
    nh.param<float>("wp_ring_front_y", wp_ring_front.y, 0.0f);
    nh.param<float>("wp_ring_front_z", wp_ring_front.z, cfg.takeoff_height);
    nh.param<float>("wp_ring_back_x", wp_ring_back.x, 2.05f);
    nh.param<float>("wp_ring_back_y", wp_ring_back.y, 0.0f);
    nh.param<float>("wp_ring_back_z", wp_ring_back.z, cfg.takeoff_height);
    nh.param<float>("wp_come_mid_x", wp_come_mid.x, -2.35f);
    nh.param<float>("wp_come_mid_y", wp_come_mid.y, -2.48f);
    nh.param<float>("wp_come_mid_z", wp_come_mid.z, cfg.takeoff_height);
    nh.param<float>("wp_back_mid_x", wp_back_mid.x, -2.35f);
    nh.param<float>("wp_back_mid_y", wp_back_mid.y, -2.48f);
    nh.param<float>("wp_back_mid_z", wp_back_mid.z, cfg.takeoff_height);
    nh.param<float>("wp_pillar_center_x", wp_pillar_center.x, -2.35f);
    nh.param<float>("wp_pillar_center_y", wp_pillar_center.y, -1.43f);
    nh.param<float>("wp_pillar_center_z", wp_pillar_center.z, cfg.takeoff_height);
    nh.param<float>("wp_drop_area_x", wp_drop_area.x, 0.45f);
    nh.param<float>("wp_drop_area_y", wp_drop_area.y, 2.0f);
    nh.param<float>("wp_drop_area_z", wp_drop_area.z, cfg.takeoff_height);
    nh.param<float>("wp_attack_area_x", wp_attack_area.x, 0.45f);
    nh.param<float>("wp_attack_area_y", wp_attack_area.y, 2.0f);
    nh.param<float>("wp_attack_area_z", wp_attack_area.z, cfg.takeoff_height);

    nh.param<float>("detection/min_confidence", cfg.detection_min_confidence, 0.5f);
    nh.param<std::string>("detection/attack_real_target", cfg.attack_real_target, "A");
    nh.param<bool>("wait_for_vision_services", cfg.wait_for_vision_services, true);

    nh.param<float>("drop_arrive_threshold", cfg.drop_arrive_threshold, 0.35f);
    nh.param<float>("drop/detect_timeout", cfg.drop_detect_timeout, 5.0f);
    nh.param<float>("drop_align_hold_time", cfg.drop_align_hold_time, 0.35f);
    nh.param<float>("drop_camera_bias_x_px", cfg.drop_camera_bias_x_px, 0.0f);
    nh.param<float>("drop_camera_bias_y_px", cfg.drop_camera_bias_y_px, 0.0f);
    nh.param<float>("drop_release_bias_x_px", cfg.drop_release_bias_x_px, 0.0f);
    nh.param<float>("drop_release_bias_y_px", cfg.drop_release_bias_y_px, 0.0f);
    nh.param<float>("drop_fine_pixel_radius", cfg.drop_fine_pixel_radius, 35.0f);
    nh.param<float>("drop_fine_vel_scale", cfg.drop_fine_vel_scale, 0.45f);
    nh.param<float>("drop_descend_distance", cfg.drop_descend_distance, 0.0f);

    nh.param<float>("land/descend_speed", cfg.land_descend_speed, 0.3f);
    nh.param<bool>("use_ego_planner_for_drop_area", cfg.use_ego_planner_for_drop_area, true);

    nh.param<float>("shoot/left_x", cfg.shoot_left_x, -0.60f);
    nh.param<float>("shoot/left_y", cfg.shoot_left_y, -2.5f);
    nh.param<float>("shoot/right_x", cfg.shoot_right_x, -0.60f);
    nh.param<float>("shoot/right_y", cfg.shoot_right_y, -1.8f);
    nh.param<float>("shoot/default_x", cfg.shoot_default_x, -0.60f);
    nh.param<float>("shoot/default_y", cfg.shoot_default_y, -2.15f);
    nh.param<float>("shoot/left_right_threshold", cfg.shoot_left_right_threshold, 20.0f);
    nh.param<float>("shoot/detect_timeout", cfg.shoot_detect_timeout, 60.0f);
    nh.param<float>("shoot/stable_time", cfg.shoot_stable_time, 0.5f);
    nh.param<float>("shoot/duration", cfg.shoot_duration, 1.5f);
    nh.param<float>("shoot/z", cfg.shoot_z, 1.0f);

    nh.param<int>("cargo/drop_angle", cfg.cargo_drop_angle, 0);
    nh.param<int>("cargo/reset_angle", cfg.cargo_reset_angle, 180);
    nh.param<float>("cargo/hold_time", cfg.cargo_hold_time, 2.0f);
    nh.param<float>("cargo/descent_timeout", cfg.descent_timeout, 10.0f);
    nh.param<float>("cargo/drop_hover_time", cfg.drop_hover_time, 2.0f);
    nh.param<float>("cargo/drop_z", cfg.drop_z, 0.8f);

    nh.param<float>("yolo/img_center_x", cfg.yolo_img_center_x, 160.0f);
    nh.param<float>("yolo/target_timeout", cfg.yolo_target_timeout, 0.5f);
    nh.param<float>("yolo/detect_timeout", cfg.yolo_detect_timeout, 60.0f);

    nh.param<int>("down/min_votes", cfg.down_min_votes, 3);
    nh.param<std::string>("shoot/a_side", cfg.shoot_a_side, "left");
    a_on_left    = (cfg.shoot_a_side != "right");
    shoot_letter = cfg.attack_real_target;

    nh.param<std::string>("pillar_nav_mode", pillar_nav_mode, "ego");
    nh.param<float>("traverse/flight_z", cfg.trav_flight_z, 1.3f);
    nh.param<float>("traverse/err_max", cfg.trav_err_max, 0.15f);
    nh.param<double>("traverse/v_max", cfg.trav_v_max, 0.5);
    nh.param<double>("traverse/a_max", cfg.trav_a_max, 0.4);
    nh.param<double>("traverse/a_lat_max", cfg.trav_a_lat_max, 0.6);
    nh.param<double>("traverse/inflation", cfg.trav_inflation, 0.3);
    nh.param<double>("traverse/sample_ds", cfg.trav_sample_ds, 0.01);
    nh.param<int>("traverse/force_fly", cfg.trav_force_fly, 0);
    nh.param<float>("traverse/traj_timeout_margin", cfg.trav_timeout_margin, 15.0f);
    nh.param<float>("traverse/scan_hover_time", cfg.scan_hover_time, 3.0f);
    nh.param<float>("traverse/scan_timeout", cfg.scan_timeout, 4.0f);
    nh.param<int>("traverse/default_case", cfg.default_case, 1);
    nh.param<int>("traverse/force_case", cfg.force_case, -1);
    nh.param<double>("map/origin_x", origin_fx, 0.65);
    nh.param<double>("map/origin_y", origin_fy, 0.75);
    nh.param<double>("map/pillar_radius", pillar_radius, 0.1);

    ROS_INFO("参数加载完成。");
}

void initROSCommunication(ros::NodeHandle &nh) {
    setpoint_pub       = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ego_goal_pub       = nh.advertise<geometry_msgs::PoseStamped>("/fsm/ego_goal", 1);
    servo_control_pub  = nh.advertise<std_msgs::UInt8>("/servo_control", 1);
    shoot_pub          = nh.advertise<std_msgs::Empty>("/shoot", 1);
    laser_control_pub  = nh.advertise<std_msgs::Bool>("/laser_control", 1);

    state_sub          = nh.subscribe("/mavros/state", 10, &stateCallback);
    odom_sub           = nh.subscribe("/mavros/local_position/odom", 10, &odomCallback);
    nav_status_sub     = nh.subscribe("/ego_controller/status", 10, &navStatusCallback);
    detected_target_sub = nh.subscribe("/detected_target", 10, &detectedTargetCallback);
    yolo_detect_sub    = nh.subscribe("/yolo_front_detect", 10, &yoloDetectCallback);
    yolo_down_detect_sub = nh.subscribe("/yolo_down_detect", 10, &yoloDownDetectCallback);
    pillar_sub         = nh.subscribe("/pcl_detection2/pillar_case_id", 10, &pillarDetectCallback);
    pillar_start_pub   = nh.advertise<std_msgs::Empty>("/pcl_detection2/start_pillar_detect", 1);

    switch_camera_client = nh.serviceClient<std_srvs::Empty>("/switch_camera");
    reset_target_client  = nh.serviceClient<std_srvs::Empty>("/reset_target");
    set_mode_client      = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client        = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
}

// ==================== 穿越赛段：地图读取 + leg2 规划 ====================
static double xmlNum(const XmlRpc::XmlRpcValue &v) {
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) return (int)v;
    return (double)v;
}

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

void loadTraverseConfig(ros::NodeHandle &nh) {
    if (pillar_nav_mode != "pcl") {
        ROS_INFO("[穿越] pillar_nav_mode=%s，不加载穿越地图", pillar_nav_mode.c_str());
        return;
    }

    if (cfg.default_case < 0 || cfg.default_case > 3) {
        ROS_ERROR("[穿越] traverse/default_case=%d 非法（必须 0~3），回退 EGO 路径", cfg.default_case);
        return;
    }
    if (cfg.force_case < -1 || cfg.force_case > 3) {
        ROS_ERROR("[穿越] traverse/force_case=%d 非法（必须 -1 或 0~3），回退 EGO 路径", cfg.force_case);
        return;
    }

    XmlRpc::XmlRpcValue wl;
    if (nh.getParam("map/walls", wl)) {
        for (int i = 0; i < wl.size(); i++)
            if (wl[i].size() >= 4) {
                SegObs sg;
                sg.x1 = xmlNum(wl[i][0]);
                sg.y1 = xmlNum(wl[i][1]);
                sg.x2 = xmlNum(wl[i][2]);
                sg.y2 = xmlNum(wl[i][3]);
                walls.push_back(sg);
            }
    }

    for (int cid = 0; cid < 4; cid++) {
        char key[64];
        snprintf(key, sizeof(key), "map/via_points_leg2_case%d", cid);
        loadPointList(nh, key, via_leg2[cid]);
    }
    loadPointList(nh, "map/pillar_candidates", pillar_cand);

    {
        XmlRpc::XmlRpcValue sh;
        if (nh.getParam("map/scan_hover", sh) && sh.size() >= 2) {
            scan_hover_fx = xmlNum(sh[0]);
            scan_hover_fy = xmlNum(sh[1]);
        }
        else {
            ROS_WARN("[穿越] map/scan_hover 读取失败，用默认值 (%.2f, %.2f)",
                     scan_hover_fx, scan_hover_fy);
        }
    }

    for (int cid = 0; cid < 4; cid++) {
        if (via_leg2[cid].size() < 2) {
            ROS_ERROR("[穿越] map/via_points_leg2_case%d 为空或点数不足，回退 EGO 路径", cid);
            return;
        }
    }
    if (pillar_cand.size() != 4) {
        ROS_ERROR("[穿越] map/pillar_candidates 必须是 4 个候选柱位（当前 %zu 个），回退 EGO 路径",
                  pillar_cand.size());
        return;
    }

    for (int cid = 0; cid < 4; cid++) {
        const Vec2f &p0 = via_leg2[cid].front();
        if (fabs(p0.x - scan_hover_fx) > 1e-6 || fabs(p0.y - scan_hover_fy) > 1e-6)
            ROS_WARN("[穿越] leg2_case%d 首点(%.2f,%.2f) != scan_hover(%.2f,%.2f)，轨迹将从其他点起画！",
                     cid, p0.x, p0.y, scan_hover_fx, scan_hover_fy);
        const Vec2f &p1  = via_leg2[cid].back();
        const Vec2f &ref = via_leg2[0].back();
        if (fabs(p1.x - ref.x) > 1e-6 || fabs(p1.y - ref.y) > 1e-6)
            ROS_WARN("[穿越] leg2_case%d 末点(%.2f,%.2f) 与 case0 末点(%.2f,%.2f) 不一致，"
                     "投放区以 case0 末点为准！", cid, p1.x, p1.y, ref.x, ref.y);
    }

    Vec2f ho  = field_to_odom(scan_hover_fx, scan_hover_fy, origin_fx, origin_fy);
    hover_ox = ho.x;
    hover_oy = ho.y;
    Vec2f ep  = field_to_odom(via_leg2[0].back().x, via_leg2[0].back().y, origin_fx, origin_fy);
    end_x    = ep.x;
    end_y    = ep.y;

    {
        bool case_ok[4];
        ROS_INFO("[穿越] 启动预检：4 套 leg2 绕柱段净距一览");
        for (int cid = 0; cid < 4; cid++) {
            TraversePlanResult tp;
            traverse_plan(tp, via_leg2[cid], origin_fx, origin_fy, walls, caseCircles(cid),
                          cfg.trav_v_max, cfg.trav_a_max, cfg.trav_a_lat_max, cfg.trav_sample_ds);
            case_ok[cid] = (tp.min_clearance >= cfg.trav_inflation);
            ROS_INFO("[穿越]   case%d（%s）：总长 %.2f m，单程 %.1f s，最小净距 %.3f m %s",
                     cid, TRAV_CASE_DESC[cid], tp.total_length, traverse_duration(tp),
                     tp.min_clearance, case_ok[cid] ? "✓" : "✗ 不达标！");
        }
        if (!case_ok[cfg.default_case] && cfg.trav_force_fly != 1) {
            ROS_ERROR("[穿越] default_case=%d 净距不达标，回退链失效，回退 EGO 路径！"
                      "请调整 map/via_points_leg2_case%d", cfg.default_case, cfg.default_case);
            return;
        }
        if (cfg.force_case >= 0 && !case_ok[cfg.force_case] && cfg.trav_force_fly != 1) {
            ROS_ERROR("[穿越] force_case=%d 净距不达标，回退 EGO 路径！"
                      "请调整对应 via_points 或改 force_case", cfg.force_case);
            return;
        }
    }

    traverse_cfg_ok = true;
    ROS_INFO("[穿越] 地图加载完成：悬停扫描点 odom(%.2f, %.2f)，投放区 odom(%.2f, %.2f)，定高 %.2f",
             hover_ox, hover_oy, end_x, end_y, cfg.trav_flight_z);
}

std::vector<CircleObs> caseCircles(int cid) {
    std::vector<CircleObs> circles;
    for (int k = 0; k < 2; k++) {
        const Vec2f &c = pillar_cand[TRAV_CASE_PILLARS[cid][k]];
        CircleObs co;
        co.x = c.x;
        co.y = c.y;
        co.r = pillar_radius;
        circles.push_back(co);
    }
    return circles;
}

void printLeg2Report(int cid) {
    ROS_INFO("╔══════════════════════════════════════════════════╗");
    ROS_INFO("║  leg2 绕柱段规划报告 case%d（%s）", cid, TRAV_CASE_DESC[cid]);
    ROS_INFO("╚══════════════════════════════════════════════════╝");
    const std::vector<Vec2f> &via = via_leg2[cid];
    for (size_t i = 0; i < via.size(); i++) {
        Vec2f o = field_to_odom(via[i].x, via[i].y, origin_fx, origin_fy);
        ROS_INFO("  [%2zu] 场地(%5.2f, %5.2f) -> odom(%6.2f, %6.2f)", i, via[i].x, via[i].y, o.x, o.y);
    }
    ROS_INFO("  轨迹总长 %.2f m，单程时长 %.1f s，采样点 %zu 个",
             planner_leg2.total_length, traverse_duration(planner_leg2), planner_leg2.traj.size());
    ROS_INFO("  碰撞检测：最小净距 %.3f m @ 场地(%.2f, %.2f)，最近障碍：%s",
             planner_leg2.min_clearance, planner_leg2.min_clear_pos.x,
             planner_leg2.min_clear_pos.y, planner_leg2.min_clear_what.c_str());
    ROS_INFO("  膨胀要求：%.2f m -> %s", cfg.trav_inflation,
             planner_leg2.min_clearance >= cfg.trav_inflation ? "✓ 通过" : "✗ 不通过！");
}

bool planLeg2ForCase(int cid) {
    if (!traverse_plan(planner_leg2, via_leg2[cid], origin_fx, origin_fy, walls, caseCircles(cid),
                       cfg.trav_v_max, cfg.trav_a_max, cfg.trav_a_lat_max, cfg.trav_sample_ds)) {
        ROS_ERROR("[穿越] case%d 轨迹规划失败（途经点异常）！", cid);
        return false;
    }
    printLeg2Report(cid);
    return planner_leg2.min_clearance >= cfg.trav_inflation;
}

bool tryPlanLeg2(int cid) {
    if (planLeg2ForCase(cid)) {
        active_case = cid;
        return true;
    }
    if (cid != cfg.default_case) {
        ROS_WARN("[穿越] case%d 净距不达标，回退 default_case=%d 重试", cid, cfg.default_case);
        if (planLeg2ForCase(cfg.default_case)) {
            active_case = cfg.default_case;
            return true;
        }
    }
    if (cfg.trav_force_fly == 1) {
        planLeg2ForCase(cid);
        active_case = cid;
        ROS_WARN("[穿越] force_fly=1，强行按 case%d 飞行（净距 %.3f < %.3f，危险！）",
                 cid, planner_leg2.min_clearance, cfg.trav_inflation);
        return true;
    }
    return false;
}

bool trackLeg(bool reverse, double goal_x, double goal_y, const char *label) {
    double t  = (ros::Time::now() - leg_start_time).toSec();
    double T  = traverse_duration(planner_leg2);
    double qt = reverse ? (T - t) : t;
    if (qt < 0.0) qt = 0.0;

    double sx, sy;
    traverse_sample(planner_leg2, qt, sx, sy);

    current_setpoint.type_mask        = TRAV_TYPE_MASK_POSITION_ONLY;
    current_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint.position.x       = sx;
    current_setpoint.position.y       = sy;
    current_setpoint.position.z       = cfg.trav_flight_z;
    current_setpoint.yaw              = init_yaw;

    double cx = local_odom.pose.pose.position.x;
    double cy = local_odom.pose.pose.position.y;
    double cz = local_odom.pose.pose.position.z;

    bool arrived = (fabs(cx - goal_x) < cfg.trav_err_max &&
                    fabs(cy - goal_y) < cfg.trav_err_max &&
                    fabs(cz - cfg.trav_flight_z) < cfg.trav_err_max);

    ROS_INFO_THROTTLE(0.5, "[跟踪%s] t=%.1f/%.1fs 设定(%.2f,%.2f) 当前(%.2f,%.2f,%.2f)",
                      label, t, T, sx, sy, cx, cy, cz);

    if (t >= T && arrived) return true;

    if (t > T + cfg.trav_timeout_margin) {
        ROS_WARN("[跟踪%s] 超时(t=%.1f > %.1f+%.1f)，当前(%.2f,%.2f) 强制进入下一状态",
                 label, t, T, cfg.trav_timeout_margin, cx, cy);
        return true;
    }
    return false;
}

#endif // MAIN_CONTROL_H
