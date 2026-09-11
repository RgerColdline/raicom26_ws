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
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
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
// 定点加速控制掩码：忽略加速度 + yaw_rate，同时使用 x/y/z + vx/vy/vz + yaw
const uint16_t TYPE_MASK_POSITION_VELOCITY = 2496;
// 投放点原地瞄准：位置/速度与 yaw/yaw_rate 同时生效，仅忽略加速度
const uint16_t TYPE_MASK_POSITION_VELOCITY_YAW_RATE = 64 + 128 + 256;
// 穿越段轨迹跟踪位置控制掩码：忽略 vx/vy/vz/afx/afy/afz + IGNORE_YAW_RATE=2048（不带 512）
const uint16_t TRAV_TYPE_MASK_POSITION_ONLY = 8 + 16 + 32 + 64 + 128 + 256 + 2048;

// ==================== 状态机枚举 ====================
enum MissionState {
    // === 穿越赛段 ===
    TRAVERSE_TO_SCAN,
    TRAVERSE_SCAN,
    TRAVERSE_LEG2,

    // === 投货与原地旋转射击 ===
    HOVER_RECOG_DROP,
    DROP_SUPPLY,
    ROTATE_TO_ATTACK_YAW,
    SHOOT_TARGET,

    // === 返程路径 ===
    TRAVERSE_RETURN_HOME,   // 返程直通：投放/射击点 -> 倒放绕柱 -> 穿环 -> 起飞点，单条轨迹不停顿
    TRAVERSE_RETURN_LEG2,
    RETURN_CROSS_RING,
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
    float err_max;
    float p_xy, p_z;

    std::string attack_real_target;

    float land_descend_speed         = 0.3f;

    float shoot_yaw_a_offset_deg = -90.0f;
    float shoot_yaw_b_offset_deg = 90.0f;
    float shoot_yaw_kp           = 1.5f;
    float shoot_yaw_rate_max     = 0.8f;
    float shoot_yaw_tolerance_deg = 3.0f;
    float shoot_stable_time       = 0.5f;
    float shoot_duration          = 1.5f;

    int   cargo_drop_angle           = 0;
    int   cargo_reset_angle          = 180;
    float drop_hover_time            = 2.0f;

    int   down_min_votes             = 3;

    float trav_flight_z        = 1.3f;
    float trav_err_max         = 0.15f;
    double trav_v_max          = 0.5;
    double trav_a_max          = 0.4;
    double trav_a_lat_max      = 0.6;
    double trav_inflation      = 0.33;   // 机架0.225+桨叶旋转半径（与traverse_map.yaml一致）
    double trav_sample_ds      = 0.01;
    int   trav_force_fly         = 0;
    float trav_timeout_margin  = 15.0f;
    float scan_timeout         = 4.0f;
    int default_case           = 1;
    int force_case             = -1;
    int trav_early_scan        = 1;   // 1=起飞后立刻触发柱子检测，边飞边扫，检出且过环后当前位置直接切入 leg2（跳过悬停点停留）；0=旧行为到悬停点才扫
    int trav_return_smooth     = 1;   // 1=射击后从当前位置单条平滑轨迹返程（倒放绕柱+穿环+回起飞点，中途不停顿）；0=旧行为分段返程
    float trav_return_land_z   = 0.45f; // 返程直通末段目标高度(m)：过环后从 flight_z 边飞边线性降到该高度，缩短最后降落；<=0 禁用降高全程平飞
} cfg;

// ==================== ROS 通信 ====================
ros::Publisher setpoint_pub;
ros::Publisher servo_control_pub;
ros::Publisher shoot_pub;
ros::Publisher laser_control_pub;
ros::Subscriber state_sub, odom_sub;
ros::Subscriber yolo_down_detect_sub;
ros::Subscriber pillar_sub;
ros::Publisher pillar_start_pub;

ros::ServiceClient set_mode_client;
ros::ServiceClient arming_client;

// ==================== 状态机数据 ====================
MissionState current_state = TRAVERSE_TO_SCAN;
ros::Time state_start_time;
bool init_pos_received = false;
bool mission_finished  = false;
bool control_cfg_ok    = true;
ros::Time mission_start_time;  // 任务计时起点：无人机 arm 解锁成功那一刻

// ==================== 无人机状态 ====================
mavros_msgs::State current_mav_state;
nav_msgs::Odometry local_odom;
double current_yaw   = 0.0;
float init_pos_x = 0.0f, init_pos_y = 0.0f, init_pos_z = 0.0f;
double init_yaw = 0.0;

// ==================== 下视字母识别（投货悬停投票） ====================
std::string shoot_letter = "A";
bool down_voting  = false;
int  down_vote_a  = 0;
int  down_vote_b  = 0;

// ==================== 投货子状态 ====================
int       drop_sub_state   = 0;
int       drop_pub_count   = 0;   // 开舱指令已发送次数（节奏：0s/0.2s/0.4s 共 3 次）
ros::Time drop_hover_start;

// ==================== 射击状态 ====================
bool      shoot_triggered = false;
ros::Time shoot_time;
double    shoot_target_yaw = 0.0;
ros::Time yaw_aligned_since;

// ==================== 穿越赛段 ====================
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

// ==================== 控制 ====================
mavros_msgs::PositionTarget current_setpoint;

Waypoint wp_ring_front;
Waypoint wp_ring_back;
Waypoint wp_drop_area;

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
TraversePlanResult planner_return;   // 返程直通轨迹（当前位置 -> 倒放leg2绕柱 -> 穿环 -> 起飞点）
bool scan_trigger_sent = false;      // 去程前移触发只发一次（多发会反复 reset 检测的连续确认计数）

// ==================== 函数声明 ====================
// 初始化
void loadParameters(ros::NodeHandle &nh);
void loadTraverseConfig(ros::NodeHandle &nh);
void initROSCommunication(ros::NodeHandle &nh);

// 回调
void stateCallback(const mavros_msgs::State::ConstPtr &msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
void yoloDownDetectCallback(const raicom_vision_laser::DetectionInfo::ConstPtr &msg);
void pillarDetectCallback(const std_msgs::Int32::ConstPtr &msg);

// 控制辅助
void sendSetpoint(const mavros_msgs::PositionTarget &sp);
void positionControl(const Eigen::Vector3f &target_pos, mavros_msgs::PositionTarget &sp);
void positionVelocityControl(const Eigen::Vector3f &target_pos,
                             mavros_msgs::PositionTarget &sp);
bool reachedTarget(const Eigen::Vector3f &target, float dist_thresh);
bool moveTo(const float x, const float y, const float z);
bool moveTo(const Waypoint wp);
bool moveToPositionVelocity(const float x, const float y, const float z);
bool moveToPositionVelocity(const Waypoint wp);
bool moveToAbs(double x, double y, double z);
double normalizeAngle(double angle);
bool holdPositionAndAim(const Waypoint &wp, double target_yaw, double *yaw_error = nullptr);

// 穿越赛段
std::vector<CircleObs> caseCircles(int cid);
bool planLeg2ForCase(int cid);
bool tryPlanLeg2(int cid);
bool tryPlanLeg2FromCurrent(int cid, const Vec2f &cur_field);
bool buildReturnVia(int cid, const Vec2f &start_field, std::vector<Vec2f> &out);
bool planReturnFromCurrent();
void printLeg2Report(int cid);
bool trackPlan(const TraversePlanResult &plan, bool reverse, double goal_x, double goal_y,
               const char *label, double z_end = NAN);
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
    double roll = 0.0, pitch = 0.0;
    tf::Matrix3x3(q).getRPY(roll, pitch, current_yaw);

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

void positionVelocityControl(const Eigen::Vector3f &target_pos,
                             mavros_msgs::PositionTarget &sp) {
    // 复用原位置误差 -> 速度指令，额外向 PX4 提供最终位置目标：
    // 位置闭环保证收敛，速度前馈加快进点，并在误差缩小时自然降速。
    positionControl(target_pos, sp);
    sp.type_mask  = TYPE_MASK_POSITION_VELOCITY;
    sp.position.x = target_pos.x();
    sp.position.y = target_pos.y();
    sp.position.z = target_pos.z();
}

bool reachedTarget(const Eigen::Vector3f &target, float dist_thresh) {
    float dx = target.x() - local_odom.pose.pose.position.x;
    float dy = target.y() - local_odom.pose.pose.position.y;
    float dz = target.z() - local_odom.pose.pose.position.z;
    return (dx * dx + dy * dy + dz * dz) < (dist_thresh * dist_thresh);
}

bool moveTo(const float x, const float y, const float z) {
    float target_x = init_pos_x + x;
    float target_y = init_pos_y + y;
    float target_z = init_pos_z + z;

    positionControl(Eigen::Vector3f(target_x, target_y, target_z), current_setpoint);
    current_setpoint.yaw = init_yaw;

    return reachedTarget(Eigen::Vector3f(target_x, target_y, target_z), cfg.err_max);
}

bool moveTo(const Waypoint wp) { return moveTo(wp.x, wp.y, wp.z); }

bool moveToPositionVelocity(const float x, const float y, const float z) {
    const Eigen::Vector3f target(init_pos_x + x, init_pos_y + y, init_pos_z + z);

    positionVelocityControl(target, current_setpoint);
    current_setpoint.yaw = init_yaw;

    return reachedTarget(target, cfg.err_max);
}

bool moveToPositionVelocity(const Waypoint wp) {
    return moveToPositionVelocity(wp.x, wp.y, wp.z);
}

bool moveToAbs(double x, double y, double z) {
    positionControl(Eigen::Vector3f(x, y, z), current_setpoint);
    current_setpoint.yaw = init_yaw;
    return reachedTarget(Eigen::Vector3f(x, y, z), cfg.err_max);
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool holdPositionAndAim(const Waypoint &wp, double target_yaw, double *yaw_error) {
    const bool position_ok = moveToPositionVelocity(wp);
    const double error = normalizeAngle(target_yaw - current_yaw);
    const double yaw_rate = std::clamp(cfg.shoot_yaw_kp * error,
                                       -(double)cfg.shoot_yaw_rate_max,
                                       (double)cfg.shoot_yaw_rate_max);

    current_setpoint.type_mask = TYPE_MASK_POSITION_VELOCITY_YAW_RATE;
    current_setpoint.yaw       = target_yaw;
    current_setpoint.yaw_rate  = yaw_rate;

    if (yaw_error != nullptr) *yaw_error = error;
    return position_ok && fabs(error) <= cfg.shoot_yaw_tolerance_deg * M_PI / 180.0;
}

// ==================== 参数加载 ====================
void loadParameters(ros::NodeHandle &nh) {
    nh.param<float>("takeoff_height", cfg.takeoff_height, 1.2f);
    nh.param<float>("max_speed", cfg.max_speed, 0.8f);
    nh.param<float>("err_max", cfg.err_max, 0.25f);
    nh.param<float>("p_xy", cfg.p_xy, 0.4f);
    nh.param<float>("p_z", cfg.p_z, 0.3f);

    nh.param<float>("wp_ring_front_x", wp_ring_front.x, -0.65f);
    nh.param<float>("wp_ring_front_y", wp_ring_front.y, 0.0f);
    nh.param<float>("wp_ring_front_z", wp_ring_front.z, cfg.takeoff_height);
    nh.param<float>("wp_ring_back_x", wp_ring_back.x, -2.05f);
    nh.param<float>("wp_ring_back_y", wp_ring_back.y, 0.0f);
    nh.param<float>("wp_ring_back_z", wp_ring_back.z, cfg.takeoff_height);
    nh.param<float>("wp_drop_area_x", wp_drop_area.x, -0.45f);
    nh.param<float>("wp_drop_area_y", wp_drop_area.y, -2.0f);
    nh.param<float>("wp_drop_area_z", wp_drop_area.z, cfg.takeoff_height);

    nh.param<std::string>("detection/attack_real_target", cfg.attack_real_target, "A");

    nh.param<float>("land/descend_speed", cfg.land_descend_speed, 0.3f);

    nh.param<float>("shoot/yaw_a_offset_deg", cfg.shoot_yaw_a_offset_deg, -90.0f);
    nh.param<float>("shoot/yaw_b_offset_deg", cfg.shoot_yaw_b_offset_deg, 90.0f);
    nh.param<float>("shoot/yaw_kp", cfg.shoot_yaw_kp, 1.5f);
    nh.param<float>("shoot/yaw_rate_max", cfg.shoot_yaw_rate_max, 0.8f);
    nh.param<float>("shoot/yaw_tolerance_deg", cfg.shoot_yaw_tolerance_deg, 3.0f);
    nh.param<float>("shoot/stable_time", cfg.shoot_stable_time, 0.5f);
    nh.param<float>("shoot/duration", cfg.shoot_duration, 1.5f);

    nh.param<int>("cargo/drop_angle", cfg.cargo_drop_angle, 0);
    nh.param<int>("cargo/reset_angle", cfg.cargo_reset_angle, 180);
    nh.param<float>("cargo/drop_hover_time", cfg.drop_hover_time, 2.0f);

    nh.param<int>("down/min_votes", cfg.down_min_votes, 3);
    shoot_letter = cfg.attack_real_target;

    nh.param<float>("traverse/flight_z", cfg.trav_flight_z, 1.3f);
    nh.param<float>("traverse/err_max", cfg.trav_err_max, 0.15f);
    nh.param<double>("traverse/v_max", cfg.trav_v_max, 0.5);
    nh.param<double>("traverse/a_max", cfg.trav_a_max, 0.4);
    nh.param<double>("traverse/a_lat_max", cfg.trav_a_lat_max, 0.6);
    nh.param<double>("traverse/inflation", cfg.trav_inflation, 0.33);
    nh.param<double>("traverse/sample_ds", cfg.trav_sample_ds, 0.01);
    nh.param<int>("traverse/force_fly", cfg.trav_force_fly, 0);
    nh.param<float>("traverse/traj_timeout_margin", cfg.trav_timeout_margin, 15.0f);
    nh.param<float>("traverse/scan_timeout", cfg.scan_timeout, 4.0f);
    nh.param<int>("traverse/default_case", cfg.default_case, 1);
    nh.param<int>("traverse/force_case", cfg.force_case, -1);
    nh.param<int>("traverse/early_scan", cfg.trav_early_scan, 1);
    nh.param<int>("traverse/return_smooth", cfg.trav_return_smooth, 1);
    nh.param<float>("traverse/return_land_z", cfg.trav_return_land_z, 0.45f);
    nh.param<double>("map/origin_x", origin_fx, 0.65);
    nh.param<double>("map/origin_y", origin_fy, 0.75);
    nh.param<double>("map/pillar_radius", pillar_radius, 0.1);

    if (cfg.max_speed <= 0.0f || cfg.err_max <= 0.0f || cfg.p_xy <= 0.0f || cfg.p_z <= 0.0f ||
        cfg.land_descend_speed <= 0.0f || cfg.drop_hover_time < 0.0f || cfg.down_min_votes < 1 ||
        cfg.cargo_drop_angle < 0 || cfg.cargo_drop_angle > 255 ||
        cfg.cargo_reset_angle < 0 || cfg.cargo_reset_angle > 255 ||
        (cfg.attack_real_target != "A" && cfg.attack_real_target != "B") ||
        !std::isfinite(cfg.shoot_yaw_a_offset_deg) || !std::isfinite(cfg.shoot_yaw_b_offset_deg) ||
        cfg.shoot_yaw_kp <= 0.0f || cfg.shoot_yaw_rate_max <= 0.0f ||
        cfg.shoot_yaw_tolerance_deg <= 0.0f || cfg.shoot_yaw_tolerance_deg > 180.0f ||
        cfg.shoot_stable_time < 0.0f || cfg.shoot_duration <= 0.0f) {
        ROS_FATAL("主控参数非法：控制速度/误差/增益和降落速度须 > 0；投票数须 >= 1；"
                  "舵机角度须在 0~255；兜底目标须为 A/B；yaw 偏移须为有限值；"
                  "yaw kp/rate/duration 须 > 0，tolerance 须在 (0,180]，等待时间须 >= 0");
        control_cfg_ok = false;
        return;
    }

    ROS_INFO("参数加载完成：A/B yaw 偏移 %.1f°/%.1f°，kp=%.2f，rate_max=%.2f rad/s，容差 %.1f°",
             cfg.shoot_yaw_a_offset_deg, cfg.shoot_yaw_b_offset_deg, cfg.shoot_yaw_kp,
             cfg.shoot_yaw_rate_max, cfg.shoot_yaw_tolerance_deg);
}

void initROSCommunication(ros::NodeHandle &nh) {
    setpoint_pub       = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    servo_control_pub  = nh.advertise<std_msgs::UInt8>("/servo_control", 1);
    shoot_pub          = nh.advertise<std_msgs::Empty>("/shoot", 1);
    laser_control_pub  = nh.advertise<std_msgs::Bool>("/laser_control", 1);

    state_sub          = nh.subscribe("/mavros/state", 10, &stateCallback);
    odom_sub           = nh.subscribe("/mavros/local_position/odom", 10, &odomCallback);
    yolo_down_detect_sub = nh.subscribe("/yolo_down_detect", 10, &yoloDownDetectCallback);
    pillar_sub         = nh.subscribe("/pcl_detection2/pillar_case_id", 10, &pillarDetectCallback);
    pillar_start_pub   = nh.advertise<std_msgs::Empty>("/pcl_detection2/start_pillar_detect", 1);

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
    if (cfg.default_case < 0 || cfg.default_case > 3) {
        ROS_ERROR("[穿越] traverse/default_case=%d 非法（必须 0~3）", cfg.default_case);
        return;
    }
    if (cfg.force_case < -1 || cfg.force_case > 3) {
        ROS_ERROR("[穿越] traverse/force_case=%d 非法（必须 -1 或 0~3）", cfg.force_case);
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
            ROS_ERROR("[穿越] map/via_points_leg2_case%d 为空或点数不足", cid);
            return;
        }
    }
    if (pillar_cand.size() != 4) {
        ROS_ERROR("[穿越] map/pillar_candidates 必须是 4 个候选柱位（当前 %zu 个）",
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
            ROS_ERROR("[穿越] default_case=%d 净距不达标，无法安全执行任务！"
                      "请调整 map/via_points_leg2_case%d", cfg.default_case, cfg.default_case);
            return;
        }
        if (cfg.force_case >= 0 && !case_ok[cfg.force_case] && cfg.trav_force_fly != 1) {
            ROS_ERROR("[穿越] force_case=%d 净距不达标，无法安全执行任务！"
                      "请调整对应 via_points 或改 force_case", cfg.force_case);
            return;
        }
    }

    // ---- 返程直通预检（return_smooth=1 时）：投放/射击点出发，倒放leg2+穿环走廊+起飞点 ----
    if (cfg.trav_return_smooth == 1) {
        const Vec2f shoot_start = via_leg2[0].back();
        ROS_INFO("[穿越] 返程直通预检：4 套返程轨迹（投放点原地射击后，倒放绕柱+穿环+回起点）净距一览");
        for (int cid = 0; cid < 4; cid++) {
            std::vector<Vec2f> via_ret;
            TraversePlanResult tp;
            if (!buildReturnVia(cid, shoot_start, via_ret)) {
                ROS_WARN("[穿越]   case%d 返程途经点构造失败", cid);
                continue;
            }
            traverse_plan(tp, via_ret, origin_fx, origin_fy, walls, caseCircles(cid),
                          cfg.trav_v_max, cfg.trav_a_max, cfg.trav_a_lat_max, cfg.trav_sample_ds);
            ROS_INFO("[穿越]   case%d 返程：总长 %.2f m，时长 %.1f s，最小净距 %.3f m %s",
                     cid, tp.total_length, traverse_duration(tp), tp.min_clearance,
                     tp.min_clearance >= cfg.trav_inflation ? "✓" : "✗ 偏紧（运行时校验兜底）");
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

bool tryPlanLeg2FromCurrent(int cid, const Vec2f &cur_field) {
    // 边飞边扫命中后：把 leg2 途经点首点（悬停点）替换为当前位置重新规划，
    // 无人机无需到悬停点停稳，直接切入绕柱轨迹。净距校验与完整版一致。
    if (cid < 0 || cid > 3 || via_leg2[cid].size() < 2) return false;

    std::vector<Vec2f> via = via_leg2[cid];
    via.front() = cur_field;

    if (!traverse_plan(planner_leg2, via, origin_fx, origin_fy, walls, caseCircles(cid),
                       cfg.trav_v_max, cfg.trav_a_max, cfg.trav_a_lat_max, cfg.trav_sample_ds)) {
        ROS_WARN("[穿越] 当前位置(场地 %.2f,%.2f)切入 case%d 规划失败", cur_field.x, cur_field.y, cid);
        return false;
    }
    ROS_INFO("[穿越] 当前位置(场地 %.2f,%.2f)切入 case%d：总长 %.2f m，时长 %.1f s，最小净距 %.3f m %s",
             cur_field.x, cur_field.y, cid, planner_leg2.total_length,
             traverse_duration(planner_leg2), planner_leg2.min_clearance,
             planner_leg2.min_clearance >= cfg.trav_inflation ? "✓" : "✗");
    if (planner_leg2.min_clearance >= cfg.trav_inflation) {
        active_case = cid;
        return true;
    }
    return false;
}

bool buildReturnVia(int cid, const Vec2f &start_field, std::vector<Vec2f> &out) {
    // 返程直通途经点（场地系）：起点 -> 倒放 leg2（绕柱区入口 -> 悬停点）-> 穿环走廊 -> 起飞点
    // 倒放跳过 leg2 末点（投放区），第一个途经点 = leg2 倒数第二点（投放/射击区与绕柱区之间），
    // 从投放/射击点直连该点。穿环走廊：悬停点与孔之间拉直 -> 过孔中心 -> 孔与出生点之间拉直 -> 起飞点
    if (cid < 0 || cid > 3 || via_leg2[cid].size() < 3) return false;
    out.clear();
    // 防重复点（重复点会让样条弦长参数 h=0 导致除零）：起点离倒放首点太近就不单独加起点
    const Vec2f &entry = via_leg2[cid][via_leg2[cid].size() - 2];   // 绕柱区入口（leg2 倒数第二点）
    if (std::hypot(start_field.x - entry.x, start_field.y - entry.y) > 0.1)
        out.push_back(start_field);                            // 起点（投放/射击点或当前位置）
    for (int i = (int)via_leg2[cid].size() - 2; i >= 0; --i)   // 倒放绕柱：绕柱区入口 -> ... -> 悬停点（跳过投放区）
        out.push_back(via_leg2[cid][i]);
    Vec2f mid1{0.5 * (scan_hover_fx + 2.0), scan_hover_fy};    // 悬停点与孔之间拉直（抑制拐弯外凸）
    Vec2f hole{2.0, scan_hover_fy};                            // 过孔中心（x=2，孔 y 中心=出生点 y）
    Vec2f mid2{0.5 * (origin_fx + 2.0), origin_fy};            // 孔与出生点之间拉直
    Vec2f home{origin_fx, origin_fy};                          // 起飞点
    out.push_back(mid1);
    out.push_back(hole);
    out.push_back(mid2);
    out.push_back(home);
    return true;
}

bool planReturnFromCurrent() {
    // 射击完成后：以当前位置为首点规划单条返程直通轨迹（中途不停顿）。
    // 净距不达标返回 false，由调用方回退分段返程（TRAVERSE_RETURN_LEG2）。
    if (active_case < 0 || active_case > 3) return false;

    Vec2f cur_field;
    cur_field.x = origin_fx - local_odom.pose.pose.position.x;   // odom -> 场地 反变换
    cur_field.y = origin_fy - local_odom.pose.pose.position.y;

    std::vector<Vec2f> via;
    if (!buildReturnVia(active_case, cur_field, via)) return false;

    if (!traverse_plan(planner_return, via, origin_fx, origin_fy, walls, caseCircles(active_case),
                       cfg.trav_v_max, cfg.trav_a_max, cfg.trav_a_lat_max, cfg.trav_sample_ds)) {
        ROS_ERROR("[返程] 直通轨迹规划失败（途经点异常）");
        return false;
    }
    ROS_INFO("[返程] 直通轨迹：起点场地(%.2f,%.2f)，总长 %.2f m，时长 %.1f s，最小净距 %.3f m %s",
             cur_field.x, cur_field.y, planner_return.total_length,
             traverse_duration(planner_return), planner_return.min_clearance,
             planner_return.min_clearance >= cfg.trav_inflation ? "✓" : "✗ 回退分段返程");
    return planner_return.min_clearance >= cfg.trav_inflation || cfg.trav_force_fly == 1;
}

bool trackPlan(const TraversePlanResult &plan, bool reverse, double goal_x, double goal_y,
               const char *label, double z_end) {
    double t  = (ros::Time::now() - leg_start_time).toSec();
    double T  = traverse_duration(plan);
    double qt = reverse ? (T - t) : t;
    if (qt < 0.0) qt = 0.0;

    double sx, sy;
    traverse_sample(plan, qt, sx, sy);

    // z_end 有效（返程直通）时：过环前（场地 x>=2.05）保持 flight_z，
    // 过环后随水平进度从 flight_z 线性降到 z_end —— 边飞边降，缩短最后降落时间
    //
    // ⚠️ 2026-09-09 修复挂网：原来直接按“当前轨迹点的场地 x<2.05 就降”，但返程
    // 起点在射击区（场地 x≈1.1，本来就 <2.05），导致刚射击完还没往环走就被拉到
    // 低高度（~0.66m）撞到靶区侧网。现在必须先确认轨迹已真正越过环进入末段
    // 回家走廊（从轨迹末端倒扫到“最后一个场地 x>=2.05 的点”即过环时刻），
    // 时刻未到一律保持 flight_z，全程不会再提前降高。
    double z_cmd = cfg.trav_flight_z;
    if (!std::isnan(z_end) && z_end > 0.0) {
        double t_ring = T;                              // 默认整段不降（安全兜底）
        const std::vector<TrajPoint> &tr = plan.traj;
        for (int k = (int)tr.size() - 1; k >= 0; --k) {
            double fxk = origin_fx - tr[k].x;           // odom -> 场地 x
            if (fxk >= 2.05) { t_ring = tr[k].t; break; }
        }
        if (qt > t_ring && t_ring < T) {
            double fx = origin_fx - sx;                 // 采样点场地 x
            double r  = (2.05 - fx) / (2.05 - origin_fx);   // 过环点 -> 起飞点 的进度 [0,1]
            r = std::max(0.0, std::min(1.0, r));
            z_cmd = cfg.trav_flight_z + (z_end - cfg.trav_flight_z) * r;
        }
    }

    current_setpoint.type_mask        = TRAV_TYPE_MASK_POSITION_ONLY;
    current_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint.position.x       = sx;
    current_setpoint.position.y       = sy;
    current_setpoint.position.z       = z_cmd;
    current_setpoint.yaw              = init_yaw;

    double cx = local_odom.pose.pose.position.x;
    double cy = local_odom.pose.pose.position.y;
    double cz = local_odom.pose.pose.position.z;

    bool arrived = (fabs(cx - goal_x) < cfg.trav_err_max &&
                    fabs(cy - goal_y) < cfg.trav_err_max &&
                    fabs(cz - z_cmd) < cfg.trav_err_max);

    ROS_INFO_THROTTLE(0.5, "[跟踪%s] t=%.1f/%.1fs 设定(%.2f,%.2f,%.2f) 当前(%.2f,%.2f,%.2f)",
                      label, t, T, sx, sy, z_cmd, cx, cy, cz);

    if (t >= T && arrived) return true;

    if (t > T + cfg.trav_timeout_margin) {
        ROS_WARN("[跟踪%s] 超时(t=%.1f > %.1f+%.1f)，当前(%.2f,%.2f) 强制进入下一状态",
                 label, t, T, cfg.trav_timeout_margin, cx, cy);
        return true;
    }
    return false;
}

bool trackLeg(bool reverse, double goal_x, double goal_y, const char *label) {
    return trackPlan(planner_leg2, reverse, goal_x, goal_y, label);
}

#endif // MAIN_CONTROL_H
