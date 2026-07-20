// ============================================================
// 文件: traverse_planner.h
// 作者: 肚腩特大哥 (Claude 辅助)
// 日期: 2026-07
// 功能: 省赛固定地图(4x4)穿越轨迹规划器 —— 纯计算模块，不依赖 ROS
//   1) 途经点(场地系) -> 自然三次样条平滑
//   2) 密集采样 + 碰撞检测(膨胀净距)
//   3) 时间参数化(梯形速度剖面 + 曲率限速)
//   4) 输出 odom 系时间轨迹，供 traverse_node 逐帧跟踪
// 坐标约定见 field_to_odom() 注释与 config/traverse_map_*.yaml 头部
// ============================================================
#ifndef TRAVERSE_PLANNER_H
#define TRAVERSE_PLANNER_H

// ==================== 依赖头文件 ====================
#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <algorithm>

// ==================== 基础数据结构 ====================
struct Vec2f { double x, y; };                 // 二维点
struct CircleObs { double x, y, r; };          // 圆形障碍（圆柱，r=实际半径，未膨胀）
struct SegObs { double x1, y1, x2, y2; };      // 线段障碍（墙 / 场地边界）
struct TrajPoint { double t, x, y; };          // 时间参数化轨迹点（odom 系）

// ==================== 坐标变换 ====================
/****************************************************************
 场地系 -> odom系
 odom 原点 = 出生点；机头全程朝场地 -X 且 odom yaw 保持 0，
 所以 odom +x 轴 = 场地 -X 方向，即绕出生点旋转 180°：
     p_odom = -(p_field - p_start) = p_start - p_field
 例：出生点 (0.65, 0.75)，柱1 场地系 (2.7, 1.55) -> odom (-2.05, -0.8)
*****************************************************************/
inline Vec2f field_to_odom(double fx, double fy, double origin_fx, double origin_fy)
{
    Vec2f p;
    p.x = origin_fx - fx;
    p.y = origin_fy - fy;
    return p;
}

// ==================== 距离工具 ====================
// 点到线段的最短距离
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

// ==================== 自然三次样条（二维，参数=累积弦长） ====================
class NaturalSpline2D
{
public:
    // 用途经点构建样条；点数 <2 返回 false
    bool build(const std::vector<Vec2f>& pts)
    {
        if (pts.size() < 2) return false;
        int N = (int)pts.size();
        u_.assign(N, 0.0);
        x_.resize(N);
        y_.resize(N);
        for (int i = 0; i < N; i++)
        {
            x_[i] = pts[i].x;
            y_[i] = pts[i].y;
        }
        for (int i = 1; i < N; i++)
            u_[i] = u_[i - 1] + std::hypot(x_[i] - x_[i - 1], y_[i] - y_[i - 1]);
        mx_ = solve_second_deriv(u_, x_);
        my_ = solve_second_deriv(u_, y_);
        return true;
    }

    // 参数 u 处的位置
    Vec2f eval(double u) const
    {
        int i = find_seg(u);
        double h = u_[i + 1] - u_[i];
        double A = (u_[i + 1] - u) / h;
        double B = (u - u_[i]) / h;
        Vec2f p;
        p.x = A * x_[i] + B * x_[i + 1]
            + ((A * A * A - A) * mx_[i] + (B * B * B - B) * mx_[i + 1]) * h * h / 6.0;
        p.y = A * y_[i] + B * y_[i + 1]
            + ((A * A * A - A) * my_[i] + (B * B * B - B) * my_[i + 1]) * h * h / 6.0;
        return p;
    }

    // 参数 u 处的一阶/二阶导数（对 u 求导，用于算曲率）
    void eval_deriv(double u, double& dx, double& dy, double& ddx, double& ddy) const
    {
        int i = find_seg(u);
        double h = u_[i + 1] - u_[i];
        double A = (u_[i + 1] - u) / h;
        double B = (u - u_[i]) / h;
        dx = (x_[i + 1] - x_[i]) / h
           + ((-3.0 * A * A + 1.0) * mx_[i] + (3.0 * B * B - 1.0) * mx_[i + 1]) * h / 6.0;
        dy = (y_[i + 1] - y_[i]) / h
           + ((-3.0 * A * A + 1.0) * my_[i] + (3.0 * B * B - 1.0) * my_[i + 1]) * h / 6.0;
        ddx = A * mx_[i] + B * mx_[i + 1];
        ddy = A * my_[i] + B * my_[i + 1];
    }

    double length_param() const { return u_.empty() ? 0.0 : u_.back(); }  // 参数总长（≈折线长）

private:
    // 托马斯算法解自然三次样条的三对角方程，返回各节点二阶导数 M
    static std::vector<double> solve_second_deriv(const std::vector<double>& u,
                                                  const std::vector<double>& v)
    {
        int N = (int)u.size();
        std::vector<double> M(N, 0.0);
        if (N < 3) return M;   // 两个点：直线，二阶导全 0

        int m = N - 2;         // 内部未知量个数
        std::vector<double> a(m), b(m), c(m), d(m);
        for (int i = 1; i <= N - 2; i++)
        {
            double h_prev = u[i] - u[i - 1];
            double h_next = u[i + 1] - u[i];
            int k = i - 1;
            a[k] = h_prev;                          // M_{i-1} 系数
            b[k] = 2.0 * (h_prev + h_next);         // M_i 系数
            c[k] = h_next;                          // M_{i+1} 系数
            d[k] = 6.0 * ((v[i + 1] - v[i]) / h_next - (v[i] - v[i - 1]) / h_prev);
        }
        // 前向消元
        for (int k = 1; k < m; k++)
        {
            double w = a[k] / b[k - 1];
            b[k] -= w * c[k - 1];
            d[k] -= w * d[k - 1];
        }
        // 回代
        std::vector<double> x(m);
        x[m - 1] = d[m - 1] / b[m - 1];
        for (int k = m - 2; k >= 0; k--)
            x[k] = (d[k] - c[k] * x[k + 1]) / b[k];
        for (int k = 0; k < m; k++)
            M[k + 1] = x[k];
        return M;   // M[0] = M[N-1] = 0（自然边界）
    }

    // 二分定位 u 所在段（夹取到 [0, N-2]）
    int find_seg(double u) const
    {
        int N = (int)u_.size();
        if (u <= u_.front()) return 0;
        if (u >= u_.back()) return N - 2;
        int lo = 0, hi = N - 1;
        while (lo + 1 < hi)
        {
            int mid = (lo + hi) / 2;
            if (u_[mid] <= u) lo = mid; else hi = mid;
        }
        return lo;
    }

    std::vector<double> u_;          // 节点参数（累积弦长）
    std::vector<double> x_, y_;      // 节点坐标
    std::vector<double> mx_, my_;    // 各节点二阶导数
};

// ==================== 穿越轨迹规划器 ====================
class TraversePlanner
{
public:
    /****************************************************************
     完整规划流程：
       field_pts   途经点（场地系，含起点/终点）
       origin      出生点（场地系），用于 field->odom 变换
       walls       线段障碍（含孔两侧实体段、墙2/墙3、场地边界）
       circles     圆形障碍（圆柱，r 为实际半径）
       v_max       最大线速度 (m/s)
       a_max       最大线加速度 (m/s^2)
       a_lat_max   最大向心加速度 (m/s^2)（弯道自动减速）
       ds          采样步长 (m)
     返回 true 成功；结果存于 traj_（odom 系时间轨迹）
    *****************************************************************/
    bool plan(const std::vector<Vec2f>& field_pts,
              double origin_fx, double origin_fy,
              const std::vector<SegObs>& walls,
              const std::vector<CircleObs>& circles,
              double v_max, double a_max, double a_lat_max, double ds)
    {
        if (field_pts.size() < 2) return false;

        NaturalSpline2D spline;
        if (!spline.build(field_pts)) return false;

        double U = spline.length_param();
        int N = std::max(2, (int)std::ceil(U / ds));
        double du = U / N;

        // ---- 1) 密集采样（场地系）+ 弧长 + 曲率 ----
        std::vector<Vec2f> p(N + 1);
        std::vector<double> s(N + 1, 0.0), kap(N + 1, 0.0);
        for (int k = 0; k <= N; k++)
            p[k] = spline.eval(k * du);
        for (int k = 1; k <= N; k++)
            s[k] = s[k - 1] + std::hypot(p[k].x - p[k - 1].x, p[k].y - p[k - 1].y);
        for (int k = 0; k <= N; k++)
        {
            double dx, dy, ddx, ddy;
            spline.eval_deriv(k * du, dx, dy, ddx, ddy);
            double denom = std::pow(dx * dx + dy * dy, 1.5);
            kap[k] = (denom > 1e-9) ? std::fabs(dx * ddy - dy * ddx) / denom : 0.0;
        }

        // ---- 2) 碰撞检测（场地系，净距 = 到障碍距离 - 圆半径）----
        min_clearance_ = 1e9;
        for (int k = 0; k <= N; k++)
        {
            std::string what;
            double c = clearance_at(p[k], walls, circles, what);
            if (c < min_clearance_)
            {
                min_clearance_ = c;
                min_clear_pos_ = p[k];
                min_clear_what_ = what;
            }
        }

        // ---- 3) 速度剖面：曲率限速 + 前向/后向梯形积分 ----
        std::vector<double> v(N + 1);
        for (int k = 0; k <= N; k++)
            v[k] = std::min(v_max, std::sqrt(a_lat_max / std::max(kap[k], 1e-6)));
        v[0] = 0.0;
        v[N] = 0.0;
        for (int k = 1; k <= N; k++)       // 前向（加速限制）
            v[k] = std::min(v[k], std::sqrt(v[k - 1] * v[k - 1] + 2.0 * a_max * (s[k] - s[k - 1])));
        for (int k = N - 1; k >= 0; k--)   // 后向（减速限制）
            v[k] = std::min(v[k], std::sqrt(v[k + 1] * v[k + 1] + 2.0 * a_max * (s[k + 1] - s[k])));

        // ---- 4) 时间戳 + 变换到 odom 系 ----
        traj_.assign(N + 1, TrajPoint{0.0, 0.0, 0.0});
        for (int k = 1; k <= N; k++)
        {
            double ds_k = s[k] - s[k - 1];
            double v_sum = v[k] + v[k - 1];
            // v 全 0 的退化段用匀加速公式兜底，防止除零
            double dt = (v_sum > 1e-6) ? 2.0 * ds_k / v_sum : 2.0 * std::sqrt(ds_k / a_max);
            traj_[k].t = traj_[k - 1].t + dt;
        }
        total_length_ = s.back();
        for (int k = 0; k <= N; k++)
        {
            Vec2f o = field_to_odom(p[k].x, p[k].y, origin_fx, origin_fy);
            traj_[k].x = o.x;
            traj_[k].y = o.y;
        }
        return true;
    }

    // 按时间取位置（线性插值），t 超界自动夹取到首/尾点
    void sample(double t, double& x, double& y) const
    {
        if (traj_.empty()) { x = 0.0; y = 0.0; return; }
        if (t <= 0.0) { x = traj_.front().x; y = traj_.front().y; return; }
        if (t >= traj_.back().t) { x = traj_.back().x; y = traj_.back().y; return; }
        int lo = 0, hi = (int)traj_.size() - 1;
        while (lo + 1 < hi)
        {
            int mid = (lo + hi) / 2;
            if (traj_[mid].t <= t) lo = mid; else hi = mid;
        }
        double span = traj_[hi].t - traj_[lo].t;
        double r = (span > 1e-9) ? (t - traj_[lo].t) / span : 0.0;
        x = traj_[lo].x + r * (traj_[hi].x - traj_[lo].x);
        y = traj_[lo].y + r * (traj_[hi].y - traj_[lo].y);
    }

    // ---- 结果查询 ----
    double duration() const { return traj_.empty() ? 0.0 : traj_.back().t; }  // 单程时长(s)
    double total_length() const { return total_length_; }                     // 轨迹总长(m)
    size_t point_count() const { return traj_.size(); }
    double min_clearance() const { return min_clearance_; }                   // 全程最小净距(m)
    Vec2f min_clearance_pos() const { return min_clear_pos_; }                // 最小净距位置(场地系)
    std::string min_clearance_what() const { return min_clear_what_; }        // 最近的障碍

private:
    // 单点净距：到所有墙/边界的最短距离、到圆柱(距离-半径) 的最小值
    double clearance_at(const Vec2f& pt, const std::vector<SegObs>& walls,
                        const std::vector<CircleObs>& circles, std::string& what) const
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

    std::vector<TrajPoint> traj_;   // odom 系时间轨迹
    double total_length_ = 0.0;
    double min_clearance_ = 1e9;
    Vec2f min_clear_pos_{0.0, 0.0};
    std::string min_clear_what_;
};

#endif // TRAVERSE_PLANNER_H
