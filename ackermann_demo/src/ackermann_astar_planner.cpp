// =============================================================================
// Ackermann Hybrid A* Planner — faithful to Dolgov, Thrun, Montemerlo & Diebel,
// "Practical Search Techniques in Path Planning for Autonomous Driving" (2008),
// layered on the RC-car integration features of astar_planner.cpp.
//
// What it implements from the paper that the original astar_planner.cpp lacked:
//   1. Reeds-Shepp analytic expansions to the goal (exact heading, can reverse),
//      replacing the straight-line finisher.
//   2. The dual heuristic h = max( non-holonomic-without-obstacles [Reeds-Shepp
//      length], holonomic-with-obstacles [2D Dijkstra from the goal] ).
//   3. A second-phase gradient-descent path smoother with the paper's
//      smoothness + obstacle + Voronoi terms (curvature kept feasible by
//      anchoring cusps and a per-step collision/curvature guard). The Voronoi
//      term descends the ρ_V field published by voronoi_costmap on
//      /voronoi_field, centring the path in narrow passages.
//
// RC-car features carried over verbatim in spirit from astar_planner.cpp:
//   escape bubble, virtual-obstacle stall recovery, signed-velocity profile in
//   pose.position.z, /followed_path + /visual_path contracts, keep-alive
//   republish, deviation-triggered replanning, rear-axle base_footprint.
//
// The Reeds-Shepp curve math (section RS) is a clean-room port of OMPL's
// ReedsSheppStateSpace (BSD-3-Clause, (c) 2010 Rice University, Mark Moll),
// which itself follows Reeds & Shepp (1990).
// =============================================================================

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int8.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

static constexpr double PI      = 3.14159265358979323846;
static constexpr double TWO_PI  = 2.0 * PI;
static constexpr double DEG2RAD = PI / 180.0;

inline double wrapPi(double a) {
    while (a >  PI) a -= TWO_PI;
    while (a < -PI) a += TWO_PI;
    return a;
}
inline double wrap2Pi(double a) {
    a = std::fmod(a, TWO_PI);
    if (a < 0.0) a += TWO_PI;
    return a;
}

// =============================================================================
// RS — Reeds-Shepp shortest paths (unit turning radius), with a sampler that
// replays the optimal word into world-frame poses carrying a gear (±1) sign.
// Port of OMPL ReedsSheppStateSpace (BSD-3-Clause, (c) 2010 Rice University).
// =============================================================================
namespace rs {

static constexpr double RS_PI  = 3.14159265358979323846;
static constexpr double RS_2PI = 2.0 * RS_PI;
static const double ZERO = 10.0 * std::numeric_limits<double>::epsilon();

enum Seg { NOP = 0, LEFT = 1, STRAIGHT = 2, RIGHT = 3 };

// 18 canonical segment-type words (the 48 RS paths are these plus the
// timeflip/reflect symmetries applied to the segment lengths).
static const int PATH_TYPE[18][5] = {
    {LEFT, RIGHT, LEFT, NOP, NOP},           // 0
    {RIGHT, LEFT, RIGHT, NOP, NOP},          // 1
    {LEFT, RIGHT, LEFT, RIGHT, NOP},         // 2
    {RIGHT, LEFT, RIGHT, LEFT, NOP},         // 3
    {LEFT, RIGHT, STRAIGHT, LEFT, NOP},      // 4
    {RIGHT, LEFT, STRAIGHT, RIGHT, NOP},     // 5
    {LEFT, STRAIGHT, RIGHT, LEFT, NOP},      // 6
    {RIGHT, STRAIGHT, LEFT, RIGHT, NOP},     // 7
    {LEFT, RIGHT, STRAIGHT, RIGHT, NOP},     // 8
    {RIGHT, LEFT, STRAIGHT, LEFT, NOP},      // 9
    {RIGHT, STRAIGHT, RIGHT, LEFT, NOP},     // 10
    {LEFT, STRAIGHT, LEFT, RIGHT, NOP},      // 11
    {LEFT, STRAIGHT, RIGHT, NOP, NOP},       // 12
    {RIGHT, STRAIGHT, LEFT, NOP, NOP},       // 13
    {LEFT, STRAIGHT, LEFT, NOP, NOP},        // 14
    {RIGHT, STRAIGHT, RIGHT, NOP, NOP},      // 15
    {LEFT, RIGHT, STRAIGHT, LEFT, RIGHT},    // 16
    {RIGHT, LEFT, STRAIGHT, RIGHT, LEFT}     // 17
};

inline double mod2pi(double x) {
    double v = std::fmod(x, RS_2PI);
    if (v < -RS_PI) v += RS_2PI;
    else if (v > RS_PI) v -= RS_2PI;
    return v;
}
inline void polar(double x, double y, double& r, double& theta) {
    r = std::hypot(x, y);
    theta = std::atan2(y, x);
}
inline void tauOmega(double u, double v, double xi, double eta, double phi,
                     double& tau, double& omega) {
    double delta = mod2pi(u - v), A = std::sin(u) - std::sin(delta),
           B = std::cos(u) - std::cos(delta) - 1.0;
    double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B),
           t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
    tau = (t2 < 0) ? mod2pi(t1 + RS_PI) : mod2pi(t1);
    omega = mod2pi(tau - u + v - phi);
}

struct Path {
    const int* type = nullptr;   // points to a PATH_TYPE row, or null for "empty"
    double len[5] = {0, 0, 0, 0, 0};
    double L = std::numeric_limits<double>::infinity();  // total |length|
};

inline void set(Path& p, const int row[5],
                double a, double b = 0, double c = 0, double d = 0, double e = 0) {
    p.type = row;
    p.len[0] = a; p.len[1] = b; p.len[2] = c; p.len[3] = d; p.len[4] = e;
    p.L = std::fabs(a) + std::fabs(b) + std::fabs(c) + std::fabs(d) + std::fabs(e);
}

// ---- leaf word generators -------------------------------------------------
inline bool LpSpLp(double x, double y, double phi, double& t, double& u, double& v) {
    polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
    if (t >= -ZERO) {
        v = mod2pi(phi - t);
        if (v >= -ZERO) return true;
    }
    return false;
}
inline bool LpSpRp(double x, double y, double phi, double& t, double& u, double& v) {
    double t1, u1;
    polar(x + std::sin(phi), y - 1.0 - std::cos(phi), u1, t1);
    u1 = u1 * u1;
    if (u1 >= 4.0) {
        u = std::sqrt(u1 - 4.0);
        double theta = std::atan2(2.0, u);
        t = mod2pi(t1 + theta);
        v = mod2pi(t - phi);
        return t >= -ZERO && v >= -ZERO;
    }
    return false;
}
inline bool LpRmL(double x, double y, double phi, double& t, double& u, double& v) {
    double xi = x - std::sin(phi), eta = y - 1.0 + std::cos(phi), u1, theta;
    polar(xi, eta, u1, theta);
    if (u1 <= 4.0) {
        u = -2.0 * std::asin(0.25 * u1);
        t = mod2pi(theta + 0.5 * u + RS_PI);
        v = mod2pi(phi - t + u);
        return t >= -ZERO && u <= ZERO;
    }
    return false;
}
inline bool LpRupLumRm(double x, double y, double phi, double& t, double& u, double& v) {
    double xi = x + std::sin(phi), eta = y - 1.0 - std::cos(phi),
           rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
    if (rho <= 1.0) {
        u = std::acos(rho);
        tauOmega(u, -u, xi, eta, phi, t, v);
        return t >= -ZERO && v <= ZERO;
    }
    return false;
}
inline bool LpRumLumRp(double x, double y, double phi, double& t, double& u, double& v) {
    double xi = x + std::sin(phi), eta = y - 1.0 - std::cos(phi),
           rho = (20.0 - xi * xi - eta * eta) / 16.0;
    if (rho >= 0 && rho <= 1.0) {
        u = -std::acos(rho);
        if (u >= -0.5 * RS_PI) {
            tauOmega(u, u, xi, eta, phi, t, v);
            return t >= -ZERO && v >= -ZERO;
        }
    }
    return false;
}
inline bool LpRmSmLm(double x, double y, double phi, double& t, double& u, double& v) {
    double xi = x - std::sin(phi), eta = y - 1.0 + std::cos(phi), rho, theta;
    polar(xi, eta, rho, theta);
    if (rho >= 2.0) {
        double r = std::sqrt(rho * rho - 4.0);
        u = 2.0 - r;
        t = mod2pi(theta + std::atan2(r, -2.0));
        v = mod2pi(phi - 0.5 * RS_PI - t);
        return t >= -ZERO && u <= ZERO && v <= ZERO;
    }
    return false;
}
inline bool LpRmSmRm(double x, double y, double phi, double& t, double& u, double& v) {
    double xi = x + std::sin(phi), eta = y - 1.0 - std::cos(phi), rho, theta;
    polar(-eta, xi, rho, theta);
    if (rho >= 2.0) {
        t = theta;
        u = 2.0 - rho;
        v = mod2pi(t + 0.5 * RS_PI - phi);
        return t >= -ZERO && u <= ZERO && v <= ZERO;
    }
    return false;
}
inline bool LpRmSLmRp(double x, double y, double phi, double& t, double& u, double& v) {
    double xi = x + std::sin(phi), eta = y - 1.0 - std::cos(phi), rho, theta;
    polar(xi, eta, rho, theta);
    if (rho >= 2.0) {
        u = 4.0 - std::sqrt(rho * rho - 4.0);
        if (u <= ZERO) {
            t = mod2pi(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
            v = mod2pi(t - phi);
            return t >= -ZERO && v >= -ZERO;
        }
    }
    return false;
}

// ---- word families (each applies the four base symmetries) ----------------
inline void CSC(double x, double y, double phi, Path& path) {
    double t, u, v, Lmin = path.L, L;
    if (LpSpLp( x,  y,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[14],  t,  u,  v); Lmin = L; }
    if (LpSpLp(-x,  y, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[14], -t, -u, -v); Lmin = L; }
    if (LpSpLp( x, -y, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[15],  t,  u,  v); Lmin = L; }
    if (LpSpLp(-x, -y,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[15], -t, -u, -v); Lmin = L; }
    if (LpSpRp( x,  y,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[12],  t,  u,  v); Lmin = L; }
    if (LpSpRp(-x,  y, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[12], -t, -u, -v); Lmin = L; }
    if (LpSpRp( x, -y, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[13],  t,  u,  v); Lmin = L; }
    if (LpSpRp(-x, -y,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[13], -t, -u, -v); }
}
inline void CCC(double x, double y, double phi, Path& path) {
    double t, u, v, Lmin = path.L, L;
    if (LpRmL( x,  y,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[0],  t,  u,  v); Lmin = L; }
    if (LpRmL(-x,  y, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[0], -t, -u, -v); Lmin = L; }
    if (LpRmL( x, -y, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[1],  t,  u,  v); Lmin = L; }
    if (LpRmL(-x, -y,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[1], -t, -u, -v); Lmin = L; }

    double xb = x * std::cos(phi) + y * std::sin(phi), yb = x * std::sin(phi) - y * std::cos(phi);
    if (LpRmL( xb,  yb,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[0],  v,  u,  t); Lmin = L; }
    if (LpRmL(-xb,  yb, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[0], -v, -u, -t); Lmin = L; }
    if (LpRmL( xb, -yb, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[1],  v,  u,  t); Lmin = L; }
    if (LpRmL(-xb, -yb,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[1], -v, -u, -t); }
}
inline void CCCC(double x, double y, double phi, Path& path) {
    double t, u, v, Lmin = path.L, L;
    if (LpRupLumRm( x,  y,  phi, t, u, v) && Lmin > (L = std::fabs(t)+2.0*std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[2],  t,  u, -u,  v); Lmin = L; }
    if (LpRupLumRm(-x,  y, -phi, t, u, v) && Lmin > (L = std::fabs(t)+2.0*std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[2], -t, -u,  u, -v); Lmin = L; }
    if (LpRupLumRm( x, -y, -phi, t, u, v) && Lmin > (L = std::fabs(t)+2.0*std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[3],  t,  u, -u,  v); Lmin = L; }
    if (LpRupLumRm(-x, -y,  phi, t, u, v) && Lmin > (L = std::fabs(t)+2.0*std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[3], -t, -u,  u, -v); Lmin = L; }

    if (LpRumLumRp( x,  y,  phi, t, u, v) && Lmin > (L = std::fabs(t)+2.0*std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[2],  t,  u,  u,  v); Lmin = L; }
    if (LpRumLumRp(-x,  y, -phi, t, u, v) && Lmin > (L = std::fabs(t)+2.0*std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[2], -t, -u, -u, -v); Lmin = L; }
    if (LpRumLumRp( x, -y, -phi, t, u, v) && Lmin > (L = std::fabs(t)+2.0*std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[3],  t,  u,  u,  v); Lmin = L; }
    if (LpRumLumRp(-x, -y,  phi, t, u, v) && Lmin > (L = std::fabs(t)+2.0*std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[3], -t, -u, -u, -v); }
}
inline void CCSC(double x, double y, double phi, Path& path) {
    double t, u, v, Lmin = path.L - 0.5 * RS_PI, L;
    if (LpRmSmLm( x,  y,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[4],  t, -0.5*RS_PI,  u,  v); Lmin = L; }
    if (LpRmSmLm(-x,  y, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[4], -t,  0.5*RS_PI, -u, -v); Lmin = L; }
    if (LpRmSmLm( x, -y, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[5],  t, -0.5*RS_PI,  u,  v); Lmin = L; }
    if (LpRmSmLm(-x, -y,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[5], -t,  0.5*RS_PI, -u, -v); Lmin = L; }

    if (LpRmSmRm( x,  y,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[8],  t, -0.5*RS_PI,  u,  v); Lmin = L; }
    if (LpRmSmRm(-x,  y, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[8], -t,  0.5*RS_PI, -u, -v); Lmin = L; }
    if (LpRmSmRm( x, -y, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[9],  t, -0.5*RS_PI,  u,  v); Lmin = L; }
    if (LpRmSmRm(-x, -y,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[9], -t,  0.5*RS_PI, -u, -v); Lmin = L; }

    double xb = x * std::cos(phi) + y * std::sin(phi), yb = x * std::sin(phi) - y * std::cos(phi);
    if (LpRmSmLm( xb,  yb,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[6],  v,  u, -0.5*RS_PI,  t); Lmin = L; }
    if (LpRmSmLm(-xb,  yb, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[6], -v, -u,  0.5*RS_PI, -t); Lmin = L; }
    if (LpRmSmLm( xb, -yb, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[7],  v,  u, -0.5*RS_PI,  t); Lmin = L; }
    if (LpRmSmLm(-xb, -yb,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[7], -v, -u,  0.5*RS_PI, -t); Lmin = L; }

    if (LpRmSmRm( xb,  yb,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[10],  v,  u, -0.5*RS_PI,  t); Lmin = L; }
    if (LpRmSmRm(-xb,  yb, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[10], -v, -u,  0.5*RS_PI, -t); Lmin = L; }
    if (LpRmSmRm( xb, -yb, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[11],  v,  u, -0.5*RS_PI,  t); Lmin = L; }
    if (LpRmSmRm(-xb, -yb,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[11], -v, -u,  0.5*RS_PI, -t); }
}
inline void CCSCC(double x, double y, double phi, Path& path) {
    double t, u, v, Lmin = path.L - RS_PI, L;
    if (LpRmSLmRp( x,  y,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[16],  t, -0.5*RS_PI,  u, -0.5*RS_PI,  v); Lmin = L; }
    if (LpRmSLmRp(-x,  y, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[16], -t,  0.5*RS_PI, -u,  0.5*RS_PI, -v); Lmin = L; }
    if (LpRmSLmRp( x, -y, -phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[17],  t, -0.5*RS_PI,  u, -0.5*RS_PI,  v); Lmin = L; }
    if (LpRmSLmRp(-x, -y,  phi, t, u, v) && Lmin > (L = std::fabs(t)+std::fabs(u)+std::fabs(v))) { set(path, PATH_TYPE[17], -t,  0.5*RS_PI, -u,  0.5*RS_PI, -v); }
}

inline Path getPath(double x, double y, double phi) {
    Path path;
    CSC(x, y, phi, path);
    CCC(x, y, phi, path);
    CCCC(x, y, phi, path);
    CCSC(x, y, phi, path);
    CCSCC(x, y, phi, path);
    return path;
}

// Shortest Reeds-Shepp distance between two oriented poses for turning radius rho.
inline double distance(double x0, double y0, double th0,
                       double x1, double y1, double th1, double rho) {
    double dx = x1 - x0, dy = y1 - y0, c = std::cos(th0), s = std::sin(th0);
    double xr = (c * dx + s * dy) / rho;
    double yr = (-s * dx + c * dy) / rho;
    double phi = th1 - th0;
    return getPath(xr, yr, phi).L * rho;
}

struct Sample { double x, y, theta; int gear; };  // gear: +1 forward, -1 reverse

// Replay the optimal word as world poses spaced ~ds apart (excludes the start
// pose, includes the goal). gear is the travel direction of each sub-segment.
inline std::vector<Sample> samplePath(double x0, double y0, double th0,
                                      double x1, double y1, double th1,
                                      double rho, double ds, double& out_len) {
    double dx = x1 - x0, dy = y1 - y0, c = std::cos(th0), s = std::sin(th0);
    double xr = (c * dx + s * dy) / rho;
    double yr = (-s * dx + c * dy) / rho;
    double phi = th1 - th0;
    Path p = getPath(xr, yr, phi);
    out_len = p.L * rho;

    std::vector<Sample> out;
    if (!p.type || !std::isfinite(p.L)) return out;

    double ux = 0.0, uy = 0.0, yaw = th0;  // accumulate in world-aligned unit frame
    for (int i = 0; i < 5; ++i) {
        if (p.type[i] == NOP) continue;
        double Li = p.len[i];
        if (std::fabs(Li) < 1e-9) continue;
        int gear = (Li >= 0.0) ? 1 : -1;
        int n = std::max(1, static_cast<int>(std::ceil(std::fabs(Li) * rho / ds)));
        double du = Li / n;
        for (int k = 0; k < n; ++k) {
            double ph = yaw;
            switch (p.type[i]) {
                case LEFT:
                    ux += std::sin(ph + du) - std::sin(ph);
                    uy += -std::cos(ph + du) + std::cos(ph);
                    yaw = ph + du; break;
                case RIGHT:
                    ux += -std::sin(ph - du) + std::sin(ph);
                    uy += std::cos(ph - du) - std::cos(ph);
                    yaw = ph - du; break;
                case STRAIGHT:
                    ux += du * std::cos(ph);
                    uy += du * std::sin(ph);
                    break;
                default: break;
            }
            out.push_back({x0 + ux * rho, y0 + uy * rho, mod2pi(yaw), gear});
        }
    }
    return out;
}

}  // namespace rs

// =============================================================================
// Planner node
// =============================================================================
struct Node3D {
    double x, y, theta;
    double kappa;
    int    direction;     // gear of the motion arriving at this node (+1/-1)
    double g, f;
    int    parent_idx;
};

struct TupleHash {
    std::size_t operator()(const std::tuple<int,int,int>& t) const noexcept {
        std::size_t h = 0;
        auto combine = [&h](int v) {
            h ^= std::hash<int>{}(v) + 0x9e3779b9u + (h << 6) + (h >> 2);
        };
        combine(std::get<0>(t));
        combine(std::get<1>(t));
        combine(std::get<2>(t));
        return h;
    }
};

using Index3D    = std::tuple<int,int,int>;
using QueueEntry = std::pair<double, Index3D>;
using WorldPoint = std::pair<double,double>;

struct PlannerParams {
    double default_tolerance;
    double turning_radius;
    double step_size;
    int    max_iterations;
    double deviation_threshold;
    int    lethal_cost_threshold;
    double unknown_cost_penalty;
    int    theta_bins;
    int    stall_dir;
    double escape_radius;
    double goal_yaw_tol;
    double heuristic_weight;        // 1.0 = admissible (paper); >1 trades optimality for speed
    bool   smoothing_enabled;
    int    smooth_iterations;
    double w_smooth;
    double w_obstacle;
    double obstacle_dmax;           // [m] range of the smoother's obstacle term
    double w_voronoi;               // weight of the Voronoi-field guidance term
};

struct EscapeBubble {
    double x = 0.0, y = 0.0, radius = 0.0;
    bool   active = false;
};

// Banded obstacle distance field used by the path smoother's obstacle term:
// for each cell within obstacle_dmax of a lethal cell, the distance and the
// world coordinates of the nearest lethal cell (so the gradient direction is
// "straight away from the obstacle").
struct ObstacleField {
    int    w = 0, h = 0;
    double res = 0.0, ox = 0.0, oy = 0.0;
    double dmax = 0.0;
    std::vector<float> dist;   // [m]; BIG where farther than dmax
    std::vector<float> nx, ny; // nearest lethal-cell centre [m]
    static constexpr float BIG = 1e9f;

    bool inBounds(int cx, int cy) const { return cx >= 0 && cx < w && cy >= 0 && cy < h; }
    int idxOf(double wx, double wy) const {
        int cx = static_cast<int>(std::floor((wx - ox) / res));
        int cy = static_cast<int>(std::floor((wy - oy) / res));
        if (!inBounds(cx, cy)) return -1;
        return cx + cy * w;
    }
};

class AckermannAStarPlannerNode : public rclcpp::Node
{
public:
    AckermannAStarPlannerNode()
      : Node("ackermann_astar_planner"), new_goal_set_(false), plan_pending_(false), stall_dir_(0)
    {
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        this->declare_parameter("default_tolerance",      0.25);
        this->declare_parameter("turning_radius",         0.35);
        this->declare_parameter("step_size",              0.15);
        this->declare_parameter("max_iterations",         100000);
        this->declare_parameter("deviation_threshold",    0.20);
        this->declare_parameter("lethal_cost_threshold",  85);
        this->declare_parameter("unknown_cost_penalty",   15.0);
        this->declare_parameter("theta_bins",             72);
        this->declare_parameter("escape_radius",          0.50);
        this->declare_parameter("goal_yaw_tolerance_deg", 15.0);
        // Paper-faithful additions:
        this->declare_parameter("heuristic_weight",       1.0);
        this->declare_parameter("smoothing_enabled",      true);
        this->declare_parameter("smooth_iterations",      120);
        this->declare_parameter("w_smooth",               0.20);
        this->declare_parameter("w_obstacle",             0.12);
        this->declare_parameter("obstacle_dmax",          0.35);
        this->declare_parameter("w_voronoi",              0.10);
        // Match to the controller's max_linear_velocity so the planned decel
        // ramp targets the speed the controller will actually reach.
        this->declare_parameter("cruise_velocity",        1.0);
        // Rear-axle-to-centre distance (= wheelbase); the planner plans the rear
        // axle, the goal is offset back by L/2 so the car centre lands on it.
        this->declare_parameter("wheelbase",              0.1688);

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/inflated_costmap", 10,
            std::bind(&AckermannAStarPlannerNode::mapCallback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&AckermannAStarPlannerNode::goalCallback, this, std::placeholders::_1));

        stall_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/robot_stall", 10,
            std::bind(&AckermannAStarPlannerNode::stallCallback, this, std::placeholders::_1));

        // Latched QoS to match voronoi_costmap's transient-local publisher.
        voronoi_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/voronoi_field", rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&AckermannAStarPlannerNode::voronoiCallback, this, std::placeholders::_1));

        path_pub_        = this->create_publisher<nav_msgs::msg::Path>("/followed_path", 10);
        visual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/visual_path",   10);

        plan_timer_ = this->create_wall_timer(
            200ms, std::bind(&AckermannAStarPlannerNode::replanTimerCallback, this));

        RCLCPP_INFO(this->get_logger(),
            "Ackermann Hybrid A* (Dolgov et al.) ready: RS analytic expansion + dual heuristic + smoother.");
    }

private:
    // ── ROS plumbing ────────────────────────────────────────────────────────
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(map_mutex_);
        map_msg_ = msg;
        map_dirty_ = true;   // a new costmap may invalidate the current path -> replan
    }
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(map_mutex_);
        latest_goal_pose_ = msg;
        new_goal_set_     = true;
        plan_pending_     = true;
    }
    void stallCallback(const std_msgs::msg::Int8::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(map_mutex_);
        stall_dir_ = msg->data;
    }
    void voronoiCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(map_mutex_);
        voronoi_msg_ = msg;
    }

    // ── grid helpers ────────────────────────────────────────────────────────
    Index3D worldTo3DIndex(double wx, double wy, double theta,
                           const nav_msgs::msg::OccupancyGrid& map, int theta_bins) const {
        int cx = static_cast<int>(std::floor((wx - map.info.origin.position.x) / map.info.resolution));
        int cy = static_cast<int>(std::floor((wy - map.info.origin.position.y) / map.info.resolution));
        int ct = static_cast<int>(std::floor(wrap2Pi(theta) / (TWO_PI / theta_bins))) % theta_bins;
        return {cx, cy, ct};
    }

    int cellValue(double wx, double wy, const nav_msgs::msg::OccupancyGrid& map) const {
        int cx = static_cast<int>(std::floor((wx - map.info.origin.position.x) / map.info.resolution));
        int cy = static_cast<int>(std::floor((wy - map.info.origin.position.y) / map.info.resolution));
        if (cx < 0 || cx >= static_cast<int>(map.info.width) || cy < 0 || cy >= static_cast<int>(map.info.height))
            return -2;
        return static_cast<int>(map.data[cx + cy * static_cast<int>(map.info.width)]);
    }

    bool cellIsFree(double wx, double wy, const nav_msgs::msg::OccupancyGrid& map, int max_cost,
                    const EscapeBubble& esc = EscapeBubble{}) const {
        for (const auto& obs : virtual_obstacles_) {
            if (std::hypot(wx - obs.first, wy - obs.second) < 0.22) return false;
        }
        int v = cellValue(wx, wy, map);
        if (v == -2) return false;
        if (v == -1) return true;
        if (v < max_cost) return true;
        return esc.active && v < 100 && std::hypot(wx - esc.x, wy - esc.y) < esc.radius;
    }

    bool propagateArc(double x0, double y0, double th0, double kappa, double arc_len, int dir,
                      const nav_msgs::msg::OccupancyGrid& map, int max_cost,
                      double& xf, double& yf, double& thf,
                      const EscapeBubble& esc = EscapeBubble{}) const {
        int steps = std::max(3, static_cast<int>(arc_len / (map.info.resolution * 0.4)));
        double ds = arc_len / steps; double x = x0, y = y0, th = th0;
        for (int i = 1; i <= steps; ++i) {
            x  += dir * ds * std::cos(th);
            y  += dir * ds * std::sin(th);
            th  = wrapPi(th + dir * ds * kappa);
            if (!cellIsFree(x, y, map, max_cost, esc)) return false;
        }
        xf = x; yf = y; thf = th;
        return true;
    }

    static std::vector<Node3D> reconstructPath(const std::vector<Node3D>& pool, int end_idx) {
        std::vector<Node3D> path;
        for (int i = end_idx; i != -1; i = pool[i].parent_idx) path.push_back(pool[i]);
        std::reverse(path.begin(), path.end());
        return path;
    }

    // True if the portion of the current path from the robot onward intersects a
    // lethal cell of the given map (densely sampled between waypoints). Used to
    // decide whether a costmap update actually invalidates the committed path.
    bool pathAheadBlocked(const std::vector<WorldPoint>& path, double sx, double sy,
                          const nav_msgs::msg::OccupancyGrid& map, int max_cost) const {
        if (path.empty()) return false;
        size_t closest = 0; double best = std::numeric_limits<double>::max();
        for (size_t i = 0; i < path.size(); ++i) {
            double d = std::hypot(path[i].first - sx, path[i].second - sy);
            if (d < best) { best = d; closest = i; }
        }
        const double ds = std::max(map.info.resolution * 0.5, 1e-3);
        for (size_t i = closest; i < path.size(); ++i) {
            if (!cellIsFree(path[i].first, path[i].second, map, max_cost)) return true;
            if (i + 1 < path.size()) {
                double x1 = path[i].first, y1 = path[i].second;
                double x2 = path[i+1].first, y2 = path[i+1].second;
                int steps = std::max(1, static_cast<int>(std::hypot(x2 - x1, y2 - y1) / ds));
                for (int s = 1; s < steps; ++s) {
                    double t = static_cast<double>(s) / steps;
                    if (!cellIsFree(x1 + t*(x2-x1), y1 + t*(y2-y1), map, max_cost)) return true;
                }
            }
        }
        return false;
    }

    // ── heuristic 2: holonomic-with-obstacles (2D Dijkstra from the goal) ────
    // Permissive free space (only hard occupancy blocks) so the result is a
    // genuine lower bound on the achievable path length — i.e. admissible.
    std::vector<float> holonomicHeuristic(const nav_msgs::msg::OccupancyGrid& map,
                                          double gx, double gy) const {
        const int w = static_cast<int>(map.info.width), h = static_cast<int>(map.info.height);
        const double res = map.info.resolution;
        std::vector<float> dist(static_cast<size_t>(w) * h, std::numeric_limits<float>::infinity());

        int gcx = static_cast<int>(std::floor((gx - map.info.origin.position.x) / res));
        int gcy = static_cast<int>(std::floor((gy - map.info.origin.position.y) / res));
        if (gcx < 0 || gcx >= w || gcy < 0 || gcy >= h) return dist;

        using QE = std::pair<float, int>;
        std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
        const int gidx = gcx + gcy * w;
        dist[gidx] = 0.0f;
        pq.push({0.0f, gidx});

        const float diag = static_cast<float>(res * std::sqrt(2.0)), ortho = static_cast<float>(res);
        while (!pq.empty()) {
            auto [d, idx] = pq.top(); pq.pop();
            if (d > dist[idx]) continue;
            int cx = idx % w, cy = idx / w;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (!dx && !dy) continue;
                    int nx = cx + dx, ny = cy + dy;
                    if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                    int nidx = nx + ny * w;
                    if (map.data[nidx] == 100) continue;           // only hard occupancy blocks
                    float nd = d + ((dx && dy) ? diag : ortho);
                    if (nd < dist[nidx]) { dist[nidx] = nd; pq.push({nd, nidx}); }
                }
            }
        }
        return dist;
    }

    float lookupHolo(const std::vector<float>& field, const nav_msgs::msg::OccupancyGrid& map,
                     double wx, double wy) const {
        const int w = static_cast<int>(map.info.width), h = static_cast<int>(map.info.height);
        int cx = static_cast<int>(std::floor((wx - map.info.origin.position.x) / map.info.resolution));
        int cy = static_cast<int>(std::floor((wy - map.info.origin.position.y) / map.info.resolution));
        if (cx < 0 || cx >= w || cy < 0 || cy >= h) return std::numeric_limits<float>::infinity();
        return field[cx + cy * w];
    }

    // ── velocity profile (unchanged contract: signed magnitude in pose.z) ────
    std::vector<double> computeVelocityProfile(const std::vector<Node3D>& nodes) const {
        const size_t n = nodes.size(); if (n == 0) return {};
        // Cruise is a parameter so it can be matched to the controller's
        // max_linear_velocity (the decel ramp is planned against this value).
        const double V_CRUISE = this->get_parameter("cruise_velocity").as_double();
        constexpr double A_MAX = 0.8; constexpr double D_MAX = 0.8; constexpr double V_MIN = 0.15;
        if (n == 1) return {0.0};

        auto seg_dir = [&nodes, n](size_t i) {
            return (i + 1 < n) ? nodes[i + 1].direction : nodes[n - 1].direction;
        };

        std::vector<double> mag(n, V_CRUISE);
        std::vector<char> is_cusp(n, 0);
        mag[0] = 0.0; mag[n - 1] = 0.0;
        for (size_t i = 1; i + 1 < n; ++i)
            if (seg_dir(i) != seg_dir(i - 1)) { mag[i] = 0.0; is_cusp[i] = 1; }   // cusp: full stop

        for (size_t i = n - 1; i-- > 0; ) {
            double ds = std::hypot(nodes[i+1].x - nodes[i].x, nodes[i+1].y - nodes[i].y);
            mag[i] = std::min(mag[i], std::sqrt(mag[i+1]*mag[i+1] + 2.0*D_MAX*ds));
        }
        for (size_t i = 1; i < n; ++i) {
            double ds = std::hypot(nodes[i].x - nodes[i-1].x, nodes[i].y - nodes[i-1].y);
            mag[i] = std::min(mag[i], std::sqrt(mag[i-1]*mag[i-1] + 2.0*A_MAX*ds));
        }
        // Floor non-final, non-cusp waypoints so the car keeps a drivable speed;
        // cusps stay at 0 so it comes to rest before reversing (the controller
        // keys its gear handoff off this stop).
        for (size_t i = 0; i + 1 < n; ++i) if (!is_cusp[i]) mag[i] = std::max(mag[i], V_MIN);
        for (size_t i = 0; i < n; ++i) if (seg_dir(i) == -1) mag[i] = -mag[i];
        return mag;
    }

    // ── second phase: banded obstacle field + gradient-descent smoother ──────
    ObstacleField buildObstacleField(const nav_msgs::msg::OccupancyGrid& map, double dmax) const {
        ObstacleField f;
        f.w = static_cast<int>(map.info.width); f.h = static_cast<int>(map.info.height);
        f.res = map.info.resolution; f.ox = map.info.origin.position.x; f.oy = map.info.origin.position.y;
        f.dmax = dmax;
        const size_t N = static_cast<size_t>(f.w) * f.h;
        f.dist.assign(N, ObstacleField::BIG);
        f.nx.assign(N, 0.0f); f.ny.assign(N, 0.0f);

        // label-correcting flood within the band: seed lethal cells, then relax
        // outward, carrying the nearest lethal-cell coordinate.
        std::queue<int> q;
        for (size_t i = 0; i < N; ++i) {
            if (map.data[i] == 100) {
                int cx = static_cast<int>(i) % f.w, cy = static_cast<int>(i) / f.w;
                f.dist[i] = 0.0f;
                f.nx[i] = static_cast<float>(f.ox + (cx + 0.5) * f.res);
                f.ny[i] = static_cast<float>(f.oy + (cy + 0.5) * f.res);
                q.push(static_cast<int>(i));
            }
        }
        while (!q.empty()) {
            int idx = q.front(); q.pop();
            int cx = idx % f.w, cy = idx / f.w;
            float sx = f.nx[idx], sy = f.ny[idx];
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (!dx && !dy) continue;
                    int nxx = cx + dx, nyy = cy + dy;
                    if (!f.inBounds(nxx, nyy)) continue;
                    int nidx = nxx + nyy * f.w;
                    if (map.data[nidx] == 100) continue;
                    float wxc = static_cast<float>(f.ox + (nxx + 0.5) * f.res);
                    float wyc = static_cast<float>(f.oy + (nyy + 0.5) * f.res);
                    float d = std::hypot(wxc - sx, wyc - sy);
                    if (d < f.dist[nidx] && d <= static_cast<float>(dmax)) {
                        f.dist[nidx] = d; f.nx[nidx] = sx; f.ny[nidx] = sy;
                        q.push(nidx);
                    }
                }
            }
        }
        return f;
    }

    // Sample ρ_V (in [0,1]) from the published Voronoi field at a world point.
    bool voronoiValue(const nav_msgs::msg::OccupancyGrid& vf,
                      double wx, double wy, double& out) const {
        int cx = static_cast<int>(std::floor((wx - vf.info.origin.position.x) / vf.info.resolution));
        int cy = static_cast<int>(std::floor((wy - vf.info.origin.position.y) / vf.info.resolution));
        if (cx < 0 || cx >= static_cast<int>(vf.info.width) || cy < 0 || cy >= static_cast<int>(vf.info.height))
            return false;
        int8_t v = vf.data[cx + cy * static_cast<int>(vf.info.width)];
        if (v < 0) return false;   // unknown: no gradient information here
        out = v / 100.0;
        return true;
    }

    // Central-difference gradient ∇ρ_V at a world point (points toward obstacles);
    // the smoother descends it (−∇ρ_V) to ride the Voronoi ridge.
    bool voronoiGradient(const nav_msgs::msg::OccupancyGrid& vf,
                         double wx, double wy, double& gx, double& gy) const {
        const double eps = vf.info.resolution;
        double rpx, rmx, rpy, rmy;
        if (!voronoiValue(vf, wx + eps, wy, rpx) || !voronoiValue(vf, wx - eps, wy, rmx) ||
            !voronoiValue(vf, wx, wy + eps, rpy) || !voronoiValue(vf, wx, wy - eps, rmy))
            return false;
        gx = (rpx - rmx) / (2.0 * eps);
        gy = (rpy - rmy) / (2.0 * eps);
        return true;
    }

    // Smooths interior, non-cusp waypoints in place. Endpoints and cusps stay
    // anchored (keeps the maneuver structure); moves that would collide or
    // exceed the curvature limit are rejected, so feasibility is preserved.
    // When a Voronoi field is supplied, a −∇ρ_V term pulls the path onto the
    // medial axis through narrow passages (the paper's Voronoi term).
    void smoothPath(std::vector<Node3D>& nodes, const ObstacleField& field,
                    const nav_msgs::msg::OccupancyGrid& map, const PlannerParams& p,
                    const EscapeBubble& esc, const nav_msgs::msg::OccupancyGrid* voronoi) const {
        const size_t n = nodes.size();
        if (n < 5) return;

        // anchor[i] true => never move this vertex (endpoints + cusps + their neighbours)
        std::vector<char> anchor(n, 0);
        anchor[0] = anchor[1] = anchor[n-1] = anchor[n-2] = 1;
        for (size_t i = 1; i + 1 < n; ++i) {
            if (nodes[i].direction != nodes[i+1].direction) {
                anchor[i] = 1;
                if (i > 0) anchor[i-1] = 1;
                if (i + 1 < n) anchor[i+1] = 1;
            }
        }

        const double kappa_max = 1.0 / p.turning_radius;
        for (int it = 0; it < p.smooth_iterations; ++it) {
            for (size_t i = 2; i + 2 < n; ++i) {
                if (anchor[i]) continue;
                double xi = nodes[i].x, yi = nodes[i].y;
                double cx = 0.0, cy = 0.0;

                // smoothness term: pull toward the average of the neighbours
                cx += p.w_smooth * (nodes[i-1].x - 2.0 * xi + nodes[i+1].x);
                cy += p.w_smooth * (nodes[i-1].y - 2.0 * yi + nodes[i+1].y);

                // obstacle term: push directly away from the nearest lethal cell
                int fidx = field.idxOf(xi, yi);
                if (fidx >= 0 && field.dist[fidx] < field.dmax) {
                    double d = field.dist[fidx];
                    if (d > 1e-3) {
                        double ox = (xi - field.nx[fidx]) / d, oy = (yi - field.ny[fidx]) / d;
                        double mag = p.w_obstacle * (field.dmax - d);
                        cx += mag * ox; cy += mag * oy;
                    }
                }

                // Voronoi term: descend ρ_V toward the medial-axis ridge so the
                // path centres itself in narrow passages (Dolgov et al. §Voronoi).
                if (voronoi && p.w_voronoi > 0.0) {
                    double vgx, vgy;
                    if (voronoiGradient(*voronoi, xi, yi, vgx, vgy)) {
                        cx -= p.w_voronoi * vgx; cy -= p.w_voronoi * vgy;
                    }
                }

                // Bound each step so the disparate term scales stay numerically
                // stable (the ρ_V gradient can be much larger than the others).
                double step = std::hypot(cx, cy), step_cap = 0.5 * map.info.resolution;
                if (step > step_cap && step > 1e-9) { double s = step_cap / step; cx *= s; cy *= s; }

                double nxp = xi + cx, nyp = yi + cy;

                // reject if the new vertex collides
                if (!cellIsFree(nxp, nyp, map, p.lethal_cost_threshold, esc)) continue;

                // reject if it makes either incident turn tighter than the car can drive
                if (discreteCurvature(nodes[i-1].x, nodes[i-1].y, nxp, nyp, nodes[i+1].x, nodes[i+1].y) > kappa_max)
                    continue;

                nodes[i].x = nxp; nodes[i].y = nyp;
            }
        }

        // recompute headings from the smoothed geometry, honouring gear (a
        // reversing segment's body heading is opposite its travel bearing).
        for (size_t i = 1; i + 1 < n; ++i) {
            double bearing = std::atan2(nodes[i+1].y - nodes[i-1].y, nodes[i+1].x - nodes[i-1].x);
            nodes[i].theta = (nodes[i].direction < 0) ? wrapPi(bearing + PI) : bearing;
        }
    }

    static double discreteCurvature(double ax, double ay, double bx, double by, double cx, double cy) {
        double d1x = bx - ax, d1y = by - ay, d2x = cx - bx, d2y = cy - by;
        double l1 = std::hypot(d1x, d1y), l2 = std::hypot(d2x, d2y);
        if (l1 < 1e-6 || l2 < 1e-6) return 0.0;
        double dot = (d1x * d2x + d1y * d2y) / (l1 * l2);
        dot = std::max(-1.0, std::min(1.0, dot));
        double dphi = std::acos(dot);
        return dphi / std::min(l1, l2);   // turn angle per unit arc ≈ curvature
    }

    // ── main timer: identical control flow to astar_planner.cpp ──────────────
    void replanTimerCallback()
    {
        nav_msgs::msg::OccupancyGrid::SharedPtr    local_map;
        nav_msgs::msg::OccupancyGrid::SharedPtr    local_voronoi;
        geometry_msgs::msg::PoseStamped::SharedPtr local_goal;
        bool local_new_goal, local_plan_pending, local_map_dirty;
        int  local_stall_dir;
        {
            std::lock_guard<std::mutex> lk(map_mutex_);
            local_map          = map_msg_;
            local_voronoi      = voronoi_msg_;
            local_goal         = latest_goal_pose_;
            local_new_goal     = new_goal_set_;
            local_plan_pending = plan_pending_;
            local_stall_dir    = stall_dir_;
            local_map_dirty    = map_dirty_;
            map_dirty_         = false;   // consume: one map update -> at most one replan
        }
        if (!local_goal) return;   // no goal requested yet — normal idle, stay quiet
        if (!local_map) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Goal received but no /inflated_costmap yet — is costmap_node running and is SLAM "
                "publishing /map? (sim needs use_slam:=true use_nav:=true)");
            return;
        }

        PlannerParams p;
        p.default_tolerance     = this->get_parameter("default_tolerance").as_double();
        p.turning_radius        = this->get_parameter("turning_radius").as_double();
        p.step_size             = this->get_parameter("step_size").as_double();
        p.max_iterations        = this->get_parameter("max_iterations").as_int();
        p.deviation_threshold   = this->get_parameter("deviation_threshold").as_double();
        p.lethal_cost_threshold = this->get_parameter("lethal_cost_threshold").as_int();
        p.unknown_cost_penalty  = this->get_parameter("unknown_cost_penalty").as_double();
        p.theta_bins            = this->get_parameter("theta_bins").as_int();
        p.stall_dir             = local_stall_dir;
        p.escape_radius         = this->get_parameter("escape_radius").as_double();
        p.goal_yaw_tol          = this->get_parameter("goal_yaw_tolerance_deg").as_double() * DEG2RAD;
        p.heuristic_weight      = this->get_parameter("heuristic_weight").as_double();
        p.smoothing_enabled     = this->get_parameter("smoothing_enabled").as_bool();
        p.smooth_iterations     = this->get_parameter("smooth_iterations").as_int();
        p.w_smooth              = this->get_parameter("w_smooth").as_double();
        p.w_obstacle            = this->get_parameter("w_obstacle").as_double();
        p.obstacle_dmax         = this->get_parameter("obstacle_dmax").as_double();
        p.w_voronoi             = this->get_parameter("w_voronoi").as_double();

        double sx = 0.0, sy = 0.0, syaw = 0.0;
        try {
            auto tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
            sx = tf.transform.translation.x; sy = tf.transform.translation.y;
            syaw = std::atan2(2.0*(tf.transform.rotation.w * tf.transform.rotation.z + tf.transform.rotation.x * tf.transform.rotation.y),
                1.0 - 2.0*(tf.transform.rotation.y * tf.transform.rotation.y + tf.transform.rotation.z * tf.transform.rotation.z));
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Goal received but no TF map->base_footprint (%s) — is SLAM/EKF up, and is RViz's "
                "Fixed Frame set to 'map'?", e.what());
            return;
        }

        double gx = local_goal->pose.position.x, gy = local_goal->pose.position.y;
        double gyaw = std::atan2(2.0*(local_goal->pose.orientation.w * local_goal->pose.orientation.z + local_goal->pose.orientation.x * local_goal->pose.orientation.y),
            1.0 - 2.0*(local_goal->pose.orientation.y * local_goal->pose.orientation.y + local_goal->pose.orientation.z * local_goal->pose.orientation.z));

        if (local_new_goal) {
            virtual_obstacles_.clear();
            RCLCPP_INFO(this->get_logger(), "🧹 Fresh destination targeted. Cleared virtual obstacle memory.");
            std::lock_guard<std::mutex> lk(map_mutex_);
            new_goal_set_ = false;
        }

        if (local_stall_dir != 0) {
            double obs_project_x = sx + local_stall_dir * 0.35 * std::cos(syaw);
            double obs_project_y = sy + local_stall_dir * 0.35 * std::sin(syaw);
            bool is_duplicate = false;
            for (const auto& obs : virtual_obstacles_) {
                if (std::hypot(obs.first - obs_project_x, obs.second - obs_project_y) < 0.20) { is_duplicate = true; break; }
            }
            if (!is_duplicate) {
                virtual_obstacles_.push_back({obs_project_x, obs_project_y});
                RCLCPP_WARN(this->get_logger(),
                    "Blind stall while moving %s. Injected virtual obstacle at (%.2f, %.2f).",
                    local_stall_dir > 0 ? "forward" : "in reverse", obs_project_x, obs_project_y);
            }
        }

        bool do_replan = local_plan_pending || local_stall_dir != 0;
        // On a fresh costmap, replan only if the committed path ahead of the
        // robot is now blocked — leave a still-valid path untouched. Gated to an
        // active, not-yet-reached goal so we don't churn while idle or at goal.
        if (!do_replan && local_map_dirty && !current_global_path_.empty() &&
            std::hypot(gx - sx, gy - sy) > p.default_tolerance &&
            pathAheadBlocked(current_global_path_, sx, sy, *local_map, p.lethal_cost_threshold)) {
            do_replan = true;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Costmap update blocks the current path — replanning.");
        }
        if (!do_replan && !current_global_path_.empty()) {
            double min_d = std::numeric_limits<double>::max();
            for (const auto& wp : current_global_path_) min_d = std::min(min_d, std::hypot(wp.first - sx, wp.second - sy));
            if (min_d > p.deviation_threshold) do_replan = true;
        }
        if (!do_replan) {
            if (!last_ctrl_msg_.poses.empty() && std::hypot(gx - sx, gy - sy) > p.default_tolerance)
                path_pub_->publish(last_ctrl_msg_);
            return;
        }

        auto path = astarSearch(sx, sy, syaw, gx, gy, gyaw, local_map, p);

        if (!path.empty()) {
            // Second phase: smooth the raw Hybrid A* path before publishing.
            if (p.smoothing_enabled) {
                EscapeBubble esc;
                if (!cellIsFree(sx, sy, *local_map, p.lethal_cost_threshold))
                    esc = {sx, sy, p.escape_radius, true};
                ObstacleField field = buildObstacleField(*local_map, p.obstacle_dmax);
                smoothPath(path, field, *local_map, p, esc, local_voronoi.get());
            }
            current_global_path_.clear();
            for (const auto& nd : path) current_global_path_.push_back({nd.x, nd.y});
            publishPath(path);
            if (local_plan_pending) {
                std::lock_guard<std::mutex> lk(map_mutex_);
                plan_pending_ = false;
            }
        } else {
            const bool goal_blocked = !cellIsFree(gx, gy, *local_map, p.lethal_cost_threshold);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Hybrid A*: no path found%s. Retrying until the goal is reached or replaced.",
                goal_blocked ? " (goal lies inside an obstacle or its inflation zone)" : "");
        }
    }

    // ── Hybrid A* with Reeds-Shepp analytic expansion + dual heuristic ───────
    std::vector<Node3D> astarSearch(double sx, double sy, double syaw, double gx, double gy, double gyaw,
                                    const nav_msgs::msg::OccupancyGrid::SharedPtr& map_ptr, const PlannerParams& p)
    {
        const nav_msgs::msg::OccupancyGrid& map = *map_ptr;
        const double max_k = 1.0 / p.turning_radius;
        const std::array<double, 5> kappas = {-max_k, -max_k*0.5, 0.0, max_k*0.5, max_k};
        const std::array<int, 2>    dirs   = {1, -1};

        EscapeBubble esc;
        if (!cellIsFree(sx, sy, map, p.lethal_cost_threshold)) {
            esc = {sx, sy, p.escape_radius, true};
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Start pose is inside the inflated lethal zone. Allowing escape arcs within %.2f m.", p.escape_radius);
        }

        // heuristic 2 precomputed once for this goal (obstacle-aware lower bound)
        std::vector<float> holo = holonomicHeuristic(map, gx, gy);

        // h = w * max( Reeds-Shepp(no obstacles), Dijkstra(with obstacles) )
        auto heuristic = [&](double x, double y, double th) -> double {
            double h_rs   = rs::distance(x, y, th, gx, gy, gyaw, p.turning_radius);
            float  h_holo = lookupHolo(holo, map, x, y);
            double h = std::max<double>(h_rs, std::isfinite(h_holo) ? h_holo : 0.0);
            return p.heuristic_weight * h;
        };

        std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open_set;
        std::unordered_set<Index3D, TupleHash>          closed;
        std::unordered_map<Index3D, int,    TupleHash>  best_pool_idx;
        std::unordered_map<Index3D, double, TupleHash>  g_best;

        std::vector<Node3D> pool;
        pool.reserve(static_cast<size_t>(p.max_iterations) * kappas.size() * dirs.size() + 256);

        pool.push_back({sx, sy, syaw, 0.0, 1, 0.0, 0.0, -1});
        Index3D s_idx = worldTo3DIndex(sx, sy, syaw, map, p.theta_bins);
        best_pool_idx[s_idx] = 0; g_best[s_idx] = 0.0;
        open_set.push({heuristic(sx, sy, syaw), s_idx});

        int iters = 0;
        const double rs_ds = std::max(map.info.resolution * 0.5, 0.05);

        while (!open_set.empty()) {
            auto [cur_f, cur_idx] = open_set.top(); open_set.pop();
            (void)cur_f;
            if (closed.count(cur_idx)) continue;
            closed.insert(cur_idx);
            if (++iters > p.max_iterations) return {};

            auto it = best_pool_idx.find(cur_idx); if (it == best_pool_idx.end()) continue;
            const int cur_pi = it->second; const Node3D cur = pool[cur_pi];

            double d_goal     = std::hypot(cur.x - gx, cur.y - gy);
            double angle_diff = std::abs(wrapPi(cur.theta - gyaw));

            // --- Reeds-Shepp analytic expansion, tried BEFORE the proximity test.
            // It lands exactly on the goal pose (position + yaw), so preferring it
            // makes the returned path END on the goal instead of up to
            // default_tolerance short (which left the controller stranded short of
            // the goal). Attempted every iteration once near the goal — where the
            // proximity fallback could otherwise fire early — and periodically far
            // out. Disabled while stalled so escape goes through kinematic arcs.
            if (p.stall_dir == 0) {
                int N = std::max(1, std::min(100, static_cast<int>(d_goal / p.step_size)));
                if (d_goal < 2.0 * p.default_tolerance || iters % N == 0) {
                    double rs_len = 0.0;
                    auto samples = rs::samplePath(cur.x, cur.y, cur.theta, gx, gy, gyaw,
                                                  p.turning_radius, rs_ds, rs_len);
                    bool ok = !samples.empty();
                    for (const auto& s : samples) {
                        if (!cellIsFree(s.x, s.y, map, p.lethal_cost_threshold, esc)) { ok = false; break; }
                    }
                    if (ok) {
                        int par = cur_pi; double g_acc = cur.g; int prev_gear = cur.direction;
                        for (const auto& s : samples) {
                            double seg = p.step_size + (s.gear == -1 ? 1.0 : 0.0);
                            if (s.gear != prev_gear) seg += 2.0;
                            g_acc += seg; prev_gear = s.gear;
                            pool.push_back({s.x, s.y, s.theta, 0.0, s.gear, g_acc, g_acc, par});
                            par = static_cast<int>(pool.size()) - 1;
                        }
                        return reconstructPath(pool, static_cast<int>(pool.size()) - 1);
                    }
                }
            }

            // Proximity fallback: accept a node within tolerance only when the RS
            // shot above could not connect (e.g. the analytic path is blocked).
            if (d_goal < p.default_tolerance && angle_diff < p.goal_yaw_tol) return reconstructPath(pool, cur_pi);

            // --- kinematic expansion: 5 steering angles × {forward, reverse} -
            for (int dir : dirs) {
                if (p.stall_dir != 0 && cur.parent_idx == -1 && dir == p.stall_dir) continue;

                for (double k : kappas) {
                    double nx, ny, nth;
                    if (!propagateArc(cur.x, cur.y, cur.theta, k, p.step_size, dir, map, p.lethal_cost_threshold, nx, ny, nth, esc)) continue;

                    int    cv      = cellValue(nx, ny, map);
                    double penalty = 0.0;
                    if      (cv == -1) penalty = p.unknown_cost_penalty * p.step_size;
                    else if (cv >   0) penalty = (cv / 100.0) * p.step_size * 2.0;

                    double ec = p.step_size + penalty;
                    if (std::abs(k - cur.kappa) > 0.01) ec += 0.40;   // steering change
                    if (std::abs(k)             > 1e-3) ec += 0.08;   // steering magnitude
                    if (dir  == -1)                     ec += 2.50;   // reverse
                    if (dir  != cur.direction)          ec += 2.00;   // gear change
                    if (p.stall_dir != 0 && dir == p.stall_dir) ec += 7.00;

                    double tent_g = cur.g + ec;
                    Index3D nb_idx = worldTo3DIndex(nx, ny, nth, map, p.theta_bins);
                    {
                        auto gi = g_best.find(nb_idx);
                        if (gi != g_best.end() && tent_g >= gi->second) continue;
                    }
                    g_best[nb_idx] = tent_g;

                    double f = tent_g + heuristic(nx, ny, nth);
                    pool.push_back({nx, ny, nth, k, dir, tent_g, f, cur_pi});
                    best_pool_idx[nb_idx] = static_cast<int>(pool.size()) - 1;
                    open_set.push({f, nb_idx});
                }
            }
        }
        return {};
    }

    // ── publish, preserving the signed-velocity-in-pose.z contract ───────────
    // Velocity profile is computed here, after smoothing, since smoothing moves
    // geometry (segment lengths) while leaving gears/cusps intact.
    void publishPath(const std::vector<Node3D>& nodes) {
        const std::vector<double> v = computeVelocityProfile(nodes);
        auto stamp = this->now(); nav_msgs::msg::Path ctrl_msg, vis_msg;
        ctrl_msg.header.frame_id = vis_msg.header.frame_id = "map";
        ctrl_msg.header.stamp = vis_msg.header.stamp = stamp;
        for (size_t i = 0; i < nodes.size(); ++i) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id = "map"; ps.header.stamp = stamp;
            ps.pose.position.x = nodes[i].x; ps.pose.position.y = nodes[i].y;
            ps.pose.orientation.z = std::sin(nodes[i].theta * 0.5);
            ps.pose.orientation.w = std::cos(nodes[i].theta * 0.5);
            ps.pose.position.z = v[i]; ctrl_msg.poses.push_back(ps);
            ps.pose.position.z = 0.0;  vis_msg.poses.push_back(ps);
        }
        path_pub_->publish(ctrl_msg); visual_path_pub_->publish(vis_msg);
        last_ctrl_msg_ = std::move(ctrl_msg);
    }

    // ── members ──────────────────────────────────────────────────────────────
    std::vector<WorldPoint> current_global_path_;
    std::vector<WorldPoint> virtual_obstacles_;
    nav_msgs::msg::Path     last_ctrl_msg_;
    std::mutex                                            map_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr               map_msg_;
    nav_msgs::msg::OccupancyGrid::SharedPtr               voronoi_msg_;    // latest /voronoi_field
    geometry_msgs::msg::PoseStamped::SharedPtr            latest_goal_pose_;
    bool                                                  new_goal_set_;
    bool                                                  plan_pending_;
    bool                                                  map_dirty_ = false;
    int                                                   stall_dir_;

    std::unique_ptr<tf2_ros::Buffer>                      tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>           tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr             stall_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    voronoi_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                visual_path_pub_;
    rclcpp::TimerBase::SharedPtr                                     plan_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannAStarPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
