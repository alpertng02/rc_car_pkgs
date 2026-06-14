// =============================================================================
// Hybrid A* Kinematic Path Planner — ROS 2 with Escape Inversion
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

struct Node3D {
    double x, y, theta;   
    double kappa;         
    int    direction;     
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
    int    stall_dir;               // 0 = free, +1 = stalled driving forward, -1 = stalled reversing
    double escape_radius;           // start-pose bubble where graded inflation is not lethal
    double goal_yaw_tol;            // max heading error at the goal [rad]
};

// Relaxation zone around a start pose that sits inside the costmap's inflated
// cushion: within it, graded inflation cells are traversable and only hard
// occupancy (cost 100) stays lethal, so the planner can free a wedged car.
struct EscapeBubble {
    double x = 0.0, y = 0.0, radius = 0.0;
    bool   active = false;
};

class AStarPlannerNode : public rclcpp::Node
{
public:
    AStarPlannerNode()
      : Node("astar_planner"), new_goal_set_(false), plan_pending_(false), stall_dir_(0)
    {
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        this->declare_parameter("default_tolerance",     0.25);
        this->declare_parameter("turning_radius",        0.35);
        this->declare_parameter("step_size",             0.15);
        // Honest goal-yaw matching needs real turn-around maneuvers; 5k
        // exhausted on those (car.launch.py runs with this default).
        this->declare_parameter("max_iterations",        100000);
        this->declare_parameter("deviation_threshold",   0.20);
        this->declare_parameter("lethal_cost_threshold", 85);
        this->declare_parameter("unknown_cost_penalty",  15.0);   
        this->declare_parameter("theta_bins",            72);
        this->declare_parameter("escape_radius",         0.50);
        this->declare_parameter("goal_yaw_tolerance_deg", 15.0);

        // Latched QoS to match costmap_node's transient-local publisher, so a
        // late-joining planner immediately receives the most recent costmap.
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/inflated_costmap", rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&AStarPlannerNode::mapCallback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&AStarPlannerNode::goalCallback, this, std::placeholders::_1));

        stall_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/robot_stall", 10,
            std::bind(&AStarPlannerNode::stallCallback, this, std::placeholders::_1));

        path_pub_        = this->create_publisher<nav_msgs::msg::Path>("/followed_path", 10);
        visual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/visual_path",   10);

        plan_timer_ = this->create_wall_timer(
            200ms, std::bind(&AStarPlannerNode::replanTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Hybrid A* Planner ready with Escape Layer.");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(map_mutex_);
        map_msg_ = msg;
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(map_mutex_);
        latest_goal_pose_ = msg;
        new_goal_set_     = true;   // one-time bookkeeping (virtual obstacle reset)
        plan_pending_     = true;   // cleared only by a successful plan
    }

    void stallCallback(const std_msgs::msg::Int8::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(map_mutex_);
        stall_dir_ = msg->data;
    }

    Index3D worldTo3DIndex(double wx, double wy, double theta,
                           const nav_msgs::msg::OccupancyGrid& map,
                           int theta_bins) const
    {
        int cx = static_cast<int>(std::floor((wx - map.info.origin.position.x) / map.info.resolution));
        int cy = static_cast<int>(std::floor((wy - map.info.origin.position.y) / map.info.resolution));
        int ct = static_cast<int>(std::floor(wrap2Pi(theta) / (TWO_PI / theta_bins))) % theta_bins;
        return {cx, cy, ct};
    }

    int cellValue(double wx, double wy, const nav_msgs::msg::OccupancyGrid& map) const {
        int cx = static_cast<int>(std::floor((wx - map.info.origin.position.x) / map.info.resolution));
        int cy = static_cast<int>(std::floor((wy - map.info.origin.position.y) / map.info.resolution));

        if (cx < 0 || cx >= static_cast<int>(map.info.width)  || cy < 0 || cy >= static_cast<int>(map.info.height))
            return -2;

        return static_cast<int>(map.data[cx + cy * static_cast<int>(map.info.width)]);
    }

    // ── 🟢 UPDATED: Checks against injected hardware safety zones ────────────
    bool cellIsFree(double wx, double wy, const nav_msgs::msg::OccupancyGrid& map, int max_cost,
                    const EscapeBubble& esc = EscapeBubble{}) const {
        // Evaluate memory buffer of blind static stall coordinates
        for (const auto& obs : virtual_obstacles_) {
            if (std::hypot(wx - obs.first, wy - obs.second) < 0.22) { // 22cm radial tracking safety boundary
                return false; // Force immediate collision rejection
            }
        }

        int v = cellValue(wx, wy, map);
        if (v == -2) return false;
        if (v == -1) return true;
        if (v < max_cost) return true;
        // Inside the escape bubble graded inflation is traversable (the chassis
        // is already parked in it); only hard occupancy stays lethal.
        return esc.active && v < 100 && std::hypot(wx - esc.x, wy - esc.y) < esc.radius;
    }

    // Integrates a constant-curvature arc with collision checks and outputs the
    // endpoint, so successor nodes land exactly on the collision-checked trajectory.
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

    bool segmentIsFree(double x1, double y1, double x2, double y2, const nav_msgs::msg::OccupancyGrid& map, int max_cost,
                       const EscapeBubble& esc = EscapeBubble{}) const {
        double dist = std::hypot(x2 - x1, y2 - y1);
        if (dist < 1e-6) return cellIsFree(x1, y1, map, max_cost, esc);
        int steps = std::max(2, static_cast<int>(dist / (map.info.resolution * 0.4)));

        for (int i = 0; i <= steps; ++i) {
            double t = static_cast<double>(i) / steps;
            if (!cellIsFree(x1 + t*(x2-x1), y1 + t*(y2-y1), map, max_cost, esc)) return false;
        }
        return true;
    }

    static std::vector<Node3D> reconstructPath(const std::vector<Node3D>& pool, int end_idx) {
        std::vector<Node3D> path;
        for (int i = end_idx; i != -1; i = pool[i].parent_idx) path.push_back(pool[i]);
        std::reverse(path.begin(), path.end());
        return path;
    }

    std::vector<double> computeVelocityProfile(const std::vector<Node3D>& nodes) const {
        const size_t n = nodes.size(); if (n == 0) return {};
        constexpr double V_CRUISE = 1.0; constexpr double A_MAX = 0.8; constexpr double D_MAX = 0.8; constexpr double V_MIN = 0.15;

        if (n == 1) return {0.0};

        // nodes[i].direction is the direction of the motion *arriving* at node i,
        // so the segment leaving node i has nodes[i+1].direction.
        auto seg_dir = [&nodes, n](size_t i) {
            return (i + 1 < n) ? nodes[i + 1].direction : nodes[n - 1].direction;
        };

        std::vector<double> mag(n, V_CRUISE);
        mag[0] = 0.0; mag[n - 1] = 0.0;
        for (size_t i = 1; i + 1 < n; ++i)
            if (seg_dir(i) != seg_dir(i - 1)) mag[i] = 0.0;   // cusp: come to rest before switching direction

        for (size_t i = n - 1; i-- > 0; ) {
            double ds = std::hypot(nodes[i+1].x - nodes[i].x, nodes[i+1].y - nodes[i].y);
            mag[i] = std::min(mag[i], std::sqrt(mag[i+1]*mag[i+1] + 2.0*D_MAX*ds));
        }
        for (size_t i = 1; i < n; ++i) {
            double ds = std::hypot(nodes[i].x - nodes[i-1].x, nodes[i].y - nodes[i-1].y);
            mag[i] = std::min(mag[i], std::sqrt(mag[i-1]*mag[i-1] + 2.0*A_MAX*ds));
        }

        // Floor everything but the final stop: the controller derives the motion
        // direction from the commanded velocity, so every drivable waypoint must
        // carry a non-zero, correctly signed value. Its own rate limiter handles
        // the actual ramp from/through zero at the start and at cusps.
        for (size_t i = 0; i + 1 < n; ++i) mag[i] = std::max(mag[i], V_MIN);

        for (size_t i = 0; i < n; ++i) if (seg_dir(i) == -1) mag[i] = -mag[i];
        return mag;
    }

    void replanTimerCallback()
    {
        nav_msgs::msg::OccupancyGrid::SharedPtr    local_map;
        geometry_msgs::msg::PoseStamped::SharedPtr local_goal;
        bool local_new_goal;
        bool local_plan_pending;
        int  local_stall_dir;
        {
            std::lock_guard<std::mutex> lk(map_mutex_);
            local_map          = map_msg_;
            local_goal         = latest_goal_pose_;
            local_new_goal     = new_goal_set_;
            local_plan_pending = plan_pending_;
            local_stall_dir    = stall_dir_;
        }
        // plan_pending_ is cleared only after a successful plan, so a goal that
        // arrives before the map/TF is ready, or whose first plan fails, is
        // retried on the next tick instead of being silently dropped.
        // new_goal_set_ is consumed on the first tick that processes the goal,
        // so per-goal bookkeeping does not repeat on every retry.
        if (!local_map || !local_goal) return;

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

        double sx = 0.0, sy = 0.0, syaw = 0.0;
        try {
            auto tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
            sx   = tf.transform.translation.x; sy   = tf.transform.translation.y;
            syaw = std::atan2(2.0*(tf.transform.rotation.w * tf.transform.rotation.z + tf.transform.rotation.x * tf.transform.rotation.y),
                1.0 - 2.0*(tf.transform.rotation.y * tf.transform.rotation.y + tf.transform.rotation.z * tf.transform.rotation.z));
        } catch (const tf2::TransformException&) { return; }

        double gx   = local_goal->pose.position.x; double gy   = local_goal->pose.position.y;
        double gyaw = std::atan2(2.0*(local_goal->pose.orientation.w * local_goal->pose.orientation.z + local_goal->pose.orientation.x * local_goal->pose.orientation.y),
            1.0 - 2.0*(local_goal->pose.orientation.y * local_goal->pose.orientation.y + local_goal->pose.orientation.z * local_goal->pose.orientation.z));

        // 🟢 FIXED: Clear virtual obstacles on fresh goal dispatch to prevent map ghosting.
        // Consumed here so retries of a failing plan do not wipe the memory again
        // (the stall-recovery obstacles must survive across retry ticks).
        if (local_new_goal) {
            virtual_obstacles_.clear();
            RCLCPP_INFO(this->get_logger(), "🧹 Fresh destination targeted. Cleared virtual obstacle memory.");
            std::lock_guard<std::mutex> lk(map_mutex_);
            new_goal_set_ = false;
        }

        // Inject a virtual obstacle on the side the robot was moving toward when it
        // stalled: ahead of the bumper for a forward stall, behind it for a reverse one.
        if (local_stall_dir != 0) {
            double obs_project_x = sx + local_stall_dir * 0.35 * std::cos(syaw);
            double obs_project_y = sy + local_stall_dir * 0.35 * std::sin(syaw);

            bool is_duplicate = false;
            for (const auto& obs : virtual_obstacles_) {
                if (std::hypot(obs.first - obs_project_x, obs.second - obs_project_y) < 0.20) {
                    is_duplicate = true;
                    break;
                }
            }
            if (!is_duplicate) {
                virtual_obstacles_.push_back({obs_project_x, obs_project_y});
                RCLCPP_WARN(this->get_logger(),
                    "Blind stall while moving %s. Injected virtual obstacle at (%.2f, %.2f).",
                    local_stall_dir > 0 ? "forward" : "in reverse",
                    obs_project_x, obs_project_y);
            }
        }

        bool do_replan = local_plan_pending || local_stall_dir != 0;
        if (!do_replan && !current_global_path_.empty()) {
            double min_d = std::numeric_limits<double>::max();
            for (const auto& wp : current_global_path_) min_d = std::min(min_d, std::hypot(wp.first - sx, wp.second - sy));
            if (min_d > p.deviation_threshold) do_replan = true;
        }
        if (!do_replan) {
            // Re-publish the unchanged plan (same stamp) so the controller's path
            // watchdog stays fed while tracking is healthy; stop once at the goal.
            if (!last_ctrl_msg_.poses.empty() && std::hypot(gx - sx, gy - sy) > p.default_tolerance)
                path_pub_->publish(last_ctrl_msg_);
            return;
        }

        auto path = astarSearch(sx, sy, syaw, gx, gy, gyaw, local_map, p);

        if (!path.empty()) {
            current_global_path_.clear();
            for (const auto& nd : path) current_global_path_.push_back({nd.x, nd.y});
            publishPath(path, computeVelocityProfile(path));
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

    std::vector<Node3D> astarSearch(double sx, double sy, double syaw, double gx, double gy, double gyaw, const nav_msgs::msg::OccupancyGrid::SharedPtr& map_ptr, const PlannerParams& p)
    {
        const nav_msgs::msg::OccupancyGrid& map = *map_ptr;
        const double max_k = 1.0 / p.turning_radius;
        const std::array<double, 5> kappas = {-max_k, -max_k*0.5, 0.0, max_k*0.5, max_k};
        const std::array<int, 2>    dirs   = {1, -1};

        // Escape bubble: when the start pose already sits inside the costmap's
        // inflated cushion (bumper centimeters from a wall), every first arc
        // from the start node would be rejected and planning could never free
        // the car. Within escape_radius of the start, graded inflation is
        // accepted and only hard occupancy stays lethal, so a short escape
        // maneuver (typically reversing out) exists.
        EscapeBubble esc;
        if (!cellIsFree(sx, sy, map, p.lethal_cost_threshold)) {
            esc = {sx, sy, p.escape_radius, true};
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Start pose is inside the inflated lethal zone. Allowing escape arcs within %.2f m.",
                p.escape_radius);
        }

        std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open_set;
        std::unordered_set<Index3D, TupleHash>          closed;
        std::unordered_map<Index3D, int,    TupleHash>  best_pool_idx; 
        std::unordered_map<Index3D, double, TupleHash>  g_best;

        std::vector<Node3D> pool;
        pool.reserve(static_cast<size_t>(p.max_iterations) * kappas.size() * dirs.size() + 256);

        pool.push_back({sx, sy, syaw, 0.0, 1, 0.0, 0.0, -1});
        Index3D s_idx = worldTo3DIndex(sx, sy, syaw, map, p.theta_bins);
        best_pool_idx[s_idx] = 0; g_best[s_idx] = 0.0;
        open_set.push({0.0, s_idx});

        int iters = 0;

        while (!open_set.empty()) {
            auto [cur_f, cur_idx] = open_set.top(); open_set.pop();
            
            if (closed.count(cur_idx)) {
                continue; 
            }
            closed.insert(cur_idx);

            if (++iters > p.max_iterations) return {};

            auto it = best_pool_idx.find(cur_idx); if (it == best_pool_idx.end()) continue;
            const int cur_pi = it->second; const Node3D cur = pool[cur_pi];        

            double d_goal     = std::hypot(cur.x - gx, cur.y - gy);
            double angle_diff = std::abs(wrapPi(cur.theta - gyaw));

            if (d_goal < p.default_tolerance && angle_diff < p.goal_yaw_tol) return reconstructPath(pool, cur_pi);

            // Analytic straight-shot finisher. Only valid when the segment is
            // kinematically honest: the car already heads along the segment
            // bearing AND that bearing matches the requested goal yaw, so the
            // final pose orientation is actually achieved, not just labeled.
            if (p.stall_dir == 0 && d_goal < 4.0 * p.turning_radius) {
                double head  = std::atan2(gy - cur.y, gx - cur.x); double align = std::abs(wrapPi(head - cur.theta));
                if (align < p.goal_yaw_tol && std::abs(wrapPi(head - gyaw)) < p.goal_yaw_tol &&
                    segmentIsFree(cur.x, cur.y, gx, gy, map, p.lethal_cost_threshold, esc)) {
                    int n_steps = std::max(1, static_cast<int>(d_goal / p.step_size)); int par = cur_pi; double g_acc = cur.g;
                    for (int s = 1; s <= n_steps; ++s) {
                        double r = static_cast<double>(s) / n_steps;
                        double ix = cur.x + r*(gx - cur.x); double iy = cur.y + r*(gy - cur.y); double ith = (s == n_steps) ? gyaw : head;
                        g_acc += p.step_size; pool.push_back({ix, iy, ith, 0.0, 1, g_acc, 0.0, par});
                        par = static_cast<int>(pool.size()) - 1;
                    }
                    return reconstructPath(pool, static_cast<int>(pool.size()) - 1);
                }
            }

            for (int dir : dirs) {
                // When stalled, never expand the start node in the blocked direction.
                if (p.stall_dir != 0 && cur.parent_idx == -1 && dir == p.stall_dir) {
                    continue;
                }

                for (double k : kappas) {
                    double nx, ny, nth;
                    if (!propagateArc(cur.x, cur.y, cur.theta, k, p.step_size, dir, map, p.lethal_cost_threshold, nx, ny, nth, esc)) continue;

                    int    cv      = cellValue(nx, ny, map);
                    double penalty = 0.0;
                    if      (cv == -1) penalty = p.unknown_cost_penalty * p.step_size;
                    else if (cv >   0) penalty = (cv / 100.0) * p.step_size * 2.0;

                    double ec = p.step_size + penalty;
                    if (std::abs(k - cur.kappa) > 0.01) ec += 0.40;   
                    if (std::abs(k)             > 1e-3) ec += 0.08;   
                    if (dir  == -1)                     ec += 2.50;   
                    if (dir  != cur.direction)          ec += 2.00;   

                    if (p.stall_dir != 0 && dir == p.stall_dir) {
                        ec += 7.00;
                    }

                    double tent_g = cur.g + ec;
                    Index3D nb_idx = worldTo3DIndex(nx, ny, nth, map, p.theta_bins);
                    {
                        auto gi = g_best.find(nb_idx);
                        if (gi != g_best.end() && tent_g >= gi->second) continue;
                    }
                    g_best[nb_idx] = tent_g;

                    double h = std::hypot(gx - nx, gy - ny) + 0.5 * std::abs(wrapPi(nth - gyaw));
                    double f = tent_g + 1.3 * h;

                    pool.push_back({nx, ny, nth, k, dir, tent_g, f, cur_pi});
                    best_pool_idx[nb_idx] = static_cast<int>(pool.size()) - 1;
                    open_set.push({f, nb_idx});
                }
            }
        }
        return {};  
    }

    void publishPath(const std::vector<Node3D>& nodes, const std::vector<double>& velocities) {
        auto stamp = this->now(); nav_msgs::msg::Path ctrl_msg, vis_msg;
        ctrl_msg.header.frame_id = vis_msg.header.frame_id = "map"; ctrl_msg.header.stamp = vis_msg.header.stamp = stamp;
        for (size_t i = 0; i < nodes.size(); ++i) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id = "map"; ps.header.stamp = stamp;
            ps.pose.position.x = nodes[i].x; ps.pose.position.y = nodes[i].y;
            ps.pose.orientation.z = std::sin(nodes[i].theta * 0.5); ps.pose.orientation.w = std::cos(nodes[i].theta * 0.5);
            ps.pose.position.z = velocities[i]; ctrl_msg.poses.push_back(ps);
            ps.pose.position.z = 0.0; vis_msg.poses.push_back(ps);
        }
        path_pub_->publish(ctrl_msg); visual_path_pub_->publish(vis_msg);
        last_ctrl_msg_ = std::move(ctrl_msg);
    }

    std::vector<WorldPoint> current_global_path_;
    std::vector<WorldPoint> virtual_obstacles_;   // blind-stall coordinates treated as obstacles
    nav_msgs::msg::Path     last_ctrl_msg_;       // last published plan, re-sent as keep-alive
    std::mutex                                            map_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr               map_msg_;
    geometry_msgs::msg::PoseStamped::SharedPtr            latest_goal_pose_;
    bool                                                  new_goal_set_;   // per-goal bookkeeping not yet done
    bool                                                  plan_pending_;   // goal has no successful plan yet
    int                                                   stall_dir_;

    std::unique_ptr<tf2_ros::Buffer>                      tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>           tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr             stall_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                visual_path_pub_;
    rclcpp::TimerBase::SharedPtr                                     plan_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarPlannerNode>());
    rclcpp::shutdown();
    return 0;
}