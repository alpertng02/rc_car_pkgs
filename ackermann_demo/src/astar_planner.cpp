// =============================================================================
// Hybrid A* Kinematic Path Planner — ROS 2
// =============================================================================
// CHANGES FROM ORIGINAL (annotated inline with ★)
//
//  BUG FIXES
//  ★1  Angle-normalization bug in goal check — abs() was taken before wrapping,
//      so the `< -PI` branch never fired.  wrapPi() fixes this everywhere.
//  ★2  Arc collision check — original only tested the arc endpoint; the arc
//      itself could pass through obstacles.  arcIsFree() sub-samples the arc.
//  ★3  Costmap penalty units — original mixed raw 0-100 costmap values with
//      metric step sizes, making costs incoherent.  Penalty is now normalised
//      to meters via (cv/100)*step_size*2.
//  ★4  pool.reserve() too small — original reserved max_iterations+100 but
//      each iteration adds up to |kappas|×|dirs| = 10 nodes.
//  ★5  unknown_cost_penalty was int, losing sub-unit precision.
//
//  ROBUSTNESS
//  ★6  TupleHash replaced with Boost-style hash_combine (far fewer collisions).
//  ★7  PlannerParams struct — all parameters read once per replan, not
//      scattered across three functions (avoids mid-search parameter drift).
//  ★8  cellValue() helper returns -2 for OOB, -1 for unknown, 0-100 for cost;
//      separates the two error cases that the original conflated.
//  ★9  reconstructPath() extracted as a static helper — was copy-pasted twice.
//  ★10 publishEmptyPath() extracted — was copy-pasted twice.
//  ★11 theta_bins promoted to a ROS parameter (default 72 = 5° resolution
//      vs the original hardcoded 24 = 15°, which discretises too coarsely).
//
//  PERFORMANCE / QUALITY
//  ★12 Velocity profile: added forward acceleration pass — original only
//      enforced deceleration, so the robot started at cruise speed.
//  ★13 Heuristic now includes an angular component:
//          h = dist_to_goal + 0.5 * |angle_diff|
//      so heading errors are penalised during search, not just at the goal.
//  ★14 std::array for kappas/dirs (stack allocation, compile-time size).
//  ★15 const-correctness on all helper methods.
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
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

// ─── Constants ────────────────────────────────────────────────────────────────
static constexpr double PI      = 3.14159265358979323846;
static constexpr double TWO_PI  = 2.0 * PI;
static constexpr double DEG2RAD = PI / 180.0;

// ─── ★1  Angle utilities ──────────────────────────────────────────────────────
/// Wrap angle to (−π, +π]
inline double wrapPi(double a) {
    while (a >  PI) a -= TWO_PI;
    while (a < -PI) a += TWO_PI;
    return a;
}
/// Wrap angle to [0, 2π)
inline double wrap2Pi(double a) {
    a = std::fmod(a, TWO_PI);
    if (a < 0.0) a += TWO_PI;
    return a;
}

// ─── Node descriptor ──────────────────────────────────────────────────────────
struct Node3D {
    double x, y, theta;   // continuous state
    double kappa;         // curvature of the incoming arc
    int    direction;     // +1 forward, −1 reverse
    double g, f;          // cost-so-far, f-score
    int    parent_idx;    // index in node pool; −1 for root
};

// ─── ★6  Boost-style hash for (int, int, int) ─────────────────────────────────
struct TupleHash {
    std::size_t operator()(const std::tuple<int,int,int>& t) const noexcept {
        std::size_t h = 0;
        auto combine = [&h](int v) {
            // Knuth multiplicative hash + avalanche
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

// ─── ★7  Planner parameters — snapshotted once per planning call ───────────────
struct PlannerParams {
    double default_tolerance;       // goal-acceptance radius [m]
    double turning_radius;          // minimum turning radius [m]
    double step_size;               // arc step length [m]
    int    max_iterations;
    double deviation_threshold;     // replan trigger threshold [m]
    int    lethal_cost_threshold;   // costmap values ≥ this are obstacles
    double unknown_cost_penalty;    // ★5 double, not int
    int    theta_bins;              // ★11 heading discretisation bins
};

// ─────────────────────────────────────────────────────────────────────────────
class AStarPlannerNode : public rclcpp::Node
{
public:
    AStarPlannerNode()
      : Node("astar_planner"), new_goal_set_(false)
    {
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        this->declare_parameter("default_tolerance",     0.25);
        this->declare_parameter("turning_radius",        0.35);
        this->declare_parameter("step_size",             0.15);
        this->declare_parameter("max_iterations",        5000);
        this->declare_parameter("deviation_threshold",   0.20);
        this->declare_parameter("lethal_cost_threshold", 85);
        this->declare_parameter("unknown_cost_penalty",  15.0);   // ★5
        this->declare_parameter("theta_bins",            72);      // ★11

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/inflated_costmap", 10,
            std::bind(&AStarPlannerNode::mapCallback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&AStarPlannerNode::goalCallback, this, std::placeholders::_1));

        path_pub_        = this->create_publisher<nav_msgs::msg::Path>("/followed_path", 10);
        visual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/visual_path",   10);

        plan_timer_ = this->create_wall_timer(
            200ms, std::bind(&AStarPlannerNode::replanTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Hybrid A* Planner ready.");
    }

private:
    // ── Callbacks ─────────────────────────────────────────────────────────────
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(map_mutex_);
        map_msg_ = msg;
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(map_mutex_);
        latest_goal_pose_ = msg;
        new_goal_set_     = true;
    }

    // ── ★8  Grid helpers ──────────────────────────────────────────────────────

    /// Discretise a continuous world pose into a 3D grid cell.
    Index3D worldTo3DIndex(double wx, double wy, double theta,
                           const nav_msgs::msg::OccupancyGrid& map,
                           int theta_bins) const
    {
        int cx = static_cast<int>(
            std::floor((wx - map.info.origin.position.x) / map.info.resolution));
        int cy = static_cast<int>(
            std::floor((wy - map.info.origin.position.y) / map.info.resolution));
        int ct = static_cast<int>(
            std::floor(wrap2Pi(theta) / (TWO_PI / theta_bins))) % theta_bins;
        return {cx, cy, ct};
    }

    /**
     * Raw costmap value at world point (wx, wy).
     *  -2  → out of bounds
     *  -1  → unknown cell (ROS convention)
     * 0-100 → occupancy cost
     */
    int cellValue(double wx, double wy,
                  const nav_msgs::msg::OccupancyGrid& map) const
    {
        int cx = static_cast<int>(
            std::floor((wx - map.info.origin.position.x) / map.info.resolution));
        int cy = static_cast<int>(
            std::floor((wy - map.info.origin.position.y) / map.info.resolution));

        if (cx < 0 || cx >= static_cast<int>(map.info.width)  ||
            cy < 0 || cy >= static_cast<int>(map.info.height))
            return -2;

        return static_cast<int>(map.data[cx + cy * static_cast<int>(map.info.width)]);
    }

    bool cellIsFree(double wx, double wy,
                    const nav_msgs::msg::OccupancyGrid& map,
                    int max_cost) const
    {
        int v = cellValue(wx, wy, map);
        if (v == -2) return false;  // out of bounds — never traversable
        if (v == -1) return true;   // unknown — traversable with penalty
        return v < max_cost;
    }

    // ── ★2  Collision checks ──────────────────────────────────────────────────

    /**
     * Check the full arc travelled by the bicycle model over one step.
     * Sub-samples at 40 % of a grid cell so no obstacle is skipped.
     */
    bool arcIsFree(double x0, double y0, double th0,
                   double kappa, double arc_len, int dir,
                   const nav_msgs::msg::OccupancyGrid& map,
                   int max_cost) const
    {
        int steps = std::max(3,
            static_cast<int>(arc_len / (map.info.resolution * 0.4)));

        double ds = arc_len / steps;
        double x = x0, y = y0, th = th0;

        for (int i = 1; i <= steps; ++i) {
            x  += dir * ds * std::cos(th);
            y  += dir * ds * std::sin(th);
            th  = wrapPi(th + dir * ds * kappa);
            if (!cellIsFree(x, y, map, max_cost)) return false;
        }
        return true;
    }

    /// Straight-line collision check used by the analytic expansion.
    bool segmentIsFree(double x1, double y1, double x2, double y2,
                       const nav_msgs::msg::OccupancyGrid& map,
                       int max_cost) const
    {
        double dist = std::hypot(x2 - x1, y2 - y1);
        if (dist < 1e-6) return cellIsFree(x1, y1, map, max_cost);

        int steps = std::max(2,
            static_cast<int>(dist / (map.info.resolution * 0.4)));

        for (int i = 0; i <= steps; ++i) {
            double t = static_cast<double>(i) / steps;
            if (!cellIsFree(x1 + t*(x2-x1), y1 + t*(y2-y1), map, max_cost))
                return false;
        }
        return true;
    }

    // ── ★9  Path reconstruction ───────────────────────────────────────────────
    static std::vector<Node3D> reconstructPath(const std::vector<Node3D>& pool,
                                               int end_idx)
    {
        std::vector<Node3D> path;
        for (int i = end_idx; i != -1; i = pool[i].parent_idx)
            path.push_back(pool[i]);
        std::reverse(path.begin(), path.end());
        return path;
    }

    // ── ★12  Velocity profile ─────────────────────────────────────────────────
    std::vector<double> computeVelocityProfile(
        const std::vector<Node3D>& nodes) const
    {
        const size_t n = nodes.size();
        if (n == 0) return {};

        constexpr double V_CRUISE  = 1.0;   // m/s
        constexpr double A_MAX     = 0.8;   // m/s²
        constexpr double D_MAX     = 0.8;   // m/s²
        constexpr double V_MIN_FWD = 0.15;  // m/s — avoids infinite stop-time

        std::vector<double> v(n, V_CRUISE);
        if (n == 1) { v[0] = 0.0; return v; }

        // Backward pass — enforce deceleration to rest at goal
        v[n - 1] = 0.0;
        for (size_t i = n - 2; i > 0; --i) {
            double ds = std::hypot(nodes[i+1].x - nodes[i].x,
                                   nodes[i+1].y - nodes[i].y);
            double v_back = std::sqrt(v[i+1]*v[i+1] + 2.0*D_MAX*ds);
            v[i] = std::min(v[i], v_back);
            v[i] = std::max(v[i], V_MIN_FWD);
        }

        // ★12  Forward pass — enforce acceleration from rest at start
        v[0] = 0.0;
        for (size_t i = 1; i < n - 1; ++i) {
            double ds = std::hypot(nodes[i].x - nodes[i-1].x,
                                   nodes[i].y - nodes[i-1].y);
            double v_fwd = std::sqrt(v[i-1]*v[i-1] + 2.0*A_MAX*ds);
            v[i] = std::min(v[i], v_fwd);
        }

        // Apply direction sign for reverse segments
        for (size_t i = 0; i < n; ++i)
            if (nodes[i].direction == -1) v[i] = -v[i];

        return v;
    }

    // ── ★7  Replan timer ──────────────────────────────────────────────────────
    void replanTimerCallback()
    {
        // Snapshot shared data under lock — minimise lock hold time
        nav_msgs::msg::OccupancyGrid::SharedPtr    local_map;
        geometry_msgs::msg::PoseStamped::SharedPtr local_goal;
        bool local_new_goal;
        {
            std::lock_guard<std::mutex> lk(map_mutex_);
            local_map      = map_msg_;
            local_goal     = latest_goal_pose_;
            local_new_goal = new_goal_set_;
            if (new_goal_set_) new_goal_set_ = false;
        }
        if (!local_map || !local_goal) return;

        // ★7  Read all parameters once into a plain struct
        PlannerParams p;
        p.default_tolerance     = this->get_parameter("default_tolerance").as_double();
        p.turning_radius        = this->get_parameter("turning_radius").as_double();
        p.step_size             = this->get_parameter("step_size").as_double();
        p.max_iterations        = this->get_parameter("max_iterations").as_int();
        p.deviation_threshold   = this->get_parameter("deviation_threshold").as_double();
        p.lethal_cost_threshold = this->get_parameter("lethal_cost_threshold").as_int();
        p.unknown_cost_penalty  = this->get_parameter("unknown_cost_penalty").as_double();
        p.theta_bins            = this->get_parameter("theta_bins").as_int();

        // Robot pose via TF
        double sx = 0.0, sy = 0.0, syaw = 0.0;
        try {
            auto tf = tf_buffer_->lookupTransform(
                "map", "base_footprint", tf2::TimePointZero);
            sx   = tf.transform.translation.x;
            sy   = tf.transform.translation.y;
            syaw = std::atan2(
                2.0*(tf.transform.rotation.w * tf.transform.rotation.z +
                     tf.transform.rotation.x * tf.transform.rotation.y),
                1.0 - 2.0*(tf.transform.rotation.y * tf.transform.rotation.y +
                            tf.transform.rotation.z * tf.transform.rotation.z));
        } catch (const tf2::TransformException&) { return; }

        // Goal pose
        double gx   = local_goal->pose.position.x;
        double gy   = local_goal->pose.position.y;
        double gyaw = std::atan2(
            2.0*(local_goal->pose.orientation.w * local_goal->pose.orientation.z +
                 local_goal->pose.orientation.x * local_goal->pose.orientation.y),
            1.0 - 2.0*(local_goal->pose.orientation.y * local_goal->pose.orientation.y +
                       local_goal->pose.orientation.z * local_goal->pose.orientation.z));

        // Validate goal cell
        if (!cellIsFree(gx, gy, *local_map, p.lethal_cost_threshold)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Goal is inside an obstacle. Ignoring.");
            std::lock_guard<std::mutex> lk(map_mutex_);
            latest_goal_pose_ = nullptr;
            return;
        }

        // Already at goal?
        if (std::hypot(gx - sx, gy - sy) <= p.default_tolerance && !local_new_goal) {
            publishEmptyPath();                         // ★10
            std::lock_guard<std::mutex> lk(map_mutex_);
            latest_goal_pose_ = nullptr;
            current_global_path_.clear();
            return;
        }

        // Decide whether to replan
        bool do_replan = local_new_goal;
        if (!do_replan && !current_global_path_.empty()) {
            double min_d = std::numeric_limits<double>::max();
            for (const auto& wp : current_global_path_)
                min_d = std::min(min_d, std::hypot(wp.first - sx, wp.second - sy));

            if (min_d > p.deviation_threshold) {
                do_replan = true;
                RCLCPP_WARN(this->get_logger(),
                    "Path deviation %.2f m. Replanning.", min_d);
            }
        }
        if (!do_replan) return;

        // Run planner
        auto path = astarSearch(sx, sy, syaw, gx, gy, gyaw, local_map, p);

        if (!path.empty()) {
            current_global_path_.clear();
            for (const auto& nd : path)
                current_global_path_.push_back({nd.x, nd.y});

            publishPath(path, computeVelocityProfile(path));
        } else {
            RCLCPP_WARN(this->get_logger(), "Hybrid A*: no path found.");
        }
    }

    // ── Core search ───────────────────────────────────────────────────────────
    std::vector<Node3D> astarSearch(
        double sx, double sy, double syaw,
        double gx, double gy, double gyaw,
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map_ptr,
        const PlannerParams& p)
    {
        const nav_msgs::msg::OccupancyGrid& map = *map_ptr;

        // ★14  std::array — stack allocation, compile-time size
        const double max_k = 1.0 / p.turning_radius;
        const std::array<double, 5> kappas = {-max_k, -max_k*0.5, 0.0,
                                               max_k*0.5, max_k};
        const std::array<int, 2>    dirs   = {1, -1};

        // Data structures
        std::priority_queue<QueueEntry,
                            std::vector<QueueEntry>,
                            std::greater<QueueEntry>> open_set;

        std::unordered_set<Index3D, TupleHash>          closed;
        std::unordered_map<Index3D, int,    TupleHash>  best_pool_idx; // cell → pool index
        std::unordered_map<Index3D, double, TupleHash>  g_best;

        // ★4  Reserve accounting for all children per iteration
        std::vector<Node3D> pool;
        pool.reserve(
            static_cast<size_t>(p.max_iterations) * kappas.size() * dirs.size() + 256);

        // Root node
        pool.push_back({sx, sy, syaw, 0.0, 1, 0.0, 0.0, -1});
        Index3D s_idx = worldTo3DIndex(sx, sy, syaw, map, p.theta_bins);
        best_pool_idx[s_idx] = 0;
        g_best[s_idx]        = 0.0;
        open_set.push({0.0, s_idx});

        int iters = 0;

        while (!open_set.empty()) {
            auto [cur_f, cur_idx] = open_set.top();
            open_set.pop();

            if (closed.count(cur_idx)) continue;
            closed.insert(cur_idx);

            if (++iters > p.max_iterations) {
                RCLCPP_WARN(this->get_logger(),
                    "A* budget exhausted (%d iters).", iters);
                return {};
            }

            auto it = best_pool_idx.find(cur_idx);
            if (it == best_pool_idx.end()) continue;    // stale open-set entry

            const int     cur_pi = it->second;
            const Node3D  cur    = pool[cur_pi];        // copy — pool may grow

            // ── Goal check ────────────────────────────────────────────────────
            double d_goal     = std::hypot(cur.x - gx, cur.y - gy);
            // ★1  normalize BEFORE taking abs
            double angle_diff = std::abs(wrapPi(cur.theta - gyaw));

            if (d_goal < p.default_tolerance && angle_diff < 30.0 * DEG2RAD)
                return reconstructPath(pool, cur_pi);   // ★9

            // ── Analytic (straight-line) expansion toward goal ────────────────
            if (d_goal < 4.0 * p.turning_radius) {
                double head  = std::atan2(gy - cur.y, gx - cur.x);
                double align = std::abs(wrapPi(head - cur.theta));

                if (align < 45.0 * DEG2RAD &&
                    segmentIsFree(cur.x, cur.y, gx, gy, map,
                                  p.lethal_cost_threshold))
                {
                    int    n_steps = std::max(1,
                        static_cast<int>(d_goal / p.step_size));
                    int    par     = cur_pi;
                    double g_acc   = cur.g;

                    for (int s = 1; s <= n_steps; ++s) {
                        double r   = static_cast<double>(s) / n_steps;
                        double ix  = cur.x + r*(gx - cur.x);
                        double iy  = cur.y + r*(gy - cur.y);
                        double ith = (s == n_steps) ? gyaw : head;
                        g_acc     += p.step_size;
                        pool.push_back({ix, iy, ith, 0.0, 1, g_acc, 0.0, par});
                        par = static_cast<int>(pool.size()) - 1;
                    }
                    RCLCPP_INFO(this->get_logger(),
                        "Analytic expansion succeeded (iter %d).", iters);
                    return reconstructPath(pool, static_cast<int>(pool.size()) - 1);
                }
            }

            // ── Primitive expansion ───────────────────────────────────────────
            for (int dir : dirs) {
                for (double k : kappas) {

                    // ★2  Arc collision check — not just the endpoint
                    if (!arcIsFree(cur.x, cur.y, cur.theta,
                                   k, p.step_size, dir,
                                   map, p.lethal_cost_threshold))
                        continue;

                    // Bicycle model — Euler integration over one step
                    double nx  = cur.x  + dir * p.step_size * std::cos(cur.theta);
                    double ny  = cur.y  + dir * p.step_size * std::sin(cur.theta);
                    double nth = wrapPi(cur.theta + dir * p.step_size * k);

                    // ★3  Costmap penalty normalised to metric units
                    int    cv      = cellValue(nx, ny, map);
                    double penalty = 0.0;
                    if      (cv == -1) penalty = p.unknown_cost_penalty * p.step_size;
                    else if (cv >   0) penalty = (cv / 100.0) * p.step_size * 2.0;

                    // Edge cost — keep base units in meters for coherence
                    double ec = p.step_size + penalty;
                    if (std::abs(k - cur.kappa) > 0.01) ec += 0.40;   // curvature smoothness
                    if (std::abs(k)             > 1e-3) ec += 0.08;   // prefer straight
                    if (dir  == -1)                     ec += 2.50;   // reverse
                    if (dir  != cur.direction)          ec += 2.00;   // gear change

                    double tent_g = cur.g + ec;

                    Index3D nb_idx = worldTo3DIndex(nx, ny, nth, map, p.theta_bins);
                    {
                        auto gi = g_best.find(nb_idx);
                        if (gi != g_best.end() && tent_g >= gi->second) continue;
                    }
                    g_best[nb_idx] = tent_g;

                    // ★13  Heuristic: Euclidean distance + angular penalty
                    double h = std::hypot(gx - nx, gy - ny)
                             + 0.5 * std::abs(wrapPi(nth - gyaw));
                    double f = tent_g + 1.3 * h;

                    pool.push_back({nx, ny, nth, k, dir, tent_g, f, cur_pi});
                    best_pool_idx[nb_idx] = static_cast<int>(pool.size()) - 1;
                    open_set.push({f, nb_idx});
                }
            }
        }

        return {};  // open set exhausted
    }

    // ── ★10  Publishing ───────────────────────────────────────────────────────
    void publishPath(const std::vector<Node3D>& nodes,
                     const std::vector<double>& velocities)
    {
        auto stamp = this->now();
        nav_msgs::msg::Path ctrl_msg, vis_msg;
        ctrl_msg.header.frame_id = vis_msg.header.frame_id = "map";
        ctrl_msg.header.stamp    = vis_msg.header.stamp    = stamp;

        for (size_t i = 0; i < nodes.size(); ++i) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id    = "map";
            ps.header.stamp       = stamp;
            ps.pose.position.x    = nodes[i].x;
            ps.pose.position.y    = nodes[i].y;
            // Heading as planar quaternion
            ps.pose.orientation.z = std::sin(nodes[i].theta * 0.5);
            ps.pose.orientation.w = std::cos(nodes[i].theta * 0.5);
            // Velocity packed into z (controller convention)
            ps.pose.position.z = velocities[i];
            ctrl_msg.poses.push_back(ps);

            ps.pose.position.z = 0.0;
            vis_msg.poses.push_back(ps);
        }
        path_pub_->publish(ctrl_msg);
        visual_path_pub_->publish(vis_msg);
    }

    void publishEmptyPath() {
        nav_msgs::msg::Path empty;
        empty.header.frame_id = "map";
        empty.header.stamp    = this->now();
        path_pub_->publish(empty);
        visual_path_pub_->publish(empty);
    }

    // ── Members ───────────────────────────────────────────────────────────────
    std::vector<WorldPoint> current_global_path_;

    std::mutex                                            map_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr               map_msg_;
    geometry_msgs::msg::PoseStamped::SharedPtr            latest_goal_pose_;
    bool                                                  new_goal_set_;

    std::unique_ptr<tf2_ros::Buffer>                      tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>           tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
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
