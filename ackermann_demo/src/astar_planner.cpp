#include <chrono>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

const double PI = 3.14159265358979323846;

struct PairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        return std::hash<T1>()(p.first) ^ (std::hash<T2>()(p.second) << 1);
    }
};

using Point2D = std::pair<int, int>;
using QueueElement = std::pair<double, Point2D>;
using WorldPoint = std::pair<double, double>;

class AStarPlannerNode : public rclcpp::Node {
public:
    AStarPlannerNode() : Node("astar_planner"), new_goal_set_(false) {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/inflated_costmap", 10, std::bind(&AStarPlannerNode::mapCallback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&AStarPlannerNode::goalCallback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/followed_path", 10);
        visual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/visual_path", 10);

        plan_timer_ = this->create_wall_timer(200ms, std::bind(&AStarPlannerNode::replanTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "🚀 Advanced Smooth A* Planner with Velocity Profiling Active!");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_msg_ = msg;
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        latest_goal_pose_ = msg;
        new_goal_set_ = true;
    }

    Point2D worldToGrid(double wx, double wy) {
        int cx = static_cast<int>((wx - map_msg_->info.origin.position.x) / map_msg_->info.resolution);
        int cy = static_cast<int>((wy - map_msg_->info.origin.position.y) / map_msg_->info.resolution);
        return { cx, cy };
    }

    WorldPoint gridToWorld(int cx, int cy) {
        double wx = map_msg_->info.origin.position.x + (cx + 0.5) * map_msg_->info.resolution;
        double wy = map_msg_->info.origin.position.y + (cy + 0.5) * map_msg_->info.resolution;
        return { wx, wy };
    }

    bool isValidAndFree(int cx, int cy) {
        if (!map_msg_) return false;

        // Hard map boundary check
        if (cx < 0 || cx >= static_cast<int>(map_msg_->info.width) ||
            cy < 0 || cy >= static_cast<int>(map_msg_->info.height)) {
            return false;
        }

        int index = cx + (cy * map_msg_->info.width);
        int8_t cell_value = map_msg_->data[index];

        // If the costmap node marks it as an obstacle (e.g., value 100) or unmapped (-1)
        if (cell_value == 100 || cell_value == -1) {
            return false;
        }

        return true;
    }

    // ✅ MODULE 1: Gradient Descent Path Smoothing Layer
    std::vector<WorldPoint> smoothPath(const std::vector<Point2D>& grid_path) {
        std::vector<WorldPoint> smoothed_path;
        if (grid_path.empty()) return smoothed_path;

        // Convert the discrete A* grid layout array to real world coordinates
        for (const auto& node : grid_path) {
            smoothed_path.push_back(gridToWorld(node.first, node.second));
        }

        // Optimization parameters
        double alpha = 0.1;     // Weight for data fidelity (staying close to original path)
        double beta = 0.3;      // Weight for smoothness (flattening turns)
        double tolerance = 0.001;
        double change = tolerance;

        std::vector<WorldPoint> optimized_path = smoothed_path;

        while (change >= tolerance) {
            change = 0.0;
            // Keep start node and goal node locked down, optimize internal nodes
            for (size_t i = 1; i < smoothed_path.size() - 1; ++i) {
                double old_x = optimized_path[i].first;
                double old_y = optimized_path[i].second;

                optimized_path[i].first += alpha * (smoothed_path[i].first - optimized_path[i].first) +
                    beta * (optimized_path[i + 1].first - 2.0 * optimized_path[i].first + optimized_path[i - 1].first);

                optimized_path[i].second += alpha * (smoothed_path[i].second - optimized_path[i].second) +
                    beta * (optimized_path[i + 1].second - 2.0 * optimized_path[i].second + optimized_path[i - 1].second);

                change += std::abs(old_x - optimized_path[i].first) + std::abs(old_y - optimized_path[i].second);
            }
        }
        return optimized_path;
    }

    // ✅ MODULE 2: Dual-Pass Kinematic Velocity Profiler
    std::vector<double> computeVelocityProfile(const std::vector<WorldPoint>& smooth_path) {
        size_t n = smooth_path.size();
        if (n == 0) return std::vector<double>();

        // Define RC vehicle physics profile configurations
        double max_v = 1.5;            // m/s max velocity down straight corridors
        double max_lat_accel = 1.8;    // m/s^2 max safe sideways cornering force
        double max_decel = 1.2;        // m/s^2 comfortable braking acceleration

        std::vector<double> target_velocities(n, max_v);

        if (n < 3) {
            if (n == 2) target_velocities[1] = 0.0;
            return target_velocities;
        }

        // Pass 1: Forward Loop -> Throttle velocities based on local curvature math constraints
        for (size_t i = 1; i < n - 1; ++i) {
            double dx1 = smooth_path[i].first - smooth_path[i - 1].first;
            double dy1 = smooth_path[i].second - smooth_path[i - 1].second;
            double dx2 = smooth_path[i + 1].first - smooth_path[i].first;
            double dy2 = smooth_path[i + 1].second - smooth_path[i].second;

            double ds1 = std::hypot(dx1, dy1);
            double ds2 = std::hypot(dx2, dy2);
            if (ds1 < 0.001 || ds2 < 0.001) continue;

            double yaw1 = std::atan2(dy1, dx1);
            double yaw2 = std::atan2(dy2, dx2);

            double dyaw = yaw2 - yaw1;
            while (dyaw > PI)  dyaw -= 2.0 * PI;
            while (dyaw < -PI) dyaw += 2.0 * PI;

            // Curvature calculation
            double curvature = std::abs(dyaw) / ((ds1 + ds2) / 2.0);

            if (curvature > 0.05) {
                // v = sqrt(a_lat / curvature)
                double safe_v = std::sqrt(max_lat_accel / curvature);
                target_velocities[i] = std::min(max_v, safe_v);
            }
        }

        // Pass 2: Backward Loop -> Generate brake deceleration ramp so the car slows down BEFORE curves
        target_velocities[n - 1] = 0.0; // Force total stop at destination node

        for (size_t i = n - 2; i > 0; --i) {
            double dx = smooth_path[i + 1].first - smooth_path[i].first;
            double dy = smooth_path[i + 1].second - smooth_path[i].second;
            double ds = std::hypot(dx, dy);

            // v_allowed = sqrt(v_next^2 + 2 * a * d)
            double max_allowed_v = std::sqrt(target_velocities[i + 1] * target_velocities[i + 1] + 2.0 * max_decel * ds);
            target_velocities[i] = std::min(target_velocities[i], max_allowed_v);
        }

        return target_velocities;
    }

    void replanTimerCallback() {
        if (!map_msg_ || !latest_goal_pose_) return;

        double start_x = 0.0, start_y = 0.0, car_yaw = 0.0;
        try {
            auto transform = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
            start_x = transform.transform.translation.x;
            start_y = transform.transform.translation.y;

            double qx = transform.transform.rotation.x;
            double qy = transform.transform.rotation.y;
            double qz = transform.transform.rotation.z;
            double qw = transform.transform.rotation.w;

            double siny_cosp = 2.0 * (qw * qz + qx * qy);
            double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            car_yaw = std::atan2(siny_cosp, cosy_cosp);
        } catch (const tf2::TransformException& ex) {
            return;
        }

        double goal_x = latest_goal_pose_->pose.position.x;
        double goal_y = latest_goal_pose_->pose.position.y;

        double distance_to_goal = std::hypot(goal_x - start_x, goal_y - start_y);
        if (distance_to_goal <= 0.25 && !new_goal_set_) {
            nav_msgs::msg::Path empty_path;
            empty_path.header.frame_id = "map";
            empty_path.header.stamp = this->now();

            // Clear both data channels instantly
            path_pub_->publish(empty_path);
            visual_path_pub_->publish(empty_path);

            latest_goal_pose_ = nullptr;
            return;
        }

        double lookahead_dist = 0.25;
        double forward_x = start_x + lookahead_dist * std::cos(car_yaw);
        double forward_y = start_y + lookahead_dist * std::sin(car_yaw);

        Point2D start_grid = worldToGrid(start_x, start_y);
        Point2D forward_grid = worldToGrid(forward_x, forward_y);
        Point2D goal_grid = worldToGrid(goal_x, goal_y);

        std::vector<Point2D> path_grid = astarSearch(forward_grid, goal_grid);

        if (!path_grid.empty()) {
            if (start_grid != forward_grid) {
                path_grid.insert(path_grid.begin(), start_grid);
            }

            // ✅ Execute modular processing pipeline
            std::vector<WorldPoint> smoothed_world_path = smoothPath(path_grid);
            std::vector<double> velocity_profile = computeVelocityProfile(smoothed_world_path);

            publishPath(smoothed_world_path, velocity_profile);
            if (new_goal_set_) new_goal_set_ = false;
        }
    }

    std::vector<Point2D> astarSearch(Point2D start, Point2D goal) {
        std::vector<Point2D> neighbors = {
            { 0, 1 }, { 0, -1 }, { 1, 0 }, { -1, 0 }, { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1 }
        };

        std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> open_set;
        open_set.push({ 0.0, start });

        std::unordered_map<Point2D, Point2D, PairHash> came_from;
        std::unordered_map<Point2D, double, PairHash> g_score;
        g_score[start] = 0.0;

        while (!open_set.empty()) {
            Point2D current = open_set.top().second;
            open_set.pop();

            if (current == goal) {
                std::vector<Point2D> path;
                while (came_from.find(current) != came_from.end()) {
                    path.push_back(current);
                    current = came_from[current];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }

            // Inside src/astar_planner.cpp -> astarSearch neighbor calculation loop:
            for (const auto& dir : neighbors) {
                Point2D neighbor = { current.first + dir.first, current.second + dir.second };

                if (!isValidAndFree(neighbor.first, neighbor.second)) continue;

                double geometric_move_cost = std::hypot(dir.first, dir.second);

                // 🔍 NEW: Fetch the graded cost value penalty directly from the costmap data matrix array index
                int map_index = neighbor.first + (neighbor.second * map_msg_->info.width);
                double costmap_penalty = static_cast<double>(map_msg_->data[map_index]);

                // Add costmap penalties directly into the tracking score. 
                // Multiplier constant (e.g., 0.05) balances physical distance vs obstacle clearance choice.
                double tentative_g_score = g_score[current] + geometric_move_cost + (costmap_penalty * 0.05);

                if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g_score;

                    double h_score = std::hypot(goal.first - neighbor.first, goal.second - neighbor.second);
                    open_set.push({ tentative_g_score + h_score, neighbor });
                }
            }
        }
        return {};
    }

    void publishPath(const std::vector<WorldPoint>& smooth_path, const std::vector<double>& velocities) {
        nav_msgs::msg::Path control_msg;
        nav_msgs::msg::Path visual_msg;

        control_msg.header.frame_id = "map";
        control_msg.header.stamp = this->now();
        visual_msg.header.frame_id = "map";
        visual_msg.header.stamp = this->now();

        size_t num_waypoints = smooth_path.size();
        double yaw = 0.0;

        for (size_t i = 0; i < num_waypoints; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";

            pose.pose.position.x = smooth_path[i].first;
            pose.pose.position.y = smooth_path[i].second;

            if (i < num_waypoints - 1) {
                yaw = std::atan2(smooth_path[i + 1].second - smooth_path[i].second,
                    smooth_path[i + 1].first - smooth_path[i].first);
            }

            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);

            // 🏎️ 1. Pack data payload for your Stanley Controller Node
            pose.pose.position.z = velocities[i];
            control_msg.poses.push_back(pose);

            // 👁️ 2. Flatten out the payload for the human RViz display
            pose.pose.position.z = 0.0;
            visual_msg.poses.push_back(pose);
        }

        // Ship both frames out across the network topology link
        path_pub_->publish(control_msg);
        visual_path_pub_->publish(visual_msg);
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    geometry_msgs::msg::PoseStamped::SharedPtr latest_goal_pose_;
    bool new_goal_set_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr visual_path_pub_;
    rclcpp::TimerBase::SharedPtr plan_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarPlannerNode>());
    rclcpp::shutdown();
    return 0;
}