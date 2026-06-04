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

struct PairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        return std::hash<T1>()(p.first) ^ (std::hash<T2>()(p.second) << 1);
    }
};

using Point2D = std::pair<int, int>;
using QueueElement = std::pair<double, Point2D>;

class AStarPlannerNode : public rclcpp::Node {
public:
    AStarPlannerNode() : Node("astar_planner"), new_goal_set_(false) {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&AStarPlannerNode::mapCallback, this, std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&AStarPlannerNode::goalCallback, this, std::placeholders::_1));
        
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/custom_astar_path", 10);
        plan_timer_ = this->create_wall_timer(200ms, std::bind(&AStarPlannerNode::replanTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "🚀 Optimized C++ A* Planner Ready!");
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
        return {cx, cy};
    }

    std::pair<double, double> gridToWorld(int cx, int cy) {
        double wx = map_msg_->info.origin.position.x + (cx + 0.5) * map_msg_->info.resolution;
        double wy = map_msg_->info.origin.position.y + (cy + 0.5) * map_msg_->info.resolution;
        return {wx, wy};
    }

    // ✅ OPTIMIZED: Added toggle to bypass inflation for start/end nodes to prevent getting stuck
    bool isValidAndFree(int cx, int cy, bool is_start_or_goal = false) {
        if (!map_msg_) return false;
        if (cx < 0 || cx >= static_cast<int>(map_msg_->info.width) || cy < 0 || cy >= static_cast<int>(map_msg_->info.height)) {
            return false;
        }

        // Hard collision check against the raw pixel value first
        int base_index = cx + (cy * map_msg_->info.width);
        if (map_msg_->data[base_index] > 0 || map_msg_->data[base_index] == -1) return false;
        if (is_start_or_goal) return true; // Bypass heavy neighborhood checks for origin/destination cells

        int inflation_cells = 6; // Optimized slightly down from 8 for cleaner performance cycles
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                if (std::hypot(dx, dy) > inflation_cells) continue;

                int check_x = cx + dx;
                int check_y = cy + dy;

                if (check_x < 0 || check_x >= static_cast<int>(map_msg_->info.width) || check_y < 0 || check_y >= static_cast<int>(map_msg_->info.height)) {
                    return false;
                }

                int index = check_x + (check_y * map_msg_->info.width);
                if (map_msg_->data[index] > 0 || map_msg_->data[index] == -1) {
                    return false;
                }
            }
        }
        return true;
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
            path_pub_->publish(empty_path);
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
            publishPath(path_grid);
            if (new_goal_set_) new_goal_set_ = false;
        }
    }

    std::vector<Point2D> astarSearch(Point2D start, Point2D goal) {
        std::vector<Point2D> neighbors = {
            {0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
        };

        std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> open_set;
        open_set.push({0.0, start});

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

            for (const auto& dir : neighbors) {
                Point2D neighbor = {current.first + dir.first, current.second + dir.second};

                // Inside the neighbor loop in astarSearch:
                bool is_escaping = (current == start || neighbor == goal);
                if (!isValidAndFree(neighbor.first, neighbor.second, is_escaping)) continue;
                
                double move_cost = std::hypot(dir.first, dir.second);
                double tentative_g_score = g_score[current] + move_cost;

                if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g_score;
                    
                    double h_score = std::hypot(goal.first - neighbor.first, goal.second - neighbor.second);
                    open_set.push({tentative_g_score + h_score, neighbor});
                }
            }
        }
        return {};
    }

    void publishPath(const std::vector<Point2D>& grid_path) {
        nav_msgs::msg::Path msg;
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();

        size_t num_waypoints = grid_path.size();
        double yaw = 0.0; // Persistent tracking variable to avoid cross-contamination of structures

        for (size_t i = 0; i < num_waypoints; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";

            auto [wx, wy] = gridToWorld(grid_path[i].first, grid_path[i].second);
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;

            if (i < num_waypoints - 1) {
                auto [next_wx, next_wy] = gridToWorld(grid_path[i+1].first, grid_path[i+1].second);
                yaw = std::atan2(next_wy - wy, next_wx - wx);
            }

            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);

            msg.poses.push_back(pose);
        }
        path_pub_->publish(msg);
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    geometry_msgs::msg::PoseStamped::SharedPtr latest_goal_pose_;
    bool new_goal_set_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_; 
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr plan_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<AStarPlannerNode>());
    rclcpp::shutdown();
    return 0;
}