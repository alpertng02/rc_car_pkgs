/**
 *@brief costmap_node.cpp
 *
 */


 // Inflates the SLAM occupancy grid into a graded costmap for the planners:
 //   • cells within robot_radius of an obstacle  -> lethal (100)
 //   • cells within the safety_margin cushion    -> graded cost (99..1)
 //   • everything else                           -> free (0); unknown stays -1


#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class CostmapInflationNode : public rclcpp::Node {
public:
    CostmapInflationNode() : Node("costmap_inflation_node") {
        // Parameters
        this->declare_parameter("robot_radius", 0.25);  // lethal inscribed radius
        this->declare_parameter("safety_margin", 0.25);  // graded cushion width beyond the radius
        this->declare_parameter("occupied_threshold", 65);    // cell value treated as a hard obstacle
        this->declare_parameter("decay", std::string("linear"));  // "linear" | "exponential"
        this->declare_parameter("cost_scaling_factor", 10.0);  // exponential steepness [1/m] (decay="exponential")
        this->declare_parameter("min_recompute_period", 1.0);   // throttle heavy recompute [s]

        const double period = std::max(0.05, this->get_parameter("min_recompute_period").as_double());

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&CostmapInflationNode::onParameterChange, this, std::placeholders::_1));

        // Latched, reliable QoS: late-joining
        // programs can immediately receive the most recent costmap.
        auto latched = rclcpp::QoS(1).reliable().transient_local();
        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/inflated_costmap", latched);
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", latched, std::bind(&CostmapInflationNode::mapCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(period), std::bind(&CostmapInflationNode::onTimer, this));

        RCLCPP_INFO(this->get_logger(),
            "Costmap inflation node online (%.2fs throttle) -> /inflated_costmap", period);
    }

private:
    // Validate live parameter edits and request a recompute.
    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& p : params) {
            const auto& name = p.get_name();
            if (name == "robot_radius" || name == "safety_margin") {
                if (p.as_double() < 0.0) {
                    result.successful = false;
                    result.reason = name + " must be >= 0";
                }
            } else if (name == "cost_scaling_factor") {
                if (p.as_double() <= 0.0) {
                    result.successful = false;
                    result.reason = "cost_scaling_factor must be > 0";
                }
            } else if (name == "occupied_threshold") {
                const long v = p.as_int();
                if (v < 1 || v > 100) {
                    result.successful = false;
                    result.reason = "occupied_threshold must be in [1, 100]";
                }
            } else if (name == "decay") {
                const std::string v = p.as_string();
                if (v != "linear" && v != "exponential") {
                    result.successful = false;
                    result.reason = "decay must be \"linear\" or \"exponential\"";
                }
            }
        }
        if (result.successful) dirty_ = true;   // apply on the next timer tick
        return result;
    }

    // Receive the latest map
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latest_map_ = msg;
        dirty_ = true;
    }

    // Check if a new map arrived. Only trigger an update if map changes to save CPU .
    void onTimer() {
        if (!dirty_ || !latest_map_) return;
        auto map = latest_map_;
        dirty_ = false;
        inflate(*map);
    }

    void inflate(const nav_msgs::msg::OccupancyGrid& map) {
        const int w = static_cast<int>(map.info.width);
        const int h = static_cast<int>(map.info.height);

        // Validate map size and resolution.
        if (w <= 0 || h <= 0 || static_cast<size_t>(w) * h != map.data.size()) return;

        const double res = map.info.resolution;
        if (res <= 0.0) return;

        const double robot_radius = this->get_parameter("robot_radius").as_double();
        const double safety_margin = this->get_parameter("safety_margin").as_double();
        const int    occ_th = static_cast<int>(this->get_parameter("occupied_threshold").as_int());
        const bool   exponential = (this->get_parameter("decay").as_string() == "exponential");
        const double scaling = this->get_parameter("cost_scaling_factor").as_double();
        const double inflation_radius = robot_radius + safety_margin;

        nav_msgs::msg::OccupancyGrid out;
        out.header = map.header;
        out.header.stamp = this->now();
        out.info = map.info;
        out.data.assign(map.data.size(), 0);

        // Obstacle mask for the distance transform: obstacle -> 0, else -> 255
        // (cv::distanceTransform measures the distance to the nearest zero pixel).
        // Unknown (-1) is treated as free here, matching the original node.
        cv::Mat obs(h, w, CV_8U);
        uchar* op = obs.data;
        int obstacle_count = 0;
        for (size_t i = 0; i < map.data.size(); ++i) {
            const bool occ = (map.data[i] >= occ_th);
            op[i] = occ ? 0 : 255;
            obstacle_count += occ ? 1 : 0;
        }

        // No obstacles -> nothing to inflate; emit free space, preserving unknown.
        if (obstacle_count == 0) {
            for (size_t i = 0; i < map.data.size(); ++i)
                if (map.data[i] < 0) out.data[i] = -1;
            costmap_pub_->publish(out);
            return;
        }

        cv::Mat dist_px;   // exact L2 distance (no labels needed -> PRECISE is fine)
        cv::distanceTransform(obs, dist_px, cv::DIST_L2, cv::DIST_MASK_PRECISE);
        const float* dp = dist_px.ptr<float>();

        for (size_t i = 0; i < map.data.size(); ++i) {
            const int8_t v = map.data[i];
            if (v < 0) { out.data[i] = -1;  continue; }    // unknown stays unknown
            if (v >= occ_th) { out.data[i] = 100; continue; }    // hard obstacle stays lethal

            const double d = static_cast<double>(dp[i]) * res;   // metres to nearest obstacle
            if (d <= robot_radius) {
                out.data[i] = 100;                               // inscribed: lethal core
            } else if (d <= inflation_radius) {
                out.data[i] = gradedCost(d, robot_radius, safety_margin, exponential, scaling);
            }
            // else: free cell beyond influence -> stays 0
        }

        costmap_pub_->publish(out);
    }

    // Cushion cost for d in (robot_radius, inflation_radius], clamped to [1, 99].
    static int8_t gradedCost(double d, double robot_radius, double safety_margin,
        bool exponential, double scaling) {
        double cost;
        if (exponential) {
            // nav2-style exponential falloff from the inscribed edge.
            cost = 99.0 * std::exp(-scaling * (d - robot_radius));
        } else {
            // Linear decay: 99 at the inscribed edge down to 1 at the cushion edge.
            const double factor = (safety_margin > 1e-6)
                ? ((robot_radius + safety_margin) - d) / safety_margin : 0.0;
            cost = 1.0 + factor * 98.0;
        }
        return static_cast<int8_t>(std::clamp<long>(std::lround(cost), 1, 99));
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr    costmap_pub_;
    rclcpp::TimerBase::SharedPtr                                  timer_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr        param_callback_handle_;

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    bool dirty_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapInflationNode>());
    rclcpp::shutdown();
    return 0;
}
