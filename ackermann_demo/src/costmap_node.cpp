#include <cmath>
#include <vector>
#include <memory>
#include <algorithm>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// Structural container to hold relative mask properties
struct MaskOffset {
    int dx;
    int dy;
    int8_t cost;
};

class CostmapInflationNode : public rclcpp::Node {
public:
    CostmapInflationNode() : Node("costmap_inflation_node"), last_inflation_cells_(-1), params_changed_(false) {
        // Declare live tunable physical configuration parameters (in meters)
        this->declare_parameter("robot_radius", 0.25);     // Physical radius of the vehicle chassis
        this->declare_parameter("safety_margin", 0.25);    // Width of the decaying cost cushion zone

        robot_radius_ = this->get_parameter("robot_radius").as_double();
        safety_margin_ = this->get_parameter("safety_margin").as_double();

        // Register the runtime parameter reconfiguration callback handle
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&CostmapInflationNode::onParameterChange, this, std::placeholders::_1));

        // Setup Pub/Sub topology pipeline links
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&CostmapInflationNode::mapCallback, this, std::placeholders::_1));
        
        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/inflated_costmap", 10);

        RCLCPP_INFO(this->get_logger(), "🛡️ Graded Costmap Inflation Node Online & Tunable!");
    }

private:
    // ✅ Parameter Callback: Catches updates live from command line or dashboards
    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
            if (param.get_name() == "robot_radius") {
                robot_radius_ = param.as_double();
                params_changed_ = true;
                RCLCPP_INFO(this->get_logger(), "🔧 Parameter updated: robot_radius = %.3fm", robot_radius_);
            }
            if (param.get_name() == "safety_margin") {
                safety_margin_ = param.as_double();
                params_changed_ = true;
                RCLCPP_INFO(this->get_logger(), "🔧 Parameter updated: safety_margin = %.3fm", safety_margin_);
            }
        }
        return result;
    }

    // ✅ Graded Cost Precomputation: Generates cost value distributions based on distance
    void precomputeMask(double resolution) {
        mask_offsets_.clear();

        for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
            for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
                // Calculate physical distance in meters to match vehicle dimension scaling
                double distance = std::hypot(dx, dy) * resolution;

                if (distance <= robot_radius_) {
                    // 💥 Inside vehicle perimeter: Explicitly mark as a Lethal Obstacle
                    mask_offsets_.push_back({dx, dy, 100});
                } 
                else if (distance <= (robot_radius_ + safety_margin_)) {
                    // 📉 Inside cushion perimeter: Smooth linear decay cost calculation (99 down to 1)
                    if (safety_margin_ > 0.001) {
                        double factor = ((robot_radius_ + safety_margin_) - distance) / safety_margin_;
                        int8_t graded_cost = static_cast<int8_t>(1 + std::round(factor * 98));
                        mask_offsets_.push_back({dx, dy, graded_cost});
                    }
                }
            }
        }
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        double resolution = msg->info.resolution;
        if (resolution <= 0.0) return;

        double total_inflation_distance = robot_radius_ + safety_margin_;
        int current_inflation_cells = static_cast<int>(std::ceil(total_inflation_distance / resolution));

        // Recompute the look-up footprint array if resolution or live parameters change
        if (current_inflation_cells != last_inflation_cells_ || params_changed_) {
            inflation_cells_ = current_inflation_cells;
            precomputeMask(resolution);
            last_inflation_cells_ = inflation_cells_;
            params_changed_ = false;
            
            RCLCPP_INFO(this->get_logger(), 
                "🛡️ Costmap Mask Regenerated: Core lethal radius uses %d cells.", inflation_cells_);
        }

        int width = msg->info.width;
        int height = msg->info.height;

        // Initialize output payload as a deep copy of the raw environment base map structural layout
        auto inflated_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>(*msg);

        // Process the array topology map
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int base_idx = x + (y * width);

                // Find solid obstacles inside the immutable input buffer data stream
                if (msg->data[base_idx] == 100) {
                    for (const auto& offset : mask_offsets_) {
                        int nx = x + offset.dx;
                        int ny = y + offset.dy;

                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            int target_idx = nx + (ny * width);
                            int8_t original_cell_value = msg->data[target_idx];

                            // Skip processing for hard walls and unknown unmapped territories (-1)
                            if (original_cell_value == -1 || original_cell_value == 100) continue;

                            // Keeping the maximum cost penalty prevents overlapping masks from diluting wall weights
                            inflated_msg->data[target_idx] = std::max(inflated_msg->data[target_idx], offset.cost);
                        }
                    }
                }
            }
        }
        costmap_pub_->publish(*inflated_msg);
    }

    // Class Attributes
    double robot_radius_;
    double safety_margin_;
    int inflation_cells_;
    int last_inflation_cells_;
    std::atomic<bool> params_changed_;

    std::vector<MaskOffset> mask_offsets_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapInflationNode>());
    rclcpp::shutdown();
    return 0;
}