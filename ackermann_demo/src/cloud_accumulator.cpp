// =============================================================================
// Colored Cloud Accumulator — ROS2 Humble
// Builds a persistent colored 3D map: transforms each /colored_scan into the
// SLAM map frame and merges it into a voxel-deduplicated cloud republished on
// /colored_map. First colored observation of a voxel wins; gray (out-of-FOV)
// points only fill voxels nothing colored has claimed yet.
// =============================================================================

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class CloudAccumulatorNode : public rclcpp::Node {
public:
    CloudAccumulatorNode() : Node("cloud_accumulator") {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        this->declare_parameter("cloud_topic", "/colored_scan");
        this->declare_parameter("map_topic", "/colored_map");
        this->declare_parameter("target_frame", "map");      // slam_toolbox corrects this frame on loop closure
        this->declare_parameter("voxel_size", 0.05);
        this->declare_parameter("publish_period_sec", 1.0);
        this->declare_parameter("max_voxels", 2000000);

        target_frame_ = this->get_parameter("target_frame").as_string();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        max_voxels_ = static_cast<size_t>(this->get_parameter("max_voxels").as_int());

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("cloud_topic").as_string(), 10,
            std::bind(&CloudAccumulatorNode::cloudCallback, this, std::placeholders::_1));

        // Transient local so a late-started RViz receives the map immediately
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("map_topic").as_string(),
            rclcpp::QoS(1).reliable().transient_local());

        publish_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(this->get_parameter("publish_period_sec").as_double()),
            std::bind(&CloudAccumulatorNode::publishMap, this));

        clear_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/clear",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                voxel_index_.clear();
                points_.clear();
                res->success = true;
                res->message = "colored map cleared";
            });
    }

private:
    struct ColoredPoint {
        float x, y, z;
        uint8_t r, g, b;
        bool colored;
    };

    static uint64_t voxelKey(const tf2::Vector3 &p, double inv_voxel) {
        // 21 bits per axis around a 2^20 bias: ±52 km at 5 cm voxels
        const auto q = [inv_voxel](double v) {
            return static_cast<uint64_t>(
                       static_cast<int64_t>(std::floor(v * inv_voxel)) + (1 << 20)) & 0x1FFFFF;
        };
        return (q(p.x()) << 42) | (q(p.y()) << 21) | q(p.z());
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        tf2::Transform to_map;
        try {
            const auto tf_msg = tf_buffer_->lookupTransform(
                target_frame_, msg->header.frame_id, msg->header.stamp,
                rclcpp::Duration::from_seconds(0.1));
            tf2::fromMsg(tf_msg.transform, to_map);
        } catch (const tf2::TransformException &e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "TF %s -> %s unavailable, scan skipped (is SLAM running?): %s",
                                 msg->header.frame_id.c_str(), target_frame_.c_str(), e.what());
            return;
        }

        const double inv_voxel = 1.0 / voxel_size_;
        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x"), it_y(*msg, "y"), it_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> it_r(*msg, "r"), it_g(*msg, "g"), it_b(*msg, "b");

        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_r, ++it_g, ++it_b) {
            if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z)) continue;
            const tf2::Vector3 p = to_map * tf2::Vector3(*it_x, *it_y, *it_z);
            // the fusion node marks out-of-FOV ring points with exactly this gray
            const bool colored = !(*it_r == 128 && *it_g == 128 && *it_b == 128);

            const uint64_t key = voxelKey(p, inv_voxel);
            const auto found = voxel_index_.find(key);
            if (found != voxel_index_.end()) {
                ColoredPoint &existing = points_[found->second];
                if (colored && !existing.colored) {
                    existing.r = *it_r; existing.g = *it_g; existing.b = *it_b;
                    existing.colored = true;
                }
            } else if (points_.size() < max_voxels_) {
                voxel_index_.emplace(key, points_.size());
                points_.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()),
                                   static_cast<float>(p.z()), *it_r, *it_g, *it_b, colored});
            }
        }
    }

    void publishMap() {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = target_frame_;
        cloud.header.stamp = this->now();
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(points_.size());

        sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x"), it_y(cloud, "y"), it_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> it_r(cloud, "r"), it_g(cloud, "g"), it_b(cloud, "b");
        for (const ColoredPoint &p : points_) {
            *it_x = p.x; *it_y = p.y; *it_z = p.z;
            *it_r = p.r; *it_g = p.g; *it_b = p.b;
            ++it_x; ++it_y; ++it_z; ++it_r; ++it_g; ++it_b;
        }
        map_pub_->publish(cloud);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unordered_map<uint64_t, size_t> voxel_index_;
    std::vector<ColoredPoint> points_;

    std::string target_frame_;
    double voxel_size_;
    size_t max_voxels_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudAccumulatorNode>());
    rclcpp::shutdown();
    return 0;
}
