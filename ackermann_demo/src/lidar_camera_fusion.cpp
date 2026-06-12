// =============================================================================
// LiDAR-Camera Fusion — ROS2 Humble
// Back-projects the camera image into 3D using the 2D laser scan as the depth
// source: each image column takes the lidar depth at its bearing (vertical-
// surface assumption), producing a dense XYZRGB point cloud. Scan points
// outside the camera FOV stay as a gray ring.
// =============================================================================

#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class LidarCameraFusionNode : public rclcpp::Node {
public:
    LidarCameraFusionNode() : Node("lidar_camera_fusion") {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        this->declare_parameter("scan_topic", "/scan_raw");
        this->declare_parameter("image_topic", "/rasppi_camera/image_raw");
        this->declare_parameter("camera_info_topic", "/rasppi_camera/camera_info");
        this->declare_parameter("cloud_topic", "/colored_scan");
        this->declare_parameter("max_image_age_sec", 0.25);   // scans paired with an older image are published uncolored
        this->declare_parameter("keep_uncolored_points", true);
        this->declare_parameter("project_full_image", true);  // false: only colorize the scan ring
        this->declare_parameter("pixel_step", 4);             // back-project every Nth pixel
        this->declare_parameter("max_column_gap_px", 24);     // widest gap between lidar columns to interpolate over
        this->declare_parameter("depth_jump_m", 0.5);         // larger depth steps are object edges, not surfaces
        this->declare_parameter("min_height_m", -0.30);       // clip band relative to the lidar plane: pixels whose
        this->declare_parameter("max_height_m", 2.50);        // vertical-surface depth puts them outside are dropped

        max_image_age_ = this->get_parameter("max_image_age_sec").as_double();
        keep_uncolored_ = this->get_parameter("keep_uncolored_points").as_bool();
        project_full_image_ = this->get_parameter("project_full_image").as_bool();
        pixel_step_ = std::max<int>(1, this->get_parameter("pixel_step").as_int());
        max_column_gap_ = this->get_parameter("max_column_gap_px").as_int();
        depth_jump_ = this->get_parameter("depth_jump_m").as_double();
        min_height_ = this->get_parameter("min_height_m").as_double();
        max_height_ = this->get_parameter("max_height_m").as_double();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("scan_topic").as_string(), rclcpp::SensorDataQoS(),
            std::bind(&LidarCameraFusionNode::scanCallback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            this->get_parameter("image_topic").as_string(), rclcpp::SensorDataQoS(),
            std::bind(&LidarCameraFusionNode::imageCallback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            this->get_parameter("camera_info_topic").as_string(), rclcpp::SensorDataQoS(),
            std::bind(&LidarCameraFusionNode::cameraInfoCallback, this, std::placeholders::_1));

        // Reliable QoS: RViz's PointCloud2 display subscribes reliable by default,
        // which is incompatible with a best-effort publisher
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("cloud_topic").as_string(), rclcpp::QoS(10));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_image_ = msg;
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (camera_model_.fromCameraInfo(msg)) ray_lut_.clear();
        camera_frame_ = msg->header.frame_id;
        has_camera_info_ = true;
    }

    // Rectified ray directions (x/z, y/z) for every sampled pixel, rebuilt only
    // when the intrinsics change — rectifyPoint per pixel per frame is too slow
    void buildRayLut(int width, int height) {
        lut_cols_ = (width + pixel_step_ - 1) / pixel_step_;
        const int lut_rows = (height + pixel_step_ - 1) / pixel_step_;
        ray_lut_.resize(static_cast<size_t>(lut_cols_) * lut_rows);
        for (int v = 0, iv = 0; v < height; v += pixel_step_, ++iv) {
            for (int u = 0, iu = 0; u < width; u += pixel_step_, ++iu) {
                const cv::Point2d rect = camera_model_.rectifyPoint(cv::Point2d(u, v));
                const cv::Point3d ray = camera_model_.projectPixelTo3dRay(rect);
                ray_lut_[iv * lut_cols_ + iu] = cv::Point2f(
                    static_cast<float>(ray.x / ray.z), static_cast<float>(ray.y / ray.z));
            }
        }
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
        sensor_msgs::msg::Image::ConstSharedPtr image;
        std::string camera_frame;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!has_camera_info_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Waiting for camera info...");
                return;
            }
            image = latest_image_;
            camera_frame = camera_frame_;
        }

        cv::Mat rgb;
        if (image) {
            const double age =
                std::abs((rclcpp::Time(scan->header.stamp) - rclcpp::Time(image->header.stamp)).seconds());
            if (age <= max_image_age_) {
                try {
                    rgb = cv_bridge::toCvCopy(image, "rgb8")->image;
                } catch (const cv_bridge::Exception &e) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                         "cv_bridge conversion failed: %s", e.what());
                }
            }
        }

        tf2::Transform laser_to_camera = tf2::Transform::getIdentity();
        bool have_tf = false;
        if (!rgb.empty()) {
            try {
                const auto tf_msg = tf_buffer_->lookupTransform(
                    camera_frame, scan->header.frame_id, tf2::TimePointZero);
                tf2::fromMsg(tf_msg.transform, laser_to_camera);
                have_tf = true;
            } catch (const tf2::TransformException &e) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "TF %s -> %s unavailable: %s",
                                     scan->header.frame_id.c_str(), camera_frame.c_str(), e.what());
            }
        }

        const bool dense = project_full_image_ && !rgb.empty() && have_tf;
        const size_t grid_points =
            dense ? static_cast<size_t>((rgb.cols + pixel_step_ - 1) / pixel_step_) *
                        ((rgb.rows + pixel_step_ - 1) / pixel_step_)
                  : 0;

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = scan->header;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(scan->ranges.size() + grid_points);

        sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> it_r(cloud, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> it_g(cloud, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> it_b(cloud, "b");

        size_t count = 0;
        auto emit = [&](const tf2::Vector3 &p, uint8_t cr, uint8_t cg, uint8_t cb) {
            *it_x = p.x(); *it_y = p.y(); *it_z = p.z();
            *it_r = cr; *it_g = cg; *it_b = cb;
            ++it_x; ++it_y; ++it_z; ++it_r; ++it_g; ++it_b;
            ++count;
        };

        // Pass 1: scan ring — colorize, and record each column's lidar depth
        std::vector<float> col_depth;
        if (dense) col_depth.assign(rgb.cols, std::numeric_limits<float>::quiet_NaN());

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            const float r = scan->ranges[i];
            if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) continue;

            const double angle = scan->angle_min + i * scan->angle_increment;
            const tf2::Vector3 p_laser(r * std::cos(angle), r * std::sin(angle), 0.0);

            uint8_t cr = 128, cg = 128, cb = 128;
            bool colored = false;
            if (have_tf && !rgb.empty()) {
                const tf2::Vector3 p_cam = laser_to_camera * p_laser;
                if (p_cam.z() > 0.0) {
                    // project3dToPixel works on rectified coordinates; unrectifyPoint
                    // maps back into the raw image (no-op when distortion is zero)
                    const cv::Point2d uv = camera_model_.unrectifyPoint(
                        camera_model_.project3dToPixel(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z())));
                    const int u = static_cast<int>(std::lround(uv.x));
                    const int v = static_cast<int>(std::lround(uv.y));
                    if (u >= 0 && u < rgb.cols && v >= 0 && v < rgb.rows) {
                        const cv::Vec3b &px = rgb.at<cv::Vec3b>(v, u);
                        cr = px[0]; cg = px[1]; cb = px[2];
                        colored = true;
                        if (dense) {
                            const float z = static_cast<float>(p_cam.z());
                            if (!(col_depth[u] <= z)) col_depth[u] = z;  // keep nearest hit
                        }
                    }
                }
            }
            if (colored || keep_uncolored_) emit(p_laser, cr, cg, cb);
        }

        if (dense) {
            // Pass 2: fill columns between lidar rays. Linear interpolation across
            // a surface; at depth discontinuities extend the nearer endpoint instead
            int last = -1;
            for (int c = 0; c < rgb.cols; ++c) {
                if (!std::isfinite(col_depth[c])) continue;
                if (last >= 0 && c - last > 1 && c - last <= max_column_gap_) {
                    const float z0 = col_depth[last], z1 = col_depth[c];
                    for (int g = last + 1; g < c; ++g) {
                        if (std::abs(z1 - z0) <= depth_jump_) {
                            const float t = static_cast<float>(g - last) / (c - last);
                            col_depth[g] = z0 + t * (z1 - z0);
                        } else {
                            col_depth[g] = (g - last <= c - g) ? z0 : z1;
                        }
                    }
                }
                last = c;
            }

            // Pass 3: back-project every sampled pixel at its column depth.
            // Constant z per column == the surface is a vertical plane through
            // the lidar hit, exact for walls and upright obstacle faces
            if (ray_lut_.empty() || lut_w_ != rgb.cols || lut_h_ != rgb.rows) {
                buildRayLut(rgb.cols, rgb.rows);
                lut_w_ = rgb.cols; lut_h_ = rgb.rows;
            }
            const tf2::Transform camera_to_laser = laser_to_camera.inverse();
            for (int v = 0, iv = 0; v < rgb.rows; v += pixel_step_, ++iv) {
                for (int u = 0, iu = 0; u < rgb.cols; u += pixel_step_, ++iu) {
                    const float z = col_depth[u];
                    if (!std::isfinite(z)) continue;
                    const cv::Point2f &ray = ray_lut_[iv * lut_cols_ + iu];
                    const tf2::Vector3 p_laser =
                        camera_to_laser * tf2::Vector3(ray.x * z, ray.y * z, z);
                    if (p_laser.z() < min_height_ || p_laser.z() > max_height_) continue;
                    const cv::Vec3b &px = rgb.at<cv::Vec3b>(v, u);
                    emit(p_laser, px[0], px[1], px[2]);
                }
            }
        }

        modifier.resize(count);
        cloud_pub_->publish(cloud);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::mutex data_mutex_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
    image_geometry::PinholeCameraModel camera_model_;
    std::string camera_frame_;
    bool has_camera_info_ = false;

    std::vector<cv::Point2f> ray_lut_;
    int lut_cols_ = 0;
    int lut_w_ = 0;
    int lut_h_ = 0;

    double max_image_age_;
    bool keep_uncolored_;
    bool project_full_image_;
    int pixel_step_;
    int max_column_gap_;
    double depth_jump_;
    double min_height_;
    double max_height_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarCameraFusionNode>());
    rclcpp::shutdown();
    return 0;
}
