// =============================================================================
// Stanley Path Tracking Controller — ROS2 Humble with Displacement Safety
// =============================================================================

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/int8.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class StanleyControllerNode : public rclcpp::Node {
public:
    StanleyControllerNode()
        : Node("stanley_controller"),
        target_wp_idx_(0),
        has_path_(false),
        current_speed_(0.0),
        latest_range_(std::numeric_limits<double>::max()),
        accel_bias_(0.0),
        accel_activity_(0.0),
        accel_peak_(0.0),
        latest_wheel_speed_(0.0),
        stall_timer_started_(false),
        is_stalled_(false),
        stall_start_x_(0.0),
        stall_start_y_(0.0) {
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Parameter declarations
        this->declare_parameter("control_hz", 50.0);
        this->declare_parameter("wheelbase", 0.1688);         
        this->declare_parameter("stanley_k", 1.3);            
        this->declare_parameter("stanley_k_soft", 0.15);      
        this->declare_parameter("max_steering_angle", 0.523);  
        this->declare_parameter("max_linear_velocity", 0.70);  
        this->declare_parameter("max_reverse_velocity", 0.45); 
        this->declare_parameter("goal_tolerance", 0.25);       
        this->declare_parameter("max_acceleration", 0.50);     
        this->declare_parameter("max_deceleration", 0.90);     
        this->declare_parameter("steer_speed_reduction", 0.40); 
        this->declare_parameter("path_timeout_sec", 3.0);      
        this->declare_parameter("ultrasonic_safety_dist", 0.35); 
        
        // Stall parameters
        this->declare_parameter("stall_velocity_threshold", 0.15);
        this->declare_parameter("stall_timeout_sec", 1.5);
        this->declare_parameter("stall_displacement_threshold", 0.05);
        this->declare_parameter("stall_odom_threshold", 0.02);      // wheel speed below which the drivetrain counts as not turning [m/s]
        this->declare_parameter("stall_accel_threshold", 0.05);     // IMU activity below which the chassis counts as physically static [m/s²]
        this->declare_parameter("collision_accel_threshold", 3.0);  // IMU forward-acceleration spike treated as an impact [m/s²]

        // ROS2 interfaces
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/followed_path", 10,
            std::bind(&StanleyControllerNode::pathCallback, this, std::placeholders::_1));

        ultrasonic_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/ultrasonic/range", rclcpp::SensorDataQoS(),
            std::bind(&StanleyControllerNode::ultrasonicCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", rclcpp::SensorDataQoS(),
            std::bind(&StanleyControllerNode::imuCallback, this, std::placeholders::_1));

        // gazebo.xacro remaps the steering controller's ~/odometry to /odom
        wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&StanleyControllerNode::wheelOdomCallback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_stanley", 10);
        // 0 = stall cleared, +1 = stalled while driving forward, -1 = stalled while reversing
        stall_pub_ = this->create_publisher<std_msgs::msg::Int8>("/robot_stall", 10);

        double initial_hz = this->get_parameter("control_hz").as_double();
        if (initial_hz <= 0.0) initial_hz = 50.0; 
        
        std::chrono::milliseconds timer_period_ms(static_cast<int>(1000.0 / initial_hz));
        control_timer_ = this->create_wall_timer(
            timer_period_ms, std::bind(&StanleyControllerNode::controlLoop, this));

        last_path_time_ = this->now();
        path_receive_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "🏎️ Stanley Controller Online with Smooth Escape Pipelines.");
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (msg->poses.empty()) {
            has_path_ = false;
            current_path_ = nullptr;
            current_speed_ = 0.0;
            return;
        }
        // The planner re-sends the unchanged plan (same stamp) as a keep-alive:
        // refresh the watchdog without resetting tracking or stall state.
        if (current_path_ && msg->header.stamp == current_path_->header.stamp &&
            msg->poses.size() == current_path_->poses.size()) {
            last_path_time_ = this->now();
            return;
        }
        current_path_ = msg;
        target_wp_idx_ = 0;
        // current_speed_ is deliberately not reset so the speed ramp stays continuous over replans
        has_path_ = true;

        if (is_stalled_) {
            is_stalled_ = false;
            std_msgs::msg::Int8 stall_msg;
            stall_msg.data = 0;
            stall_pub_->publish(stall_msg);
        }
        stall_timer_started_ = false;
        last_path_time_ = this->now();
        path_receive_time_ = this->now(); // Track plan handshake to initialize grace windows
    }

    void ultrasonicCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(ultrasonic_mutex_);
        if (msg->range >= msg->min_range && msg->range <= msg->max_range) {
            latest_range_ = msg->range;
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        // A slow EMA tracks the static offset (mounting tilt, sensor bias); the
        // residual is the dynamic part of the forward acceleration. A faster EMA
        // of its magnitude gives "is the chassis physically moving", and a
        // peak-hold catches impact spikes between control cycles.
        constexpr double BIAS_ALPHA     = 0.002;  // ~5 s at 100 Hz
        constexpr double ACTIVITY_ALPHA = 0.05;   // ~0.2 s at 100 Hz
        accel_bias_ += BIAS_ALPHA * (msg->linear_acceleration.x - accel_bias_);
        const double dyn = std::abs(msg->linear_acceleration.x - accel_bias_);
        accel_activity_ += ACTIVITY_ALPHA * (dyn - accel_activity_);
        accel_peak_ = std::max(accel_peak_, dyn);
    }

    void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        latest_wheel_speed_ = msg->twist.twist.linear.x;
    }

    inline double normalizeAngle(double a) const {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    inline double quaternionToYaw(double qx, double qy, double qz, double qw) const {
        return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }

    void publishStop() {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_footprint";
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = 0.0;
        cmd_pub_->publish(cmd);
        current_speed_ = 0.0;
    }

    // =========================================================================
    // MAIN CONTROL LOOP
    // =========================================================================
    void controlLoop() {
        const double control_hz = this->get_parameter("control_hz").as_double();
        const double wheelbase = this->get_parameter("wheelbase").as_double();
        const double stanley_k = this->get_parameter("stanley_k").as_double();
        const double stanley_k_soft = this->get_parameter("stanley_k_soft").as_double();
        const double max_steer = this->get_parameter("max_steering_angle").as_double();
        const double max_v_fwd = this->get_parameter("max_linear_velocity").as_double();
        const double max_v_rev = this->get_parameter("max_reverse_velocity").as_double();
        const double goal_tol = this->get_parameter("goal_tolerance").as_double();
        const double max_accel = this->get_parameter("max_acceleration").as_double();
        const double max_decel = this->get_parameter("max_deceleration").as_double();
        const double steer_reduce = this->get_parameter("steer_speed_reduction").as_double();
        const double path_timeout = this->get_parameter("path_timeout_sec").as_double();
        const double safety_dist = this->get_parameter("ultrasonic_safety_dist").as_double();
        
        const double stall_v_thresh = this->get_parameter("stall_velocity_threshold").as_double();
        const double stall_timeout = this->get_parameter("stall_timeout_sec").as_double();
        const double stall_disp_thresh = this->get_parameter("stall_displacement_threshold").as_double();
        const double stall_odom_thresh = this->get_parameter("stall_odom_threshold").as_double();
        const double stall_accel_thresh = this->get_parameter("stall_accel_threshold").as_double();
        const double collision_accel_thresh = this->get_parameter("collision_accel_threshold").as_double();

        const double dt = (control_hz > 0.0) ? (1.0 / control_hz) : 0.02;

        nav_msgs::msg::Path::SharedPtr local_path;
        bool local_has_path;
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            local_path = current_path_;
            local_has_path = has_path_;
        }

        if (local_has_path) {
            const double age = (this->now() - last_path_time_).seconds();
            if (age > path_timeout) {
                publishStop();
                return;
            }
        }

        double robot_x, robot_y, robot_yaw;
        try {
            auto tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
            robot_x = tf.transform.translation.x;
            robot_y = tf.transform.translation.y;
            robot_yaw = quaternionToYaw(
                tf.transform.rotation.x, tf.transform.rotation.y,
                tf.transform.rotation.z, tf.transform.rotation.w);
        } catch (const tf2::TransformException&) {
            return;  
        }

        if (!local_has_path || !local_path || local_path->poses.empty()) {
            publishStop();
            return;
        }

        const size_t N = local_path->poses.size();

        const double goal_x = local_path->poses.back().pose.position.x;
        const double goal_y = local_path->poses.back().pose.position.y;
        if (std::hypot(goal_x - robot_x, goal_y - robot_y) < goal_tol) {
            RCLCPP_INFO(this->get_logger(), "🏁 Goal reached. Halting.");
            publishStop();
            std::lock_guard<std::mutex> lock(path_mutex_);
            has_path_ = false;
            current_path_ = nullptr;
            return;
        }

        target_wp_idx_ = std::min(target_wp_idx_, N - 1);

        // The motion direction comes from the first non-zero commanded velocity at or
        // after the target waypoint: zero entries (path end) carry no direction, and
        // looking ahead lets the sign flip as a cusp is reached.
        auto signedVelFrom = [&local_path, N](size_t from) {
            for (size_t i = from; i < N; ++i) {
                const double vz = local_path->poses[i].pose.position.z;
                if (std::abs(vz) > 1e-6) return vz;
            }
            return 0.0;
        };
        const bool reversing = (signedVelFrom(target_wp_idx_) < 0.0);

        double ref_x, ref_y, motion_yaw;
        if (!reversing) {
            ref_x = robot_x + wheelbase * std::cos(robot_yaw);
            ref_y = robot_y + wheelbase * std::sin(robot_yaw);
            motion_yaw = robot_yaw;
        } else {
            // 🟢 FIXED: base_footprint sits directly on the rear axle center.
            // Erased the translation calculation to point directly to robot chassis frame origins.
            ref_x = robot_x;
            ref_y = robot_y;
            motion_yaw = normalizeAngle(robot_yaw + M_PI);
        }

        const size_t search_start = (target_wp_idx_ > 4) ? target_wp_idx_ - 4 : 0;
        const size_t search_end = std::min(N - 1, target_wp_idx_ + 30);

        double min_dist = std::numeric_limits<double>::max();
        size_t best_idx = target_wp_idx_;

        for (size_t i = search_start; i <= search_end; ++i) {
            const double dx = local_path->poses[i].pose.position.x - ref_x;
            const double dy = local_path->poses[i].pose.position.y - ref_y;
            const double d = std::hypot(dx, dy);
            if (d < min_dist) { min_dist = d; best_idx = i; }
        }
        target_wp_idx_ = best_idx;
        const double path_vel_raw = signedVelFrom(target_wp_idx_);

        const size_t next_idx = std::min(target_wp_idx_ + 1, N - 1);
        const double cur_wx = local_path->poses[target_wp_idx_].pose.position.x;
        const double cur_wy = local_path->poses[target_wp_idx_].pose.position.y;
        const double nxt_wx = local_path->poses[next_idx].pose.position.x;
        const double nxt_wy = local_path->poses[next_idx].pose.position.y;

        double path_tangent;
        if (std::hypot(nxt_wx - cur_wx, nxt_wy - cur_wy) > 1e-5) {
            path_tangent = std::atan2(nxt_wy - cur_wy, nxt_wx - cur_wx);
        } else {
            path_tangent = quaternionToYaw(
                local_path->poses[target_wp_idx_].pose.orientation.x,
                local_path->poses[target_wp_idx_].pose.orientation.y,
                local_path->poses[target_wp_idx_].pose.orientation.z,
                local_path->poses[target_wp_idx_].pose.orientation.w);
            if (reversing) path_tangent = normalizeAngle(path_tangent + M_PI);
        }

        const double heading_error = normalizeAngle(path_tangent - motion_yaw);
        const double err_x = ref_x - cur_wx;
        const double err_y = ref_y - cur_wy;
        const double cte = std::sin(path_tangent) * err_x - std::cos(path_tangent) * err_y;

        const double v_eff = std::max(std::abs(path_vel_raw), stanley_k_soft);
        const double cte_term = std::atan2(stanley_k * cte, v_eff);
        const double raw_delta = heading_error + cte_term;
        double steering_angle = std::clamp(raw_delta, -max_steer, max_steer);

        if (reversing) steering_angle = -steering_angle;

        const double v_limit = reversing ? max_v_rev : max_v_fwd;
        double speed_target = std::clamp(path_vel_raw, -v_limit, v_limit);

        double current_range = std::numeric_limits<double>::max();
        {
            std::lock_guard<std::mutex> lock(ultrasonic_mutex_);
            current_range = latest_range_;
        }

        const bool ultrasonic_block = (!reversing && current_range < safety_dist);

        double accel_activity, accel_peak;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            accel_activity = accel_activity_;
            accel_peak = accel_peak_;
            accel_peak_ = 0.0;   // peak-hold is consumed once per control cycle
        }
        double wheel_speed;
        {
            std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
            wheel_speed = std::abs(latest_wheel_speed_);
        }

        // 1.0 second motor spin-up grace window right after plan switches
        // to prevent zero-displacement false flags while gears drop into reverse.
        bool plan_grace_active = (this->now() - path_receive_time_).seconds() < 1.0;

        // The stall check runs on the desired speed *before* the ultrasonic override:
        // an obstacle that holds the car still must still arm the stall timer, so the
        // escape replan fires instead of the car waiting in place forever.
        if (std::abs(speed_target) >= stall_v_thresh && !plan_grace_active) {
            if (!stall_timer_started_) {
                stall_start_x_ = robot_x;
                stall_start_y_ = robot_y;
                stall_start_time_ = this->now();
                last_wheel_motion_time_ = this->now();
                stall_timer_started_ = true;
            } else {
                const double traveled_distance = std::hypot(robot_x - stall_start_x_, robot_y - stall_start_y_);
                const double elapsed_seconds = (this->now() - stall_start_time_).seconds();

                if (wheel_speed >= stall_odom_thresh) last_wheel_motion_time_ = this->now();
                const double wheels_dead_seconds = (this->now() - last_wheel_motion_time_).seconds();

                const char* stall_cause = nullptr;
                if (accel_peak > collision_accel_thresh) {
                    stall_cause = "impact spike on IMU";
                } else if (wheels_dead_seconds > stall_timeout) {
                    // Wheel odometry never turned: detected without the map TF, so
                    // this path survives localization freezes and SLAM jumps.
                    stall_cause = ultrasonic_block ? "held by ultrasonic obstacle"
                                                   : "drivetrain not turning";
                } else if (traveled_distance > stall_disp_thresh) {
                    stall_start_x_ = robot_x;
                    stall_start_y_ = robot_y;
                    stall_start_time_ = this->now();
                } else if (elapsed_seconds > stall_timeout) {
                    if (accel_activity < stall_accel_thresh) {
                        stall_cause = (wheel_speed >= stall_odom_thresh)
                            ? "wheels slipping in place" : "no displacement";
                    } else {
                        // The IMU says the chassis is physically moving while the map
                        // pose says it is not: distrust the pose rather than forcing
                        // an escape maneuver from a position that may be wrong.
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "No map displacement but IMU activity is %.3f m/s²; suspecting localization, holding off stall.",
                            accel_activity);
                    }
                }

                if (stall_cause) {
                    is_stalled_ = true;

                    std_msgs::msg::Int8 stall_msg;
                    stall_msg.data = reversing ? -1 : 1;
                    stall_pub_->publish(stall_msg);

                    RCLCPP_ERROR(this->get_logger(),
                        "STALL (%s) while commanding %s motion. Requesting escape replan.",
                        stall_cause, reversing ? "reverse" : "forward");
                }
            }
        } else {
            stall_timer_started_ = false;
        }

        if (is_stalled_) {
            publishStop();
            std::lock_guard<std::mutex> lock(path_mutex_);
            has_path_ = false;
            current_path_ = nullptr; 
            return;
        }

        if (ultrasonic_block) {
            speed_target = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Ultrasonic obstacle at %.2f m (limit %.2f m). Holding the car stopped.",
                current_range, safety_dist);
        }

        const double steer_ratio = std::abs(steering_angle) / max_steer;
        const double speed_scale = std::max(0.20, 1.0 - steer_reduce * steer_ratio);
        speed_target *= speed_scale;

        const double dv_desired = speed_target - current_speed_;
        const bool decelerating = (dv_desired * current_speed_ <= 0.0) || (std::abs(speed_target) < std::abs(current_speed_));
        const double rate = decelerating ? max_decel : max_accel;
        const double dv_cap = rate * dt;
        current_speed_ += std::clamp(dv_desired, -dv_cap, dv_cap);

        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_footprint";
        cmd.twist.linear.x = current_speed_;
        cmd.twist.angular.z = steering_angle;
        cmd_pub_->publish(cmd);
    }

    size_t        target_wp_idx_;
    bool          has_path_;
    double        current_speed_;     
    rclcpp::Time  last_path_time_;
    rclcpp::Time  path_receive_time_; // 🟢 Added state asset tracking

    std::mutex                         path_mutex_;
    nav_msgs::msg::Path::SharedPtr     current_path_;

    std::mutex                         ultrasonic_mutex_;
    double                             latest_range_;

    std::mutex                         imu_mutex_;
    double                             accel_bias_;       // slow EMA of raw accel x (gravity/mounting offset)
    double                             accel_activity_;   // fast EMA of |dynamic accel x|
    double                             accel_peak_;       // peak |dynamic accel x| since last control cycle

    std::mutex                         wheel_odom_mutex_;
    double                             latest_wheel_speed_;

    bool         stall_timer_started_;
    rclcpp::Time stall_start_time_;
    rclcpp::Time last_wheel_motion_time_;
    bool         is_stalled_;
    double       stall_start_x_;
    double       stall_start_y_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr           path_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr       ultrasonic_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       wheel_odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr              stall_pub_;
    rclcpp::TimerBase::SharedPtr                                   control_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StanleyControllerNode>());
    rclcpp::shutdown();
    return 0;
}