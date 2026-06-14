// =============================================================================
// Stanley Path Tracking Controller — ROS2 Humble with Displacement Safety
// =============================================================================

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

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
        this->declare_parameter("goal_yaw_tolerance", 0.35);   // accept the goal heading within ~20°
        this->declare_parameter("max_acceleration", 0.50);
        this->declare_parameter("max_deceleration", 0.90);
        this->declare_parameter("steer_speed_reduction", 0.40);
        this->declare_parameter("reverse_steer_speed_reduction", 0.40);  // speed cut per steer fraction while reversing
        this->declare_parameter("path_timeout_sec", 3.0);
        this->declare_parameter("ultrasonic_safety_dist", 0.35);

        // Cusp handoff: switch to the next motion segment only once the chassis
        // has PHYSICALLY stopped, gated on measured wheel speed (not the command
        // ramp), with a timeout fail-safe if odometry never reads a clean zero.
        this->declare_parameter("cusp_settle_speed", 0.03);    // measured wheel speed treated as standstill [m/s]
        this->declare_parameter("cusp_settle_timeout", 2.0);   // hand off anyway after this long at the cusp [s]

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
    // Split the path into maximal runs of one travel direction. Cusps (the
    // planner zeroes velocity there) and the endpoints are segment boundaries,
    // so a reverse-curve plan from the Hybrid A* RS expansion becomes a clean
    // sequence of forward/reverse segments the tracker can follow one at a time.
    void computeSegments(const nav_msgs::msg::Path& path) {
        segments_.clear(); seg_dir_.clear();
        const size_t N = path.poses.size();
        if (N == 0) return;
        std::vector<int> dir(N, 0);
        int last = 0;
        for (size_t i = 0; i < N; ++i) {
            const double vz = path.poses[i].pose.position.z;
            int s = (vz > 1e-6) ? 1 : (vz < -1e-6 ? -1 : 0);
            if (s != 0) last = s;
            dir[i] = (s != 0) ? s : last;     // zeros inherit the preceding direction
        }
        if (last == 0) { segments_.push_back({ 0, N - 1 }); seg_dir_.push_back(1); return; }
        if (dir[0] == 0) {                    // back-fill leading zeros (path starts at rest)
            int first = 1;
            for (size_t i = 0; i < N; ++i) if (dir[i] != 0) { first = dir[i]; break; }
            for (size_t i = 0; i < N && dir[i] == 0; ++i) dir[i] = first;
        }
        size_t start = 0;
        for (size_t i = 1; i < N; ++i) {
            if (dir[i] != dir[start]) {
                segments_.push_back({ start, i - 1 }); seg_dir_.push_back(dir[start]);
                start = i;
            }
        }
        segments_.push_back({ start, N - 1 }); seg_dir_.push_back(dir[start]);
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (msg->poses.empty()) {
            has_path_ = false;
            current_path_ = nullptr;
            current_speed_ = 0.0;
            segments_.clear(); seg_dir_.clear(); current_seg_ = 0;
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
        computeSegments(*msg);
        current_seg_ = 0;
        seg_end_settling_ = false;
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
        constexpr double BIAS_ALPHA = 0.002;  // ~5 s at 100 Hz
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
        const double rev_steer_reduce = this->get_parameter("reverse_steer_speed_reduction").as_double();
        const double path_timeout = this->get_parameter("path_timeout_sec").as_double();
        const double safety_dist = this->get_parameter("ultrasonic_safety_dist").as_double();
        const double cusp_settle_speed = this->get_parameter("cusp_settle_speed").as_double();
        const double cusp_settle_timeout = this->get_parameter("cusp_settle_timeout").as_double();

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

        const double half_wb = 0.5 * wheelbase;

        const auto& goal_pose = local_path->poses.back().pose;
        const double goal_x = goal_pose.position.x;
        const double goal_y = goal_pose.position.y;
        const double dist_to_goal = std::hypot(goal_x - robot_x, goal_y - robot_y);

        // Watchdog: stop on a stale plan — but exempt the final approach, where
        // the planner intentionally drops the keep-alive once we are within its
        // arrival tolerance while we still close to our tighter goal_tolerance.
        if (local_has_path) {
            const double age = (this->now() - last_path_time_).seconds();
            if (age > path_timeout && dist_to_goal > 0.40) {
                publishStop();
                return;
            }
        }

        // Goal reached once the car has actually SETTLED at the path end. Gate on
        // the MEASURED wheel speed, not just the command ramp: current_speed_
        // reaches ~0 before the chassis stops, so latching the goal on it alone
        // can drop the path while the car is still drifting, leaving it stopped
        // short or mis-aligned.
        const double goal_yaw = quaternionToYaw(goal_pose.orientation.x, goal_pose.orientation.y,
            goal_pose.orientation.z, goal_pose.orientation.w);
        const double goal_yaw_err = std::abs(normalizeAngle(goal_yaw - robot_yaw));
        double goal_measured_speed;
        {
            std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
            goal_measured_speed = std::abs(latest_wheel_speed_);
        }
        if (dist_to_goal < goal_tol && std::abs(current_speed_) < 0.05 && goal_measured_speed < 0.05) {
            RCLCPP_INFO(this->get_logger(), "🏁 Goal reached (heading err %.1f°). Halting.",
                goal_yaw_err * 180.0 / M_PI);
            publishStop();
            std::lock_guard<std::mutex> lock(path_mutex_);
            has_path_ = false;
            current_path_ = nullptr;
            segments_.clear(); seg_dir_.clear(); current_seg_ = 0;
            return;
        }

        // Segment-aware tracking: stay within the current motion segment
        if (segments_.empty()) { publishStop(); return; }
        current_seg_ = std::min(current_seg_, segments_.size() - 1);
        const size_t seg_s = segments_[current_seg_].first;
        const size_t seg_e = segments_[current_seg_].second;
        target_wp_idx_ = std::clamp(target_wp_idx_, seg_s, seg_e);
        const bool reversing = (seg_dir_[current_seg_] < 0);

        // Hybrid reference. The path is the REAR-AXLE trajectory (exact bicycle kinematics).
        // Forward uses front-axle Stanley for crisp heading. Reverse tracks the rear axle.
        double ref_x, ref_y, motion_yaw;
        if (!reversing) {
            ref_x = robot_x + half_wb * std::cos(robot_yaw);   // front axle = centre + L/2
            ref_y = robot_y + half_wb * std::sin(robot_yaw);
            motion_yaw = robot_yaw;
        } else {
            ref_x = robot_x - half_wb * std::cos(robot_yaw);   // rear axle = centre - L/2
            ref_y = robot_y - half_wb * std::sin(robot_yaw);
            motion_yaw = normalizeAngle(robot_yaw + M_PI);
        }

        auto track_pt = [&](size_t i, double& tx, double& ty) {
            const auto& wp = local_path->poses[i].pose;
            if (!reversing) {
                double th = quaternionToYaw(wp.orientation.x, wp.orientation.y, wp.orientation.z, wp.orientation.w);
                tx = wp.position.x + wheelbase * std::cos(th);
                ty = wp.position.y + wheelbase * std::sin(th);
            } else {
                tx = wp.position.x; ty = wp.position.y;
            }
        };

        const size_t search_start = (target_wp_idx_ > seg_s + 4) ? target_wp_idx_ - 4 : seg_s;
        const size_t search_end = std::min(seg_e, target_wp_idx_ + 30);

        double min_dist = std::numeric_limits<double>::max();
        size_t best_idx = target_wp_idx_;
        for (size_t i = search_start; i <= search_end; ++i) {
            double tx, ty; track_pt(i, tx, ty);
            const double d = std::hypot(tx - ref_x, ty - ref_y);
            if (d < min_dist) { min_dist = d; best_idx = i; }
        }
        target_wp_idx_ = best_idx;
        const double path_vel_raw = local_path->poses[target_wp_idx_].pose.position.z;

        const size_t next_idx = std::min(target_wp_idx_ + 1, seg_e);
        double cur_wx, cur_wy, nxt_wx, nxt_wy;
        track_pt(target_wp_idx_, cur_wx, cur_wy);
        track_pt(next_idx, nxt_wx, nxt_wy);

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
        const double steering_angle = std::clamp(raw_delta, -max_steer, max_steer);

        const double v_limit = reversing ? max_v_rev : max_v_fwd;
        double speed_target = std::clamp(path_vel_raw, -v_limit, v_limit);

        // =====================================================================
        // 1. KINEMATIC REDUCTIONS
        // =====================================================================

        // Steering speed reduction
        const double steer_ratio = std::abs(steering_angle) / max_steer;
        const double reduce = reversing ? rev_steer_reduce : steer_reduce;
        const double speed_scale = std::max(0.20, 1.0 - reduce * steer_ratio);
        speed_target *= speed_scale;

        // Cusp approach and stop
        const bool cusp_ahead = (current_seg_ + 1 < segments_.size());
        const bool at_seg_end = (target_wp_idx_ >= seg_e);

        if (cusp_ahead) {
            const auto& cusp_pose = local_path->poses[seg_e].pose;
            const double dist_to_cusp = std::hypot(cusp_pose.position.x - robot_x,
                cusp_pose.position.y - robot_y);

            // Kinematic braking curve: v = sqrt(2 * a * d)
            const double safe_approach_speed = std::sqrt(2.0 * (max_decel * 0.8) * std::max(0.0, dist_to_cusp));

            // Clamp the target speed to the braking curve, preserving travel direction (sign)
            if (std::abs(speed_target) > safe_approach_speed) {
                speed_target = (speed_target > 0.0) ? safe_approach_speed : -safe_approach_speed;
            }

            // Hard stop precisely at the end of the segment
            if (at_seg_end) speed_target = 0.0;
        }

        // =====================================================================
        // 2. SENSOR & STATE GATHERING
        // =====================================================================

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

        bool plan_grace_active = (this->now() - path_receive_time_).seconds() < 1.0;

        // =====================================================================
        // 3. STALL DETECTION
        // =====================================================================

        // We now check against the INTENDED kinematic speed, avoiding false flags
        // when the car is intentionally creeping through a curve or braking for a cusp.
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
                    stall_cause = ultrasonic_block ? "held by ultrasonic obstacle" : "drivetrain not turning";
                } else if (traveled_distance > stall_disp_thresh) {
                    stall_start_x_ = robot_x;
                    stall_start_y_ = robot_y;
                    stall_start_time_ = this->now();
                } else if (elapsed_seconds > stall_timeout) {
                    if (accel_activity < stall_accel_thresh) {
                        stall_cause = (wheel_speed >= stall_odom_thresh) ? "wheels slipping in place" : "no displacement";
                    } else {
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
                    RCLCPP_ERROR(this->get_logger(), "STALL (%s) while commanding %s motion. Requesting escape replan.",
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

        // =====================================================================
        // 4. ULTRASONIC OVERRIDE & EXECUTION
        // =====================================================================

        // This runs after the stall detector. If we are blocked while trying to move fast,
        // the stall timer will keep ticking in the background until it triggers an escape.
        if (ultrasonic_block) {
            speed_target = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Ultrasonic obstacle at %.2f m (limit %.2f m). Holding the car stopped.",
                current_range, safety_dist);
        }

        const double dv_desired = speed_target - current_speed_;
        const bool decelerating = (current_speed_ != 0.0 && dv_desired * current_speed_ < 0.0)
            || (std::abs(speed_target) < std::abs(current_speed_));
        const double rate = decelerating ? max_decel : max_accel;
        const double dv_cap = rate * dt;
        current_speed_ += std::clamp(dv_desired, -dv_cap, dv_cap);

        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_footprint";
        cmd.twist.linear.x = current_speed_;

        // Explicitly invert steering for reverse.
        // Because our correct kinematic yaw-rate conversion cancels out the velocity sign 
        // inside the Ackermann controller, we no longer get a "free" sign flip.
        const double active_steer = reversing ? -steering_angle : steering_angle;

        // Convert steering angle to proper yaw rate for AckermannSteeringController
        // Prevents division-by-zero wheel slams at extremely low approach speeds
        if (std::abs(current_speed_) > 0.01) {
            cmd.twist.angular.z = (current_speed_ * std::tan(active_steer)) / wheelbase;
        } else {
            cmd.twist.angular.z = 0.0; // Hold steering steady when practically stopped
        }

        cmd_pub_->publish(cmd);

        // Cusp handoff: switch to the next forward/reverse segment only once the
        // car has PHYSICALLY come to rest. current_speed_ is the internal command
        // ramp and reaches ~0 a beat before the chassis actually stops; handing
        // off on it alone flipped the steering to the next segment's direction
        // while the car was still rolling through the cusp, throwing it off path.
        // We wait for the measured wheel speed to confirm standstill, with a
        // timeout fail-safe in case odometry never reports a clean zero.
        if (cusp_ahead && at_seg_end && std::abs(current_speed_) < 0.001) {
            if (!seg_end_settling_) {
                seg_end_settling_ = true;
                seg_end_stop_time_ = this->now();
            }
            const double settle_elapsed = (this->now() - seg_end_stop_time_).seconds();
            if (wheel_speed < cusp_settle_speed || settle_elapsed > cusp_settle_timeout) {
                current_seg_++;
                target_wp_idx_ = segments_[current_seg_].first;
                path_receive_time_ = this->now();   // reset the grace window for the new segment
                seg_end_settling_ = false;
            }
        } else {
            seg_end_settling_ = false;
        }
    }

    size_t        target_wp_idx_;
    bool          has_path_;
    double        current_speed_;
    rclcpp::Time  last_path_time_;
    rclcpp::Time  path_receive_time_;

    std::mutex                         path_mutex_;
    nav_msgs::msg::Path::SharedPtr     current_path_;
    std::vector<std::pair<size_t, size_t>> segments_;   // [start,end] index of each motion segment
    std::vector<int>                   seg_dir_;         // +1 forward / -1 reverse per segment
    size_t                             current_seg_ = 0; // segment currently being tracked
    bool                               seg_end_settling_ = false; // waiting for a physical stop at a cusp
    rclcpp::Time                       seg_end_stop_time_;        // when the command ramp first hit zero at the cusp

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