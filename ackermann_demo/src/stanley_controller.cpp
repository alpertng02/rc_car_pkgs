// =============================================================================
// Stanley Path Tracking Controller — ROS2 Humble
// =============================================================================
//
// Subscribes:
//   /followed_path  (nav_msgs/msg/Path)
//       pose.position.z  → target velocity  [m/s], sign encodes direction:
//                          positive = forward, negative = reverse
//       pose.orientation → vehicle heading quaternion at each waypoint
//
// Publishes:
//   /cmd_vel_stanley  (geometry_msgs/msg/TwistStamped)
//       twist.linear.x  → longitudinal velocity command [m/s]
//       twist.angular.z → steering angle command [rad]
//                         positive = left / CCW  (ROS convention)
//
//   NOTE — Differential-drive conversion:
//     If your actuator driver expects a yaw rate instead of a steering angle:
//       omega = (linear.x * tan(angular.z)) / wheelbase
//
// Algorithm:
//   δ(t) = θ_e(t) + arctan( k · e_fa(t) / (v(t) + k_soft) )
//
//   θ_e  = path tangent angle − vehicle heading  (heading error)
//   e_fa = signed lateral distance of the reference axle from the path
//          positive → axle is to the RIGHT of the path direction
//   k    = cross-track gain  (stanley_k parameter)
//   k_soft prevents division-by-zero at low / zero velocity
//
// Reverse handling:
//   Reference point switches from front axle → rear axle.
//   Effective heading = robot_yaw + π  (motion is backwards).
//   Final steering output is negated  (rear-axle tracking mirrors correction).
//
// =============================================================================

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

// =============================================================================
class StanleyControllerNode : public rclcpp::Node {
public:
    StanleyControllerNode()
        : Node("stanley_controller"),
        target_wp_idx_(0),
        has_path_(false),
        current_speed_(0.0) {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // -----------------------------------------------------------------
        // Parameter declarations — all tunable at runtime via ros2 param set
        // -----------------------------------------------------------------

        // Vehicle geometry
        this->declare_parameter("wheelbase", 0.26);  // rear-to-front axle [m]

        // Stanley gains
        this->declare_parameter("stanley_k", 1.5);   // cross-track gain
        this->declare_parameter("stanley_k_soft", 0.5);   // softening constant [m/s]

        // Steering limits
        this->declare_parameter("max_steering_angle", 0.75);  // [rad]

        // Speed limits
        this->declare_parameter("max_linear_velocity", 0.50);  // forward  [m/s]
        this->declare_parameter("max_reverse_velocity", 0.30);  // reverse  [m/s]

        // Stopping
        this->declare_parameter("goal_tolerance", 0.25);  // [m]

        // Dynamics
        this->declare_parameter("max_acceleration", 0.40);  // [m/s²]
        this->declare_parameter("max_deceleration", 0.80);  // [m/s²]

        // Cornering speed reduction: speed *= max(0.2, 1 − factor × |δ|/δ_max)
        this->declare_parameter("steer_speed_reduction", 0.50);  // [0–1]

        // Safety: halt if no fresh path received within this window
        this->declare_parameter("path_timeout_sec", 30.0);   // [s]

        // -----------------------------------------------------------------
        // ROS2 interfaces
        // -----------------------------------------------------------------
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/followed_path", 10,
            std::bind(&StanleyControllerNode::pathCallback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel_stanley", 10);

        // 20 Hz control loop (50 ms period)
        control_timer_ = this->create_wall_timer(
            50ms, std::bind(&StanleyControllerNode::controlLoop, this));

        last_path_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "🏎️  Stanley Controller Online. "
            "Awaiting path on /followed_path...");
    }

private:
    // =========================================================================
    // PATH SUBSCRIPTION CALLBACK
    // =========================================================================
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(path_mutex_);

        if (msg->poses.empty()) {
            has_path_ = false;
            current_path_ = nullptr;
            current_speed_ = 0.0;
            RCLCPP_INFO(this->get_logger(),
                "📭 Empty path received — halting controller.");
            return;
        }

        current_path_ = msg;
        target_wp_idx_ = 0;
        current_speed_ = 0.0;          // reset ramp on every new plan
        has_path_ = true;
        last_path_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "📬 New path: %zu waypoints. Starting Stanley tracking.",
            msg->poses.size());
    }

    // =========================================================================
    // UTILITY HELPERS
    // =========================================================================

    // Wrap angle into (−π, +π]
    inline double normalizeAngle(double a) const {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    // Extract 2-D yaw from a quaternion (z, w sufficient in planar nav)
    inline double quaternionToYaw(double qx, double qy,
        double qz, double qw) const {
        return std::atan2(2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz));
    }

    // Publish a zero-velocity command and reset integrator
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
    // MAIN CONTROL LOOP  —  20 Hz
    // =========================================================================
    void controlLoop() {

        // ---- Snapshot all live parameters (supports runtime tuning) ----------
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

        // ---- Snapshot current path under lock --------------------------------
        nav_msgs::msg::Path::SharedPtr local_path;
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            local_path = current_path_;
        }

        // ---- Path freshness guard --------------------------------------------
        if (has_path_) {
            const double age = (this->now() - last_path_time_).seconds();
            if (age > path_timeout) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "⚠️  Path data stale (%.1fs > %.1fs). Halting for safety.",
                    age, path_timeout);
                publishStop();
                return;
            }
        }

        // ---- Robot pose via TF2  (map ← base_footprint) ---------------------
        double robot_x, robot_y, robot_yaw;
        try {
            auto tf = tf_buffer_->lookupTransform(
                "map", "base_footprint", tf2::TimePointZero);
            robot_x = tf.transform.translation.x;
            robot_y = tf.transform.translation.y;
            robot_yaw = quaternionToYaw(
                tf.transform.rotation.x, tf.transform.rotation.y,
                tf.transform.rotation.z, tf.transform.rotation.w);
        } catch (const tf2::TransformException&) {
            return;  // TF not yet available — silent wait
        }

        // ---- Guard: no active path -------------------------------------------
        if (!has_path_ || !local_path || local_path->poses.empty()) {
            publishStop();
            return;
        }

        const size_t N = local_path->poses.size();

        // ---- Goal-reached check ---------------------------------------------
        const double goal_x = local_path->poses.back().pose.position.x;
        const double goal_y = local_path->poses.back().pose.position.y;
        if (std::hypot(goal_x - robot_x, goal_y - robot_y) < goal_tol) {
            RCLCPP_INFO(this->get_logger(),
                "🏁 Goal reached. Halting.");
            publishStop();
            std::lock_guard<std::mutex> lock(path_mutex_);
            has_path_ = false;
            current_path_ = nullptr;
            return;
        }

        // ---- Clamp waypoint index to valid range -----------------------------
        target_wp_idx_ = std::min(target_wp_idx_, N - 1);

        // ---- Motion direction from velocity stored in path -------------------
        //  The Hybrid A* planner encodes its computed velocity profile in
        //  pose.position.z:  positive → forward segment,  negative → reverse
        const double path_vel_raw = local_path->poses[target_wp_idx_].pose.position.z;
        const bool   reversing = (path_vel_raw < 0.0);

        // ====================================================================
        // REFERENCE AXLE POSITION
        //   Forward  → FRONT axle  (wheelbase ahead along robot heading)
        //              Classic Stanley lateral tracking reference
        //   Reverse  → REAR  axle  (half-wheelbase behind robot heading)
        //              Motion direction flipped to robot_yaw + π
        // ====================================================================
        double ref_x, ref_y, motion_yaw;
        if (!reversing) {
            ref_x = robot_x + wheelbase * std::cos(robot_yaw);
            ref_y = robot_y + wheelbase * std::sin(robot_yaw);
            motion_yaw = robot_yaw;
        } else {
            ref_x = robot_x - (wheelbase * 0.5) * std::cos(robot_yaw);
            ref_y = robot_y - (wheelbase * 0.5) * std::sin(robot_yaw);
            motion_yaw = normalizeAngle(robot_yaw + M_PI);
        }

        // ====================================================================
        // PROGRESSIVE WAYPOINT SEARCH
        //   Search a bounded window forward from the current index to avoid
        //   snapping back to a previously visited waypoint.
        // ====================================================================
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

        // ====================================================================
        // PATH TANGENT AT CLOSEST WAYPOINT
        //   Derived geometrically from consecutive waypoint positions —
        //   more robust than relying solely on stored quaternions.
        //   Falls back to stored orientation only at the final waypoint.
        //
        //   For reverse segments the planner expands via:
        //     next = current + (−step) · (cos θ, sin θ)
        //   so atan2(next − current) already gives θ + π  (motion direction).
        // ====================================================================
        const size_t next_idx = std::min(target_wp_idx_ + 1, N - 1);

        const double cur_wx = local_path->poses[target_wp_idx_].pose.position.x;
        const double cur_wy = local_path->poses[target_wp_idx_].pose.position.y;
        const double nxt_wx = local_path->poses[next_idx].pose.position.x;
        const double nxt_wy = local_path->poses[next_idx].pose.position.y;

        double path_tangent;
        if (std::hypot(nxt_wx - cur_wx, nxt_wy - cur_wy) > 1e-5) {
            // Geometric tangent — direction of motion along the path segment
            path_tangent = std::atan2(nxt_wy - cur_wy, nxt_wx - cur_wx);
        } else {
            // Final waypoint: fall back to stored heading
            path_tangent = quaternionToYaw(
                local_path->poses[target_wp_idx_].pose.orientation.x,
                local_path->poses[target_wp_idx_].pose.orientation.y,
                local_path->poses[target_wp_idx_].pose.orientation.z,
                local_path->poses[target_wp_idx_].pose.orientation.w);
            if (reversing) path_tangent = normalizeAngle(path_tangent + M_PI);
        }

        // ====================================================================
        // HEADING ERROR  θ_e
        //   Difference between path tangent direction and the vehicle's
        //   effective motion heading.  Normalised to (−π, +π].
        // ====================================================================
        const double heading_error = normalizeAngle(path_tangent - motion_yaw);

        // ====================================================================
        // CROSS-TRACK ERROR  e_fa
        //   Signed lateral displacement of the reference axle from the path.
        //   Computed as the dot product of the error vector with the
        //   right-hand (clockwise) normal of the path tangent:
        //     right_normal = ( sin(path_tangent), −cos(path_tangent) )
        //
        //   Sign convention (matching Stanley formula below):
        //     e_fa > 0  →  axle to the RIGHT of path  →  steer LEFT  (δ > 0)
        //     e_fa < 0  →  axle to the LEFT  of path  →  steer RIGHT (δ < 0)
        // ====================================================================
        const double err_x = ref_x - cur_wx;
        const double err_y = ref_y - cur_wy;
        const double cte = std::sin(path_tangent) * err_x
            - std::cos(path_tangent) * err_y;

        // ====================================================================
        // STANLEY STEERING LAW
        //   δ = θ_e + arctan( k · e_fa / (|v| + k_soft) )
        //
        //   k_soft prevents division-by-zero at standstill and limits the
        //   correction magnitude at very low speeds.
        // ====================================================================
        const double v_eff = std::max(std::abs(path_vel_raw), stanley_k_soft);
        const double cte_term = std::atan2(stanley_k * cte, v_eff);
        const double raw_delta = heading_error + cte_term;
        double steering_angle = std::clamp(raw_delta, -max_steer, max_steer);

        // Reverse motion: invert correction because the rear axle is the
        // tracking reference and its steering geometry is mirrored.
        if (reversing) steering_angle = -steering_angle;

        // ====================================================================
        // SPEED COMMAND
        //   1. Clamp velocity profile to configured limits.
        //   2. Proportional reduction when steering hard (cornering stability).
        //   3. Apply asymmetric acceleration / deceleration ramp.
        // ====================================================================
        const double v_limit = reversing ? max_v_rev : max_v_fwd;
        double speed_target = std::clamp(path_vel_raw, -v_limit, v_limit);

        // Cornering speed reduction
        const double steer_ratio = std::abs(steering_angle) / max_steer;   // [0, 1]
        const double speed_scale = std::max(0.20, 1.0 - steer_reduce * steer_ratio);
        speed_target *= speed_scale;

        // Acceleration / deceleration ramp  (dt = 50 ms)
        constexpr double dt = 0.05;
        const double dv_desired = speed_target - current_speed_;

        // Use the stricter deceleration limit when slowing or reversing polarity
        const bool decelerating = (dv_desired * current_speed_ <= 0.0) ||
            (std::abs(speed_target) < std::abs(current_speed_));
        const double rate = decelerating ? max_decel : max_accel;
        const double dv_cap = rate * dt;
        current_speed_ += std::clamp(dv_desired, -dv_cap, dv_cap);

        // ====================================================================
        // PUBLISH
        // ====================================================================
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_footprint";
        cmd.twist.linear.x = current_speed_;
        cmd.twist.angular.z = steering_angle;
        cmd_pub_->publish(cmd);

        RCLCPP_DEBUG(this->get_logger(),
            "WP[%zu/%zu] | CTE: %+.3fm | θ_e: %+.1f° | δ: %+.1f° | v: %+.2fm/s%s",
            target_wp_idx_, N,
            cte,
            heading_error * 180.0 / M_PI,
            steering_angle * 180.0 / M_PI,
            current_speed_,
            reversing ? " [REV]" : "");
    }

    // =========================================================================
    // MEMBER VARIABLES
    // =========================================================================

    // Controller state
    size_t        target_wp_idx_;
    bool          has_path_;
    double        current_speed_;     // last commanded speed [m/s] for ramp
    rclcpp::Time  last_path_time_;

    // Thread-safe path storage
    std::mutex                         path_mutex_;
    nav_msgs::msg::Path::SharedPtr     current_path_;

    // TF2
    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr          path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr                                   control_timer_;
};

// =============================================================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StanleyControllerNode>());
    rclcpp::shutdown();
    return 0;
}