// =============================================================================
// Voronoi Field Costmap — Dolgov, Thrun, Montemerlo & Diebel (2008), §"Voronoi
// field". Publishes the generalized-Voronoi potential field that the paper uses
// to guide a car through narrow passages (the term the Hybrid A* smoother in
// ackermann_astar_planner.cpp consumes via /voronoi_field).
//
//   ρ_V(x,y) = ( α / (α + d_O) ) · ( d_V / (d_O + d_V) ) · ( (d_O - d_O^max)² / (d_O^max)² )
//
// for d_O ≤ d_O^max, and 0 beyond. d_O = distance to the nearest obstacle,
// d_V = distance to the nearest edge of the generalized Voronoi diagram (GVD).
// ρ_V ∈ [0,1]: 1 against obstacle boundaries, 0 on the GVD ridges down the
// centre of free corridors, so descending ρ_V pushes the car toward gap centres.
//
// Pipeline mirrors costmap_node.cpp: subscribe /map, publish /voronoi_field as
// an OccupancyGrid (0 = ridge / out of range, 100 = obstacle, -1 = unknown).
//
// Implementation: OpenCV distance transforms instead of a hand-rolled Dijkstra
// brushfire. cv::distanceTransformWithLabels computes d_O *and* the nearest
// obstacle's connected-component id in one O(N) pass, so the GVD is simply where
// adjacent free cells are nearest to *different* obstacle blobs (Lau, Sprunk &
// Burgard, IROS 2010). A second cv::distanceTransform yields d_V. The whole
// thing is throttled to min_recompute_period so it does not burn Jetson CPU on
// every SLAM map tick.
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class VoronoiCostmapNode : public rclcpp::Node {
public:
    VoronoiCostmapNode() : Node("voronoi_costmap") {
        this->declare_parameter("alpha",                     0.3);   // field falloff rate [m]
        this->declare_parameter("d_o_max",                   0.7);   // field range from obstacles [m]
        this->declare_parameter("occupied_threshold",        65);    // cell value treated as obstacle
        this->declare_parameter("min_clearance",             0.20);  // suppress GVD in gaps tighter than this [m]
        this->declare_parameter("treat_unknown_as_obstacle", false);
        this->declare_parameter("border_is_obstacle",        false); // online SLAM: edge is frontier, not wall
        this->declare_parameter("min_recompute_period",      1.0);   // throttle heavy recompute [s]

        double period = std::max(0.05, this->get_parameter("min_recompute_period").as_double());

        // Latched, reliable QoS so RViz and late-joining consumers receive the
        // most recent field (the SLAM /map is published transient-local too).
        auto latched = rclcpp::QoS(1).reliable().transient_local();
        field_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/voronoi_field", latched);
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", latched, std::bind(&VoronoiCostmapNode::mapCallback, this, std::placeholders::_1));

        // Recompute on a throttle off the latest map rather than per /map message.
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(period), std::bind(&VoronoiCostmapNode::onTimer, this));

        RCLCPP_INFO(this->get_logger(),
            "Voronoi field node online (OpenCV DT, %.2fs throttle) -> /voronoi_field", period);
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latest_map_ = msg;   // single-threaded executor: no lock needed
        dirty_ = true;
    }

    void onTimer() {
        if (!dirty_ || !latest_map_) return;
        auto map = latest_map_;
        dirty_ = false;
        compute(*map);
    }

    void compute(const nav_msgs::msg::OccupancyGrid& map) {
        const int w = static_cast<int>(map.info.width), h = static_cast<int>(map.info.height);
        if (w <= 0 || h <= 0 || static_cast<int>(map.data.size()) != w * h) return;
        const double res = map.info.resolution;
        if (res <= 0.0) return;

        const double alpha    = this->get_parameter("alpha").as_double();
        const double d_o_max  = this->get_parameter("d_o_max").as_double();
        const int    occ_th   = static_cast<int>(this->get_parameter("occupied_threshold").as_int());
        const double min_clr  = this->get_parameter("min_clearance").as_double();
        const bool   unk_obs  = this->get_parameter("treat_unknown_as_obstacle").as_bool();
        const bool   bord_obs = this->get_parameter("border_is_obstacle").as_bool();

        // 1. obstacle mask: obstacle -> 0, free -> 255 (DT measures distance to 0)
        cv::Mat obs(h, w, CV_8U);
        uchar* op = obs.data;
        for (int i = 0; i < w * h; ++i) {
            int8_t v = map.data[i];
            bool occ = (v >= occ_th) || (unk_obs && v < 0);
            op[i] = occ ? 0 : 255;
        }
        if (bord_obs) {
            for (int x = 0; x < w; ++x) { op[x] = 0; op[x + (h - 1) * w] = 0; }
            for (int y = 0; y < h; ++y) { op[y * w] = 0; op[(w - 1) + y * w] = 0; }
        }

        nav_msgs::msg::OccupancyGrid out;
        out.header = map.header;
        out.header.stamp = this->now();
        out.info = map.info;
        out.data.assign(static_cast<size_t>(w) * h, 0);

        // No obstacles at all -> field is uniformly 0 (preserve unknown as -1).
        if (cv::countNonZero(obs) == w * h) {
            for (int i = 0; i < w * h; ++i) if (map.data[i] < 0) out.data[i] = -1;
            field_pub_->publish(out);
            return;
        }

        // 2. d_O + nearest-obstacle connected-component label in one pass.
        //    Labelled DT only supports the 3x3/5x5 chamfer mask (no PRECISE);
        //    the ~1-2% error is irrelevant for a guidance gradient.
        cv::Mat distO_px, labels;
        cv::distanceTransform(obs, distO_px, labels,
                              cv::DIST_L2, cv::DIST_MASK_5, cv::DIST_LABEL_CCOMP);

        // 3. GVD ridge mask (ridge -> 0, else -> 255): a free cell whose 8-neighbour
        //    is nearest to a *different* obstacle blob, with enough clearance that
        //    the gap is actually drivable (d_O at a ridge ≈ half the gap width).
        cv::Mat ridge(h, w, CV_8U, cv::Scalar(255));
        uchar* rp = ridge.data;
        const float* dOp = distO_px.ptr<float>();
        const int*   lbp = labels.ptr<int>();
        const float min_clr_px = static_cast<float>(min_clr / res);
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                int i = x + y * w;
                if (op[i] == 0) continue;                 // obstacle
                if (dOp[i] < min_clr_px) continue;        // gap too tight for the car
                int lab = lbp[i];
                bool edge = false;
                for (int dy = -1; dy <= 1 && !edge; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (!dx && !dy) continue;
                        int nx = x + dx, ny = y + dy;
                        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                        int n = nx + ny * w;
                        if (op[n] == 0) continue;          // neighbour is obstacle
                        if (lbp[n] != lab) { edge = true; break; }
                    }
                }
                if (edge) rp[i] = 0;
            }
        }

        // 4. d_V: exact distance to the nearest ridge cell (no labels -> PRECISE ok).
        cv::Mat distV_px;
        bool any_ridge = cv::countNonZero(ridge) < w * h;
        if (any_ridge)
            cv::distanceTransform(ridge, distV_px, cv::DIST_L2, cv::DIST_MASK_PRECISE);

        // 5. assemble ρ_V and publish ----------------------------------------
        const double dmax2 = d_o_max * d_o_max;
        const double dV_fallback = d_o_max * 4.0;        // open space: no GVD nearby
        const float* dVp = any_ridge ? distV_px.ptr<float>() : nullptr;
        for (int i = 0; i < w * h; ++i) {
            if (op[i] == 0) { out.data[i] = 100; continue; }
            if (map.data[i] < 0) { out.data[i] = -1; continue; }   // keep unknown unknown
            double dO = dOp[i] * res;
            if (dO > d_o_max) { out.data[i] = 0; continue; }

            double dV = dVp ? (dVp[i] * res) : dV_fallback;
            double diff = dO - d_o_max;
            double rho = (alpha / (alpha + dO)) * (dV / (dO + dV)) * (diff * diff / dmax2);
            rho = std::clamp(rho, 0.0, 1.0);
            out.data[i] = static_cast<int8_t>(std::lround(rho * 100.0));
        }

        field_pub_->publish(out);
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr                       latest_map_;
    bool                                                         dirty_ = false;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr    field_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr                                  timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoronoiCostmapNode>());
    rclcpp::shutdown();
    return 0;
}
