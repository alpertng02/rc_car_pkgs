#!/usr/bin/env python3
"""Odometry drift evaluation against Gazebo ground truth.

Compares the EKF output (/odometry/filtered) with the p3d ground-truth pose
(/ground_truth). Both trajectories are re-expressed relative to their own
starting pose, so the constant world->odom offset drops out and what remains
is pure drift. Prints a report every few seconds; final report on Ctrl+C.

Usage: ros2 run ackermann_demo odom_drift_eval.py
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class Pose2D:
    def __init__(self, msg):
        p = msg.pose.pose
        self.x, self.y, self.yaw = p.position.x, p.position.y, yaw_of(p.orientation)

    def relative_to(self, origin):
        """This pose expressed in the frame of an origin pose."""
        dx, dy = self.x - origin.x, self.y - origin.y
        c, s = math.cos(-origin.yaw), math.sin(-origin.yaw)
        out = object.__new__(Pose2D)
        out.x, out.y = c * dx - s * dy, s * dx + c * dy
        out.yaw = wrap_pi(self.yaw - origin.yaw)
        return out


class DriftEval(Node):
    def __init__(self):
        super().__init__('odom_drift_eval')
        self.declare_parameter('report_period_sec', 5.0)
        self.truth = None        # latest ground truth
        self.truth_origin = None
        self.ekf_origin = None
        self.prev_truth = None
        self.distance = 0.0      # ground-truth distance traveled
        self.max_trans_err = 0.0
        self.max_yaw_err = 0.0
        self.last_trans_err = 0.0
        self.last_yaw_err = 0.0
        self.n_samples = 0

        self.create_subscription(Odometry, '/ground_truth', self.truth_cb, 20)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_cb, 20)
        period = self.get_parameter('report_period_sec').value
        self.create_timer(period, self.report)
        self.get_logger().info('Comparing /odometry/filtered against /ground_truth ...')

    def truth_cb(self, msg):
        pose = Pose2D(msg)
        if self.prev_truth is not None:
            self.distance += math.hypot(pose.x - self.prev_truth.x,
                                        pose.y - self.prev_truth.y)
        self.prev_truth = pose
        self.truth = pose

    def ekf_cb(self, msg):
        if self.truth is None:
            return
        ekf = Pose2D(msg)
        if self.ekf_origin is None:
            self.ekf_origin, self.truth_origin = ekf, self.truth
            return
        t = self.truth.relative_to(self.truth_origin)
        e = ekf.relative_to(self.ekf_origin)
        self.last_trans_err = math.hypot(t.x - e.x, t.y - e.y)
        self.last_yaw_err = abs(wrap_pi(t.yaw - e.yaw))
        self.max_trans_err = max(self.max_trans_err, self.last_trans_err)
        self.max_yaw_err = max(self.max_yaw_err, self.last_yaw_err)
        self.n_samples += 1

    def report(self):
        if self.n_samples == 0:
            self.get_logger().warn('No synchronized samples yet '
                                   '(are /ground_truth and /odometry/filtered alive?)')
            return
        per_m = self.last_trans_err / self.distance * 100.0 if self.distance > 0.05 else 0.0
        self.get_logger().info(
            f'dist={self.distance:6.2f} m | drift now={self.last_trans_err * 100:6.2f} cm '
            f'({per_m:4.1f}% of dist) max={self.max_trans_err * 100:6.2f} cm | '
            f'yaw err now={math.degrees(self.last_yaw_err):5.2f} deg '
            f'max={math.degrees(self.max_yaw_err):5.2f} deg | n={self.n_samples}')


def main():
    rclpy.init()
    node = DriftEval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.report()
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
