#!/usr/bin/env python3
"""Autonomous frontier exploration for hands-off planner/controller testing.

Drives the car around on its own so the Hybrid A* planner and the Stanley
controller can be exercised end to end without any operator input. It scans the
slam_toolbox occupancy map for frontiers (free cells bordering unknown space),
publishes the best one as a goal on /goal_pose, and moves on once the goal is
reached, times out, or proves unreachable. When no reachable frontiers remain,
it drives back to the pose it started from.

Inputs : /map (nav_msgs/OccupancyGrid), /inflated_costmap (optional, for goal
         clearance), TF map -> base_footprint.
Output : /goal_pose (geometry_msgs/PoseStamped) -- the same goal the planner
         already accepts from RViz.

Usage: ros2 launch ackermann_demo sim.launch.py use_nav:=true use_slam:=true use_search:=true
       (or: ros2 run ackermann_demo exploration_node.py)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def latched_qos():
    # Matches costmap_node / slam_toolbox: reliable + transient-local so a
    # late-joining subscriber still receives the most recent map.
    return QoSProfile(depth=1,
                      history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('exploration_node')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('costmap_topic', '/inflated_costmap')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('tick_period_sec', 1.0)
        self.declare_parameter('goal_reached_dist', 0.35)   # [m] count a goal as reached
        self.declare_parameter('goal_timeout_sec', 45.0)    # [s] hard cap per goal
        self.declare_parameter('stuck_timeout_sec', 15.0)   # [s] abandon if no progress
        self.declare_parameter('progress_eps', 0.10)        # [m] improvement that counts as progress
        self.declare_parameter('min_frontier_cells', 6)     # ignore tiny frontier specks
        self.declare_parameter('min_travel_dist', 0.50)     # [m] don't bother with goals already underfoot
        self.declare_parameter('blacklist_radius', 0.60)    # [m] keep clear of abandoned goals
        self.declare_parameter('free_threshold', 20)        # cell <= this (and >= 0) counts as free
        self.declare_parameter('lethal_threshold', 85)      # costmap cell >= this is non-traversable
        self.declare_parameter('return_to_start', True)
        self.declare_parameter('no_frontier_confirmations', 3)  # ticks of "nothing left" before returning

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.goal_reached_dist = self.get_parameter('goal_reached_dist').value
        self.goal_timeout = self.get_parameter('goal_timeout_sec').value
        self.stuck_timeout = self.get_parameter('stuck_timeout_sec').value
        self.progress_eps = self.get_parameter('progress_eps').value
        self.min_frontier_cells = self.get_parameter('min_frontier_cells').value
        self.min_travel_dist = self.get_parameter('min_travel_dist').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.free_thresh = self.get_parameter('free_threshold').value
        self.lethal_thresh = self.get_parameter('lethal_threshold').value
        self.return_to_start = self.get_parameter('return_to_start').value
        self.no_frontier_confirm = self.get_parameter('no_frontier_confirmations').value

        self.map = None
        self.costmap = None
        self.start_pose = None       # (x, y, yaw) captured on first valid TF
        self.state = 'INIT'          # INIT -> EXPLORE -> RETURN -> DONE
        self.active_goal = None      # (x, y, yaw)
        self.goal_sent_time = None
        self.goal_best_dist = math.inf
        self.goal_progress_time = None
        self.blacklist = []          # [(x, y), ...] abandoned/unreachable goals
        self.no_frontier_count = 0
        self.goals_sent = 0

        self.create_subscription(OccupancyGrid,
                                 self.get_parameter('map_topic').value,
                                 self.map_cb, latched_qos())
        self.create_subscription(OccupancyGrid,
                                 self.get_parameter('costmap_topic').value,
                                 self.costmap_cb, latched_qos())
        self.goal_pub = self.create_publisher(
            PoseStamped, self.get_parameter('goal_topic').value, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(self.get_parameter('tick_period_sec').value, self.tick)
        self.get_logger().info('Frontier explorer started (waiting for map + TF)...')

    # ── inputs ───────────────────────────────────────────────────────────────
    def map_cb(self, msg):
        self.map = msg

    def costmap_cb(self, msg):
        self.costmap = msg

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return None
        tr = t.transform.translation
        return (tr.x, tr.y, yaw_of(t.transform.rotation))

    # ── goal handling ─────────────────────────────────────────────────────────
    def send_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self.goal_pub.publish(msg)
        self.active_goal = (x, y, yaw)
        self.goal_sent_time = self.get_clock().now()
        self.goal_progress_time = self.goal_sent_time
        self.goal_best_dist = math.inf
        self.goals_sent += 1

    def is_blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < self.blacklist_radius
                   for bx, by in self.blacklist)

    def elapsed(self, since):
        return (self.get_clock().now() - since).nanoseconds * 1e-9

    # ── main loop ──────────────────────────────────────────────────────────────
    def tick(self):
        pose = self.robot_pose()
        if pose is None:
            return
        if self.start_pose is None:
            self.start_pose = pose
            self.get_logger().info(
                f'Recorded start pose ({pose[0]:.2f}, {pose[1]:.2f}, '
                f'{math.degrees(pose[2]):.0f} deg).')
        if self.map is None:
            return
        if self.state == 'INIT':
            self.state = 'EXPLORE'

        if self.state == 'EXPLORE':
            self.explore_tick(pose)
        elif self.state == 'RETURN':
            self.return_tick(pose)
        # DONE: idle.

    def goal_finished(self, pose):
        """Return True once the active goal is reached, stalled, or timed out
        (and clear it). False while it should keep being pursued."""
        if self.active_goal is None:
            return True
        gx, gy, _ = self.active_goal
        d = math.hypot(gx - pose[0], gy - pose[1])
        if d < self.goal_reached_dist:
            self.get_logger().info(f'Reached goal ({gx:.2f}, {gy:.2f}).')
            self.active_goal = None
            return True
        if d < self.goal_best_dist - self.progress_eps:
            self.goal_best_dist = d
            self.goal_progress_time = self.get_clock().now()
        no_progress = self.elapsed(self.goal_progress_time) > self.stuck_timeout
        timed_out = self.elapsed(self.goal_sent_time) > self.goal_timeout
        if no_progress or timed_out:
            why = 'no progress' if no_progress else 'timeout'
            self.get_logger().warn(
                f'Abandoning goal ({gx:.2f}, {gy:.2f}) [{why}] — blacklisting.')
            self.blacklist.append((gx, gy))
            self.active_goal = None
            return True
        return False

    def explore_tick(self, pose):
        if not self.goal_finished(pose):
            return
        target = self.select_frontier(pose)
        if target is None:
            self.no_frontier_count += 1
            self.get_logger().info(
                f'No reachable frontier '
                f'({self.no_frontier_count}/{self.no_frontier_confirm}).')
            if self.no_frontier_count >= self.no_frontier_confirm:
                if self.return_to_start:
                    self.get_logger().info('Map explored — returning to start.')
                    self.state = 'RETURN'
                else:
                    self.get_logger().info('Map explored — done.')
                    self.state = 'DONE'
            return
        self.no_frontier_count = 0
        gx, gy = target
        yaw = math.atan2(gy - pose[1], gx - pose[0])
        self.send_goal(gx, gy, yaw)
        self.get_logger().info(
            f'Goal #{self.goals_sent} -> frontier ({gx:.2f}, {gy:.2f}).')

    def return_tick(self, pose):
        sx, sy, syaw = self.start_pose
        if self.active_goal is None:
            self.send_goal(sx, sy, syaw)
            self.get_logger().info(f'Returning to start ({sx:.2f}, {sy:.2f}).')
            return
        if math.hypot(sx - pose[0], sy - pose[1]) < self.goal_reached_dist:
            self.get_logger().info(
                f'Back at start. Exploration complete after '
                f'{self.goals_sent} goals.')
            self.state = 'DONE'
            self.active_goal = None
        elif self.elapsed(self.goal_sent_time) > self.goal_timeout:
            self.get_logger().warn('Return goal timed out — resending.')
            self.active_goal = None

    # ── frontier detection ─────────────────────────────────────────────────────
    def find_frontiers(self):
        """Cluster free cells that border unknown space; return a list of
        (world_x, world_y, cluster_size)."""
        m = self.map
        w, h = m.info.width, m.info.height
        if w * h != len(m.data):
            return []
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        grid = np.asarray(m.data, dtype=np.int16).reshape(h, w)

        unknown = grid < 0
        free = (grid >= 0) & (grid <= self.free_thresh)

        # a free cell is a frontier if any 4-neighbour is unknown
        unknown_nbr = np.zeros_like(unknown)
        unknown_nbr[:-1, :] |= unknown[1:, :]
        unknown_nbr[1:, :] |= unknown[:-1, :]
        unknown_nbr[:, :-1] |= unknown[:, 1:]
        unknown_nbr[:, 1:] |= unknown[:, :-1]
        frontier = free & unknown_nbr

        ys, xs = np.nonzero(frontier)
        cells = set(zip(xs.tolist(), ys.tolist()))
        clusters = []
        visited = set()
        for seed in cells:
            if seed in visited:
                continue
            stack = [seed]
            visited.add(seed)
            comp = []
            while stack:
                cx, cy = stack.pop()
                comp.append((cx, cy))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        nb = (cx + dx, cy + dy)
                        if nb in cells and nb not in visited:
                            visited.add(nb)
                            stack.append(nb)
            if len(comp) < self.min_frontier_cells:
                continue
            mx = sum(p[0] for p in comp) / len(comp)
            my = sum(p[1] for p in comp) / len(comp)
            # representative = the member cell nearest the centroid (guaranteed free)
            rep = min(comp, key=lambda p: (p[0] - mx) ** 2 + (p[1] - my) ** 2)
            wx = ox + (rep[0] + 0.5) * res
            wy = oy + (rep[1] + 0.5) * res
            clusters.append((wx, wy, len(comp)))
        return clusters

    def costmap_is_blocked(self, x, y):
        cm = self.costmap
        if cm is None:
            return False
        cx = int((x - cm.info.origin.position.x) / cm.info.resolution)
        cy = int((y - cm.info.origin.position.y) / cm.info.resolution)
        if cx < 0 or cx >= cm.info.width or cy < 0 or cy >= cm.info.height:
            return False
        v = cm.data[cx + cy * cm.info.width]
        return v >= self.lethal_thresh

    def select_frontier(self, pose):
        rx, ry, _ = pose
        frontiers = self.find_frontiers()
        viable = []
        for wx, wy, size in frontiers:
            if self.is_blacklisted(wx, wy) or self.costmap_is_blocked(wx, wy):
                continue
            viable.append((math.hypot(wx - rx, wy - ry), size, wx, wy))
        if not viable:
            return None
        # prefer frontiers worth driving to; fall back to whatever is left
        far = [c for c in viable if c[0] >= self.min_travel_dist]
        pool = far if far else viable
        pool.sort(key=lambda c: c[0])     # nearest first -> predictable coverage
        return (pool[0][2], pool[0][3])


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
