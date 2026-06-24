#!/usr/bin/env python3
"""Slowly move the robot to an absolute Cartesian position via the admittance node.

Streams a slow, straight-line ramp of intermediate targets from the current pose to
the goal at a fixed speed, so the robot CREEPS there safely no matter how far it is
(avoids the fast traverse that trips a reflex). Orientation is kept fixed.

    python3 goto.py X Y Z [speed_m_s]
    python3 goto.py 0.4 0.0 0.4          # default 0.03 m/s
    python3 goto.py 0.4 0.0 0.4 0.05     # faster
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

POSE_TOPIC = "/admittance_node/current_pose"
CMD_TOPIC = "/admittance_node/target_pose"
RATE_HZ = 100.0


class GoTo(Node):
    def __init__(self):
        super().__init__("goto")
        self.pub = self.create_publisher(PoseStamped, CMD_TOPIC, 10)
        self.latest = None
        self.create_subscription(PoseStamped, POSE_TOPIC, self._cb, 10)

    def _cb(self, msg):
        self.latest = msg.pose

    def wait_pose(self, timeout=10.0):
        t0 = time.time()
        while self.latest is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest is None:
            raise RuntimeError(f"no pose on {POSE_TOPIC}")
        return self.latest


def main():
    if len(sys.argv) not in (4, 5):
        print("usage: python3 goto.py X Y Z [speed_m_s]   (meters, base frame)")
        return
    goal = np.array([float(v) for v in sys.argv[1:4]])
    speed = float(sys.argv[4]) if len(sys.argv) == 5 else 0.03

    rclpy.init()
    node = GoTo()
    p = node.wait_pose()
    ori = p.orientation  # keep current orientation
    start = np.array([p.position.x, p.position.y, p.position.z])

    dist = float(np.linalg.norm(goal - start))
    duration = max(dist / speed, 0.01)
    n_steps = max(int(duration * RATE_HZ), 1)
    print(f"[goto] start=({start[0]*1000:.0f},{start[1]*1000:.0f},{start[2]*1000:.0f})mm"
          f" -> goal=({goal[0]*1000:.0f},{goal[1]*1000:.0f},{goal[2]*1000:.0f})mm"
          f"  dist={dist*1000:.0f}mm  speed={speed}m/s  ~{duration:.1f}s")

    msg = PoseStamped()
    msg.header.frame_id = "base"
    msg.pose.orientation = ori

    dt = 1.0 / RATE_HZ
    for k in range(1, n_steps + 1):
        a = k / n_steps
        pos = start + a * (goal - start)            # straight-line interpolation
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos.tolist()
        node.pub.publish(msg)
        t_end = time.time() + dt
        while time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.001)

    # hold final target briefly so it settles
    t0 = time.time()
    while time.time() - t0 < 1.0:
        msg.header.stamp = node.get_clock().now().to_msg()
        node.pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.02)
    print("[goto] done")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
