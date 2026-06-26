#!/usr/bin/env python3
"""Slowly move the robot to a target Cartesian pose (position AND orientation) via the
admittance node's absolute equilibrium input.

Streams a slow straight-line + SLERP ramp from the current pose to the goal at bounded
linear/angular speed, so the robot CREEPS there safely no matter how far. Default
orientation is "straight down" (a good ready pose); override with --quat or --rpy.

    python3 goto.py 0.4 0.0 0.4                 # position, default down orientation
    python3 goto.py --home                       # ready pose: 0.4 0 0.4, down
    python3 goto.py 0.4 0.0 0.4 --rpy 180 0 0    # orientation by roll/pitch/yaw (deg)
    python3 goto.py 0.4 0.0 0.4 --quat 1 0 0 0   # orientation by quaternion (x y z w)
    python3 goto.py 0.4 0.0 0.4 --speed 0.05 --ang-speed 0.4
"""
import argparse
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R, Slerp

# Controller namespace: works on EITHER controller. Default = cartesian_impedance_node;
# set CTRL_NS=/admittance_node to target the admittance node instead.
NS = os.environ.get("CTRL_NS", "/cartesian_impedance_node")
POSE_TOPIC = f"{NS}/current_pose"
CMD_TOPIC = f"{NS}/target_pose"
RATE_HZ = 100.0
DOWN_QUAT = [1.0, 0.0, 0.0, 0.0]  # x y z w: EE pointing straight down (Franka "ready")


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
    ap = argparse.ArgumentParser()
    ap.add_argument("xyz", nargs="*", type=float, help="X Y Z (meters, base frame)")
    ap.add_argument("--home", action="store_true", help="ready pose: 0.4 0 0.4 + down")
    ap.add_argument("--quat", nargs=4, type=float, metavar=("X", "Y", "Z", "W"))
    ap.add_argument("--rpy", nargs=3, type=float, metavar=("R", "P", "Y"), help="degrees")
    ap.add_argument("--speed", type=float, default=0.03, help="m/s")
    ap.add_argument("--ang-speed", type=float, default=0.3, help="rad/s")
    args = ap.parse_args()

    if args.home:
        goal_p = np.array([0.4, 0.0, 0.4])
    elif len(args.xyz) == 3:
        goal_p = np.array(args.xyz)
    else:
        ap.error("give X Y Z (or --home)")

    if args.quat:
        goal_q = np.array(args.quat, dtype=float)
        goal_q /= np.linalg.norm(goal_q)
    elif args.rpy:
        goal_q = R.from_euler("xyz", args.rpy, degrees=True).as_quat()  # x y z w
    else:
        goal_q = np.array(DOWN_QUAT)

    rclpy.init()
    node = GoTo()
    p = node.wait_pose()
    start_p = np.array([p.position.x, p.position.y, p.position.z])
    start_q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    # slerp setup (shortest path)
    if np.dot(start_q, goal_q) < 0:
        goal_q = -goal_q
    slerp = Slerp([0.0, 1.0], R.from_quat(np.vstack([start_q, goal_q])))

    lin_dist = float(np.linalg.norm(goal_p - start_p))
    ang_dist = float((R.from_quat(start_q).inv() * R.from_quat(goal_q)).magnitude())  # rad
    duration = max(lin_dist / args.speed, ang_dist / args.ang_speed, 0.01)
    n_steps = max(int(duration * RATE_HZ), 1)
    print(f"[goto] pos {np.round(start_p*1000).astype(int)} -> {np.round(goal_p*1000).astype(int)} mm "
          f"({lin_dist*1000:.0f}mm), rot {np.degrees(ang_dist):.0f}deg, ~{duration:.1f}s")

    msg = PoseStamped()
    msg.header.frame_id = "base"
    dt = 1.0 / RATE_HZ
    for k in range(1, n_steps + 1):
        a = k / n_steps
        pos = start_p + a * (goal_p - start_p)
        q = slerp([a]).as_quat()[0]  # x y z w
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos.tolist()
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = q.tolist()
        node.pub.publish(msg)
        t_end = time.time() + dt
        while time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.001)

    # hold final pose briefly so it settles
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
