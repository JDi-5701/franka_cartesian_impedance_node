#!/usr/bin/env python3
"""
Static positioning-accuracy test for the Cartesian impedance node.

Quasi-static assembly metric: command a sequence of STEP targets along one axis,
wait for the robot to SETTLE, then measure the steady-state error (measured minus
commanded). This is what matters for slow assembly, unlike dynamic tracking lag.

    python3 static_test.py

Tune the constants below.
"""

import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# ---- config ----
AXIS = "y"                       # axis to step along: "x", "y" or "z"
STEPS_MM = [0, 20, 40, 20, 0, -20, -40, -20, 0]  # offsets from start [mm]
SETTLE_T = 3.0                   # [s] wait for settle before measuring
# Controller namespace: works on EITHER controller (set CTRL_NS=/admittance_node to switch).
NS = os.environ.get("CTRL_NS", "/cartesian_impedance_node")
POSE_TOPIC = f"{NS}/current_pose"
CMD_TOPIC = f"{NS}/target_pose"
# ----------------

AXES = {"x": 0, "y": 1, "z": 2}


class StaticTest(Node):
    def __init__(self):
        super().__init__("static_test")
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

    def publish(self, xyz, ori):
        m = PoseStamped()
        m.header.frame_id = "base"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        m.pose.orientation.x, m.pose.orientation.y = ori[0], ori[1]
        m.pose.orientation.z, m.pose.orientation.w = ori[2], ori[3]
        self.pub.publish(m)

    def spin_for(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.02)

    def measured(self, i):
        return [self.latest.position.x, self.latest.position.y,
                self.latest.position.z][i]


def main():
    i = AXES[AXIS]
    rclpy.init()
    node = StaticTest()

    p = node.wait_pose()
    ori = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    base = [p.position.x, p.position.y, p.position.z]
    center = base[i]
    print(f"\n[static] axis={AXIS}  start={center*1000:.1f}mm  settle={SETTLE_T}s\n")
    print("  target_mm   meas_mm    err_mm")

    max_err = 0.0
    for off in STEPS_MM:
        cmd = center + off / 1000.0
        xyz = list(base); xyz[i] = cmd
        # publish a few times so it is received, then wait to settle
        for _ in range(5):
            node.publish(xyz, ori)
            rclpy.spin_once(node, timeout_sec=0.02)
        node.spin_for(SETTLE_T)
        meas = node.measured(i)
        err = (meas - cmd) * 1000.0
        max_err = max(max_err, abs(err))
        print(f"   {cmd*1000:+7.1f}   {meas*1000:+7.1f}   {err:+6.2f}")

    print(f"\n  max steady-state error = {max_err:.2f} mm\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
