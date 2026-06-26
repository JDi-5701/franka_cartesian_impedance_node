#!/usr/bin/env python3
"""Standalone franka_gripper connectivity + control + LATENCY check.

Run BEFORE wiring the SpaceMouse buttons, to confirm the gripper server is up, the
hardware is reachable, Move/Grasp actually move the fingers, AND to measure the delay
between sending a command and the fingers actually starting to move.

Latency is measured from the joint_states width: NOTE the gripper publishes
velocity=0 always, so "motion start" = first time the width changes by more than
--motion-eps from its value at send time. Reported numbers therefore include the
joint_states publish period as quantization.

PREREQUISITE: gripper action server running, e.g.
    ros2 launch franka_gripper gripper.launch.py robot_ip:=<IP> namespace:=/

Then:
    python3 gripper_check.py                 # full sequence: homing -> open -> grasp -> open
    python3 gripper_check.py --action open
    python3 gripper_check.py --action grasp --force 30 --width 0.0
    python3 gripper_check.py --ns /franka_gripper --motion-eps 0.001
"""
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from franka_msgs.action import Homing, Move, Grasp


class GripperCheck(Node):
    def __init__(self, ns, motion_eps, timeout):
        super().__init__('gripper_check')
        self.motion_eps = motion_eps
        self.timeout = timeout
        self.homing_client = ActionClient(self, Homing, f'{ns}/homing')
        self.move_client = ActionClient(self, Move, f'{ns}/move')
        self.grasp_client = ActionClient(self, Grasp, f'{ns}/grasp')

        # latest measured width (sum of the two finger positions) + its arrival time
        self.width = None
        self.width_stamp = None
        self.create_subscription(JointState, f'{ns}/joint_states', self._js_cb, 10)

    def _js_cb(self, msg):
        if len(msg.position) >= 2:
            self.width = float(msg.position[0] + msg.position[1])
            self.width_stamp = self.get_clock().now()

    def _wait_for_width(self):
        """Block until we have at least one joint_states sample."""
        for _ in range(200):
            if self.width is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def _wait_settled(self, settle_time=0.4, max_wait=4.0):
        """Block until the width stops changing, so the next command's baseline and
        latency are not contaminated by the previous command's residual motion."""
        stable_since = None
        last = self.width
        start = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            now = self.get_clock().now()
            if self.width is not None and last is not None:
                if abs(self.width - last) <= self.motion_eps:
                    if stable_since is None:
                        stable_since = now
                    elif (now - stable_since).nanoseconds / 1e9 >= settle_time:
                        return True
                else:
                    stable_since = None
            last = self.width
            if (now - start).nanoseconds / 1e9 > max_wait:
                return False

    def _run(self, client, goal, label):
        """Send one goal; measure send->first-motion latency AND wait for the result.

        Returns (ok, latency_ms_or_None)."""
        self.get_logger().info(f'[{label}] waiting for action server ...')
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'[{label}] SERVER NOT FOUND (is the gripper node running?)')
            return False, None
        if not self._wait_for_width():
            self.get_logger().warn(f'[{label}] no joint_states yet -> latency unavailable')
        self._wait_settled()   # ensure a clean baseline (no residual motion from before)

        width0 = self.width
        t_send = self.get_clock().now()
        send_future = client.send_goal_async(goal)

        # accept handshake
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'[{label}] goal REJECTED')
            return False, None

        result_future = handle.get_result_async()
        t_move = None
        latency_ms = None
        # spin until motion is detected AND the action result is back (or timeout)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if t_move is None and width0 is not None and self.width is not None \
                    and abs(self.width - width0) > self.motion_eps:
                t_move = self.get_clock().now()
                latency_ms = (t_move - t_send).nanoseconds / 1e6
                self.get_logger().info(f'[{label}] FIRST MOTION after {latency_ms:.1f} ms '
                                       f'(width {width0:.4f} -> {self.width:.4f})')
            if result_future.done():
                break
            if (self.get_clock().now() - t_send).nanoseconds / 1e9 > self.timeout:
                self.get_logger().warn(f'[{label}] timeout waiting for result')
                break

        if t_move is None:
            self.get_logger().warn(f'[{label}] no motion detected (>|{self.motion_eps}| m) '
                                   '-> already at target, or eps too large')

        res = result_future.result().result if result_future.done() else None
        ok = getattr(res, 'success', False) if res is not None else False
        if ok:
            self.get_logger().info(f'[{label}] OK')
        else:
            err = getattr(res, 'error', 'no result') if res is not None else 'no result'
            self.get_logger().error(f'[{label}] FAILED: {err}')
        return ok, latency_ms

    def homing(self):
        return self._run(self.homing_client, Homing.Goal(), 'homing')

    def open(self, width, speed):
        g = Move.Goal(); g.width = width; g.speed = speed
        return self._run(self.move_client, g, f'open(width={width})')

    def grasp(self, width, speed, force, eps):
        g = Grasp.Goal(); g.width = width; g.speed = speed; g.force = force
        g.epsilon.inner = eps; g.epsilon.outer = eps
        return self._run(self.grasp_client, g, f'grasp(force={force})')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--ns', default='/franka_gripper', help='gripper action namespace')
    p.add_argument('--action', default='all', choices=['all', 'homing', 'open', 'grasp'])
    p.add_argument('--width', type=float, default=None, help='target width [m]')
    p.add_argument('--speed', type=float, default=0.1, help='[m/s]')
    p.add_argument('--force', type=float, default=40.0, help='grasp force [N]')
    p.add_argument('--epsilon', type=float, default=0.08, help='grasp success tolerance [m]')
    p.add_argument('--motion-eps', type=float, default=0.001,
                   help='width change [m] that counts as "started moving"')
    p.add_argument('--timeout', type=float, default=15.0, help='per-action result timeout [s]')
    p.add_argument('--repeat', type=int, default=0,
                   help='benchmark mode: N open<->close cycles, report Move vs Grasp '
                        'latency stats (min/median/max)')
    args = p.parse_args()

    rclpy.init()
    node = GripperCheck(args.ns, args.motion_eps, args.timeout)
    ok = True
    try:
        if args.repeat > 0:
            ok = _bench(node, args)
        else:
            if args.action in ('all', 'homing'):
                ok &= node.homing()[0]
            if args.action in ('all', 'open'):
                ok &= node.open(0.08 if args.width is None else args.width, args.speed)[0]
            if args.action in ('all', 'grasp'):
                ok &= node.grasp(0.0 if args.width is None else args.width,
                                 args.speed, args.force, args.epsilon)[0]
            if args.action == 'all':             # re-open so we end in a known state
                ok &= node.open(0.08, args.speed)[0]
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print('\n=== RESULT:', 'ALL OK' if ok else 'SOMETHING FAILED', '===')
    sys.exit(0 if ok else 1)


def _stats(name, vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        print(f'  {name:6s}: no latency samples')
        return
    s = sorted(vals)
    median = s[len(s) // 2]
    print(f'  {name:6s}: n={len(s)}  min={min(s):.0f}  median={median:.0f}  '
          f'max={max(s):.0f} ms')


def _bench(node, args):
    """N cycles of Move-open (0.08) <-> Grasp-close, collecting first-motion latency
    for each command type to compare them with stats."""
    node.homing()                                # start from a known calibrated state
    move_lat, grasp_lat = [], []
    ok = True
    for i in range(args.repeat):
        node.get_logger().info(f'--- cycle {i + 1}/{args.repeat} ---')
        g_ok, g_l = node.grasp(0.0 if args.width is None else args.width,
                               args.speed, args.force, args.epsilon)
        ok &= g_ok; grasp_lat.append(g_l)
        m_ok, m_l = node.open(0.08, args.speed)
        ok &= m_ok; move_lat.append(m_l)
    print('\n=== LATENCY (send -> first motion) ===')
    _stats('Move', move_lat)
    _stats('Grasp', grasp_lat)
    return ok


if __name__ == '__main__':
    main()
