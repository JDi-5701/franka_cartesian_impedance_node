#!/usr/bin/env python3
"""
Continuous smooth oscillation test for the direct-libfranka Cartesian impedance node.

Drives the target TCP pose sinusoidally back and forth along ONE axis for
TOTAL_TIME seconds, so you can watch the robot move continuously and judge
smoothness / tracking quality. Logs commanded vs measured and reports the
max / RMS tracking error (the EE lags the moving target; smaller & smoother
= stiffer / better tracking).

    python3 sigmoid_move.py

Tune the constants below (axis, amplitude, period, total time).
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# ---- config ----
AXIS = "y"            # axis to oscillate: "x", "y" or "z"
AMPLITUDE = 0.1       # [m] half-stroke (peak-to-peak = 2*AMPLITUDE)
PERIOD = 10.0         # [s] one full back-and-forth cycle
TOTAL_TIME = 12.0     # [s] how long to keep oscillating
RATE = 100.0          # [Hz] command publish rate
POSE_TOPIC = "/admittance_node/current_pose"
CMD_TOPIC = "/admittance_node/target_pose"
# ----------------

AXES = {"x": 0, "y": 1, "z": 2}


class Oscillate(Node):
    def __init__(self):
        super().__init__("sigmoid_move")
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

    def measured(self, i):
        return [self.latest.position.x, self.latest.position.y,
                self.latest.position.z][i]


def main():
    i = AXES[AXIS]
    rclpy.init()
    node = Oscillate()

    p = node.wait_pose()
    ori = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    base = [p.position.x, p.position.y, p.position.z]
    center = base[i]
    print(f"\n[oscillate] axis={AXIS}  center={center*1000:.1f}mm  "
          f"+/-{AMPLITUDE*1000:.0f}mm  period={PERIOD}s  for {TOTAL_TIME}s\n")

    dt = 1.0 / RATE
    t0 = time.time()
    max_err = 0.0
    sum_sq = 0.0
    n = 0
    next_print = 1.0
    rows = []  # (t, cmd_mm, meas_mm, err_mm) every cycle
    while True:
        t = time.time() - t0
        if t >= TOTAL_TIME:
            break
        cmd = center + AMPLITUDE * math.sin(2.0 * math.pi * t / PERIOD)
        xyz = list(base); xyz[i] = cmd
        node.publish(xyz, ori)
        rclpy.spin_once(node, timeout_sec=0.0)
        meas = node.measured(i)
        err = meas - cmd
        max_err = max(max_err, abs(err))
        sum_sq += err * err
        n += 1
        rows.append((t, cmd * 1000.0, meas * 1000.0, err * 1000.0))
        if t >= next_print:
            print(f"  t={t:4.1f}s  cmd={cmd*1000:+6.1f}  meas={meas*1000:+6.1f}  "
                  f"err={err*1000:+5.1f} mm")
            next_print += 1.0
        t_end = time.time() + dt
        while time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.001)

    # smooth ramp back to center over RAMP_T seconds (a step here slams the follow
    # filter -> command acceleration jump -> joint_velocity_discontinuity reflex).
    RAMP_T = 3.0
    last = cmd  # last commanded value from the loop
    n_ramp = int(RAMP_T * RATE)
    for k in range(1, n_ramp + 1):
        a = k / n_ramp
        val = last + a * (center - last)
        xyz = list(base); xyz[i] = val
        node.publish(xyz, ori)
        t_end = time.time() + dt
        while time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.001)

    rms = (sum_sq / n) ** 0.5 if n else 0.0

    # Estimate pure time-lag: shift meas back by k samples to best match cmd.
    avg_dt = rows[-1][0] / (n - 1) if n > 1 else dt
    best_lag_s, best_rms = 0.0, rms * 1000.0
    cmds = [r[1] for r in rows]
    meass = [r[2] for r in rows]
    max_shift = int(0.5 / avg_dt) if avg_dt > 0 else 0  # search up to 0.5 s
    for k in range(0, max_shift + 1):
        s = sum((meass[j] - cmds[j - k]) ** 2 for j in range(k, n))
        cnt = n - k
        if cnt <= 0:
            break
        r = (s / cnt) ** 0.5
        if r < best_rms:
            best_rms, best_lag_s = r, k * avg_dt

    print(f"\n  max tracking err   = {max_err*1000:.2f} mm")
    print(f"  rms tracking err   = {rms*1000:.2f} mm")
    print(f"  est. time lag      = {best_lag_s*1000:.0f} ms "
          f"(residual rms {best_rms:.2f} mm after lag removed)\n")

    # Write full log next to this script.
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sigmoid_log.txt")
    with open(out, "w") as f:
        f.write("# sigmoid tracking test\n")
        f.write(f"# axis={AXIS} amplitude_mm={AMPLITUDE*1000:.1f} period_s={PERIOD} "
                f"total_s={TOTAL_TIME} rate_hz={RATE}\n")
        f.write(f"# center_mm={center*1000:.2f} samples={n} avg_dt_s={avg_dt:.4f}\n")
        f.write(f"# max_err_mm={max_err*1000:.2f} rms_err_mm={rms*1000:.2f} "
                f"est_time_lag_ms={best_lag_s*1000:.0f} residual_rms_mm={best_rms:.2f}\n")
        f.write("# t_s\tcmd_mm\tmeas_mm\terr_mm\n")
        for (t, c, m, e) in rows:
            f.write(f"{t:.4f}\t{c:.3f}\t{m:.3f}\t{e:.3f}\n")
    print(f"  log written: {out}\n")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
