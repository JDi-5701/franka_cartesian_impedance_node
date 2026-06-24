#!/usr/bin/env python3
"""Teleop diagnostic monitor (read-only). Works on EITHER controller.

Set CTRL_NS to choose the controller namespace (default /cartesian_impedance_node;
use /admittance_node for the admittance node). Uses only the unified topics:
  <ns>/current_pose, <ns>/target_pose (commanded), <ns>/ext_wrench, and
  <ns>/joint_states if the controller publishes it (admittance does; impedance may not).

Event-triggered: stays quiet, prints a block only on an anomaly, and prints a SESSION
SUMMARY (worst oscillation seen) on Ctrl-C.

  python3 teleop_monitor.py
  CTRL_NS=/admittance_node python3 teleop_monitor.py
"""
import collections
import os

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState

NS = os.environ.get("CTRL_NS", "/cartesian_impedance_node")
WINDOW_S = 2.0
SAMPLE_HZ = 100.0
REPORT_HZ = 2.0
DRIFT_MM_S = 1.0
CMD_MOVE_MM_S = 1.0       # target moving faster than this = "commanding"
VIB_P2P_MM = 0.8
JOINT_DQ_P2P = 0.03
BUZZ_FREQ = 1.0
TCP_STILL_MM = 8.0


def _p(m):
    return np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])


def _dom(sig, fs):
    n = len(sig)
    if n < 8:
        return 0.0, float(np.ptp(sig)) if n else 0.0
    s = sig - np.mean(sig)
    amp = np.abs(np.fft.rfft(s * np.hanning(n)))
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    if len(amp) < 3:
        return 0.0, float(np.ptp(sig))
    k = 1 + int(np.argmax(amp[1:]))
    return float(freqs[k]), float(np.ptp(sig))


class Monitor(Node):
    def __init__(self):
        super().__init__("teleop_monitor")
        self.pos = None
        self.tgt = None
        self.force = np.zeros(3)
        self.dq = np.zeros(7)
        self.have_joints = False

        self.create_subscription(PoseStamped, f"{NS}/current_pose", self._cp, 10)
        self.create_subscription(PoseStamped, f"{NS}/target_pose", self._tg, 10)
        self.create_subscription(WrenchStamped, f"{NS}/ext_wrench", self._w, 10)
        self.create_subscription(JointState, f"{NS}/joint_states", self._js, 10)

        self.buf = collections.deque(maxlen=int(WINDOW_S * SAMPLE_HZ))
        self.t0 = None
        self.last_print = -1e9
        self.last_hb = -1e9
        self.peak_buzz = 0.0
        self.peak_info = None
        self.create_timer(1.0 / SAMPLE_HZ, self._sample)
        self.create_timer(1.0 / REPORT_HZ, self._report)
        self.get_logger().info(f"teleop_monitor on {NS} (read-only). Quiet until an anomaly.")

    def _cp(self, m): self.pos = _p(m)
    def _tg(self, m): self.tgt = _p(m)
    def _w(self, m):
        self.force = np.array([m.wrench.force.x, m.wrench.force.y, m.wrench.force.z])
    def _js(self, m):
        if m.velocity:
            self.dq = np.array(m.velocity[:7])
            self.have_joints = True

    def _t(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    def _sample(self):
        if self.pos is None:
            return
        tgt = self.tgt if self.tgt is not None else self.pos
        self.buf.append((self._t(), self.pos.copy(), tgt.copy(), self.force.copy(), self.dq.copy()))

    def _report(self):
        if len(self.buf) < int(0.5 * WINDOW_S * SAMPLE_HZ):
            return
        r = list(self.buf)
        t = np.array([x[0] for x in r])
        pos = np.array([x[1] for x in r])
        tgt = np.array([x[2] for x in r])
        f = np.array([x[3] for x in r])
        dq = np.array([x[4] for x in r])
        dur = max(t[-1] - t[0], 1e-3)
        fs = (len(t) - 1) / dur

        off = (pos - tgt)[-1] * 1000                       # tracking error / compliance displ.
        f_bias = np.mean(f, axis=0)
        tgt_vel = np.linalg.norm((tgt[-1] - tgt[0]) / dur * 1000)   # mm/s
        commanding = tgt_vel > CMD_MOVE_MM_S
        off_vel = np.linalg.norm(((pos - tgt)[-1] - (pos - tgt)[0]) / dur * 1000)
        tcp_p2p = np.ptp(pos, axis=0) * 1000
        tcp_max = float(np.max(tcp_p2p))

        jp2p = np.ptp(dq, axis=0)
        jfreq = np.array([_dom(dq[:, j], fs)[0] for j in range(7)])
        jmax = int(np.argmax(jp2p))
        if self.have_joints and jfreq[jmax] > BUZZ_FREQ and jp2p[jmax] > self.peak_buzz:
            self.peak_buzz = jp2p[jmax]
            self.peak_info = (jmax + 1, jp2p[jmax], jfreq[jmax], t[-1], tcp_max, commanding)

        events = []
        if self.have_joints and jp2p[jmax] > JOINT_DQ_P2P and jfreq[jmax] > BUZZ_FREQ and tcp_max < TCP_STILL_MM:
            events.append(f"JOINT/NULLSPACE BUZZ: J{jmax+1} dq swing {jp2p[jmax]:.2f} rad/s "
                          f"@ {jfreq[jmax]:.1f} Hz, TCP p2p only {tcp_max:.1f} mm -> redundant motion.")
        if not commanding and off_vel > DRIFT_MM_S:
            events.append(f"DRIFT: tracking error growing {off_vel:.2f} mm/s with no command, "
                          f"|force bias|={np.linalg.norm(f_bias):.2f} N")
        if not commanding and tcp_max > VIB_P2P_MM:
            ax = int(np.argmax(tcp_p2p))
            fr, _ = _dom(pos[:, ax], fs)
            events.append(f"CART VIBRATION on {'xyz'[ax]}: TCP p2p {tcp_p2p[ax]:.2f} mm @ {fr:.1f} Hz")

        now = t[-1]
        if events:
            if now - self.last_print > 1.0:
                self.last_print = now
                print(f"\n==== ANOMALY @ t={now:6.1f}s ({'commanding' if commanding else 'idle'}) ====")
                print(f" pos[mm]=[{pos[-1,0]*1000:6.1f} {pos[-1,1]*1000:6.1f} {pos[-1,2]*1000:6.1f}]"
                      f"  err[mm]=[{off[0]:5.2f} {off[1]:5.2f} {off[2]:5.2f}]"
                      f"  force[N]=[{f[-1,0]:5.2f} {f[-1,1]:5.2f} {f[-1,2]:5.2f}]")
                if self.have_joints:
                    print(f" dq_p2p=[" + " ".join(f"{v:5.2f}" for v in jp2p) + "]"
                          f"  dq_freq=[" + " ".join(f"{v:4.1f}" for v in jfreq) + "]Hz")
                for e in events:
                    print(f" >> {e}")
        elif now - self.last_hb > 15.0:
            self.last_hb = now
            jtxt = f"top joint J{jmax+1} {jp2p[jmax]:.2f} rad/s" if self.have_joints else "no joint_states"
            print(f"[t={now:6.1f}s] monitoring ({NS}), nothing abnormal (TCP p2p {tcp_max:.1f}mm, {jtxt})")


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            print("\n========== SESSION SUMMARY ==========")
            if node.peak_info:
                j, p2p, fr, t, tcp, cmd = node.peak_info
                print(f" worst joint oscillation: J{j}  dq_p2p={p2p:.3f} rad/s @ {fr:.1f} Hz  "
                      f"(t={t:.1f}s, TCP p2p {tcp:.1f}mm, {'commanding' if cmd else 'idle'})")
                if tcp < TCP_STILL_MM:
                    print(" -> joint oscillating while TCP ~still => NULLSPACE (redundant joint).")
            elif not node.have_joints:
                print(f" (no joint_states on {NS}; joint analysis unavailable for this controller)")
            else:
                print(" no joint oscillation above the buzz band was seen.")
        except BrokenPipeError:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
