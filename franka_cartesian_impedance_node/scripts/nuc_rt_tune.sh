#!/usr/bin/env bash
# One-shot NUC tuning for the Franka FCI 1kHz link. Run with sudo AFTER EVERY BOOT
# (settings do not survive a reboot):
#
#   sudo bash nuc_rt_tune.sh
#
# Why (2026-07-24 findings): the robot aborts with communication_constraints_violation /
# *_discontinuity even while holding still with zero target traffic -> pure transport
# timing. eno1 (robot) IRQ lands on CPU15, enp2s0 (GPU PC) on CPU0 — separate cores —
# BUT the actual packet processing (NAPI/softirq) runs at NORMAL priority, so any busy
# thread scheduled onto CPU15 delays robot packets (robot success_rate drops in lockstep
# with late callbacks). Fix: dedicate CPU15 to eno1 packet processing at RT priority.
#
# This script:
#   1. pins the eno1 IRQ to CPU15 and the enp2s0 IRQ to CPU0 (freeze current layout)
#   2. switches eno1 to threaded NAPI and runs its NAPI thread on CPU15 at SCHED_FIFO 50
#   3. raises ksoftirqd/15 to SCHED_FIFO 49 (fallback path for work still deferred there)
#
# Pair with the controller side: cartesian_impedance.launch.py confines the node
# process to CPUs 0-13 (taskset prefix) and the libfranka control thread pins itself
# to CPU14 (control_thread_cpu param).
set -eu

ROBOT_IF=eno1;  ROBOT_CPU=15
GPU_IF=enp2s0;  GPU_CPU=0

[ "$(id -u)" -eq 0 ] || { echo "run with sudo"; exit 1; }

irq_of() { awk -v ifc="$1" '$NF==ifc {sub(":","",$1); print $1; exit}' /proc/interrupts; }

r=$(irq_of "$ROBOT_IF"); g=$(irq_of "$GPU_IF")
[ -n "$r" ] || { echo "ERROR: no IRQ found for $ROBOT_IF"; exit 1; }
[ -n "$g" ] || { echo "ERROR: no IRQ found for $GPU_IF"; exit 1; }

printf '%x' $((1 << ROBOT_CPU)) > "/proc/irq/$r/smp_affinity"
printf '%x' $((1 << GPU_CPU))  > "/proc/irq/$g/smp_affinity"
echo "IRQ $r ($ROBOT_IF) -> CPU$ROBOT_CPU;  IRQ $g ($GPU_IF) -> CPU$GPU_CPU"

# Threaded NAPI: moves eno1 RX processing out of anonymous softirq context into a
# dedicated kernel thread we can prioritize and pin.
if echo 1 > "/sys/class/net/$ROBOT_IF/threaded" 2>/dev/null; then
  sleep 0.3
  found=0
  for pid in $(ps -e -o pid=,comm= | awk '$2 ~ /^napi\// {print $1}'); do
    # match this NIC's NAPI thread(s) by name: napi/<ifname>-<n>
    comm=$(cat "/proc/$pid/comm")
    case "$comm" in
      napi/"$ROBOT_IF"*)
        chrt -f -p 50 "$pid"
        taskset -pc "$ROBOT_CPU" "$pid" > /dev/null
        echo "$comm (pid $pid) -> SCHED_FIFO 50, CPU$ROBOT_CPU"
        found=1;;
    esac
  done
  [ "$found" -eq 1 ] || echo "WARNING: threaded NAPI enabled but no napi/$ROBOT_IF thread found"
else
  echo "WARNING: could not enable threaded NAPI on $ROBOT_IF (driver too old?)"
fi

kpid=$(ps -e -o pid=,comm= | awk -v c="ksoftirqd/$ROBOT_CPU" '$2==c {print $1}')
[ -n "$kpid" ] && chrt -f -p 49 "$kpid" && echo "ksoftirqd/$ROBOT_CPU (pid $kpid) -> SCHED_FIFO 49"

# CPU idle states (added 2026-07-24, after the priority fix alone changed
# nothing): a DEDICATED core is idle most of every 1 ms cycle and drops into
# deep C-states — C3 on this NUC costs 350 us exit latency, paid on every robot
# packet / control-thread wakeup. That chronic sub-ms lateness is what the robot
# counts as late commands (sr 0.58 with an otherwise clean system). Disable
# every idle state deeper than 10 us on the dedicated CPUs 14/15 AND their SMT
# siblings 6/7 (same physical cores — the core only sleeps deep if both
# hyperthreads do). POLL/C1 (<=1 us) stay enabled.
for cpu in 6 7 14 15; do
  for st in /sys/devices/system/cpu/cpu"$cpu"/cpuidle/state*; do
    lat=$(cat "$st/latency")
    [ "$lat" -gt 10 ] && echo 1 > "$st/disable"
  done
  echo "cpu$cpu: idle states deeper than 10us disabled"
done

echo "done. Verify: ps -eLo rtprio,policy,psr,comm | grep -E 'napi/$ROBOT_IF|ksoftirqd/$ROBOT_CPU'"
