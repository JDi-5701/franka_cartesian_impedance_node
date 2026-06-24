# franka_cartesian_impedance_node

Direct-libfranka Cartesian control for the Franka FR3 (ROS 2), **bypassing
ros2_control / franka_hardware**. Connects with libfranka, runs a 1 kHz control loop,
subscribes to a target pose, publishes pose / wrench / (joint) state.

Two interchangeable controllers sharing a **unified topic interface** (command in
`~/target_pose`, observe `~/current_pose` + `~/ext_wrench`), so the same scripts and
teleop work on either — pick the one whose contact/tracking trade-off you want.

## Controllers

### `cartesian_impedance_node` — built-in (true) Cartesian impedance
`robot.control(cb, ControllerMode::kCartesianImpedance)` + `setCartesianImpedance`.
A real impedance (force-out) controller: **naturally stable on contact** (yields, no
buzz/smash) — the KUKA-like behavior. Trade-off: at low (compliant) stiffness the
dynamic tracking lags. **Use this for contact / teleoperation.**
- Subscribes `~/target_pose`. Publishes `~/current_pose`, `~/ext_wrench`.
```bash
ros2 launch franka_cartesian_impedance_node cartesian_impedance.launch.py   # config/impedance_params.yaml
```

### `admittance_node` — high stiffness + software admittance
High built-in stiffness (fast/accurate inner servo) + compliance rendered in software
from the measured force (`M·ẍ + D·ẋ + Kv·x = F_ext`). **Fast, low-lag in free space**,
but admittance-on-stiff-loop **can self-excite (oscillate/smash) on contact**. Good for
free-space following; not for sustained rigid contact. The node OWNS the equilibrium.
- Subscribes `~/target_pose` (absolute), `~/cmd_twist` (tool-frame vel, integrated +
  force-limited here), `~/reset` (`Bool`). Publishes `~/current_pose`, `~/equilibrium`,
  `~/equilibrium_target`, `~/ext_wrench`, `~/joint_states` (q/dq/tau).
```bash
ros2 launch franka_cartesian_impedance_node admittance.launch.py             # config/admittance_params.yaml
```

Both classify the cause of any libfranka reflex (collision / command discontinuity /
joint-velocity limit / hardware limit) and log the offending `O_F_ext` / `tau_ext`
channel vs thresholds + the last command-derivative cycles — read that, don't guess.

## Unified interface — tools work on either controller
Pick the controller namespace with the `CTRL_NS` env var (default
`/cartesian_impedance_node`):
```bash
python3 scripts/goto.py --home                              # -> cartesian_impedance_node
CTRL_NS=/admittance_node python3 scripts/goto.py --home     # -> admittance_node
```
`sigmoid_move.py`, `static_test.py`, `teleop_monitor.py` all honor `CTRL_NS`.

## Build
```bash
colcon build --packages-select franka_cartesian_impedance_node --symlink-install
source install/setup.bash
```
Deps: libfranka (`Franka` CMake), rclcpp, geometry_msgs, sensor_msgs, std_msgs, Eigen3.

## Scripts (`scripts/`)
- `goto.py [X Y Z] [--home] [--rpy R P Y | --quat x y z w] [--speed] [--ang-speed]` —
  slowly move to an absolute pose (creeps via interpolated waypoints; `--home` =
  `0.4 0 0.4` + pointing down).
- `sigmoid_move.py` — sinusoidal tracking test; reports max/rms error + est. time lag.
- `static_test.py` — static positioning-accuracy test.
- `teleop_monitor.py` — read-only diagnostic; event-triggered (drift / vibration /
  joint-nullspace buzz) + session summary on Ctrl-C.

## Parameters
`config/impedance_params.yaml` (cartesian_impedance_node: stiffness, command filter,
collision thresholds, cutoff) and `config/admittance_params.yaml` (admittance_node:
stiffness, follow gain, admittance M/D/Kv, force deadband/sign, twist/force-limit,
collision, cutoff). Both have `debug_buffer_samples` / `debug_dump_samples`.
