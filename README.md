# franka_cartesian_impedance

Direct-**libfranka** Cartesian control for the Franka **FR3** (ROS 2 Jazzy), **bypassing
`ros2_control` / `franka_hardware`**. The node opens its own libfranka connection, runs the
1 kHz real-time control loop itself, takes a target TCP pose over ROS, and republishes the
robot state. A Franka Hand (gripper) server is brought up alongside it.

This repo is a **monorepo with two packages** (the repo root is intentionally *not* a
package, so colcon discovers both):

| Package | Depends on libfranka? | Purpose |
|---------|:---:|---------|
| **`franka_cartesian_impedance_msgs`** | no | interface-only (`srv/GoToPose.srv`). Build this alone on the operator PC to call the services. |
| **`franka_cartesian_impedance_node`** | yes | the controllers, launch files, config, scripts. |

---

## 1. Architecture

- No `ros2_control`, no `franka_hardware`, no URDF/MoveIt. The node links libfranka and
  calls `robot.control(...)` directly.
- The **1 kHz loop lives on the PC wired to the robot** and never crosses the network. You
  send a ~100 Hz target pose; the node interpolates it to 1 kHz locally with a 2nd-order
  critically-damped filter (so cross-machine commands stay smooth and don't trip reflexes).
- Single-publisher rule: only **one** publisher to `~/target_pose` at a time (teleop *or*
  a policy, not both).

For the two-PC (operator GPU PC ↔ robot NUC) network/DDS/daily-operation guide, see
`README_2PC_TELEOP.md` in the parent directory.

---

## 2. `cartesian_impedance_node` (primary, tested)

Uses the robot's **built-in (true) Cartesian impedance** controller
(`robot.control(cb, ControllerMode::kCartesianImpedance)` + `setCartesianImpedance`). A real
force-out impedance controller: **naturally stable on contact** (yields, no buzz/smash —
the KUKA-like behavior). Trade-off: at low (compliant) stiffness the dynamic tracking lags.
This is the controller used for teleoperation, data recording, and contact tasks.

### Franka robot data used (from libfranka `RobotState`)
The node reads these fields each cycle and republishes the useful ones as ROS topics:

| libfranka field | meaning | exposed as |
|-----------------|---------|------------|
| `O_T_EE`              | current TCP pose (base frame) | `~/current_pose` |
| `O_T_EE_d`            | impedance *desired* TCP pose  | `~/desired_pose` (compare vs current → tracking lag) |
| `O_dP_EE_d`           | desired TCP twist             | `~/ee_twist` |
| `q`, `dq`, `tau_J`    | joint position / velocity / torque | `~/joint_states` |
| `O_F_ext_hat_K`       | estimated external wrench at TCP | `~/ext_wrench` |
| `tau_ext_hat_filtered`| estimated external joint torque  | reflex diagnostics |
| `O_T_EE_c`            | last commanded pose | used to latch the equilibrium at startup |

Commands sent to the robot: `franka::CartesianPose` (the interpolated equilibrium),
plus `setCartesianImpedance(stiffness)` and `setCollisionBehavior(thresholds)` at start.

### ROS interface (namespace `/cartesian_impedance_node`)

**Subscribes**
| Topic | Type | Meaning |
|-------|------|---------|
| `~/target_pose` | `geometry_msgs/PoseStamped` (frame `base`) | commanded desired TCP pose (~100 Hz) |

**Publishes**
| Topic | Type | Rate | Meaning |
|-------|------|------|---------|
| `~/current_pose` | `geometry_msgs/PoseStamped`  | 1 kHz | measured TCP pose |
| `~/desired_pose` | `geometry_msgs/PoseStamped`  | 1 kHz | impedance-desired TCP pose |
| `~/ee_twist`     | `geometry_msgs/TwistStamped` | 1 kHz | desired TCP twist |
| `~/joint_states` | `sensor_msgs/JointState`     | 1 kHz | q / dq / tau_J (`fr3_joint1..7`) |
| `~/ext_wrench`   | `geometry_msgs/WrenchStamped`| 1 kHz | external force/torque at TCP |

**Services**
| Service | Type | Behavior |
|---------|------|----------|
| `~/go_home` | `std_srvs/srv/Trigger` | smoothly creep to the configured `home_pose`, ignoring `target_pose` until reached; blocks until reached/timeout. **Standard type → callable from any PC without building the custom srv.** |
| `~/go_pose` | `franka_cartesian_impedance_msgs/srv/GoToPose` | creep to an **arbitrary** pose (`pose` + optional `max_velocity`); empty/zero-quaternion request falls back to `home_pose`. Blocks until reached/timeout. |

Homing creeps at a velocity cap (param `homing_velocity`, request can override), considers
itself "reached" within `homing_position_tolerance` / `homing_rotation_tolerance`, and gives
up after `homing_timeout`. On success it latches the goal as the standing target so the arm
holds there until a teleop/policy takes over.

### Reflex diagnostics
On any libfranka reflex the node classifies the cause (collision / command discontinuity /
joint-velocity limit / hardware limit) and logs the offending `O_F_ext` / `tau_ext` channel
vs the configured thresholds, plus the last command-derivative cycles. Read that — don't guess.

### Key parameters (`franka_cartesian_impedance_node/config/impedance_params.yaml`)
- `robot_ip` — FR3 IP (default `192.168.3.100`; launch arg overrides).
- `cartesian_stiffness` `[x y z r p y]` — N/m and Nm/rad; lower = more compliant.
- `max_translational_velocity` / `_acceleration`, `max_rotational_velocity` / `_acceleration`
  — interpolation safety caps.
- `trans_filter_gain` / `rot_filter_gain` — 2nd-order command-filter stiffness.
- `command_cutoff_hz` — libfranka command low-pass.
- `collision_torque_threshold` / `collision_force_threshold` — reflex thresholds.
- `homing_velocity` / `homing_position_tolerance` / `homing_rotation_tolerance` /
  `homing_timeout` — homing behavior for `~/go_home` and `~/go_pose`.
- `home_pose` `[x y z qx qy qz qw]` (base frame) — the default home; `~/go_home` always goes
  here, `~/go_pose` falls back here on an empty request. `(1,0,0,0)` = TCP pointing down.
- `debug_buffer_samples` / `debug_dump_samples`.

---

## 3. Franka Hand (gripper)

`bringup.launch.py` also starts the standard **`franka_gripper`** action server (separate,
EXCLUSIVE libfranka connection). It exposes the usual interface under `/franka_gripper`:

| Interface | Type | Use |
|-----------|------|-----|
| `/franka_gripper/homing` | action `franka_msgs/action/Homing` | calibrate stroke |
| `/franka_gripper/move`   | action `franka_msgs/action/Move`   | open/close to a width |
| `/franka_gripper/grasp`  | action `franka_msgs/action/Grasp`  | grasp with force |
| `/franka_gripper/gripper_action` | action `control_msgs/action/GripperCommand` | simple width/effort |
| `/franka_gripper/stop`   | service | stop current motion |
| `/franka_gripper/joint_states` | `sensor_msgs/JointState` | finger state |

`scripts/gripper_check.py` is a standalone connectivity/latency test (robot side).

---

## 4. Other executables

- **`admittance_node`** — high built-in stiffness + software admittance
  (`M·ẍ + D·ẋ + Kv·x = F_ext`). Fast/low-lag in free space, but admittance-on-a-stiff-loop
  can self-excite on sustained rigid contact. Shares the `~/target_pose` / `~/current_pose`
  / `~/ext_wrench` interface; also takes `~/cmd_twist` and `~/reset`. Config:
  `config/admittance_params.yaml`. Launch: `admittance.launch.py`.
- **`cartesian_impedance_torque_node`** — an experimental torque-level Cartesian impedance
  controller (factorization-damping design). ⚠️ **Not yet hardware-tested — functionality
  intentionally not documented here yet.** See `TORQUE_IMPEDANCE_DESIGN.md` for the design.

---

## 5. Build

**Robot side (NUC, has libfranka)** — build both packages (deps resolved automatically):
```bash
cd ~/franka_ros2_ws
colcon build --packages-up-to franka_cartesian_impedance_node
source install/setup.bash
```

**Operator side (no libfranka, e.g. a RoboStack/conda env)** — interface package only, so you
can call the services without libfranka:
```bash
cd ~/ros_ml_ws
colcon build --packages-select franka_cartesian_impedance_msgs
source install/setup.bash
```
To keep the libfranka-dependent node package from being built on a machine without libfranka
(so others' `colcon build` never trips over it), drop a `COLCON_IGNORE` file in
`franka_cartesian_impedance_node/` on that machine (local only — do **not** commit it).

---

## 6. Run

```bash
# Robot side: controller + gripper server (the everyday bring-up)
ros2 launch franka_cartesian_impedance_node bringup.launch.py robot_ip:=192.168.3.100

# or controller only / gripper only
ros2 launch franka_cartesian_impedance_node cartesian_impedance.launch.py robot_ip:=192.168.3.100
ros2 launch franka_cartesian_impedance_node gripper_server.launch.py robot_ip:=192.168.3.100
```

Send a target / call the services:
```bash
# go to the configured home (standard type — works from any PC)
ros2 service call /cartesian_impedance_node/go_home std_srvs/srv/Trigger "{}"

# go to an arbitrary pose (e.g. per-episode data-recording resets)
ros2 service call /cartesian_impedance_node/go_pose \
  franka_cartesian_impedance_msgs/srv/GoToPose \
  "{pose: {position: {x: 0.4, y: 0.0, z: 0.45}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}, max_velocity: 0.05}"
```

---

## 7. Scripts (`franka_cartesian_impedance_node/scripts/`)

All honor the `CTRL_NS` env var (default `/cartesian_impedance_node`) so they work on either
controller, e.g. `CTRL_NS=/admittance_node python3 scripts/goto.py --home`.

- `goto.py [X Y Z] [--home] [--rpy R P Y | --quat x y z w] [--speed] [--ang-speed]` — creep
  to an absolute pose via interpolated waypoints.
- `sigmoid_move.py` — sinusoidal tracking test; reports max/rms error + estimated lag.
- `static_test.py` — static positioning-accuracy test.
- `teleop_monitor.py` — read-only diagnostic (drift / vibration / nullspace buzz) + summary.
- `gripper_check.py` — gripper connectivity/latency test.
