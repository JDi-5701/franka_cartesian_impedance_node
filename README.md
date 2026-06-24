# franka_cartesian_impedance_node

Direct-libfranka Cartesian control for the Franka FR3 (ROS 2), **bypassing
ros2_control / franka_hardware**. Connects to the robot with libfranka, runs a 1 kHz
control loop, subscribes to a target pose, and publishes pose / wrench / joint state.

Two executables:

### `admittance_node` (the main one)
KUKA-FRI-style compliant teleoperation. Runs the robot's built-in controller at
**high stiffness** (fast, accurate, low-lag position servo) and renders **compliance
in software** from the measured external force via a virtual admittance model
`M·ẍ + D·ẋ + Kv·x = F_ext`. This decouples "fast following" from "soft", which the
built-in low-stiffness Cartesian impedance alone cannot do (it lags).

- Subscribes: `~/target_pose` (`geometry_msgs/PoseStamped`)
- Publishes: `~/current_pose` (measured), `~/commanded_pose`, `~/ext_wrench`,
  `~/joint_states` (q / dq / tau)
- Push the TCP → it yields compliantly and returns; tracks a moving target tightly.
- On a reflex it logs the last command-derivative history (ROS log) for diagnosis.

```bash
ros2 launch franka_cartesian_impedance_node admittance.launch.py
```

### `cartesian_impedance_node`
Uses the robot's built-in **Cartesian impedance** directly
(`robot.control(cb, ControllerMode::kCartesianImpedance)` + `setCartesianImpedance`).
Mainly used to characterize the built-in impedance (it is compliant but lags at low
stiffness — see `admittance_node` for the production path).

```bash
ros2 launch franka_cartesian_impedance_node cartesian_impedance.launch.py
```

## Build
```bash
colcon build --packages-select franka_cartesian_impedance_node --symlink-install
source install/setup.bash
```
Requires libfranka (`Franka` CMake package), rclcpp, geometry_msgs, sensor_msgs, Eigen3.

## Scripts (`scripts/`)
- `goto.py X Y Z [speed]` — slowly move to an absolute Cartesian pose (creeps via
  interpolated waypoints; safe from far poses).
- `sigmoid_move.py` — sinusoidal tracking test; reports max/rms error + estimated
  time lag.
- `static_test.py` — static positioning-accuracy test.

## Parameters
See `config/admittance_params.yaml` (stiffness, follow gain, admittance M/D/Kv, force
deadband/sign, command low-pass, collision thresholds) and `config/params.yaml`.
