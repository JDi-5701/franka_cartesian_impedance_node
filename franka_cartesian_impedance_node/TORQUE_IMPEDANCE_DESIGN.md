# Torque-level Cartesian Impedance — Design Notes

Spec for writing our **own** torque-level Cartesian/joint impedance controller that
matches the reference controller, instead of the naive constant-damping version we
had. This collects (1) where the two reference code bases live, (2) exactly what the
"correct" version does differently, (3) the libfranka interface we need to build it.

Note: the reference code bases live **outside** this package (in the parent
`franka_ros2/` dir) and are NOT committed to this repo.

---

## 1. Reference code — exact locations

### Version A — what we STARTED from (naive, constant damping)
The official Franka `franka_ros2` example. Still in this repo:
- `../../src/cartesian_impedance_example_controller.cpp`
- `../../include/franka_cartesian_impedance/cartesian_impedance_example_controller.hpp`

Key line (the shortcut): damping is computed **once** from scalar stiffness, assuming
unit Cartesian inertia (`Λ = I`):
```cpp
// cartesian_impedance_example_controller.cpp  (~line 197)
damping_.topLeftCorner(3,3)     = 2.0 * damping_ratio_ * std::sqrt(ts) * I3;   // CONSTANT
damping_.bottomRightCorner(3,3) = 2.0 * damping_ratio_ * std::sqrt(rs) * I3;   // CONSTANT
tau_task = Jᵀ ( -K·error - D·(J·dq) );
tau_d    = tau_task + tau_nullspace + coriolis + tau_friction;
```
It *does* add coriolis + nullspace + friction, and exposes a `damping_ratio` param —
which is exactly what made it *look* complete. But `D` never uses the mass matrix, so
it is wrong as the arm configuration (and thus the apparent inertia) changes.

### Version B — the CORRECT reference (factorization damping)
Two reference trees, unzipped under `../` (i.e. `franka_ros2/`, local only — not committed):

- `../reference_ros1/` — ROS1 Melodic framework (state-machine → task → skill →
  controller). Real controllers are Simulink-compiled `.so`; only the **wrapper I/O
  contracts** + model layer are source:
  - `reference_ros1/src/panda/usr/include/plugins/cntr_cart_imp_wrapper.hpp` — Cartesian
    impedance I/O (inputs: `T_EE_d, T_EE, F_ff, M, B_J_EE, dtheta`; params:
    `K_x, xi_x(=damping ratio), tau_max, dtau_max`; outputs: `tau_J_d, tilde_x`).
  - `reference_ros1/src/panda/usr/include/plugins/cntr_joint_imp_wrapper.hpp` — joint
    impedance I/O (inputs: `theta, dtheta, theta_d, dtheta_d, ddtheta_d, M, tau_ff`;
    params: `K_theta, D_theta, enable_ffwd_vel, enable_ffwd_acc`).
  - `reference_ros1/src/panda/src/model.h` — wraps `franka::Model`: `mass, coriolis,
    gravity, zeroJacobian, bodyJacobian, pose`.
  - `reference_ros1/src/panda/usr/skills/cartesian_impedance/skill_cart_impedance.cpp` —
    note its comment: *"The robot is already in gravity compensation mode"*.
  - `reference_ros1/endeffector-config.json` — end-effector load + TCP frame
    (mass, COM, inertia, flange→EE transform) configured on the robot side.

- `../reference_simulink/` — the actual control-law models (`.mdl` are text):
  - `mdl/cntr_joint_imp/cntr_joint_imp.mdl` — contains subsystem
    **"Damping Design via Generalized Eigenvalues (QR right)"**. Annotation gives the
    math (A=K, B=M): `V'BV=I`, `inv(V)·inv(B)·A·V = diag(lam)`, `H = inv(V') = B·V`,
    plus a `sqrt` block and a `2*Damping Fac` gain. → This is the factorization
    damping design (see §2).
  - `config/cntr_joint_imp.json`, `config/motion_error_cart.json` — I/O bus shapes.
  - `examples/joint_impedance_controller.cpp` — a plain libfranka usage example.

### Our current node (built-in impedance, pose-commanded; keep as-is)
- `src/cartesian_impedance_node.cpp` — uses the robot's **built-in** Cartesian impedance
  (`setCartesianImpedance` + `ControllerMode::kCartesianImpedance`). Pose-commanded,
  compliant, contact-stable, but lags on moving targets (no trajectory feedforward).
  The new torque controller below is a **separate** node, not a replacement.

---

## 2. What the correct version does differently

| # | Topic | Version A (ours, naive) | Version B (reference — target) |
|---|-------|--------------------------|--------------------------------|
| 1 | **Damping** | `D = 2ξ√K`, **constant**, assumes `Λ=I`, computed once | `D(q)` from **generalized-eigenvalue factorization** of `(K, Λ(q))`, recomputed every cycle. **This is the #1 fix.** |
| 2 | **Inertia feedforward** | none (pure spring-damper) | `Λ·ẍ_d` (Cartesian) / `M(q)·q̈_d` (joint) — KUKA-FRI trick: spring only handles residual → soft *and* accurate |
| 3 | **Force/torque feedforward** | none | explicit `F_ff` / `tau_ff` path |
| 4 | **Coriolis** | added ✓ | added ✓ (both correct; never re-add gravity — robot auto-compensates it) |
| 5 | **Joint side used** | link-side `q` | motor-side `theta`/`dtheta` (flexible-joint model) for the stiffness term |
| 6 | **Torque smoothing** | manual `saturateTorqueRate` | torque **and** torque-rate saturation (`tau_max`, `dtau_max`) → no joint-elasticity excitation / reflex |
| 7 | **Nullspace** | basic projector | same idea; keep it |

### 2.1 Factorization Damping Design (the core algorithm)
Goal: pick a damping matrix `D` so that **every modal direction** of the
inertia-stiffness pair is damped to the desired ratio `ξ_i`, in the *current*
configuration. Recipe (Cartesian; for joint, replace `Λ→M`, `K_x→K_θ`, 6→7):

```
1. Cartesian inertia:        Λ = (J · M⁻¹ · Jᵀ)⁻¹            # 6×6, SPD
2. Generalized eigenproblem: solve  K_x · V = Λ · V · diag(λ)
      with eigenvectors normalized so that   Vᵀ Λ V = I  and  Vᵀ K_x V = diag(λ_i)
      (Eigen: GeneralizedSelfAdjointEigenSolver<MatrixXd>(K_x, Λ))
3. λ_i are squared modal frequencies (ω_i² = λ_i).
4. Back-transform:           H = V⁻ᵀ = Λ V
5. Damping matrix:           D = H · diag(2 ξ_i √λ_i) · Hᵀ
```
This matches the `.mdl` annotation (`V'ΛV=I`, `H=inv(V')=ΛV`). `ξ_i ≈ 1` (critical).
Near kinematic singularities `J M⁻¹ Jᵀ` is ill-conditioned → damp the inverse
(`+ ε·I`) before inverting.

### 2.2 Full control laws
**Cartesian** (target node):
```
Λ      = (J M⁻¹ Jᵀ)⁻¹
D_x    = FactorizationDamping(K_x, Λ, ξ)              # §2.1
x̃      = pose error (translation + so(3) log of orientation)        # 6×1
ẋ      = J · dq
τ_task = Jᵀ ( -K_x·x̃ - D_x·ẋ + F_ff + Λ·ẍ_d )       # ẍ_d, F_ff optional (0 if pure setpoint)
τ_null = N · ( K_n·(q_null - q) - D_n·dq ),   N = I - Jᵀ (Jᵀ)⁺
τ_d    = τ_task + τ_null + coriolis                   # NO gravity term
return saturateTorqueRate( τ_d )                       # robot adds gravity internally
```
**Joint** (simpler, good first milestone):
```
D_θ  = FactorizationDamping(K_θ, M(q), ξ)
τ_d  = K_θ(θ_d-θ) + D_θ(dθ_d-dθ)
       + enable_acc · M(q)·θ̈_d + τ_ff + coriolis
return saturateTorqueRate(τ_d)
```

---

## 3. Required libfranka interface

Build as a **direct-libfranka torque node** (same pattern as our
`cartesian_impedance_node.cpp`, but torque control). Needed pieces:

### 3.1 Model — `franka::Model model = robot.loadModel();`
| call | returns | use |
|------|---------|-----|
| `model.mass(state)` | `array<double,49>` 7×7 col-major | `M(q)` for Λ and inertia FF |
| `model.coriolis(state)` | `array<double,7>` | add to τ (NOT auto-added) |
| `model.gravity(state)` | `array<double,7>` | usually **unused** — robot auto-compensates in torque mode |
| `model.zeroJacobian(Frame::kEndEffector, state)` | `array<double,42>` 6×7 | base-frame Jacobian `J` |
| `model.bodyJacobian(...)` | `array<double,42>` | if body-frame errors preferred |
| `model.pose(Frame::kEndEffector, state)` | `array<double,16>` | FK if needed |

All arrays are **column-major** → `Eigen::Map<const Matrix<double,7,7>>(mass.data())`.

### 3.2 RobotState fields
`O_T_EE` (current pose), `q`,`dq` (link side), `theta`,`dtheta` (motor side, for the
joint-stiffness term), `tau_J`,`tau_ext_hat_filtered`, `O_F_ext_hat_K` (publish force).

### 3.3 Control loop — torque mode
```cpp
robot.setCollisionBehavior(/* high thresholds, tunable like our existing node */);
// NO setCartesianImpedance / setJointImpedance — we compute torque ourselves.
robot.control([&](const franka::RobotState& s, franka::Duration) -> franka::Torques {
    // ... compute τ_d per §2.2 ...
    std::array<double,7> tau_d_arr;  Eigen::VectorXd::Map(&tau_d_arr[0],7) = tau_d;
    return tau_d_arr;                 // gravity added internally; do NOT add it
});
```
- **Gravity:** auto-compensated by the robot in torque control — never add it (double
  compensation → drift/instability). **Coriolis:** must be added by us.
- **Rate limit:** keep `saturateTorqueRate(τ_d, τ_prev, ΔτMax≈1.0 Nm/ms)` (see Version A),
  or `franka::limitRate(...)`. Optionally pass libfranka's command low-pass cutoff like
  our existing node does.
- Returning a torque that jumps → command discontinuity reflex; rate-limit + start from
  the measured torque on the first cycle.

### 3.4 Eigen
`GeneralizedSelfAdjointEigenSolver` (in `<Eigen/Eigenvalues>`) for §2.1. `M⁻¹` via
`ldlt()`/`inverse()` (M is SPD). Orientation error via quaternion → `AngleAxis` log
(reuse the shortest-path handling already in `cartesian_impedance_node.cpp`).

---

## 4. Suggested build order
1. **Joint** torque impedance with factorization damping (no FF) — validate D(q) alone
   fixes the "oscillate-when-soft / sluggish-when-stiff" problem vs constant D.
2. Add inertia + `tau_ff` feedforward (§2.2 joint).
3. **Cartesian** version: Λ, factorization damping, Jᵀ task torque, nullspace.
4. Add `Λ·ẍ_d` / `F_ff` feedforward → KUKA-FRI-like compliant-and-fast tracking.

Reuse from `cartesian_impedance_node.cpp`: ROS wiring, `~/target_pose` interface,
state/force publishing, reflex classification + debug ring buffer, collision-threshold
params. Only the control callback body changes (built-in impedance → our torque law).

See also memory: `torque-impedance-factorization-damping`,
`real-goal-teleop-force-feedback`, `cartesian-impedance-node-design`.
