// Direct-libfranka Cartesian impedance node (bypasses ros2_control / franka_hardware).
// Uses the robot's BUILT-IN Cartesian impedance controller via libfranka:
//   - setCartesianImpedance(...) sets the stiffness (compliance)
//   - robot.control(cartesian_pose_callback) runs the internal Cartesian impedance ctrl
// You publish a target TCP pose; the robot tracks it compliantly. IK/dynamics/gravity/
// friction are all handled inside the robot. We only do rate-limited interpolation.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <franka_cartesian_impedance_msgs/srv/go_to_pose.hpp>
#include <franka_cartesian_impedance_msgs/msg/control_state.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace {

// Convert a column-major 4x4 homogeneous transform to (translation, quaternion).
void matrixToPose(const std::array<double, 16>& m, Eigen::Vector3d& p, Eigen::Quaterniond& q) {
  Eigen::Matrix4d T = Eigen::Map<const Eigen::Matrix4d>(m.data());  // column-major
  p = T.block<3, 1>(0, 3);
  q = Eigen::Quaterniond(T.block<3, 3>(0, 0));
}

// Convert (translation, quaternion) to a column-major 4x4 homogeneous transform.
std::array<double, 16> poseToMatrix(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  T.block<3, 1>(0, 3) = p;
  std::array<double, 16> m{};
  Eigen::Map<Eigen::Matrix4d>(m.data()) = T;  // column-major
  return m;
}

}  // namespace

class CartesianImpedanceNode : public rclcpp::Node {
 public:
  CartesianImpedanceNode() : Node("cartesian_impedance_node") {
    robot_ip_ = declare_parameter<std::string>("robot_ip", "192.168.3.100");
    stiffness_ = declare_parameter<std::vector<double>>(
        "cartesian_stiffness", {200.0, 200.0, 200.0, 20.0, 20.0, 20.0});
    // Per-cycle (1ms) limits to keep the commanded pose smooth -> no reflex stops.
    max_v_ = declare_parameter<double>("max_translational_velocity", 0.05);  // m/s
    max_a_ = declare_parameter<double>("max_translational_acceleration", 0.5);  // m/s^2
    max_w_ = declare_parameter<double>("max_rotational_velocity", 0.5);      // rad/s
    max_walpha_ = declare_parameter<double>("max_rotational_acceleration", 5.0);  // rad/s^2
    // Jerk clamps: bound the CHANGE of the commanded accel per cycle. The amplitude
    // clamps above do NOT bound jerk: a stepped target (30 Hz policy targets can jump
    // >1 cm = max_a/gain between messages) saturates the accel clamp in ONE 1ms cycle
    // -> accel step -> *_acceleration_discontinuity reflex. Franka's own limit is
    // ~4500 m/s^3; stay well under. Inactive during smooth teleop tracking.
    max_j_ = declare_parameter<double>("max_translational_jerk", 1000.0);   // m/s^3
    max_jw_ = declare_parameter<double>("max_rotational_jerk", 1000.0);     // rad/s^3
    // 2nd-order filter stiffness (higher = snappier tracking; damping is auto-critical).
    trans_gain_ = declare_parameter<double>("trans_filter_gain", 100.0);
    rot_gain_ = declare_parameter<double>("rot_filter_gain", 100.0);
    // Collision reflex thresholds (higher = push harder before it stops).
    coll_torque_ = declare_parameter<double>("collision_torque_threshold", 50.0);  // Nm
    coll_force_ = declare_parameter<double>("collision_force_threshold", 80.0);    // N
    // libfranka command low-pass (Hz): smooths command jerk -> fewer discontinuity reflexes.
    cutoff_hz_ = declare_parameter<double>("command_cutoff_hz", 30.0);
    // Homing (~/go_pose, ~/go_home): slow, safe creep to a commanded pose, ignoring target_pose.
    homing_velocity_ = declare_parameter<double>("homing_velocity", 0.08);          // m/s
    homing_pos_tol_ = declare_parameter<double>("homing_position_tolerance", 0.004);  // m
    homing_rot_tol_ = declare_parameter<double>("homing_rotation_tolerance", 0.02);   // rad
    homing_timeout_ = declare_parameter<double>("homing_timeout", 20.0);             // s
    // GUARD: after homing (and at startup) the controller IGNORES target_pose until an
    // incoming target comes within these tolerances of the measured pose -> then resumes
    // following it. Safe hand-over without any external lock; clients just keep publishing.
    guard_pos_tol_ = declare_parameter<double>("guard_position_tolerance", 0.10);          // m
    guard_rot_tol_ = declare_parameter<double>("guard_rotation_tolerance_deg", 20.0)
                     * M_PI / 180.0;                                                       // rad
    // Default home pose [x, y, z, qx, qy, qz, qw] (base frame). Used by ~/go_home (Trigger)
    // and by ~/go_pose when its request carries no pose (empty / zero quaternion).
    auto hp = declare_parameter<std::vector<double>>(
        "home_pose", {0.3, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0});
    if (hp.size() == 7) {
      home_p_ = Eigen::Vector3d(hp[0], hp[1], hp[2]);
      home_q_ = Eigen::Quaterniond(hp[6], hp[3], hp[4], hp[5]);  // (w, x, y, z)
      home_pose_valid_ = home_q_.norm() > 1e-6;
      if (home_pose_valid_) home_q_.normalize();
    } else {
      RCLCPP_WARN(get_logger(), "home_pose must have 7 elements [x y z qx qy qz qw]; "
                  "got %zu -> ~/go_home and empty ~/go_pose will be rejected", hp.size());
    }
    // Debug: ring buffer of command derivatives; on a reflex the cause is classified and
    // the raw external wrench / joint external torque are logged vs thresholds.
    dbg_n_ = (size_t)declare_parameter<int>("debug_buffer_samples", 1000);
    dbg_dump_ = (size_t)declare_parameter<int>("debug_dump_samples", 40);
    dbg_.assign(dbg_n_, std::array<double, 13>{});
    // Joint name prefix for ~/joint_states (fr3_joint1..7); matches admittance_node.
    joint_prefix_ = declare_parameter<std::string>("joint_prefix", "fr3_joint");

    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/target_pose", 1, std::bind(&CartesianImpedanceNode::targetCb, this, _1));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/current_pose", 10);
    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("~/ext_wrench", 10);
    // Full robot state for downstream (RViz / LeRobot recording / policies).
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("~/ee_twist", 10);
    desired_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/desired_pose", 10);

    // ~/go_pose (GoToPose): go to an arbitrary commanded pose (custom interface).
    go_pose_srv_ = create_service<franka_cartesian_impedance_msgs::srv::GoToPose>(
        "~/go_pose", std::bind(&CartesianImpedanceNode::goPoseCb, this, _1, _2));
    // ~/go_home (std_srvs/Trigger): go to the configured home_pose. Standard type -> callable
    // from any PC (RoboStack operator side) without building this package's custom srv.
    go_home_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/go_home", std::bind(&CartesianImpedanceNode::goHomeCb, this, _1, _2));

    // Advisory control-mode status (TOPIC/HOMING/GUARD + target gap). Latched so a late
    // subscriber (GUI) immediately sees the current mode. Read-only for everyone else.
    control_state_pub_ = create_publisher<franka_cartesian_impedance_msgs::msg::ControlState>(
        "~/control_state", rclcpp::QoS(1).transient_local());

    control_thread_ = std::thread(&CartesianImpedanceNode::controlLoop, this);
  }

  ~CartesianImpedanceNode() override {
    running_ = false;
    if (control_thread_.joinable()) control_thread_.join();
  }

 private:
  void targetCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(target_mtx_);
    target_p_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    target_q_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                   msg->pose.orientation.y, msg->pose.orientation.z);
    have_target_ = true;
  }

  // Shared homing motion: drive the equilibrium to (gp, gq) at velocity cap v, ignoring
  // target_pose until reached. Blocks until reached (or timeout). The control loop does the
  // motion and signals completion via homing_cv_; on success it latches target_ = goal so
  // the arm holds there until a teleop/policy re-takes over. Returns success + fills msg.
  bool driveToPose(const Eigen::Vector3d& gp, const Eigen::Quaterniond& gq, double v,
                   std::string& msg) {
    {
      std::lock_guard<std::mutex> lk(homing_mtx_);
      homing_p_ = gp;
      homing_q_ = gq.normalized();
      homing_v_eff_ = v > 0.0 ? v : homing_velocity_;
      homing_done_ = false;
      homing_fresh_ = true;   // control loop re-seeds the creep setpoint at the cmd pose
      homing_active_ = true;
    }
    RCLCPP_INFO(get_logger(), "homing: to [% .3f % .3f % .3f] @ %.2f m/s",
                gp.x(), gp.y(), gp.z(), homing_v_eff_);
    std::unique_lock<std::mutex> lk(homing_mtx_);
    const bool ok = homing_cv_.wait_for(
        lk, std::chrono::duration<double>(homing_timeout_), [this] { return homing_done_; });
    if (!ok) homing_active_ = false;  // give up; the loop falls back to target_ (last cmd)
    msg = ok ? "reached home pose" : "homing timeout / not reached";
    RCLCPP_INFO(get_logger(), "homing: %s", msg.c_str());
    return ok;
  }

  // ~/go_pose (GoToPose): drive to the requested pose. An empty request (zero quaternion)
  // falls back to the configured default home_pose.
  void goPoseCb(
      const std::shared_ptr<franka_cartesian_impedance_msgs::srv::GoToPose::Request> req,
      std::shared_ptr<franka_cartesian_impedance_msgs::srv::GoToPose::Response> resp) {
    Eigen::Vector3d gp(req->pose.position.x, req->pose.position.y, req->pose.position.z);
    Eigen::Quaterniond gq(req->pose.orientation.w, req->pose.orientation.x,
                          req->pose.orientation.y, req->pose.orientation.z);
    if (gq.norm() < 1e-6) {  // no pose in request -> default home_pose
      if (!home_pose_valid_) {
        resp->success = false;
        resp->message = "no pose in request and no valid default home_pose param";
        return;
      }
      gp = home_p_;
      gq = home_q_;
      RCLCPP_INFO(get_logger(), "go_pose: empty request -> using default home_pose");
    }
    resp->success = driveToPose(gp, gq, req->max_velocity, resp->message);
  }

  // ~/go_home (std_srvs/Trigger): standard, no custom type needed -> callable from any machine
  // (e.g. the RoboStack operator PC) with zero custom packages. Always drives to home_pose.
  void goHomeCb(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
              std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
    if (!home_pose_valid_) {
      resp->success = false;
      resp->message = "no valid default home_pose param";
      return;
    }
    resp->success = driveToPose(home_p_, home_q_, homing_velocity_, resp->message);
  }

  void controlLoop() {
    try {
      franka::Robot robot(robot_ip_);

      // Collision reflex thresholds (tunable so light/hand contact does not trip a reflex).
      const double t = coll_torque_;
      const double f = coll_force_;
      robot.setCollisionBehavior(
          {{t, t, t, t, t, t, t}}, {{t, t, t, t, t, t, t}},
          {{f, f, f, f, f, f}}, {{f, f, f, f, f, f}});

      // THE point of this node: enable the built-in Cartesian impedance controller.
      robot.setCartesianImpedance(
          {{stiffness_[0], stiffness_[1], stiffness_[2], stiffness_[3], stiffness_[4], stiffness_[5]}});

      RCLCPP_INFO(get_logger(), "Connected to %s, Cartesian stiffness set. Starting control.",
                  robot_ip_.c_str());

      bool initialized = false;
      int status_ctr = 0;          // throttles ~/control_state publishing
      Eigen::Vector3d cmd_p;       // current commanded translation
      Eigen::Vector3d cmd_v = Eigen::Vector3d::Zero();  // current commanded translational velocity
      Eigen::Quaterniond cmd_q;    // current commanded orientation
      Eigen::Vector3d cmd_w = Eigen::Vector3d::Zero();  // current commanded angular velocity
      Eigen::Vector3d setp_p = Eigen::Vector3d::Zero();       // homing creep setpoint
      Eigen::Quaterniond setp_q = Eigen::Quaterniond::Identity();
      Eigen::Vector3d cmd_a = Eigen::Vector3d::Zero();   // last commanded accel (jerk clamp state)
      Eigen::Vector3d cmd_aw = Eigen::Vector3d::Zero();  // last commanded angular accel

      robot.control([&](const franka::RobotState& state,
                        franka::Duration period) -> franka::CartesianPose {
        if (!initialized) {
          matrixToPose(state.O_T_EE_c, cmd_p, cmd_q);
          {
            std::lock_guard<std::mutex> lk(target_mtx_);
            target_p_ = cmd_p;
            target_q_ = cmd_q;
          }
          hold_p_ = cmd_p;          // launch in GUARD, parked at the start pose, until a
          hold_q_ = cmd_q;          // near target_pose arrives (guard_active_ defaults true)
          initialized = true;
        }

        publishState(state);

        // measured pose (for the GUARD gap check + status)
        Eigen::Vector3d meas_p;
        Eigen::Quaterniond meas_q;
        matrixToPose(state.O_T_EE, meas_p, meas_q);

        // ---- control-mode arbitration: HOMING > GUARD > TOPIC ----------------------------
        // HOMING (a go_* srv) overrides target_pose and creeps to the goal. After homing (and
        // at startup) we sit in GUARD: hold the parked pose and IGNORE target_pose until an
        // incoming target comes within (guard_pos_tol_, guard_rot_tol_) of the measured pose,
        // then resume TOPIC. Safe hand-over is the controller's job; clients just publish.
        Eigen::Vector3d tp;
        Eigen::Quaterniond tq;
        double vcap = max_v_;
        const char* mode_str = "TOPIC";
        bool homing = false;
        Eigen::Vector3d goal_p;
        Eigen::Quaterniond goal_q;
        double homing_v = 0.0;
        {
          std::lock_guard<std::mutex> lk(homing_mtx_);
          homing = homing_active_;
          if (homing) {
            goal_p = homing_p_;
            goal_q = homing_q_;
            homing_v = homing_v_eff_;
            if (homing_fresh_) {   // new homing request: creep starts at the current command
              setp_p = cmd_p;
              setp_q = cmd_q;
              homing_fresh_ = false;
            }
            vcap = std::min(max_v_, homing_v_eff_);
          }
        }
        // gap between the latest target_pose and the measured pose (guard + status readout)
        double tgt_pos_err, tgt_rot_err;
        bool has_tgt;
        {
          std::lock_guard<std::mutex> lk(target_mtx_);
          has_tgt = have_target_;
          tgt_pos_err = (target_p_ - meas_p).norm();
          Eigen::Quaterniond dqt = target_q_ * meas_q.inverse();
          if (dqt.w() < 0) dqt.coeffs() *= -1.0;
          tgt_rot_err = 2.0 * std::acos(std::min(1.0, std::abs(dqt.w())));
        }
        if (homing) {
          mode_str = "HOMING";
        } else if (guard_active_) {
          // release the guard once a fresh target is close enough to the current pose
          if (has_tgt && tgt_pos_err < guard_pos_tol_ && tgt_rot_err < guard_rot_tol_) {
            guard_active_ = false;
            RCLCPP_INFO(get_logger(), "GUARD released -> TOPIC control");
          }
        }
        if (!homing && guard_active_) {
          tp = hold_p_;             // hold the parked pose; ignore target_pose
          tq = hold_q_;
          mode_str = "GUARD";
        } else if (!homing) {
          std::lock_guard<std::mutex> lk(target_mtx_);
          tp = target_p_;
          tq = target_q_;
        }

        double dt = period.toSec();
        if (dt <= 0.0) dt = 0.001;

        // HOMING setpoint creep: do NOT feed the far goal to the tracking filter directly —
        // a 0.3 m target jump saturates the accel clamp in one 1ms cycle (accel step ->
        // *_acceleration_discontinuity reflex). Instead move an internal setpoint toward
        // the goal at exactly the requested velocity and track THAT: the filter then only
        // ever sees a millimeter-scale moving target, same as the verified teleop stream.
        if (homing) {
          const double step = homing_v * dt;
          Eigen::Vector3d d = goal_p - setp_p;
          const double dist = d.norm();
          Eigen::Quaterniond dqs = goal_q * setp_q.inverse();
          if (dqs.w() < 0) dqs.coeffs() *= -1.0;
          const double ang = 2.0 * std::acos(std::min(1.0, std::abs(dqs.w())));
          if (dist > step) {
            const double frac = step / dist;      // rotation shares the fraction -> both
            setp_p += d * frac;                   // translation and rotation arrive together
            if (ang > 1e-6) setp_q = setp_q.slerp(frac, goal_q);
          } else {
            setp_p = goal_p;
            // residual pure rotation: creep at a slow fixed angular rate
            const double wstep = std::min(0.3, max_w_) * dt;   // rad per cycle
            setp_q = (ang <= wstep) ? goal_q
                                    : setp_q.slerp(wstep / std::max(ang, 1e-9), goal_q);
          }
          setp_q.normalize();
          tp = setp_p;
          tq = setp_q;
        }

        // Velocity- AND acceleration-limited interpolation toward target (trapezoidal
        // profile). Smooth start/stop -> no velocity/acceleration discontinuity reflex.
        // Critically-damped 2nd-order filter toward target: smooth, no overshoot, and
        // no near-target chatter (bounded gain, unlike a sqrt braking law). kd=2*sqrt(kp).
        double kd_t = 2.0 * std::sqrt(trans_gain_);
        Eigen::Vector3d acc = trans_gain_ * (tp - cmd_p) - kd_t * cmd_v;
        if (acc.norm() > max_a_) acc = acc.normalized() * max_a_;        // accel safety clamp
        // jerk clamp: a stepped target saturating the accel clamp must ramp over a few
        // cycles, not step in one (accel step = discontinuity reflex, see param comment)
        Eigen::Vector3d da = acc - cmd_a;
        if (da.norm() > max_j_ * dt) acc = cmd_a + da.normalized() * (max_j_ * dt);
        cmd_a = acc;
        cmd_v += acc * dt;
        // Velocity safety clamp (homing -> slower cap). When the cap SHRINKS mid-motion
        // (TOPIC max_v -> homing_velocity on a go_home/go_pose while the arm still moves),
        // truncating in one 1ms cycle is a velocity step -> discontinuity reflex after the
        // robot-side IK. Decelerate down to the cap at max_a_ instead.
        double vn = cmd_v.norm();
        if (vn > vcap) {
          double vlim = std::max(vcap, vn - max_a_ * dt);
          cmd_v *= vlim / vn;
        }
        cmd_p += cmd_v * dt;

        // orientation: velocity- AND acceleration-limited (same scheme as position)
        Eigen::Quaterniond dq = tq * cmd_q.inverse();
        if (dq.w() < 0) dq.coeffs() *= -1.0;             // shortest path
        Eigen::AngleAxisd aa(dq);
        Eigen::Vector3d err_rot = aa.angle() * aa.axis();  // rotation-vector error
        double kd_r = 2.0 * std::sqrt(rot_gain_);
        Eigen::Vector3d ang_acc = rot_gain_ * err_rot - kd_r * cmd_w;
        if (ang_acc.norm() > max_walpha_) ang_acc = ang_acc.normalized() * max_walpha_;
        Eigen::Vector3d daw = ang_acc - cmd_aw;   // jerk clamp (same as translation)
        if (daw.norm() > max_jw_ * dt) ang_acc = cmd_aw + daw.normalized() * (max_jw_ * dt);
        cmd_aw = ang_acc;
        cmd_w += ang_acc * dt;
        if (cmd_w.norm() > max_w_) cmd_w = cmd_w.normalized() * max_w_;
        double wn = cmd_w.norm();
        if (wn > 1e-9) {
          cmd_q = Eigen::Quaterniond(Eigen::AngleAxisd(wn * dt, cmd_w / wn)) * cmd_q;
        }
        cmd_q.normalize();

        // homing complete? (close in position + orientation + nearly stopped) -> latch the
        // home as the standing target and signal the waiting service call.
        if (homing) {
          // completion is measured against the GOAL, not the creeping setpoint (the
          // setpoint starts AT the command pose -> comparing against it fires instantly)
          double perr = (goal_p - cmd_p).norm();
          Eigen::Quaterniond dqh = goal_q * cmd_q.inverse();
          if (dqh.w() < 0) dqh.coeffs() *= -1.0;
          double rerr = 2.0 * std::acos(std::min(1.0, std::abs(dqh.w())));
          if (perr < homing_pos_tol_ && rerr < homing_rot_tol_ && cmd_v.norm() < 0.005) {
            {
              std::lock_guard<std::mutex> lk(target_mtx_);
              target_p_ = goal_p;
              target_q_ = goal_q;
              have_target_ = true;
            }
            {
              std::lock_guard<std::mutex> lk(homing_mtx_);
              homing_active_ = false;
              homing_done_ = true;
            }
            hold_p_ = goal_p;        // park here and GUARD until a near target_pose arrives
            hold_q_ = goal_q;        // (else teleop's stale equilibrium would yank the arm)
            guard_active_ = true;
            homing_cv_.notify_all();
          }
        }

        // publish advisory control-mode status at ~20 Hz (state + gap to the latest target).
        if (++status_ctr >= 50) {
          status_ctr = 0;
          franka_cartesian_impedance_msgs::msg::ControlState cs;
          cs.state = mode_str;
          cs.position_error = tgt_pos_err;
          cs.orientation_error = tgt_rot_err * 180.0 / M_PI;
          control_state_pub_->publish(cs);
        }

        // debug: command derivatives on BOTH channels (what libfranka's continuity
        // checks see) + raw signals, so a reflex can be attributed to an exact channel.
        if (!deriv_init_) {
          prev_cmd_p_ = cmd_p; prev_cmd_v_.setZero();
          prev_cmd_q_ = cmd_q; prev_cmd_w_.setZero();
          deriv_init_ = true;
        }
        Eigen::Vector3d d_v = (cmd_p - prev_cmd_p_) / dt;
        Eigen::Vector3d d_a = (d_v - prev_cmd_v_) / dt;
        Eigen::Quaterniond dq_dbg = cmd_q * prev_cmd_q_.inverse();
        if (dq_dbg.w() < 0) dq_dbg.coeffs() *= -1.0;
        Eigen::AngleAxisd aa_dbg(dq_dbg);
        Eigen::Vector3d d_w = aa_dbg.angle() * aa_dbg.axis() / dt;   // commanded ang vel
        Eigen::Vector3d d_alpha = (d_w - prev_cmd_w_) / dt;          // commanded ang accel
        prev_cmd_p_ = cmd_p; prev_cmd_v_ = d_v;
        prev_cmd_q_ = cmd_q; prev_cmd_w_ = d_w;
        Eigen::Quaterniond dq_lag = tq * cmd_q.inverse();
        if (dq_lag.w() < 0) dq_lag.coeffs() *= -1.0;
        double rot_lag_deg =
            2.0 * std::acos(std::min(1.0, std::abs(dq_lag.w()))) * 180.0 / M_PI;
        dbg_t_ += dt;
        peak_v_ = std::max(peak_v_, d_v.norm());
        peak_a_ = std::max(peak_a_, d_a.norm());
        peak_w_ = std::max(peak_w_, d_w.norm());
        peak_alpha_ = std::max(peak_alpha_, d_alpha.norm());
        dbg_[dbg_i_] = {dbg_t_, d_v.x(), d_v.y(), d_v.z(), d_a.norm(),
                       d_w.norm(), d_alpha.norm(),
                       state.O_F_ext_hat_K[0], state.O_F_ext_hat_K[1], state.O_F_ext_hat_K[2],
                       (tp - cmd_p).norm(), rot_lag_deg,
                       homing ? 1.0 : (guard_active_ ? 2.0 : 0.0)};
        dbg_i_ = (dbg_i_ + 1) % dbg_n_;
        if (dbg_i_ == 0) dbg_full_ = true;
        for (int i = 0; i < 6; ++i) last_wrench_[i] = state.O_F_ext_hat_K[i];
        for (int j = 0; j < 7; ++j) last_tau_ext_[j] = state.tau_ext_hat_filtered[j];

        franka::CartesianPose out(poseToMatrix(cmd_p, cmd_q));
        if (!running_ || !rclcpp::ok()) {
          return franka::MotionFinished(out);
        }
        return out;
      },
      // CRITICAL: select the built-in Cartesian impedance controller. Default is
      // kJointImpedance, under which setCartesianImpedance() has NO effect (stiff).
      franka::ControllerMode::kCartesianImpedance, true, cutoff_hz_);
    } catch (const franka::Exception& e) {
      reportReflex(e.what());
      dumpDebug();
    }
    running_ = false;
  }

  // Attribute a reflex to an exact channel: classify the cause, then print the raw
  // external wrench and joint external torque vs the configured thresholds.
  void reportReflex(const std::string& reason) {
    RCLCPP_ERROR(get_logger(), "==== REFLEX: %s ====", reason.c_str());
    std::string cls;
    if (reason.find("discontinuity") != std::string::npos)
      cls = "COMMAND DISCONTINUITY -> Franka command-rate limit, our command too jerky. "
            "Fix: lower trans/rot_filter_gain, raise command_cutoff_hz, slower target.";
    else if (reason.find("velocity_violation") != std::string::npos ||
             reason.find("velocity_limit") != std::string::npos)
      cls = "JOINT VELOCITY LIMIT -> a joint exceeded its max speed (near singularity / "
            "workspace edge, or commanded too fast). Fix: lower max_translational_velocity, "
            "stay away from singularities.";
    else if (reason.find("_reflex") != std::string::npos)
      cls = "COLLISION -> exceeded YOUR collision_*_threshold (see channels below).";
    else if (reason.find("limit") != std::string::npos || reason.find("violation") != std::string::npos)
      cls = "LIMIT -> Franka hardware limit (joint position/torque/singularity).";
    else
      cls = "OTHER -> see message above.";
    RCLCPP_ERROR(get_logger(), "CLASS: %s", cls.c_str());

    const char* wn[6] = {"Fx[N]", "Fy[N]", "Fz[N]", "Tx[Nm]", "Ty[Nm]", "Tz[Nm]"};
    for (int i = 0; i < 6; ++i) {
      bool hit = std::abs(last_wrench_[i]) > coll_force_;
      RCLCPP_WARN(get_logger(), "  O_F_ext %-6s = % 7.2f   (thr %.1f)%s",
                  wn[i], last_wrench_[i], coll_force_, hit ? "   <== EXCEEDED" : "");
    }
    for (int j = 0; j < 7; ++j) {
      bool hit = std::abs(last_tau_ext_[j]) > coll_torque_;
      RCLCPP_WARN(get_logger(), "  tau_ext J%d   = % 7.2f   (thr %.1f)%s",
                  j + 1, last_tau_ext_[j], coll_torque_, hit ? "   <== EXCEEDED" : "");
    }
    RCLCPP_WARN(get_logger(),
                "  peak cmd |v| = %.3f m/s (Franka cmd limit ~1.7)   |a| = %.1f m/s^2 (~13)",
                peak_v_, peak_a_);
    RCLCPP_WARN(get_logger(),
                "  peak cmd |w| = %.3f rad/s (~2.5)   |alpha| = %.1f rad/s^2 (~25)",
                peak_w_, peak_alpha_);
  }

  void dumpDebug() {
    size_t avail = dbg_full_ ? dbg_n_ : dbg_i_;
    size_t count = std::min(avail, dbg_dump_);
    size_t first = (dbg_i_ + dbg_n_ - count) % dbg_n_;
    RCLCPP_WARN(get_logger(), "---- last %zu cycles (t | cmd_v | |a| | |w| |alpha| | "
                "O_F_ext xyz | pos/rot lag | mode 0=TOPIC 1=HOMING 2=GUARD) ----", count);
    for (size_t k = 0; k < count; ++k) {
      const auto& r = dbg_[(first + k) % dbg_n_];
      RCLCPP_WARN(get_logger(),
                  "t=%.3f v=[% .3f % .3f % .3f] |a|=%6.1f |w|=%6.3f |al|=%6.1f "
                  "F=[% .1f % .1f % .1f] lag=%.3f/%5.1fdeg m=%.0f",
                  r[0], r[1], r[2], r[3], r[4], r[5], r[6],
                  r[7], r[8], r[9], r[10], r[11], r[12]);
    }
  }

  void publishState(const franka::RobotState& state) {
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    matrixToPose(state.O_T_EE, p, q);

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = now();
    ps.header.frame_id = "base";
    ps.pose.position.x = p.x();
    ps.pose.position.y = p.y();
    ps.pose.position.z = p.z();
    ps.pose.orientation.w = q.w();
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    pose_pub_->publish(ps);

    geometry_msgs::msg::WrenchStamped ws;
    ws.header = ps.header;
    ws.wrench.force.x = state.O_F_ext_hat_K[0];
    ws.wrench.force.y = state.O_F_ext_hat_K[1];
    ws.wrench.force.z = state.O_F_ext_hat_K[2];
    ws.wrench.torque.x = state.O_F_ext_hat_K[3];
    ws.wrench.torque.y = state.O_F_ext_hat_K[4];
    ws.wrench.torque.z = state.O_F_ext_hat_K[5];
    wrench_pub_->publish(ws);

    // Joint state: q / dq / tau_J in one standard message (RViz / recording / policies).
    sensor_msgs::msg::JointState js;
    js.header.stamp = ps.header.stamp;
    js.name.resize(7); js.position.resize(7); js.velocity.resize(7); js.effort.resize(7);
    for (int i = 0; i < 7; ++i) {
      js.name[i] = joint_prefix_ + std::to_string(i + 1);
      js.position[i] = state.q[i];
      js.velocity[i] = state.dq[i];
      js.effort[i] = state.tau_J[i];
    }
    joint_pub_->publish(js);

    // Desired EE twist in base frame (O_dP_EE_d): [vx vy vz wx wy wz].
    geometry_msgs::msg::TwistStamped ts;
    ts.header = ps.header;
    ts.twist.linear.x = state.O_dP_EE_d[0];
    ts.twist.linear.y = state.O_dP_EE_d[1];
    ts.twist.linear.z = state.O_dP_EE_d[2];
    ts.twist.angular.x = state.O_dP_EE_d[3];
    ts.twist.angular.y = state.O_dP_EE_d[4];
    ts.twist.angular.z = state.O_dP_EE_d[5];
    twist_pub_->publish(ts);

    // Impedance-desired EE pose (O_T_EE_d): compare vs current_pose to read tracking lag.
    Eigen::Vector3d pd;
    Eigen::Quaterniond qd;
    matrixToPose(state.O_T_EE_d, pd, qd);
    geometry_msgs::msg::PoseStamped pds;
    pds.header = ps.header;
    pds.pose.position.x = pd.x();
    pds.pose.position.y = pd.y();
    pds.pose.position.z = pd.z();
    pds.pose.orientation.w = qd.w();
    pds.pose.orientation.x = qd.x();
    pds.pose.orientation.y = qd.y();
    pds.pose.orientation.z = qd.z();
    desired_pose_pub_->publish(pds);
  }

  std::string robot_ip_;
  std::vector<double> stiffness_;
  double max_v_, max_a_, max_w_, max_walpha_, max_j_, max_jw_;
  double trans_gain_, rot_gain_;
  double coll_torque_, coll_force_, cutoff_hz_;
  double homing_velocity_, homing_pos_tol_, homing_rot_tol_, homing_timeout_;
  double guard_pos_tol_, guard_rot_tol_;
  std::string joint_prefix_;

  // debug
  // row: t, dv.xyz, |da|, |w|, |dalpha|, F.xyz, pos_lag, rot_lag_deg, mode(0=TOPIC 1=HOMING 2=GUARD)
  std::vector<std::array<double, 13>> dbg_;
  size_t dbg_n_{1000}, dbg_i_{0}, dbg_dump_{40};
  bool dbg_full_{false};
  double dbg_t_{0.0};
  Eigen::Vector3d prev_cmd_p_{Eigen::Vector3d::Zero()}, prev_cmd_v_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond prev_cmd_q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d prev_cmd_w_{Eigen::Vector3d::Zero()};
  bool deriv_init_{false};
  double peak_v_{0.0}, peak_a_{0.0}, peak_w_{0.0}, peak_alpha_{0.0};
  std::array<double, 6> last_wrench_{};
  std::array<double, 7> last_tau_ext_{};

  std::mutex target_mtx_;
  Eigen::Vector3d target_p_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond target_q_{Eigen::Quaterniond::Identity()};
  bool have_target_{false};

  // homing (~/go_home): control thread reads homing_active_; service waits on homing_cv_.
  std::mutex homing_mtx_;
  std::condition_variable homing_cv_;
  bool homing_active_{false};
  bool homing_done_{false};
  bool homing_fresh_{false};   // set per request; control loop re-seeds the creep setpoint
  Eigen::Vector3d homing_p_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond homing_q_{Eigen::Quaterniond::Identity()};
  double homing_v_eff_{0.08};

  // default home pose for ~/go_home when the request has no pose (see home_pose param)
  Eigen::Vector3d home_p_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond home_q_{Eigen::Quaterniond::Identity()};
  bool home_pose_valid_{false};

  // GUARD: hold this parked pose and ignore target_pose until a near target arrives. Written
  // and read only by the control thread (start-up + homing-complete), so no mutex needed.
  std::atomic<bool> guard_active_{true};   // start guarded at launch
  Eigen::Vector3d hold_p_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond hold_q_{Eigen::Quaterniond::Identity()};

  std::atomic<bool> running_{true};
  std::thread control_thread_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_pub_;
  rclcpp::Service<franka_cartesian_impedance_msgs::srv::GoToPose>::SharedPtr go_pose_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_home_srv_;
  rclcpp::Publisher<franka_cartesian_impedance_msgs::msg::ControlState>::SharedPtr control_state_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // MultiThreaded so the blocking ~/go_home service callback (waits for homing to finish)
  // does not stall the rest of the node's callbacks.
  auto node = std::make_shared<CartesianImpedanceNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
