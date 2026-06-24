// Direct-libfranka Cartesian impedance node (bypasses ros2_control / franka_hardware).
// Uses the robot's BUILT-IN Cartesian impedance controller via libfranka:
//   - setCartesianImpedance(...) sets the stiffness (compliance)
//   - robot.control(cartesian_pose_callback) runs the internal Cartesian impedance ctrl
// You publish a target TCP pose; the robot tracks it compliantly. IK/dynamics/gravity/
// friction are all handled inside the robot. We only do rate-limited interpolation.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
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
#include <geometry_msgs/msg/wrench_stamped.hpp>

using std::placeholders::_1;

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
    // 2nd-order filter stiffness (higher = snappier tracking; damping is auto-critical).
    trans_gain_ = declare_parameter<double>("trans_filter_gain", 100.0);
    rot_gain_ = declare_parameter<double>("rot_filter_gain", 100.0);
    // Collision reflex thresholds (higher = push harder before it stops).
    coll_torque_ = declare_parameter<double>("collision_torque_threshold", 50.0);  // Nm
    coll_force_ = declare_parameter<double>("collision_force_threshold", 80.0);    // N
    // libfranka command low-pass (Hz): smooths command jerk -> fewer discontinuity reflexes.
    cutoff_hz_ = declare_parameter<double>("command_cutoff_hz", 30.0);
    // Debug: ring buffer of command derivatives; on a reflex the cause is classified and
    // the raw external wrench / joint external torque are logged vs thresholds.
    dbg_n_ = (size_t)declare_parameter<int>("debug_buffer_samples", 1000);
    dbg_dump_ = (size_t)declare_parameter<int>("debug_dump_samples", 40);
    dbg_.assign(dbg_n_, std::array<double, 11>{});

    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/target_pose", 1, std::bind(&CartesianImpedanceNode::targetCb, this, _1));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/current_pose", 10);
    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("~/ext_wrench", 10);

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
      Eigen::Vector3d cmd_p;       // current commanded translation
      Eigen::Vector3d cmd_v = Eigen::Vector3d::Zero();  // current commanded translational velocity
      Eigen::Quaterniond cmd_q;    // current commanded orientation
      Eigen::Vector3d cmd_w = Eigen::Vector3d::Zero();  // current commanded angular velocity

      robot.control([&](const franka::RobotState& state,
                        franka::Duration period) -> franka::CartesianPose {
        if (!initialized) {
          matrixToPose(state.O_T_EE_c, cmd_p, cmd_q);
          {
            std::lock_guard<std::mutex> lk(target_mtx_);
            target_p_ = cmd_p;
            target_q_ = cmd_q;
          }
          initialized = true;
        }

        publishState(state);

        // Fetch latest target.
        Eigen::Vector3d tp;
        Eigen::Quaterniond tq;
        {
          std::lock_guard<std::mutex> lk(target_mtx_);
          tp = target_p_;
          tq = target_q_;
        }

        double dt = period.toSec();
        if (dt <= 0.0) dt = 0.001;

        // Velocity- AND acceleration-limited interpolation toward target (trapezoidal
        // profile). Smooth start/stop -> no velocity/acceleration discontinuity reflex.
        // Critically-damped 2nd-order filter toward target: smooth, no overshoot, and
        // no near-target chatter (bounded gain, unlike a sqrt braking law). kd=2*sqrt(kp).
        double kd_t = 2.0 * std::sqrt(trans_gain_);
        Eigen::Vector3d acc = trans_gain_ * (tp - cmd_p) - kd_t * cmd_v;
        if (acc.norm() > max_a_) acc = acc.normalized() * max_a_;        // accel safety clamp
        cmd_v += acc * dt;
        if (cmd_v.norm() > max_v_) cmd_v = cmd_v.normalized() * max_v_;  // velocity safety clamp
        cmd_p += cmd_v * dt;

        // orientation: velocity- AND acceleration-limited (same scheme as position)
        Eigen::Quaterniond dq = tq * cmd_q.inverse();
        if (dq.w() < 0) dq.coeffs() *= -1.0;             // shortest path
        Eigen::AngleAxisd aa(dq);
        Eigen::Vector3d err_rot = aa.angle() * aa.axis();  // rotation-vector error
        double kd_r = 2.0 * std::sqrt(rot_gain_);
        Eigen::Vector3d ang_acc = rot_gain_ * err_rot - kd_r * cmd_w;
        if (ang_acc.norm() > max_walpha_) ang_acc = ang_acc.normalized() * max_walpha_;
        cmd_w += ang_acc * dt;
        if (cmd_w.norm() > max_w_) cmd_w = cmd_w.normalized() * max_w_;
        double wn = cmd_w.norm();
        if (wn > 1e-9) {
          cmd_q = Eigen::Quaterniond(Eigen::AngleAxisd(wn * dt, cmd_w / wn)) * cmd_q;
        }
        cmd_q.normalize();

        // debug: command derivatives (what libfranka's continuity checks see) + raw
        // signals so a reflex can be attributed to an exact channel.
        if (!deriv_init_) { prev_cmd_p_ = cmd_p; prev_cmd_v_.setZero(); deriv_init_ = true; }
        Eigen::Vector3d d_v = (cmd_p - prev_cmd_p_) / dt;
        Eigen::Vector3d d_a = (d_v - prev_cmd_v_) / dt;
        prev_cmd_p_ = cmd_p; prev_cmd_v_ = d_v;
        dbg_t_ += dt;
        peak_v_ = std::max(peak_v_, d_v.norm());
        peak_a_ = std::max(peak_a_, d_a.norm());
        dbg_[dbg_i_] = {dbg_t_, d_v.x(), d_v.y(), d_v.z(), d_a.x(), d_a.y(), d_a.z(),
                       state.O_F_ext_hat_K[0], state.O_F_ext_hat_K[1], state.O_F_ext_hat_K[2],
                       (tp - cmd_p).norm()};
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
  }

  void dumpDebug() {
    size_t avail = dbg_full_ ? dbg_n_ : dbg_i_;
    size_t count = std::min(avail, dbg_dump_);
    size_t first = (dbg_i_ + dbg_n_ - count) % dbg_n_;
    RCLCPP_WARN(get_logger(), "---- last %zu cycles (t | cmd_v | cmd_a | O_F_ext xyz | "
                "cmd-target lag) ----", count);
    for (size_t k = 0; k < count; ++k) {
      const auto& r = dbg_[(first + k) % dbg_n_];
      RCLCPP_WARN(get_logger(),
                  "t=%.3f v=[% .3f % .3f % .3f] a=[% 7.1f % 7.1f % 7.1f] F=[% .1f % .1f % .1f] lag=%.3f",
                  r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10]);
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
  }

  std::string robot_ip_;
  std::vector<double> stiffness_;
  double max_v_, max_a_, max_w_, max_walpha_;
  double trans_gain_, rot_gain_;
  double coll_torque_, coll_force_, cutoff_hz_;

  // debug
  std::vector<std::array<double, 11>> dbg_;
  size_t dbg_n_{1000}, dbg_i_{0}, dbg_dump_{40};
  bool dbg_full_{false};
  double dbg_t_{0.0};
  Eigen::Vector3d prev_cmd_p_{Eigen::Vector3d::Zero()}, prev_cmd_v_{Eigen::Vector3d::Zero()};
  bool deriv_init_{false};
  double peak_v_{0.0}, peak_a_{0.0};
  std::array<double, 6> last_wrench_{};
  std::array<double, 7> last_tau_ext_{};

  std::mutex target_mtx_;
  Eigen::Vector3d target_p_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond target_q_{Eigen::Quaterniond::Identity()};
  bool have_target_{false};

  std::atomic<bool> running_{true};
  std::thread control_thread_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianImpedanceNode>());
  rclcpp::shutdown();
  return 0;
}
