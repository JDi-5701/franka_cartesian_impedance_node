// Admittance teleoperation node (direct libfranka, bypasses ros2_control).
//
// The node OWNS the equilibrium (like KUKA FRI): clients command the equilibrium,
// never the measured pose; the measured pose is published as an observation.
//
//   equilibrium (eq)  <- absolute (~/target_pose) OR integrated twist (~/cmd_twist)
//   nominal (nom)      <- eq smoothed by a 2nd-order critically-damped follow filter
//   offset x           <- M*xddot + D*xdot + Kv*x = F_ext   (software compliance)
//   command            =  nom + x   (built-in high-stiffness inner loop tracks it)
//
// Inner loop runs at high stiffness (fast/accurate); compliance is rendered in
// software from the measured external force, decoupling "fast following" from "soft".
//
// Subscribes:
//   ~/target_pose      (PoseStamped, absolute)  -- equilibrium command (policy / goto / teleop)
//   ~/cmd_twist        (Twist, tool-frame vel)  -- for teleop jogging (integrated here)
//   ~/reset            (Bool true)              -- snap equilibrium to measured pose
// Publishes:
//   ~/current_pose       measured TCP pose (observation)
//   ~/equilibrium        current (smoothed) equilibrium the robot is compliant about
//   ~/equilibrium_target raw equilibrium setpoint being converged to
//   ~/ext_wrench         estimated external wrench (base frame)
//   ~/joint_states       q / dq / tau_J

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
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

using std::placeholders::_1;

namespace {

void matrixToPose(const std::array<double, 16>& m, Eigen::Vector3d& p, Eigen::Quaterniond& q) {
  Eigen::Matrix4d T = Eigen::Map<const Eigen::Matrix4d>(m.data());
  p = T.block<3, 1>(0, 3);
  q = Eigen::Quaterniond(T.block<3, 3>(0, 0));
}

std::array<double, 16> poseToMatrix(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  T.block<3, 1>(0, 3) = p;
  std::array<double, 16> m{};
  Eigen::Map<Eigen::Matrix4d>(m.data()) = T;
  return m;
}

}  // namespace

class AdmittanceNode : public rclcpp::Node {
 public:
  AdmittanceNode() : Node("admittance_node") {
    robot_ip_ = declare_parameter<std::string>("robot_ip", "192.168.3.100");
    stiffness_ = declare_parameter<std::vector<double>>(
        "cartesian_stiffness", {2000.0, 2000.0, 2000.0, 150.0, 150.0, 150.0});

    // equilibrium-follow filter (2nd-order, critically damped)
    follow_gain_ = declare_parameter<double>("follow_gain", 600.0);
    follow_max_v_ = declare_parameter<double>("follow_max_velocity", 1.0);
    follow_max_a_ = declare_parameter<double>("follow_max_acceleration", 10.0);
    follow_max_w_ = declare_parameter<double>("follow_max_rot_velocity", 2.0);
    follow_max_wa_ = declare_parameter<double>("follow_max_rot_acceleration", 20.0);

    // virtual admittance (translation): M*xddot + D*xdot + Kv*x = F_ext
    M_ = declare_parameter<double>("virtual_mass", 5.0);
    D_ = declare_parameter<double>("virtual_damping", 14.0);
    Kv_ = declare_parameter<double>("virtual_stiffness", 200.0);
    force_deadband_ = declare_parameter<double>("force_deadband", 2.0);
    force_alpha_ = declare_parameter<double>("force_lowpass_alpha", 0.05);
    force_sign_ = declare_parameter<double>("force_sign", -1.0);
    max_offset_ = declare_parameter<double>("max_offset", 0.15);
    max_off_v_ = declare_parameter<double>("max_offset_velocity", 0.5);
    max_off_a_ = declare_parameter<double>("max_offset_acceleration", 2.0);

    // twist jogging: integrated into the equilibrium; force-limited; staleness timeout
    twist_timeout_ = declare_parameter<double>("twist_timeout", 0.2);
    enable_force_limit_ = declare_parameter<bool>("enable_force_limit", true);
    force_limit_ = declare_parameter<double>("force_limit", 20.0);

    coll_torque_ = declare_parameter<double>("collision_torque_threshold", 20.0);
    coll_force_ = declare_parameter<double>("collision_force_threshold", 30.0);
    joint_prefix_ = declare_parameter<std::string>("joint_prefix", "fr3_joint");
    cutoff_hz_ = declare_parameter<double>("command_cutoff_hz", 30.0);

    dbg_n_ = (size_t)declare_parameter<int>("debug_buffer_samples", 1000);
    dbg_dump_ = (size_t)declare_parameter<int>("debug_dump_samples", 40);
    dbg_.assign(dbg_n_, std::array<double, 11>{});

    // Absolute equilibrium command. Named "target_pose" to match cartesian_impedance_node
    // so the same scripts/teleop work on either controller (unified interface).
    eq_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/target_pose", 1, std::bind(&AdmittanceNode::eqPoseCb, this, _1));
    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "~/cmd_twist", 1, std::bind(&AdmittanceNode::twistCb, this, _1));
    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
        "~/reset", 1, std::bind(&AdmittanceNode::resetCb, this, _1));

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/current_pose", 10);
    eq_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/equilibrium", 10);
    eq_tgt_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/equilibrium_target", 10);
    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("~/ext_wrench", 10);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);

    control_thread_ = std::thread(&AdmittanceNode::controlLoop, this);
  }

  ~AdmittanceNode() override {
    running_ = false;
    if (control_thread_.joinable()) control_thread_.join();
  }

 private:
  // ---- command inputs (callbacks only stash requests; the control loop owns eq) ----
  void eqPoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr m) {
    std::lock_guard<std::mutex> lk(in_mtx_);
    abs_p_ = Eigen::Vector3d(m->pose.position.x, m->pose.position.y, m->pose.position.z);
    abs_q_ = Eigen::Quaterniond(m->pose.orientation.w, m->pose.orientation.x,
                                m->pose.orientation.y, m->pose.orientation.z);
    abs_new_ = true;
  }
  void twistCb(const geometry_msgs::msg::Twist::SharedPtr m) {
    std::lock_guard<std::mutex> lk(in_mtx_);
    twist_v_ = Eigen::Vector3d(m->linear.x, m->linear.y, m->linear.z);
    twist_w_ = Eigen::Vector3d(m->angular.x, m->angular.y, m->angular.z);
    twist_stamp_ = now();
  }
  void resetCb(const std_msgs::msg::Bool::SharedPtr m) {
    if (m->data) reset_req_ = true;
  }

  void controlLoop() {
    try {
      franka::Robot robot(robot_ip_);
      const double t = coll_torque_, f = coll_force_;
      robot.setCollisionBehavior({{t, t, t, t, t, t, t}}, {{t, t, t, t, t, t, t}},
                                 {{f, f, f, f, f, f}}, {{f, f, f, f, f, f}});
      robot.setCartesianImpedance({{stiffness_[0], stiffness_[1], stiffness_[2],
                                    stiffness_[3], stiffness_[4], stiffness_[5]}});
      RCLCPP_INFO(get_logger(), "Connected to %s. Equilibrium-owning admittance teleop active.",
                  robot_ip_.c_str());

      bool initialized = false;
      Eigen::Vector3d eq_p, nom_p, nom_v = Eigen::Vector3d::Zero();
      Eigen::Quaterniond eq_q, nom_q;
      Eigen::Vector3d nom_w = Eigen::Vector3d::Zero();
      Eigen::Vector3d x = Eigen::Vector3d::Zero(), xv = Eigen::Vector3d::Zero();
      Eigen::Vector3d f_filt = Eigen::Vector3d::Zero();

      robot.control([&](const franka::RobotState& state,
                        franka::Duration period) -> franka::CartesianPose {
        double dt = period.toSec();
        if (dt <= 0.0) dt = 0.001;

        Eigen::Vector3d meas_p;
        Eigen::Quaterniond meas_q;
        matrixToPose(state.O_T_EE_c, meas_p, meas_q);

        if (!initialized) {
          eq_p = meas_p; eq_q = meas_q; nom_p = meas_p; nom_q = meas_q;
          initialized = true;
        }

        // ---- reset: snap equilibrium to the measured pose ----
        if (reset_req_.exchange(false)) {
          eq_p = meas_p; eq_q = meas_q;
          nom_p = meas_p; nom_q = meas_q;
          nom_v.setZero(); nom_w.setZero();
          x.setZero(); xv.setZero();
        }

        // ---- absolute equilibrium command (policy / goto) ----
        Eigen::Vector3d tv = Eigen::Vector3d::Zero(), tw = Eigen::Vector3d::Zero();
        bool twist_fresh = false;
        {
          std::lock_guard<std::mutex> lk(in_mtx_);
          if (abs_new_) { eq_p = abs_p_; eq_q = abs_q_; abs_new_ = false; }
          tv = twist_v_; tw = twist_w_;
          twist_fresh = (now() - twist_stamp_).seconds() < twist_timeout_;
        }

        // ---- twist jogging: integrate into the equilibrium (tool-frame velocity) ----
        if (twist_fresh && (tv.norm() > 0.0 || tw.norm() > 0.0)) {
          Eigen::Vector3d base_v = eq_q * tv;  // tool -> base
          if (enable_force_limit_) {           // block motion into a contact (raw O_F_ext)
            for (int i = 0; i < 3; ++i) {
              double fb = state.O_F_ext_hat_K[i];
              if (fb > force_limit_ && base_v(i) > 0) base_v(i) = 0.0;
              else if (fb < -force_limit_ && base_v(i) < 0) base_v(i) = 0.0;
            }
          }
          eq_p += base_v * dt;
          double wn = tw.norm();
          if (wn > 1e-9) eq_q = eq_q * Eigen::Quaterniond(Eigen::AngleAxisd(wn * dt, tw / wn));
          eq_q.normalize();
        }

        // ---- nominal follows equilibrium: 2nd-order critically damped (translation) ----
        double kdt = 2.0 * std::sqrt(follow_gain_);
        Eigen::Vector3d nacc = follow_gain_ * (eq_p - nom_p) - kdt * nom_v;
        if (nacc.norm() > follow_max_a_) nacc = nacc.normalized() * follow_max_a_;
        nom_v += nacc * dt;
        if (nom_v.norm() > follow_max_v_) nom_v = nom_v.normalized() * follow_max_v_;
        nom_p += nom_v * dt;

        // ---- nominal follows equilibrium: orientation ----
        Eigen::Quaterniond dq = eq_q * nom_q.inverse();
        if (dq.w() < 0) dq.coeffs() *= -1.0;
        Eigen::AngleAxisd aa(dq);
        Eigen::Vector3d erot = aa.angle() * aa.axis();
        Eigen::Vector3d wacc = follow_gain_ * erot - kdt * nom_w;
        if (wacc.norm() > follow_max_wa_) wacc = wacc.normalized() * follow_max_wa_;
        nom_w += wacc * dt;
        if (nom_w.norm() > follow_max_w_) nom_w = nom_w.normalized() * follow_max_w_;
        double wn2 = nom_w.norm();
        if (wn2 > 1e-9) nom_q = Eigen::Quaterniond(Eigen::AngleAxisd(wn2 * dt, nom_w / wn2)) * nom_q;
        nom_q.normalize();

        // ---- software compliance: force-driven offset (translation admittance) ----
        Eigen::Vector3d f_raw(state.O_F_ext_hat_K[0], state.O_F_ext_hat_K[1],
                              state.O_F_ext_hat_K[2]);
        f_raw *= force_sign_;
        f_filt += force_alpha_ * (f_raw - f_filt);
        Eigen::Vector3d f_eff = f_filt;
        for (int i = 0; i < 3; ++i) {
          if (std::abs(f_eff(i)) < force_deadband_) f_eff(i) = 0.0;
          else f_eff(i) -= (f_eff(i) > 0 ? 1.0 : -1.0) * force_deadband_;
        }
        Eigen::Vector3d a = (f_eff - D_ * xv - Kv_ * x) / M_;
        if (a.norm() > max_off_a_) a = a.normalized() * max_off_a_;
        xv += a * dt;
        if (xv.norm() > max_off_v_) xv = xv.normalized() * max_off_v_;
        x += xv * dt;
        if (x.norm() > max_offset_) { x = x.normalized() * max_offset_; xv.setZero(); }

        Eigen::Vector3d cmd_p = nom_p + x;

        // ---- debug: command derivatives (what libfranka's continuity checks see) ----
        if (!deriv_init_) { prev_cmd_p_ = cmd_p; prev_cmd_v_.setZero(); deriv_init_ = true; }
        Eigen::Vector3d cmd_v = (cmd_p - prev_cmd_p_) / dt;
        Eigen::Vector3d cmd_a = (cmd_v - prev_cmd_v_) / dt;
        prev_cmd_p_ = cmd_p; prev_cmd_v_ = cmd_v;
        dbg_t_ += dt;
        double vn = cmd_v.norm(), an = cmd_a.norm();
        peak_v_ = std::max(peak_v_, vn);
        peak_a_ = std::max(peak_a_, an);
        dbg_[dbg_i_] = {dbg_t_, cmd_v.x(), cmd_v.y(), cmd_v.z(), cmd_a.x(), cmd_a.y(), cmd_a.z(),
                       f_eff.x(), f_eff.y(), f_eff.z(), x.norm()};
        dbg_i_ = (dbg_i_ + 1) % dbg_n_;
        if (dbg_i_ == 0) dbg_full_ = true;

        // snapshot raw signals so a reflex can be attributed to an exact channel
        for (int i = 0; i < 6; ++i) last_wrench_[i] = state.O_F_ext_hat_K[i];
        for (int j = 0; j < 7; ++j) last_tau_ext_[j] = state.tau_ext_hat_filtered[j];

        publishState(state, meas_p, meas_q, nom_p, nom_q, eq_p, eq_q);

        franka::CartesianPose out(poseToMatrix(cmd_p, nom_q));
        if (!running_ || !rclcpp::ok()) return franka::MotionFinished(out);
        return out;
      },
      // kJointImpedance = stiff default inner loop; software admittance gives compliance.
      franka::ControllerMode::kJointImpedance, true, cutoff_hz_);
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
      cls = "COMMAND DISCONTINUITY -> Franka command-rate limit, triggered by OUR command "
            "being too jerky. Fix: lower follow_gain / raise command_cutoff_hz / smaller twist.";
    else if (reason.find("_reflex") != std::string::npos)
      cls = "COLLISION -> exceeded YOUR collision_*_threshold (see channels below). "
            "Fix: raise the threshold, or interact more softly.";
    else if (reason.find("limit") != std::string::npos || reason.find("violation") != std::string::npos)
      cls = "LIMIT -> Franka HARDWARE limit (joint position/velocity/torque or singularity). "
            "Fix: stay away from limits/singularities, move slower.";
    else
      cls = "OTHER -> see message above.";
    RCLCPP_ERROR(get_logger(), "CLASS: %s", cls.c_str());

    // Cartesian external wrench vs collision_force_threshold (we set all 6 to coll_force_).
    const char* wn[6] = {"Fx[N]", "Fy[N]", "Fz[N]", "Tx[Nm]", "Ty[Nm]", "Tz[Nm]"};
    for (int i = 0; i < 6; ++i) {
      bool hit = std::abs(last_wrench_[i]) > coll_force_;
      RCLCPP_WARN(get_logger(), "  O_F_ext %-6s = % 7.2f   (thr %.1f)%s",
                  wn[i], last_wrench_[i], coll_force_, hit ? "   <== EXCEEDED" : "");
    }
    // Joint external torque vs collision_torque_threshold.
    for (int j = 0; j < 7; ++j) {
      bool hit = std::abs(last_tau_ext_[j]) > coll_torque_;
      RCLCPP_WARN(get_logger(), "  tau_ext J%d   = % 7.2f   (thr %.1f)%s",
                  j + 1, last_tau_ext_[j], coll_torque_, hit ? "   <== EXCEEDED" : "");
    }
    // Our commanded motion peaks vs Franka's hard command limits (approx, informational).
    RCLCPP_WARN(get_logger(),
                "  peak cmd |v| = %.3f m/s (Franka cmd limit ~1.7)   |a| = %.1f m/s^2 (~13)",
                peak_v_, peak_a_);
  }

  void dumpDebug() {
    size_t avail = dbg_full_ ? dbg_n_ : dbg_i_;
    size_t count = std::min(avail, dbg_dump_);
    size_t first = (dbg_i_ + dbg_n_ - count) % dbg_n_;
    RCLCPP_WARN(get_logger(), "---- last %zu cycles before reflex (t | cmd_v | cmd_a | f_eff "
                "| offset) ----", count);
    for (size_t k = 0; k < count; ++k) {
      const auto& r = dbg_[(first + k) % dbg_n_];
      RCLCPP_WARN(get_logger(),
                  "t=%.3f v=[% .3f % .3f % .3f] a=[% 7.1f % 7.1f % 7.1f] f=[% .1f % .1f % .1f] off=%.3f",
                  r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10]);
    }
  }

  static geometry_msgs::msg::PoseStamped toPoseMsg(const rclcpp::Time& stamp,
                                                   const Eigen::Vector3d& p,
                                                   const Eigen::Quaterniond& q) {
    geometry_msgs::msg::PoseStamped m;
    m.header.stamp = stamp;
    m.header.frame_id = "base";
    m.pose.position.x = p.x(); m.pose.position.y = p.y(); m.pose.position.z = p.z();
    m.pose.orientation.w = q.w(); m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y(); m.pose.orientation.z = q.z();
    return m;
  }

  void publishState(const franka::RobotState& state,
                    const Eigen::Vector3d& meas_p, const Eigen::Quaterniond& meas_q,
                    const Eigen::Vector3d& nom_p, const Eigen::Quaterniond& nom_q,
                    const Eigen::Vector3d& eq_p, const Eigen::Quaterniond& eq_q) {
    auto stamp = now();
    pose_pub_->publish(toPoseMsg(stamp, meas_p, meas_q));      // measured (observation)
    eq_pub_->publish(toPoseMsg(stamp, nom_p, nom_q));          // current equilibrium
    eq_tgt_pub_->publish(toPoseMsg(stamp, eq_p, eq_q));        // target equilibrium

    geometry_msgs::msg::WrenchStamped ws;
    ws.header.stamp = stamp;
    ws.header.frame_id = "base";
    ws.wrench.force.x = state.O_F_ext_hat_K[0];
    ws.wrench.force.y = state.O_F_ext_hat_K[1];
    ws.wrench.force.z = state.O_F_ext_hat_K[2];
    ws.wrench.torque.x = state.O_F_ext_hat_K[3];
    ws.wrench.torque.y = state.O_F_ext_hat_K[4];
    ws.wrench.torque.z = state.O_F_ext_hat_K[5];
    wrench_pub_->publish(ws);

    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name.resize(7); js.position.resize(7); js.velocity.resize(7); js.effort.resize(7);
    for (int i = 0; i < 7; ++i) {
      js.name[i] = joint_prefix_ + std::to_string(i + 1);
      js.position[i] = state.q[i];
      js.velocity[i] = state.dq[i];
      js.effort[i] = state.tau_J[i];
    }
    joint_pub_->publish(js);
  }

  std::string robot_ip_;
  std::vector<double> stiffness_;
  double follow_gain_, follow_max_v_, follow_max_a_, follow_max_w_, follow_max_wa_;
  double M_, D_, Kv_;
  double force_deadband_, force_alpha_, force_sign_, max_offset_, max_off_v_, max_off_a_;
  double twist_timeout_, force_limit_;
  bool enable_force_limit_;
  double coll_torque_, coll_force_, cutoff_hz_;
  std::string joint_prefix_;

  // command inputs (mutex-protected)
  std::mutex in_mtx_;
  Eigen::Vector3d abs_p_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond abs_q_{Eigen::Quaterniond::Identity()};
  bool abs_new_{false};
  Eigen::Vector3d twist_v_{Eigen::Vector3d::Zero()}, twist_w_{Eigen::Vector3d::Zero()};
  rclcpp::Time twist_stamp_{0, 0, RCL_ROS_TIME};
  std::atomic<bool> reset_req_{false};

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

  std::atomic<bool> running_{true};
  std::thread control_thread_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr eq_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr eq_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr eq_tgt_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdmittanceNode>());
  rclcpp::shutdown();
  return 0;
}
