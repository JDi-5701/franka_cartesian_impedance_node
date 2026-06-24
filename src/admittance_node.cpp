// Admittance teleoperation node (direct libfranka, bypasses ros2_control).
//
// Inner loop: robot's built-in Cartesian impedance at HIGH stiffness -> fast, accurate,
// low-lag pose servo. Outer compliance is rendered IN SOFTWARE from the measured
// external force via a virtual admittance model, decoupling "fast following" from "soft".
//
//   nominal pose  <- smoothly follows the teleop target (~/target_pose)
//   offset x      <- M*xddot + D*xdot + Kv*x = F_ext      (force-driven compliance)
//   command       =  nominal + x   (orientation follows the target)
//
// Publishes everything teleoperation needs:
//   ~/current_pose    measured TCP pose (base frame)        [PoseStamped]
//   ~/commanded_pose  pose actually sent to the robot       [PoseStamped]
//   ~/ext_wrench      estimated external wrench (base frame) [WrenchStamped]
//   ~/joint_states    q / dq / tau_J (joint-level)          [JointState]

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
#include <sensor_msgs/msg/joint_state.hpp>

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

    // Nominal-follow filter (2nd-order, critically damped) toward the teleop target.
    follow_gain_ = declare_parameter<double>("follow_gain", 400.0);
    follow_max_v_ = declare_parameter<double>("follow_max_velocity", 1.0);        // m/s
    follow_max_a_ = declare_parameter<double>("follow_max_acceleration", 10.0);   // m/s^2
    follow_max_w_ = declare_parameter<double>("follow_max_rot_velocity", 2.0);    // rad/s
    follow_max_wa_ = declare_parameter<double>("follow_max_rot_acceleration", 20.0);

    // Virtual admittance (translation): M*xddot + D*xdot + Kv*x = F_ext.
    M_ = declare_parameter<double>("virtual_mass", 2.0);
    D_ = declare_parameter<double>("virtual_damping", 14.0);
    Kv_ = declare_parameter<double>("virtual_stiffness", 50.0);
    force_deadband_ = declare_parameter<double>("force_deadband", 3.0);
    force_alpha_ = declare_parameter<double>("force_lowpass_alpha", 0.05);
    force_sign_ = declare_parameter<double>("force_sign", -1.0);
    max_offset_ = declare_parameter<double>("max_offset", 0.15);
    max_off_v_ = declare_parameter<double>("max_offset_velocity", 0.3);
    max_off_a_ = declare_parameter<double>("max_offset_acceleration", 2.0);

    coll_torque_ = declare_parameter<double>("collision_torque_threshold", 50.0);
    coll_force_ = declare_parameter<double>("collision_force_threshold", 80.0);
    joint_prefix_ = declare_parameter<std::string>("joint_prefix", "fr3_joint");
    // libfranka command low-pass (Hz): smooths command jerk -> fewer discontinuity reflexes.
    cutoff_hz_ = declare_parameter<double>("command_cutoff_hz", 30.0);
    // Debug: ring-buffer of command derivatives etc.; the last samples are emitted to
    // the ROS2 log on a reflex/exception so you can see what spiked.
    dbg_n_ = (size_t)declare_parameter<int>("debug_buffer_samples", 1000);   // ~1 s at 1 kHz
    dbg_period_ = declare_parameter<int>("debug_print_period_cycles", 2000); // ~2 s status line
    dbg_dump_ = (size_t)declare_parameter<int>("debug_dump_samples", 40);    // logged on reflex
    dbg_.assign(dbg_n_, std::array<double, 11>{});

    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/target_pose", 1, std::bind(&AdmittanceNode::targetCb, this, _1));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/current_pose", 10);
    cmd_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/commanded_pose", 10);
    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("~/ext_wrench", 10);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);

    control_thread_ = std::thread(&AdmittanceNode::controlLoop, this);
  }

  ~AdmittanceNode() override {
    running_ = false;
    if (control_thread_.joinable()) control_thread_.join();
  }

 private:
  void targetCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(target_mtx_);
    target_p_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    target_q_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                   msg->pose.orientation.y, msg->pose.orientation.z);
  }

  void controlLoop() {
    try {
      franka::Robot robot(robot_ip_);
      const double t = coll_torque_, f = coll_force_;
      robot.setCollisionBehavior({{t, t, t, t, t, t, t}}, {{t, t, t, t, t, t, t}},
                                 {{f, f, f, f, f, f}}, {{f, f, f, f, f, f}});
      robot.setCartesianImpedance({{stiffness_[0], stiffness_[1], stiffness_[2],
                                    stiffness_[3], stiffness_[4], stiffness_[5]}});
      RCLCPP_INFO(get_logger(), "Connected to %s. HIGH stiffness + admittance teleop active.",
                  robot_ip_.c_str());

      bool initialized = false;
      Eigen::Vector3d nom_p, nom_v = Eigen::Vector3d::Zero();   // nominal pose + filter velocity
      Eigen::Quaterniond nom_q;
      Eigen::Vector3d nom_w = Eigen::Vector3d::Zero();          // nominal angular velocity
      Eigen::Vector3d x = Eigen::Vector3d::Zero();              // admittance offset
      Eigen::Vector3d xv = Eigen::Vector3d::Zero();             // offset velocity
      Eigen::Vector3d f_filt = Eigen::Vector3d::Zero();

      robot.control([&](const franka::RobotState& state,
                        franka::Duration period) -> franka::CartesianPose {
        double dt = period.toSec();
        if (dt <= 0.0) dt = 0.001;

        if (!initialized) {
          matrixToPose(state.O_T_EE_c, nom_p, nom_q);
          std::lock_guard<std::mutex> lk(target_mtx_);
          target_p_ = nom_p;
          target_q_ = nom_q;
          initialized = true;
        }

        // Latest teleop target.
        Eigen::Vector3d tp;
        Eigen::Quaterniond tq;
        {
          std::lock_guard<std::mutex> lk(target_mtx_);
          tp = target_p_;
          tq = target_q_;
        }

        // --- Nominal follows target: 2nd-order critically damped filter (translation) ---
        double kdt = 2.0 * std::sqrt(follow_gain_);
        Eigen::Vector3d nacc = follow_gain_ * (tp - nom_p) - kdt * nom_v;
        if (nacc.norm() > follow_max_a_) nacc = nacc.normalized() * follow_max_a_;
        nom_v += nacc * dt;
        if (nom_v.norm() > follow_max_v_) nom_v = nom_v.normalized() * follow_max_v_;
        nom_p += nom_v * dt;

        // --- Nominal follows target: orientation ---
        Eigen::Quaterniond dq = tq * nom_q.inverse();
        if (dq.w() < 0) dq.coeffs() *= -1.0;
        Eigen::AngleAxisd aa(dq);
        Eigen::Vector3d erot = aa.angle() * aa.axis();
        double kdr = 2.0 * std::sqrt(follow_gain_);
        Eigen::Vector3d wacc = follow_gain_ * erot - kdr * nom_w;
        if (wacc.norm() > follow_max_wa_) wacc = wacc.normalized() * follow_max_wa_;
        nom_w += wacc * dt;
        if (nom_w.norm() > follow_max_w_) nom_w = nom_w.normalized() * follow_max_w_;
        double wn = nom_w.norm();
        if (wn > 1e-9) nom_q = Eigen::Quaterniond(Eigen::AngleAxisd(wn * dt, nom_w / wn)) * nom_q;
        nom_q.normalize();

        // --- Force-driven compliant offset (translation admittance) ---
        Eigen::Vector3d f_raw(state.O_F_ext_hat_K[0], state.O_F_ext_hat_K[1],
                              state.O_F_ext_hat_K[2]);
        f_raw *= force_sign_;
        f_filt += force_alpha_ * (f_raw - f_filt);
        Eigen::Vector3d f_eff = f_filt;
        for (int i = 0; i < 3; ++i) {
          if (std::abs(f_eff(i)) < force_deadband_)
            f_eff(i) = 0.0;
          else
            f_eff(i) -= (f_eff(i) > 0 ? 1.0 : -1.0) * force_deadband_;
        }
        Eigen::Vector3d a = (f_eff - D_ * xv - Kv_ * x) / M_;
        if (a.norm() > max_off_a_) a = a.normalized() * max_off_a_;
        xv += a * dt;
        if (xv.norm() > max_off_v_) xv = xv.normalized() * max_off_v_;
        x += xv * dt;
        if (x.norm() > max_offset_) {
          x = x.normalized() * max_offset_;
          xv.setZero();
        }

        Eigen::Vector3d cmd_p = nom_p + x;

        // --- debug: command velocity/acceleration (what libfranka's continuity checks see) ---
        if (!deriv_init_) { prev_cmd_p_ = cmd_p; prev_cmd_v_.setZero(); deriv_init_ = true; }
        Eigen::Vector3d cmd_v = (cmd_p - prev_cmd_p_) / dt;
        Eigen::Vector3d cmd_a = (cmd_v - prev_cmd_v_) / dt;
        prev_cmd_p_ = cmd_p;
        prev_cmd_v_ = cmd_v;
        dbg_t_ += dt;
        double vn = cmd_v.norm(), an = cmd_a.norm();
        peak_v_ = std::max(peak_v_, vn);
        peak_a_ = std::max(peak_a_, an);
        dbg_[dbg_i_] = {dbg_t_, cmd_v.x(), cmd_v.y(), cmd_v.z(), cmd_a.x(), cmd_a.y(), cmd_a.z(),
                       f_eff.x(), f_eff.y(), f_eff.z(), x.norm()};
        dbg_i_ = (dbg_i_ + 1) % dbg_n_;
        if (dbg_i_ == 0) dbg_full_ = true;
        if (++cycle_ % dbg_period_ == 0) {
          RCLCPP_INFO(get_logger(),
                      "cmd |v|=%.3f |a|=%.1f (peak v=%.3f a=%.1f) |offset|=%.3f |F|=%.1f",
                      vn, an, peak_v_, peak_a_, x.norm(), f_eff.norm());
        }

        publishState(state, cmd_p, nom_q);

        franka::CartesianPose out(poseToMatrix(cmd_p, nom_q));
        if (!running_ || !rclcpp::ok()) return franka::MotionFinished(out);
        return out;
      },
      // kJointImpedance = the (stiff) default inner loop that tracked well; software
      // admittance provides the compliance. limit_rate + low-pass smooth the command.
      franka::ControllerMode::kJointImpedance, true, cutoff_hz_);
    } catch (const franka::Exception& e) {
      RCLCPP_ERROR(get_logger(),
                   "libfranka exception: %s | peak cmd |v|=%.3f m/s  |a|=%.1f m/s^2",
                   e.what(), peak_v_, peak_a_);
      dumpDebug();
    }
    running_ = false;
  }

  // Emit the most recent command-derivative history to the ROS2 log (oldest -> newest)
  // so the spike that caused the reflex is visible without any file.
  void dumpDebug() {
    size_t avail = dbg_full_ ? dbg_n_ : dbg_i_;
    size_t count = std::min(avail, dbg_dump_);
    size_t first = (dbg_i_ + dbg_n_ - count) % dbg_n_;  // 'count' newest, in order
    RCLCPP_WARN(get_logger(), "---- last %zu cycles before reflex (t | cmd_v xyz | cmd_a xyz "
                "| f_eff xyz | offset) ----", count);
    for (size_t k = 0; k < count; ++k) {
      const auto& r = dbg_[(first + k) % dbg_n_];
      RCLCPP_WARN(get_logger(),
                  "t=%.3f v=[% .3f % .3f % .3f] a=[% 7.1f % 7.1f % 7.1f] f=[% .1f % .1f % .1f] off=%.3f",
                  r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10]);
    }
  }

  void publishState(const franka::RobotState& state, const Eigen::Vector3d& cmd_p,
                    const Eigen::Quaterniond& cmd_q) {
    auto stamp = now();

    // Measured TCP pose (actual).
    Eigen::Vector3d mp;
    Eigen::Quaterniond mq;
    matrixToPose(state.O_T_EE, mp, mq);
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = "base";
    ps.pose.position.x = mp.x(); ps.pose.position.y = mp.y(); ps.pose.position.z = mp.z();
    ps.pose.orientation.w = mq.w(); ps.pose.orientation.x = mq.x();
    ps.pose.orientation.y = mq.y(); ps.pose.orientation.z = mq.z();
    pose_pub_->publish(ps);

    // Commanded pose (nominal + offset) sent to the robot.
    geometry_msgs::msg::PoseStamped cs;
    cs.header.stamp = stamp;
    cs.header.frame_id = "base";
    cs.pose.position.x = cmd_p.x(); cs.pose.position.y = cmd_p.y(); cs.pose.position.z = cmd_p.z();
    cs.pose.orientation.w = cmd_q.w(); cs.pose.orientation.x = cmd_q.x();
    cs.pose.orientation.y = cmd_q.y(); cs.pose.orientation.z = cmd_q.z();
    cmd_pub_->publish(cs);

    // External wrench (base frame) for haptic feedback.
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

    // Joint-level state: position / velocity / measured link-side torque.
    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name.resize(7);
    js.position.resize(7);
    js.velocity.resize(7);
    js.effort.resize(7);
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
  double coll_torque_, coll_force_, cutoff_hz_;
  std::string joint_prefix_;

  // debug logging
  std::vector<std::array<double, 11>> dbg_;
  size_t dbg_n_{1000}, dbg_i_{0}, dbg_dump_{40};
  bool dbg_full_{false};
  double dbg_t_{0.0};
  int dbg_period_{2000};
  long cycle_{0};
  Eigen::Vector3d prev_cmd_p_{Eigen::Vector3d::Zero()}, prev_cmd_v_{Eigen::Vector3d::Zero()};
  bool deriv_init_{false};
  double peak_v_{0.0}, peak_a_{0.0};

  std::mutex target_mtx_;
  Eigen::Vector3d target_p_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond target_q_{Eigen::Quaterniond::Identity()};

  std::atomic<bool> running_{true};
  std::thread control_thread_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdmittanceNode>());
  rclcpp::shutdown();
  return 0;
}
