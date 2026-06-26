// Torque-level Cartesian impedance node (direct libfranka, ControllerMode = torque).
// Unlike cartesian_impedance_node.cpp (which uses the robot's BUILT-IN Cartesian
// impedance), this computes the joint torques ourselves so we can do what the built-in
// and the naive franka example cannot:
//   - Factorization Damping Design: D(q) from the generalized eigenproblem of
//     (K_x, Lambda(q)) -> every modal direction is critically damped in the CURRENT
//     configuration (NOT a constant D = 2*xi*sqrt(K) that assumes unit inertia).
//   - Coriolis compensation added explicitly (gravity is auto-compensated by the robot
//     in torque mode -> we must NOT add it).
//   - Nullspace posture control for the 7th DOF.
// See TORQUE_IMPEDANCE_DESIGN.md for the full derivation + reference code locations.
//
// DEBUG: the 1 kHz control loop is RT-critical (no file I/O / logging there). It only
// fills a ring buffer with rich per-cycle samples + self-verification metrics; a
// separate logger thread drains them to a CSV file and prints a throttled summary.
//
// Same topic interface as cartesian_impedance_node so the existing teleop/scripts work:
//   sub  ~/target_pose (PoseStamped)
//   pub  ~/current_pose (PoseStamped), ~/ext_wrench (WrenchStamped), ~/joint_states.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using std::placeholders::_1;

namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;

// FR3 / Panda per-joint torque limits [Nm] (for headroom reporting).
const Vector7d kTauLimit = (Vector7d() << 87, 87, 87, 87, 12, 12, 12).finished();

void matrixToPose(const std::array<double, 16>& m, Eigen::Vector3d& p, Eigen::Quaterniond& q) {
  Eigen::Matrix4d T = Eigen::Map<const Eigen::Matrix4d>(m.data());  // column-major
  p = T.block<3, 1>(0, 3);
  q = Eigen::Quaterniond(T.block<3, 3>(0, 0));
}

// Factorization Damping Design (generalized-eigenvalue / "double diagonalization").
// Solve K v = lambda * Lambda v with V'*Lambda*V = I, V'*K*V = diag(lambda).
// Then H = V^-T = Lambda*V and D = H * diag(2*xi*sqrt(lambda)) * H'.  -> every modal
// direction damped to ratio xi in the current configuration. Matches the reference impl.
Matrix6d factorizationDamping(const Matrix6d& K, const Matrix6d& Lambda, double xi,
                              Matrix6d& V_out, Vector6d& lambda_out) {
  Eigen::GeneralizedSelfAdjointEigenSolver<Matrix6d> ges(K, Lambda);
  lambda_out = ges.eigenvalues();                // omega_i^2 (>= 0)
  V_out = ges.eigenvectors();                    // V' Lambda V = I
  Matrix6d H = Lambda * V_out;                   // = V^-T
  Vector6d d_modal = 2.0 * xi * lambda_out.array().max(0.0).sqrt();
  return H * d_modal.asDiagonal() * H.transpose();
}

// Damped pseudo-inverse of J^T (7x6 -> 6x7):  (J^T)^+ = (J J^T + eps I)^-1 J.
Eigen::Matrix<double, 6, 7> jacobianTransposePinv(const Eigen::Matrix<double, 6, 7>& J,
                                                  double eps) {
  Matrix6d jjt = J * J.transpose();
  jjt.diagonal().array() += eps;
  return jjt.ldlt().solve(J);
}

// One per-cycle debug record. Plain doubles so the logger thread can dump CSV directly.
struct Sample {
  double t = 0.0;            // accumulated control time [s]
  double err_pos = 0.0;      // ||position error|| [m]
  double err_rot = 0.0;      // ||rotation error|| [rad]
  double ee_speed = 0.0;     // ||J*dq|| translational+rot magnitude [mixed]
  double manip = 0.0;        // sqrt(det(J J^T)) manipulability (0 = singular)
  double lambda_min = 0.0;   // generalized eigenvalues = omega^2
  double lambda_max = 0.0;
  double fact_resid = 0.0;   // ||V'*Lambda*V - I||      (should ~0: factorization OK)
  double damp_resid = 0.0;   // ||V'*D*V - diag(2 xi sqrt(lam))|| (should ~0: damping OK)
  double tau_task_norm = 0.0;
  double tau_null_norm = 0.0;
  double coriolis_norm = 0.0;
  double tau_d_norm = 0.0;
  double tau_headroom = 0.0; // min_j (tau_limit_j - |tau_d_j|) [Nm]  (<=0 => at limit)
  int rate_clip = 0;         // # joints clipped by delta_tau_max this cycle
  std::array<double, 7> tau_d{};
  std::array<double, 6> fext{};
};

}  // namespace

class CartesianImpedanceTorqueNode : public rclcpp::Node {
 public:
  CartesianImpedanceTorqueNode() : Node("cartesian_impedance_torque_node") {
    robot_ip_ = declare_parameter<std::string>("robot_ip", "192.168.3.100");
    auto K = declare_parameter<std::vector<double>>(
        "cartesian_stiffness", {400.0, 400.0, 400.0, 30.0, 30.0, 30.0});
    K_.setZero();
    for (int i = 0; i < 6; ++i) K_(i, i) = K[i];
    damping_ratio_ = declare_parameter<double>("damping_ratio", 1.0);   // 1.0 = critical
    nullspace_stiffness_ = declare_parameter<double>("nullspace_stiffness", 10.0);
    trans_gain_ = declare_parameter<double>("trans_filter_gain", 200.0);
    rot_gain_ = declare_parameter<double>("rot_filter_gain", 200.0);
    max_v_ = declare_parameter<double>("max_translational_velocity", 0.5);    // m/s
    max_a_ = declare_parameter<double>("max_translational_acceleration", 2.0);
    max_w_ = declare_parameter<double>("max_rotational_velocity", 1.0);       // rad/s
    max_walpha_ = declare_parameter<double>("max_rotational_acceleration", 2.0);
    delta_tau_max_ = declare_parameter<double>("delta_tau_max", 1.0);
    sing_eps_ = declare_parameter<double>("singularity_eps", 1e-6);
    coll_torque_ = declare_parameter<double>("collision_torque_threshold", 20.0);  // Nm
    coll_force_ = declare_parameter<double>("collision_force_threshold", 30.0);     // N
    joint_prefix_ = declare_parameter<std::string>("joint_prefix", "fr3_joint");
    // --- debug ---
    dbg_n_ = (size_t)declare_parameter<int>("debug_buffer_samples", 4000);
    dbg_file_ = declare_parameter<std::string>("debug_log_file",
                                               "impedance_torque_debug.csv");
    dbg_log_hz_ = declare_parameter<double>("debug_log_rate_hz", 200.0);  // CSV decimation
    dbg_print_hz_ = declare_parameter<double>("debug_print_rate_hz", 1.0);  // console summary
    dbg_.assign(dbg_n_, Sample{});

    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/target_pose", 1, std::bind(&CartesianImpedanceTorqueNode::targetCb, this, _1));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/current_pose", 10);
    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("~/ext_wrench", 10);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);

    control_thread_ = std::thread(&CartesianImpedanceTorqueNode::controlLoop, this);
    logger_thread_ = std::thread(&CartesianImpedanceTorqueNode::loggerLoop, this);
  }

  ~CartesianImpedanceTorqueNode() override {
    running_ = false;
    if (control_thread_.joinable()) control_thread_.join();
    if (logger_thread_.joinable()) logger_thread_.join();
  }

 private:
  void targetCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(target_mtx_);
    target_p_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    target_q_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                   msg->pose.orientation.y, msg->pose.orientation.z);
  }

  // RT-cheap: copy one sample into the ring buffer (no I/O, no logging).
  void pushSample(const Sample& s) {
    std::lock_guard<std::mutex> lk(dbg_mtx_);
    dbg_[dbg_head_] = s;
    dbg_head_ = (dbg_head_ + 1) % dbg_n_;
    dbg_count_++;
  }

  void controlLoop() {
    try {
      franka::Robot robot(robot_ip_);
      franka::Model model = robot.loadModel();

      const double t = coll_torque_;
      const double f = coll_force_;
      robot.setCollisionBehavior(
          {{t, t, t, t, t, t, t}}, {{t, t, t, t, t, t, t}},
          {{f, f, f, f, f, f}}, {{f, f, f, f, f, f}});

      bool initialized = false;
      Eigen::Vector3d cmd_p;
      Eigen::Vector3d cmd_v = Eigen::Vector3d::Zero();
      Eigen::Quaterniond cmd_q;
      Eigen::Vector3d cmd_w = Eigen::Vector3d::Zero();
      Vector7d q_nullspace = Vector7d::Zero();
      Vector7d tau_prev = Vector7d::Zero();
      double t_acc = 0.0;

      RCLCPP_INFO(get_logger(),
                  "Connected to %s. Torque Cartesian impedance starting. "
                  "K=[%.0f %.0f %.0f %.0f %.0f %.0f] xi=%.2f Kn=%.1f -> CSV '%s'",
                  robot_ip_.c_str(), K_(0, 0), K_(1, 1), K_(2, 2), K_(3, 3), K_(4, 4),
                  K_(5, 5), damping_ratio_, nullspace_stiffness_, dbg_file_.c_str());

      robot.control([&](const franka::RobotState& state,
                        franka::Duration period) -> franka::Torques {
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(state.O_T_EE.data()));
        Eigen::Vector3d position = transform.translation();
        Eigen::Quaterniond orientation(transform.rotation());
        Eigen::Map<const Vector7d> q(state.q.data());
        Eigen::Map<const Vector7d> dq(state.dq.data());

        std::array<double, 7> grav_arr = model.gravity(state);
        if (!initialized) {
          cmd_p = position;
          cmd_q = orientation;
          q_nullspace = q;
          {
            std::lock_guard<std::mutex> lk(target_mtx_);
            target_p_ = position;
            target_q_ = orientation;
          }
          tau_prev = Eigen::Map<const Vector7d>(state.tau_J.data()) -
                     Eigen::Map<const Vector7d>(grav_arr.data());
          initialized = true;
        }

        publishState(state);

        Eigen::Vector3d tp;
        Eigen::Quaterniond tq;
        {
          std::lock_guard<std::mutex> lk(target_mtx_);
          tp = target_p_;
          tq = target_q_;
        }
        double dt = period.toSec();
        if (dt <= 0.0) dt = 0.001;
        t_acc += dt;

        // --- smooth equilibrium toward target (no steps) ---
        double kd_t = 2.0 * std::sqrt(trans_gain_);
        Eigen::Vector3d acc = trans_gain_ * (tp - cmd_p) - kd_t * cmd_v;
        if (acc.norm() > max_a_) acc = acc.normalized() * max_a_;
        cmd_v += acc * dt;
        if (cmd_v.norm() > max_v_) cmd_v = cmd_v.normalized() * max_v_;
        cmd_p += cmd_v * dt;

        Eigen::Quaterniond dq_rot = tq * cmd_q.inverse();
        if (dq_rot.w() < 0) dq_rot.coeffs() *= -1.0;
        Eigen::AngleAxisd aa(dq_rot);
        Eigen::Vector3d err_rot_eq = aa.angle() * aa.axis();
        double kd_r = 2.0 * std::sqrt(rot_gain_);
        Eigen::Vector3d ang_acc = rot_gain_ * err_rot_eq - kd_r * cmd_w;
        if (ang_acc.norm() > max_walpha_) ang_acc = ang_acc.normalized() * max_walpha_;
        cmd_w += ang_acc * dt;
        if (cmd_w.norm() > max_w_) cmd_w = cmd_w.normalized() * max_w_;
        double wn = cmd_w.norm();
        if (wn > 1e-9) cmd_q = Eigen::Quaterniond(Eigen::AngleAxisd(wn * dt, cmd_w / wn)) * cmd_q;
        cmd_q.normalize();

        // --- dynamics (keep libfranka arrays alive; Eigen::Map does not copy) ---
        std::array<double, 49> mass_arr = model.mass(state);
        std::array<double, 7> coriolis_arr = model.coriolis(state);
        std::array<double, 42> jac_arr = model.zeroJacobian(franka::Frame::kEndEffector, state);
        Eigen::Map<const Matrix7d> M(mass_arr.data());
        Eigen::Map<const Vector7d> coriolis(coriolis_arr.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(jac_arr.data());

        Matrix7d M_inv = M.inverse();
        Matrix6d Lambda_inv = J * M_inv * J.transpose();
        Lambda_inv.diagonal().array() += sing_eps_;
        Matrix6d Lambda = Lambda_inv.inverse();

        Matrix6d V;
        Vector6d lambda;
        Matrix6d D = factorizationDamping(K_, Lambda, damping_ratio_, V, lambda);

        Vector6d error;
        error.head(3) = position - cmd_p;
        Eigen::Quaterniond oc = orientation;
        if (cmd_q.coeffs().dot(oc.coeffs()) < 0.0) oc.coeffs() *= -1.0;
        Eigen::Quaterniond eq = oc.inverse() * cmd_q;
        error.tail(3) = eq.vec();
        error.tail(3) = -(transform.rotation() * error.tail(3));

        Vector6d xdot = J * dq;
        Vector7d tau_task = J.transpose() * (-K_ * error - D * xdot);

        Eigen::Matrix<double, 6, 7> Jt_pinv = jacobianTransposePinv(J, sing_eps_);
        Matrix7d N = Matrix7d::Identity() - J.transpose() * Jt_pinv;
        Vector7d tau_null = N * (nullspace_stiffness_ * (q_nullspace - q) -
                                 2.0 * std::sqrt(nullspace_stiffness_) * dq);

        Vector7d tau_raw = tau_task + tau_null + coriolis;  // NO gravity (robot adds it)

        // rate limit + count clipped joints (debug)
        Vector7d tau_d;
        int clip = 0;
        for (int i = 0; i < 7; ++i) {
          double diff = tau_raw[i] - tau_prev[i];
          if (std::abs(diff) > delta_tau_max_) clip++;
          tau_d[i] = tau_prev[i] + std::max(std::min(diff, delta_tau_max_), -delta_tau_max_);
        }
        tau_prev = tau_d;

        // --- assemble + self-verify debug sample (all cheap, RT-safe: no I/O) ---
        Sample s;
        s.t = t_acc;
        s.err_pos = error.head(3).norm();
        s.err_rot = error.tail(3).norm();
        s.ee_speed = xdot.norm();
        s.manip = std::sqrt(std::max(0.0, (J * J.transpose()).determinant()));
        s.lambda_min = lambda.minCoeff();
        s.lambda_max = lambda.maxCoeff();
        s.fact_resid = (V.transpose() * Lambda * V - Matrix6d::Identity()).norm();
        Vector6d d_modal = 2.0 * damping_ratio_ * lambda.array().max(0.0).sqrt();
        s.damp_resid = (V.transpose() * D * V - Matrix6d(d_modal.asDiagonal())).norm();
        s.tau_task_norm = tau_task.norm();
        s.tau_null_norm = tau_null.norm();
        s.coriolis_norm = coriolis.norm();
        s.tau_d_norm = tau_d.norm();
        s.tau_headroom = (kTauLimit - tau_d.cwiseAbs()).minCoeff();
        s.rate_clip = clip;
        Vector7d::Map(s.tau_d.data()) = tau_d;
        for (int i = 0; i < 6; ++i) s.fext[i] = state.O_F_ext_hat_K[i];
        pushSample(s);

        std::array<double, 7> tau_out{};
        Vector7d::Map(tau_out.data()) = tau_d;
        if (!running_ || !rclcpp::ok()) return franka::MotionFinished(franka::Torques(tau_out));
        return franka::Torques(tau_out);
      });
    } catch (const franka::Exception& e) {
      reflex_msg_ = e.what();
      RCLCPP_ERROR(get_logger(), "==== REFLEX / libfranka exception: %s ====", e.what());
    }
    running_ = false;
  }

  // Separate thread: drain the ring buffer to a CSV file + throttled console summary.
  // File I/O lives ONLY here, never in the 1 kHz control loop.
  void loggerLoop() {
    std::ofstream csv(dbg_file_, std::ios::out | std::ios::trunc);
    if (csv) {
      csv << "t,err_pos_m,err_rot_rad,ee_speed,manip,lambda_min,lambda_max,fact_resid,"
             "damp_resid,tau_task,tau_null,coriolis,tau_d_norm,tau_headroom,rate_clip,"
             "tau1,tau2,tau3,tau4,tau5,tau6,tau7,Fx,Fy,Fz,Tx,Ty,Tz\n";
    }
    size_t drained = 0;
    int decim = std::max(1, (int)std::lround(1000.0 / std::max(1.0, dbg_log_hz_)));
    auto last_print = std::chrono::steady_clock::now();
    double print_dt = 1.0 / std::max(0.1, dbg_print_hz_);

    while (rclcpp::ok() && (running_ || drained < total())) {
      // copy the new range out under a short lock, then write outside the lock
      std::vector<Sample> batch;
      {
        std::lock_guard<std::mutex> lk(dbg_mtx_);
        size_t cnt = dbg_count_;
        if (cnt > drained) {
          size_t n = cnt - drained;
          if (n > dbg_n_) { drained = cnt - dbg_n_; n = dbg_n_; }  // overflow: lost oldest
          batch.reserve(n);
          for (size_t k = 0; k < n; ++k) {
            size_t idx = (dbg_head_ + dbg_n_ - n + k) % dbg_n_;
            batch.push_back(dbg_[idx]);
          }
          drained = cnt;
        }
      }
      if (csv) {
        for (size_t i = 0; i < batch.size(); ++i) {
          if (((drained - batch.size() + i) % decim) != 0) continue;  // decimate
          writeRow(csv, batch[i]);
        }
        csv.flush();
      }
      // throttled human summary so we can watch live without opening the CSV
      auto nowc = std::chrono::steady_clock::now();
      if (!batch.empty() &&
          std::chrono::duration<double>(nowc - last_print).count() >= print_dt) {
        last_print = nowc;
        const Sample& s = batch.back();
        RCLCPP_INFO(get_logger(),
            "err p=%.1fmm r=%.1fmrad | manip=%.4f lam=[%.0f..%.0f] | resid f=%.1e d=%.1e | "
            "tau task=%.1f null=%.1f cor=%.1f |tau|=%.1f headroom=%.1fNm clip=%d | |Fext|=%.1fN",
            s.err_pos * 1e3, s.err_rot * 1e3, s.manip, s.lambda_min, s.lambda_max,
            s.fact_resid, s.damp_resid, s.tau_task_norm, s.tau_null_norm, s.coriolis_norm,
            s.tau_d_norm, s.tau_headroom, s.rate_clip,
            std::sqrt(s.fext[0] * s.fext[0] + s.fext[1] * s.fext[1] + s.fext[2] * s.fext[2]));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!reflex_msg_.empty() && csv) {
      csv << "# REFLEX: " << reflex_msg_ << "\n";
    }
    if (csv) csv.close();
    RCLCPP_INFO(get_logger(), "Logger stopped. %zu samples -> %s", total(), dbg_file_.c_str());
  }

  void writeRow(std::ofstream& csv, const Sample& s) {
    csv << std::fixed << std::setprecision(6) << s.t << ',' << s.err_pos << ',' << s.err_rot
        << ',' << s.ee_speed << ',' << s.manip << ',' << std::setprecision(2) << s.lambda_min
        << ',' << s.lambda_max << ',' << std::scientific << s.fact_resid << ',' << s.damp_resid
        << std::fixed << std::setprecision(3) << ',' << s.tau_task_norm << ',' << s.tau_null_norm
        << ',' << s.coriolis_norm << ',' << s.tau_d_norm << ',' << s.tau_headroom << ','
        << s.rate_clip;
    for (int i = 0; i < 7; ++i) csv << ',' << s.tau_d[i];
    for (int i = 0; i < 6; ++i) csv << ',' << s.fext[i];
    csv << '\n';
  }

  size_t total() {
    std::lock_guard<std::mutex> lk(dbg_mtx_);
    return dbg_count_;
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
  }

  std::string robot_ip_;
  Matrix6d K_;
  double damping_ratio_, nullspace_stiffness_;
  double trans_gain_, rot_gain_;
  double max_v_, max_a_, max_w_, max_walpha_;
  double delta_tau_max_, sing_eps_;
  double coll_torque_, coll_force_;
  std::string joint_prefix_;

  // debug
  std::vector<Sample> dbg_;
  size_t dbg_n_{4000}, dbg_head_{0}, dbg_count_{0};
  std::mutex dbg_mtx_;
  std::string dbg_file_, reflex_msg_;
  double dbg_log_hz_{200.0}, dbg_print_hz_{1.0};
  std::thread logger_thread_;

  std::mutex target_mtx_;
  Eigen::Vector3d target_p_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond target_q_{Eigen::Quaterniond::Identity()};

  std::atomic<bool> running_{true};
  std::thread control_thread_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianImpedanceTorqueNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
