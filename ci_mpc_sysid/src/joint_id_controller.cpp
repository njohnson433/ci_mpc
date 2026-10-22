#include "ci_mpc_sysid/joint_id_controller.hpp"

#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>

namespace ci_mpc_sysid {

using controller_interface::CallbackReturn;

namespace {

double sign(double x) { return (x > 0.0) - (x < 0.0); }

// Raised-cosine window that ramps in over the first and out over the last ramp seconds.
// Returns w, w', w''.
std::array<double, 3> window(double t, double duration, double ramp) {
  const double k = M_PI / ramp;
  if (t < ramp) {
    const double u = k * t;
    return {0.5 * (1.0 - std::cos(u)), 0.5 * k * std::sin(u), 0.5 * k * k * std::cos(u)};
  }
  if (t > duration - ramp) {
    const double u = k * (duration - t);
    return {0.5 * (1.0 - std::cos(u)), -0.5 * k * std::sin(u), 0.5 * k * k * std::cos(u)};
  }
  return {1.0, 0.0, 0.0};
}

}  // namespace

controller_interface::InterfaceConfiguration JointIdController::command_interface_configuration()
    const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration JointIdController::state_interface_configuration()
    const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

CallbackReturn JointIdController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<int64_t>>("joints", {1, 2, 3, 4, 5, 6, 7});
    auto_declare<std::vector<std::string>>("experiments", {"sweep", "breakaway", "sine"});
    auto_declare<std::vector<double>>("velocities", {0.05, 0.1, 0.2, 0.4, 0.8});
    auto_declare<double>("sweep_amplitude", 0.25);
    auto_declare<double>("accel", 2.0);
    // Gains/limits per joint. Hold gains keep the other joints still; test gains track the reference.
    auto_declare<std::vector<double>>("hold_kp", {600, 600, 600, 600, 250, 150, 50});
    auto_declare<std::vector<double>>("hold_kd", {50, 50, 50, 50, 20, 15, 8});
    auto_declare<std::vector<double>>("test_kp", {600, 600, 600, 600, 250, 150, 50});
    auto_declare<std::vector<double>>("test_kd", {50, 50, 50, 50, 20, 15, 8});
    auto_declare<std::vector<double>>("tau_limit", {20, 20, 20, 20, 8, 8, 8});
    auto_declare<double>("breakaway_rate", 0.5);
    auto_declare<double>("breakaway_max", 3.0);
    auto_declare<double>("breakaway_pos_threshold", 0.003);
    auto_declare<double>("breakaway_vel_threshold", 0.02);
    auto_declare<std::vector<double>>("sine_freqs", {0.5, 1.0, 2.0});
    auto_declare<double>("sine_amplitude", 0.2);
    auto_declare<double>("sine_max_velocity", 1.0);
    auto_declare<double>("sine_cycles", 4.0);
    auto_declare<double>("settle_time", 1.0);
    auto_declare<double>("max_excursion", 0.6);
    auto_declare<double>("max_velocity", 1.5);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointIdController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  auto node = get_node();
  arm_id_ = node->get_parameter("arm_id").as_string();
  joints_ = node->get_parameter("joints").as_integer_array();
  experiments_ = node->get_parameter("experiments").as_string_array();
  velocities_ = node->get_parameter("velocities").as_double_array();
  sweep_amplitude_ = node->get_parameter("sweep_amplitude").as_double();
  accel_ = node->get_parameter("accel").as_double();
  hold_kp_ = node->get_parameter("hold_kp").as_double_array();
  hold_kd_ = node->get_parameter("hold_kd").as_double_array();
  test_kp_ = node->get_parameter("test_kp").as_double_array();
  test_kd_ = node->get_parameter("test_kd").as_double_array();
  tau_limit_ = node->get_parameter("tau_limit").as_double_array();
  breakaway_rate_ = node->get_parameter("breakaway_rate").as_double();
  breakaway_max_ = node->get_parameter("breakaway_max").as_double();
  breakaway_pos_threshold_ = node->get_parameter("breakaway_pos_threshold").as_double();
  breakaway_vel_threshold_ = node->get_parameter("breakaway_vel_threshold").as_double();
  sine_freqs_ = node->get_parameter("sine_freqs").as_double_array();
  sine_amplitude_ = node->get_parameter("sine_amplitude").as_double();
  sine_max_velocity_ = node->get_parameter("sine_max_velocity").as_double();
  sine_cycles_ = node->get_parameter("sine_cycles").as_double();
  settle_time_ = node->get_parameter("settle_time").as_double();
  max_excursion_ = node->get_parameter("max_excursion").as_double();
  max_velocity_ = node->get_parameter("max_velocity").as_double();

  for (const auto* v : {&hold_kp_, &hold_kd_, &test_kp_, &test_kd_, &tau_limit_}) {
    if (v->size() != kNumJoints) {
      RCLCPP_ERROR(node->get_logger(), "gain/limit arrays need %d entries", kNumJoints);
      return CallbackReturn::ERROR;
    }
  }
  for (auto j : joints_) {
    if (j < 1 || j > kNumJoints) {
      RCLCPP_ERROR(node->get_logger(), "joints must be in 1..%d", kNumJoints);
      return CallbackReturn::ERROR;
    }
  }

  log_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("~/log", rclcpp::SystemDefaultsQoS());
  rt_log_pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(log_pub_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointIdController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  readState();
  q_hold_ = q_;
  tau_prev_.fill(0.0);
  t_ = 0.0;
  joint_idx_ = 0;
  done_ = joints_.empty();
  aborted_ = false;
  if (!done_) {
    planJoint(static_cast<int>(joints_[0]) - 1);
  }
  RCLCPP_INFO(get_node()->get_logger(), "joint identification started (%zu joints), log layout: %s",
              joints_.size(), kLogLayout);
  return CallbackReturn::SUCCESS;
}

void JointIdController::readState() {
  for (int i = 0; i < kNumJoints; ++i) {
    q_[i] = state_interfaces_.at(2 * i).get_value();
    dq_[i] = state_interfaces_.at(2 * i + 1).get_value();
  }
}

void JointIdController::planJoint(int joint) {
  joint_ = joint;
  segments_.clear();
  const double q0 = q_hold_[joint];
  const double a = sweep_amplitude_;
  const double approach_v = 0.3;
  segments_.push_back({Phase::kSettle, 0, 0, 0, settle_time_, 0, 0});
  for (const auto& exp : experiments_) {
    if (exp == "sweep") {
      segments_.push_back({Phase::kReturn, 0, q0 - a, approach_v, 0, 0, 0});
      for (size_t k = 0; k < velocities_.size(); ++k) {
        const int code = static_cast<int>(k) + 1;
        segments_.push_back({Phase::kSweep, code, q0 + a, velocities_[k], 0, 0, 0});
        segments_.push_back({Phase::kSweep, -code, q0 - a, velocities_[k], 0, 0, 0});
      }
      segments_.push_back({Phase::kReturn, 0, q0, approach_v, 0, 0, 0});
      segments_.push_back({Phase::kSettle, 0, 0, 0, settle_time_, 0, 0});
    } else if (exp == "breakaway") {
      for (double dir : {1.0, -1.0}) {
        segments_.push_back({Phase::kBreakaway, static_cast<int>(dir), dir, 0,
                             breakaway_max_ / breakaway_rate_ + 1.0, 0, 0});
        segments_.push_back({Phase::kReturn, 0, q0, 0.2, 0, 0, 0});
        segments_.push_back({Phase::kSettle, 0, 0, 0, settle_time_, 0, 0});
      }
    } else if (exp == "sine") {
      for (size_t k = 0; k < sine_freqs_.size(); ++k) {
        const double f = sine_freqs_[k];
        const double amp = std::min(sine_amplitude_, sine_max_velocity_ / (2.0 * M_PI * f));
        // integer cycles plus one ramp period at each end
        segments_.push_back({Phase::kSine, static_cast<int>(k) + 1, 0, 0, (sine_cycles_ + 2.0) / f, f, amp});
      }
      segments_.push_back({Phase::kSettle, 0, 0, 0, settle_time_, 0, 0});
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "unknown experiment '%s' skipped", exp.c_str());
    }
  }
  seg_idx_ = 0;
  q_ref_ = q0;
  dq_ref_ = 0.0;
  startSegment();
}

void JointIdController::startSegment() {
  t_seg_ = 0.0;
  seg_start_q_ = q_[joint_];
  breakaway_tau_ = 0.0;
}

controller_interface::return_type JointIdController::update(const rclcpp::Time& /*time*/,
                                                            const rclcpp::Duration& period) {
  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1) {
    dt = 0.001;
  }
  t_ += dt;
  readState();

  // safety: excursion / velocity limits -> hold everything where it is
  if (!aborted_) {
    for (int i = 0; i < kNumJoints; ++i) {
      // the joint under test moves by up to sweep_amplitude_ (sine amplitudes are smaller)
      const double limit = (i == joint_) ? sweep_amplitude_ + 0.15 : max_excursion_;
      if (std::abs(q_[i] - q_hold_[i]) > limit || std::abs(dq_[i]) > max_velocity_) {
        RCLCPP_ERROR(get_node()->get_logger(), "joint %d out of bounds (q %.3f, dq %.3f): abort", i + 1,
                     q_[i], dq_[i]);
        aborted_ = true;
        q_hold_ = q_;
        joint_ = -1;
        break;
      }
    }
  }

  Vec7 tau{};
  for (int i = 0; i < kNumJoints; ++i) {
    tau[i] = hold_kp_[i] * (q_hold_[i] - q_[i]) - hold_kd_[i] * dq_[i];
  }

  double ddq_ref = 0.0;
  int phase = static_cast<int>(aborted_ ? Phase::kAbort : (done_ ? Phase::kDone : Phase::kSettle));
  int code = 0;
  const bool testing = !aborted_ && !done_ && joint_ >= 0;
  if (testing) {
    const int j = joint_;
    const Segment& seg = segments_[seg_idx_];
    phase = static_cast<int>(seg.phase);
    bool finished = false;
    t_seg_ += dt;
    switch (seg.phase) {
      case Phase::kSettle:
        q_ref_ = q_hold_[j];
        dq_ref_ = 0.0;
        finished = t_seg_ >= seg.duration;
        break;
      case Phase::kSweep:
      case Phase::kReturn: {
        // online trapezoidal profile towards seg.target
        const double d = seg.target - q_ref_;
        const double dir = sign(d);
        const double stop = dq_ref_ * dq_ref_ / (2.0 * accel_);
        double dq_new;
        if (std::abs(d) <= stop + 1e-9 && sign(dq_ref_) == dir) {
          dq_new = dq_ref_ - dir * accel_ * dt;
          if (sign(dq_new) != dir) dq_new = 0.0;
        } else {
          dq_new = std::clamp(dq_ref_ + dir * accel_ * dt, -seg.vmax, seg.vmax);
        }
        ddq_ref = (dq_new - dq_ref_) / dt;
        dq_ref_ = dq_new;
        q_ref_ += dq_ref_ * dt;
        const bool cruising = std::abs(std::abs(dq_ref_) - seg.vmax) < 1e-9 && ddq_ref == 0.0;
        code = (seg.phase == Phase::kSweep && cruising) ? seg.code : 0;
        if (std::abs(seg.target - q_ref_) < 1e-4 && std::abs(dq_ref_) < accel_ * dt * 1.01) {
          q_ref_ = seg.target;
          dq_ref_ = 0.0;
          finished = true;
        }
        break;
      }
      case Phase::kBreakaway: {
        // open-loop torque ramp on the tested joint, others held
        breakaway_tau_ = seg.target * breakaway_rate_ * t_seg_;
        code = seg.code;
        q_ref_ = q_[j];
        dq_ref_ = 0.0;
        finished = std::abs(q_[j] - seg_start_q_) > breakaway_pos_threshold_ ||
                   std::abs(dq_[j]) > breakaway_vel_threshold_ ||
                   std::abs(breakaway_tau_) > breakaway_max_ || t_seg_ >= seg.duration;
        break;
      }
      case Phase::kSine: {
        const double w = 2.0 * M_PI * seg.freq;
        const auto [win, dwin, ddwin] = window(t_seg_, seg.duration, 1.0 / seg.freq);
        const double s = seg.amp * std::sin(w * t_seg_);
        const double ds = seg.amp * w * std::cos(w * t_seg_);
        const double dds = -seg.amp * w * w * std::sin(w * t_seg_);
        q_ref_ = q_hold_[j] + win * s;
        dq_ref_ = dwin * s + win * ds;
        ddq_ref = ddwin * s + 2.0 * dwin * ds + win * dds;
        code = seg.code;
        finished = t_seg_ >= seg.duration;
        break;
      }
      default:
        finished = true;
    }

    if (seg.phase == Phase::kBreakaway) {
      tau[j] = breakaway_tau_;
    } else {
      tau[j] = test_kp_[j] * (q_ref_ - q_[j]) + test_kd_[j] * (dq_ref_ - dq_[j]);
    }

    if (finished) {
      if (seg.phase == Phase::kBreakaway) {
        // continue the return from where the joint is now
        q_ref_ = q_[j];
        dq_ref_ = 0.0;
      }
      if (++seg_idx_ >= segments_.size()) {
        if (++joint_idx_ >= joints_.size()) {
          done_ = true;
          joint_ = -1;
          RCLCPP_INFO(get_node()->get_logger(), "joint identification done");
        } else {
          planJoint(static_cast<int>(joints_[joint_idx_]) - 1);
        }
      } else {
        startSegment();
      }
    }
  }

  // torque and torque-rate limits
  const double max_step = delta_tau_max_ * dt / 0.001;
  for (int i = 0; i < kNumJoints; ++i) {
    tau[i] = std::clamp(tau[i], -tau_limit_[i], tau_limit_[i]);
    tau[i] = tau_prev_[i] + std::clamp(tau[i] - tau_prev_[i], -max_step, max_step);
    tau_prev_[i] = tau[i];
    command_interfaces_[i].set_value(tau[i]);
  }

  if (rt_log_pub_ && rt_log_pub_->trylock()) {
    auto& data = rt_log_pub_->msg_.data;
    data.resize(4 + 3 * kNumJoints + 3);
    data[0] = t_;
    data[1] = testing ? joint_ + 1 : 0;
    data[2] = phase;
    data[3] = code;
    for (int i = 0; i < kNumJoints; ++i) {
      data[4 + i] = q_[i];
      data[4 + kNumJoints + i] = dq_[i];
      data[4 + 2 * kNumJoints + i] = tau[i];
    }
    data[4 + 3 * kNumJoints] = q_ref_;
    data[5 + 3 * kNumJoints] = dq_ref_;
    data[6 + 3 * kNumJoints] = ddq_ref;
    rt_log_pub_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

}  // namespace ci_mpc_sysid

PLUGINLIB_EXPORT_CLASS(ci_mpc_sysid::JointIdController, controller_interface::ControllerInterface)
