// Joint-level system identification controller (friction + effective inertia).
//
// Uses only generic interfaces (<arm_id>_jointN/position, /velocity state; /effort command), so the
// same plugin runs in Isaac Sim (isaacsim.ros2.control) and on franka_hardware. All joints are held
// with a joint PD at the pose they had at activation; then, for each joint under test in turn:
//   sweep:     constant-velocity passes in both directions  -> Coulomb, viscous, offset friction
//   breakaway: slow torque ramp from rest, both directions   -> static friction
//   sine:      sinusoidal position tracking                  -> effective inertia (armature)
// Every cycle a row is published on ~/log (std_msgs/Float64MultiArray, layout in kLogLayout).
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace ci_mpc_sysid {

constexpr int kNumJoints = 7;

// Phase codes in the log
enum class Phase : int {
  kSettle = 0,
  kSweep = 1,
  kBreakaway = 2,
  kSine = 3,
  kReturn = 4,
  kDone = 9,
  kAbort = -1,
};

// Log row: t, joint (1-based, 0 = none), phase, segment, q[7], dq[7], tau[7], q_ref, dq_ref, ddq_ref
// segment: sweep -> signed velocity index (+k / -k for the k-th velocity, 0 = not cruising),
//          breakaway -> +1 / -1 direction, sine -> frequency index (1-based)
constexpr const char* kLogLayout =
    "t,joint,phase,segment,q1..q7,dq1..dq7,tau1..tau7,q_ref,dq_ref,ddq_ref";

class JointIdController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  using Vec7 = std::array<double, kNumJoints>;

  // One reference segment for the joint under test
  struct Segment {
    Phase phase;
    int code;            // logged segment code
    double target;       // sweep/return: end position; breakaway: direction (+1/-1); settle: unused
    double vmax;         // sweep/return: cruise velocity
    double duration;     // settle / sine: duration [s]; breakaway: max duration
    double freq;         // sine
    double amp;          // sine
  };

  void readState();
  void planJoint(int joint);  // fills segments_ for joint (0-based)
  void startSegment();

  // parameters
  std::string arm_id_;
  std::vector<int64_t> joints_;  // 1-based joints to test, in order
  std::vector<std::string> experiments_;
  std::vector<double> velocities_;
  double sweep_amplitude_{0.25};
  double accel_{2.0};
  std::vector<double> hold_kp_, hold_kd_, test_kp_, test_kd_, tau_limit_;
  double breakaway_rate_{0.5};
  double breakaway_max_{3.0};
  double breakaway_pos_threshold_{0.003};
  double breakaway_vel_threshold_{0.02};
  std::vector<double> sine_freqs_;
  double sine_amplitude_{0.2};
  double sine_max_velocity_{1.0};
  double sine_cycles_{4.0};
  double settle_time_{1.0};
  double max_excursion_{0.6};
  double max_velocity_{1.5};
  double delta_tau_max_{1.0};  // per 1 ms, scaled with dt (as franka's example controllers)

  // state
  Vec7 q_{}, dq_{}, q_hold_{}, tau_prev_{};
  std::vector<Segment> segments_;
  size_t seg_idx_{0};
  size_t joint_idx_{0};
  int joint_{-1};  // 0-based joint under test, -1 = none
  double t_{0.0}, t_seg_{0.0};
  double q_ref_{0.0}, dq_ref_{0.0};  // reference for the joint under test
  double seg_start_q_{0.0};
  double breakaway_tau_{0.0};
  bool done_{false};
  bool aborted_{false};

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> log_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_log_pub_;
};

}  // namespace ci_mpc_sysid
