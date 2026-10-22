// Simulation stand-in for the Franka Hand: claims the two finger position interfaces and drives both
// fingers to width/2 (std_msgs/Float64 on ~/width, metres, clamped to 0..0.08). In Isaac the position command
// becomes a PhysX drive target, so the grip force comes from the drive gains (set by the sim script).
// The real FR3 gripper is commanded through the franka_gripper action server instead.
#include <algorithm>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/msg/float64.hpp>

namespace ci_mpc_sim_gripper {

using controller_interface::CallbackReturn;

class FingerWidthController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {arm_id_ + "_finger_joint1/position", arm_id_ + "_finger_joint2/position"}};
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }
  CallbackReturn on_init() override {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<double>("initial_width", 0.08);
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    arm_id_ = get_node()->get_parameter("arm_id").as_string();
    width_.writeFromNonRT(get_node()->get_parameter("initial_width").as_double());
    sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
        "~/width", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { width_.writeFromNonRT(msg->data); });
    return CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override {
    const double half = std::clamp(*width_.readFromRT(), 0.0, 0.08) / 2.0;
    for (auto& cmd : command_interfaces_) {
      cmd.set_value(half);
    }
    return controller_interface::return_type::OK;
  }

 private:
  std::string arm_id_{"fr3"};
  realtime_tools::RealtimeBuffer<double> width_{0.08};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
};

}  // namespace ci_mpc_sim_gripper

PLUGINLIB_EXPORT_CLASS(ci_mpc_sim_gripper::FingerWidthController, controller_interface::ControllerInterface)
