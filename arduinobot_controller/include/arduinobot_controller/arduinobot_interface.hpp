#ifndef ARDUINOBOT_INTERFACE_H
#define ARDUINOBOT_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <vector>
#include <string>
#include "std_msgs/msg/float32_multi_array.hpp"

namespace arduinobot_controller
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ArduinobotInterface : public rclcpp::Node, public hardware_interface::SystemInterface
{
public:
  explicit ArduinobotInterface(const std::string &node_name = "odrive_command_node");
  virtual ~ArduinobotInterface();

  // Lifecycle management
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // Hardware Interface management
  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  virtual hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  std::vector<double> position_commands_;
  std::vector<double> prev_position_commands_;
  std::vector<double> position_states_;

  // Publisher for ODrive commands
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr odrive_command_publisher_;
  bool warning_printed_;
};

}  // namespace arduinobot_controller

#endif  // ARDUINOBOT_INTERFACE_H
