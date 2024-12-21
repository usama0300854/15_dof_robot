#include "arduinobot_controller/arduinobot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <iomanip>  
#include <sstream> 

namespace arduinobot_controller
{

ArduinobotInterface::ArduinobotInterface(const std::string &node_name) 
: hardware_interface::SystemInterface(), rclcpp::Node(node_name), warning_printed_(false) 
{
    // Publisher for ODrive commands
    odrive_command_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("odrive_commands", rclcpp::QoS(10));
    // Reserve space for position commands, states, and previous commands
    position_commands_.resize(15, 0.0);  // 15 actuators
    prev_position_commands_.resize(15, 0.0);
    position_states_.resize(15, 0.0);
}

ArduinobotInterface::~ArduinobotInterface()
{
}

CallbackReturn ArduinobotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    // Initialize the hardware info, publisher was already initialized in constructor
    position_commands_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
    prev_position_commands_.reserve(info_.joints.size());

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinobotInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Provide position interfaces for each joint
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinobotInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Provide position command interfaces for each joint
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
    }

    return command_interfaces;
}

CallbackReturn ArduinobotInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    // Reset commands and states when activated
    position_commands_.assign(15, 0.0);
    prev_position_commands_.assign(15, 0.0);
    position_states_.assign(15, 0.0);

    return CallbackReturn::SUCCESS;
}

CallbackReturn ArduinobotInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinobotInterface::read(const rclcpp::Time &time,
                                                          const rclcpp::Duration &period)
{
    // Simulate feedback by assigning the current commands as the state (replace this with real feedback in the future)
    position_states_ = position_commands_;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinobotInterface::write(const rclcpp::Time &time,
                                                           const rclcpp::Duration &period)
{
    // Only send new commands if they differ from the previous ones
    // if (position_commands_ == prev_position_commands_)
    // {
    //     return hardware_interface::return_type::OK;
    // }

    auto float_msg = std_msgs::msg::Float32MultiArray();
    float_msg.data.clear();

    // Publish commands for each joint, converting from radians to degrees
    for (size_t i = 0; i < position_commands_.size(); ++i)
    {
        float angle = static_cast<float>((((position_commands_.at(i) - (M_PI / 2)) * 180) / M_PI) + 90);
        float rounded_angle = std::round(angle * 100.0f) / 100.0f;

        if (std::fabs(rounded_angle) < std::numeric_limits<float>::epsilon())
        {
            rounded_angle = 0.0f; 
        }

        std::stringstream stream;
        stream << std::fixed << std::setprecision(1) << rounded_angle;
        float formatted_angle = std::stof(stream.str());

        float_msg.data.push_back(formatted_angle);

    }

    // Publish the ODrive command
    odrive_command_publisher_->publish(float_msg);

    prev_position_commands_ = position_commands_;

    if (!warning_printed_)
    {
        RCLCPP_WARN(this->get_logger(), "Commands are being published to ODrive.");
        warning_printed_ = true;
    }

    return hardware_interface::return_type::OK;
}

}  // namespace arduinobot_controller
PLUGINLIB_EXPORT_CLASS(arduinobot_controller::ArduinobotInterface, hardware_interface::SystemInterface)
