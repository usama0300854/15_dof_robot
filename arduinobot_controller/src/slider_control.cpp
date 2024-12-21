#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SliderControl : public rclcpp::Node
{
public:
  SliderControl() : Node("slider_control")
  {
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_commands", 10, std::bind(&SliderControl::sliderCallback, this, _1));

    torso_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "torso_controller/joint_trajectory", 10);

    left_arm_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "left_arm_controller/joint_trajectory", 10);

    left_arm_gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "left_arm_gripper_controller/joint_trajectory", 10);  

    right_arm_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "right_arm_controller/joint_trajectory", 10);

    right_arm_gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "right_arm_gripper_controller/joint_trajectory", 10);  
    
    RCLCPP_INFO(get_logger(), "Slider Control Node started");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr torso_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_arm_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_arm_gripper_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_arm_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_arm_gripper_pub_;

  void sliderCallback(const sensor_msgs::msg::JointState::SharedPtr msg) const
  {
    // Prepare trajectory messages for torso, left arm, and right arm
    trajectory_msgs::msg::JointTrajectory torso_command, left_arm_command, left_arm_gripper_command, right_arm_command, right_arm_gripper_command;
    torso_command.joint_names = {"torso_joint"};
    left_arm_command.joint_names = {"left_joint_1", "left_joint_2", "left_joint_3", "left_joint_4", "left_joint_5", "left_joint_6"};
    left_arm_gripper_command.joint_names = {"left_gripper_left_claw_joint"};
    right_arm_command.joint_names = {"right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4", "right_joint_5", "right_joint_6"};
    right_arm_gripper_command.joint_names = {"right_gripper_left_claw_joint"};

    // Create joint trajectory points for each command
    trajectory_msgs::msg::JointTrajectoryPoint torso_goal, left_arm_goal, left_arm_gripper_goal, right_arm_goal, right_arm_gripper_goal;

    
    if (msg->position.size() >= 15) {
        torso_goal.positions.insert(torso_goal.positions.end(), msg->position.begin(), msg->position.begin() + 1);  // Only first position for torso
        for (size_t i = 1; i <= 6; ++i) {
            left_arm_goal.positions.push_back(msg->position[i]);  // Positions for the left arm
        }

        left_arm_gripper_goal.positions.push_back(msg->position[7]);

        for (size_t i = 8; i < 13; ++i) {
            right_arm_goal.positions.push_back(msg->position[i]);  // Positions for the right arm
        }

        right_arm_gripper_goal.positions.push_back(msg->position[14]);

       
        torso_goal.time_from_start = rclcpp::Duration(1s);
        left_arm_goal.time_from_start = rclcpp::Duration(1s);
        left_arm_gripper_goal.time_from_start = rclcpp::Duration(1s);
        right_arm_goal.time_from_start = rclcpp::Duration(1s);
        right_arm_gripper_goal.time_from_start = rclcpp::Duration(1s);

        // Add goals to commands
        torso_command.points.push_back(torso_goal);
        left_arm_command.points.push_back(left_arm_goal);
        left_arm_gripper_command.points.push_back(left_arm_gripper_goal);
        right_arm_command.points.push_back(right_arm_goal);
        right_arm_gripper_command.points.push_back(right_arm_gripper_goal);

        // Publish the commands
        torso_pub_->publish(torso_command);
        left_arm_pub_->publish(left_arm_command);
        left_arm_gripper_pub_->publish(left_arm_gripper_command);
        right_arm_pub_->publish(right_arm_command);
        right_arm_gripper_pub_->publish(right_arm_gripper_command);

    } else {
        RCLCPP_WARN(get_logger(), "Insufficient joint positions received.");
        return; 
    }
    
    
    
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SliderControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
