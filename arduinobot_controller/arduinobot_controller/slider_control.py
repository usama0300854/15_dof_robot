#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



class SliderControl(Node):

    def __init__(self):
        super().__init__("slider_control")
      
        self.torso_pub_ = self.create_publisher(JointTrajectory, "torso_controller/joint_trajectory", 10)
        self.left_arm_pub_ = self.create_publisher(JointTrajectory, "left_arm_controller/joint_trajectory", 10)
        self.left_arm_gripper_pub_ = self.create_publisher(JointTrajectory, "left_arm_gripper_controller/joint_trajectory", 10)
        self.right_arm_pub_ = self.create_publisher(JointTrajectory, "right_arm_controller/joint_trajectory", 10)
        self.right_arm_gripper_pub_ = self.create_publisher(JointTrajectory, "right_arm_gripper_controller/joint_trajectory", 10)

        self.sub_ = self.create_subscription(JointState, "joint_commands", self.sliderCallback, 10)
        self.get_logger().info("Slider Control Node started")

    def sliderCallback(self, msg):
        torso_controller = JointTrajectory()
        left_arm_controller = JointTrajectory()
        left_arm_gripper_controller = JointTrajectory()
        right_arm_controller = JointTrajectory()
        right_arm_gripper_controller = JointTrajectory()
    
        torso_controller.joint_names = ["torso_joint"]
        left_arm_controller.joint_names = ["left_joint_1", "left_joint_2", "left_joint_3", "left_joint_4", "left_joint_5", "left_joint_6"]
        left_arm_gripper_controller.joint_names = ["left_gripper_left_claw_joint"]
        right_arm_controller.joint_names = ["right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4", "right_joint_5", "right_joint_6"]
        right_arm_gripper_controller.joint_names = ["right_gripper_left_claw_joint"]

        torso_goal = JointTrajectoryPoint()
        left_arm_goal = JointTrajectoryPoint()
        left_arm_gripper_goal = JointTrajectoryPoint()
        right_arm_goal = JointTrajectoryPoint()
        right_arm_gripper_goal = JointTrajectoryPoint()

     

        torso_goal.positions = [msg.position[0]]
        left_arm_goal.positions = [msg.position[1:6]]
        left_arm_gripper_goal.positions = [msg.position[7]]
        right_arm_goal.positions = [msg.position[8:13]]
        right_arm_gripper_goal.positions = [msg.position[14]]

        torso_controller.points.append(torso_goal)
        left_arm_controller.points.append(left_arm_goal)
        left_arm_gripper_controller.points.append(left_arm_gripper_goal)
        right_arm_controller.points.append(right_arm_goal)
        right_arm_gripper_controller.points.append(right_arm_gripper_goal)

        self.torso_pub_.publish(torso_controller)
        self.left_arm_pub_.publish(left_arm_controller)
        self.left_arm_gripper_pub_.publish(left_arm_gripper_controller)
        self.right_arm_pub_.publish(right_arm_controller)
        self.right_arm_gripper_pub_.publish(right_arm_gripper_controller)


def main():
    rclpy.init()

    simple_publisher = SliderControl()
    rclpy.spin(simple_publisher)
    
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

