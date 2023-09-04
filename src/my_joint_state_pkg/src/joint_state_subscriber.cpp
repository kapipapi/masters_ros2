#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Handle the joint_state message
  RCLCPP_INFO(rclcpp::get_logger("joint_state_subscriber"), "Received Joint 0 State: %f", msg->position[0]);
  RCLCPP_INFO(rclcpp::get_logger("joint_state_subscriber"), "Received Joint 1 State: %f", msg->position[1]);
  RCLCPP_INFO(rclcpp::get_logger("joint_state_subscriber"), "Received Joint 2 State: %f", msg->position[2]);
  RCLCPP_INFO(rclcpp::get_logger("joint_state_subscriber"), "Received Joint 3 State: %f", msg->position[3]);
  RCLCPP_INFO(rclcpp::get_logger("joint_state_subscriber"), "Received Joint 4 State: %f", msg->position[4]);
  RCLCPP_INFO(rclcpp::get_logger("joint_state_subscriber"), "Received Joint 5 State: %f", msg->position[5]);
  RCLCPP_INFO(rclcpp::get_logger("joint_state_subscriber"), "Received Joint 6 State: %f", msg->position[6]);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("joint_state_subscriber");

  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    10,
    joint_state_callback
  );

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
