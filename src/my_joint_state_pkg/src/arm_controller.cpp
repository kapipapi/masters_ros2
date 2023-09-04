#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <vector>
#include <cmath>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("arm_controller");

  auto publisher = node->create_publisher<sensor_msgs::msg::JointState>("/joint_command", 10);

  sensor_msgs::msg::JointState msg;
  msg.name = {
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7"
    };

  rclcpp::WallRate loop_rate(1);

  while (rclcpp::ok()) {
    // Dummy joint angles; replace these with angles calculated from IK
    msg.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    msg.header.stamp = node->now();

    publisher->publish(msg);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
