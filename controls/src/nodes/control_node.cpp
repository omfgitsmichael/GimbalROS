#include "nodes/control_node.h"

namespace controls{

ControlNode::ControlNode() : Node("control_node")
{
  // Set up the publishers
  controlPublisher_ = this->create_publisher<messages::msg::Control>("control_topic", 10);

  // Set up the subscribers
  estimationSubscriber_ = this->create_subscription<messages::msg::AttitudeEstimation>(
    "estimation_topic", 10, std::bind(&ControlNode::estimationCallback, this, _1));

  // Set up the timers
  controlTimer_ = this->create_wall_timer(20ms, std::bind(&ControlNode::controllerCallback, this));
}

void ControlNode::controllerCallback()
{
  auto message = messages::msg::Control();
  message.torque[0] = 0.0;
  message.torque[1] = 0.0;
  message.torque[2] = 0.0;
  message.valid = false;

  // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); // Keeping this line so I know how to use logger
  controlPublisher_->publish(message);
}

void ControlNode::estimationCallback(const messages::msg::AttitudeEstimation& msg)
{
  attitude::Quaternion<double> quat = attitude::Quaternion<double>::Zero();
  quat(0) = msg.quat.vector[0];

  // RCLCPP_INFO(this->get_logger(), "I heard:"); // Keeping this line so I know how to use logger
}

} // namespace controls

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controls::ControlNode>());
  rclcpp::shutdown();

  return 0;
}
