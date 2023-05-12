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
  // Create copies of the current data sets
  const EstimationData estimationData = this->getEstimationData();

  // if the system is not engaged or any of the data is invalid then return early without publishing data
  if (!estimationData.valid){
    return;
  }

  // Populate the controller data and then run the controller
  populateControllerData(estimationData);

  bool result = passivityBasedAdaptiveControl(params_, data_);

  if (result) {
    auto message = messages::msg::Control();
    message.torque[0] = data_.u(0);
    message.torque[1] = data_.u(1);
    message.torque[2] = data_.u(2);
    message.valid = true;

    controlPublisher_->publish(message);
  }
}

void ControlNode::estimationCallback(const messages::msg::AttitudeEstimation& msg)
{
  estimationData_.valid = false;
  if (!msg.valid) {
    RCLCPP_INFO(this->get_logger(), "ControlNode::Estimation data is not valid!");

    return;
  }

  estimationData_.valid = msg.valid;

  estimationData_.quat(0) = msg.quat.vector[0];
  estimationData_.quat(1) = msg.quat.vector[1];
  estimationData_.quat(2) = msg.quat.vector[2];
  estimationData_.quat(3) = msg.quat.scalar;

  estimationData_.omega(0) = msg.omega.x;
  estimationData_.omega(1) = msg.omega.y;
  estimationData_.omega(2) = msg.omega.z;
}

void ControlNode::populateControllerData(const EstimationData& estimation)
{
  // Populate the necessary state estimation data
  data_.quat = estimation.quat;
  data_.omega = estimation.omega;
}

} // namespace controls

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controls::ControlNode>());
  rclcpp::shutdown();

  return 0;
}
