#include "nodes/control_node.h"

namespace controls{

ControlNode::ControlNode() : Node("control_node")
{
  auto controllerRegistry = registry::controllerRegistry<double>();

  // Initialize the controller params
  controller_ = controllerRegistry.at("passivityBasedAdaptiveControl");
  params_ = std::make_shared<attitude::BaseParams<double>>();
  data_ = std::make_shared<attitude::ControllerData<double>>();

  // Set up the publishers
  controlPublisher_ = this->create_publisher<messages::msg::Control>("control_topic", 10);

  // Set up the subscribers
  estimationSubscriber_ = this->create_subscription<messages::msg::AttitudeEstimation>(
    "estimation_topic", 10, std::bind(&ControlNode::estimationCallback, this, _1));
  
  attitudeGenerationSubscriber_ = this->create_subscription<messages::msg::DesiredAttitude>(
    "attitude_generation_topic", 10, std::bind(&ControlNode::attitudeGenerationCallback, this, _1));

  systemEngagedSubscriber_ = this->create_subscription<messages::msg::SystemEngaged>(
    "system_engaged_topic", 10, std::bind(&ControlNode::systemEngagedCallback, this, _1));

  // Set up the timers
  controlTimer_ = this->create_wall_timer(20ms, std::bind(&ControlNode::controllerCallback, this));
}

void ControlNode::controllerCallback()
{
  // Check if system is engaged
  const CANData canData = this->getCANData();
  if (!canData.systemEngaged) {
    this->reset();
  }

  // Create copies of the current data sets
  const EstimationData estimationData = this->getEstimationData();
  const DesiredData desiredData = this->getDesiredData();

  // If the system is not engaged or any of the data is invalid then return early without publishing data
  if (!canData.valid || !canData.systemEngaged || !estimationData.valid || !desiredData.valid){
    return;
  }

  // Populate the controller data and then run the controller
  this->populateControllerData(estimationData, desiredData);

  bool result = controller_(*params_, *data_);

  auto message = messages::msg::Control();
  if (result) {
    previousControl_ = data_->u;

    message.torque[0] = data_->u(0);
    message.torque[1] = data_->u(1);
    message.torque[2] = data_->u(2);
    message.controller_valid = true;
  } else {
    this->reset();

    message.torque[0] = previousControl_(0);
    message.torque[1] = previousControl_(1);
    message.torque[2] = previousControl_(2);
    message.controller_valid = false;
  }

  message.valid = true;
  controlPublisher_->publish(message);
}

void ControlNode::estimationCallback(const messages::msg::AttitudeEstimation& msg)
{
  estimationData_.valid = false;
  if (!msg.valid) {
    RCLCPP_INFO(this->get_logger(), "ControlNode::Estimation messa geis not valid!");
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

void ControlNode::attitudeGenerationCallback(const messages::msg::DesiredAttitude& msg)
{
  desiredData_.valid = false;
  if (!msg.valid) {
    RCLCPP_INFO(this->get_logger(), "ControlNode::Desired attitude message is not valid!");
    return;
  }

  desiredData_.valid = msg.valid;

  desiredData_.quat(0) = msg.quat.vector[0];
  desiredData_.quat(1) = msg.quat.vector[1];
  desiredData_.quat(2) = msg.quat.vector[2];
  desiredData_.quat(3) = msg.quat.scalar;

  desiredData_.omega(0) = msg.omega.x;
  desiredData_.omega(1) = msg.omega.y;
  desiredData_.omega(2) = msg.omega.z;

  desiredData_.omegaDot(0) = msg.omega_dot.x;
  desiredData_.omegaDot(1) = msg.omega_dot.y;
  desiredData_.omegaDot(2) = msg.omega_dot.z;
}

void ControlNode::systemEngagedCallback(const messages::msg::SystemEngaged& msg)
{
  canData_.valid = false;
  if (!msg.valid) {
    RCLCPP_INFO(this->get_logger(), "ControlNode::System engaged message is not valid!");
    return;
  }

  canData_.valid = msg.valid;
  canData_.systemEngaged = msg.system_engaged;
}

void ControlNode::populateControllerData(const EstimationData& estimation, const DesiredData& desired)
{
  // Populate the necessary state estimation data
  data_->quat = estimation.quat;
  data_->omega = estimation.omega;

  // Populate the necessary desired attitude information
  data_->quatDesired = desired.quat;
  data_->omegaDesired = desired.omega;
  data_->omegaDotDesired = desired.omegaDot;
}

void ControlNode::reset()
{
  // TODO::Michael::Need to add a function registry for controller reset functions.
  // data_->u = attitude::Control<double>::Zero();
  // data_->theta = attitude::control::Theta<double>::Zero();
}

} // namespace controls

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controls::ControlNode>());
  rclcpp::shutdown();

  return 0;
}
