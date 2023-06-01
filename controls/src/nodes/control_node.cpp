#include "nodes/control_node.h"

namespace controls {

ControlNode::ControlNode() : Node("control_node")
{
  // Initialize node static params
  this->setStaticParams();

  // Initialize the controller static params
  this->initializeControllerInformation();

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
  controlTimer_ = this->create_wall_timer(
    std::chrono::duration<double>(staticParams_.controlRate), std::bind(&ControlNode::controllerCallback, this));
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
  data_->u = attitude::Control<double>::Zero();
  reset_(*data_);
}

void ControlNode::setStaticParams()
{
  this->declare_parameter("controlRate", 0.01);
  this->declare_parameter("controllerName", "passivityBasedAdaptiveControl");

  staticParams_.controlRate = this->get_parameter("controlRate").as_double();
  staticParams_.controllerName = this->get_parameter("controllerName").as_string();
}

void ControlNode::initializeControllerInformation()
{
  auto controllerRegistry = registry::controllerRegistry<double>();
  auto controllerResetRegistry = registry::controllerResetRegistry<double>();

  controller_ = controllerRegistry.at(staticParams_.controllerName);
  reset_ = controllerResetRegistry.at(staticParams_.controllerName + "Reset");
  params_ = std::make_shared<attitude::BaseParams<double>>();
  data_ = std::make_shared<attitude::ControllerData<double>>();

  // Default to the passivity based adaptive control scheme
  this->setPassivityBasedAdaptiveControlParams();

  // Set base parameter information 
  params_->dt = staticParams_.controlRate;
}

void ControlNode::setPassivityBasedAdaptiveControlParams()
{
  this->declare_parameter("lambda", std::vector<double>(3, 1.0));
  this->declare_parameter("k", std::vector<double>(3, 2.0));
  this->declare_parameter("gammaInv", std::vector<double>(3, 10.0));
  this->declare_parameter("del", 0.5);
  this->declare_parameter("e0", 0.025);
  this->declare_parameter("epsilon", 0.5);
  this->declare_parameter("thetaMax", 100.0);
  this->declare_parameter("theta", std::vector<double>(3, 0.0));

  std::shared_ptr<attitude::control::PassivityParams<double>> params = std::make_shared<attitude::control::PassivityParams<double>>();

  const auto lambda = this->get_parameter("lambda").as_double_array();
  for (unsigned int i = 0; i < lambda.size(); i++) {
    params->lambda(i, i) = lambda[i];
  }

  const auto k = this->get_parameter("k").as_double_array();
  for (unsigned int i = 0; i < k.size(); i++) {
    params->k(i, i) = k[i];
  }

  const auto gammaInv = this->get_parameter("gammaInv").as_double_array();
  for (unsigned int i = 0; i < gammaInv.size(); i++) {
    params->gammaInv(i, i) = gammaInv[i];
  }

  params->deadzoneParams.del = this->get_parameter("del").as_double();
  params->deadzoneParams.e0 = this->get_parameter("e0").as_double();
  params->projectionParams.epsilon(0) = this->get_parameter("epsilon").as_double();
  params->projectionParams.thetaMax(0) = this->get_parameter("thetaMax").as_double();

  std::shared_ptr<attitude::control::PassivityControlData<double>> data = std::make_shared<attitude::control::PassivityControlData<double>>();

  const auto theta = this->get_parameter("theta").as_double_array();
  for (unsigned int i = 0; i < gammaInv.size(); i++) {
    data->theta(i) = theta[i];
  }

  params_ = params;
  data_ = data;
}

} // namespace controls

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controls::ControlNode>());
  rclcpp::shutdown();

  return 0;
}
