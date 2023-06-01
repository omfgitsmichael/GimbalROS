#include "nodes/estimation_node.h"

namespace estimation {

EstimationNode::EstimationNode() : Node("estimation_node")
{
  // Initialize node static params
  this->setStaticParams();

  // Initialize the filter static params
  this->initializeFilterInformation();
}

void EstimationNode::setStaticParams()
{
  this->declare_parameter("filterRate", 0.01);
  this->declare_parameter("filterName", "AHRSKalmanFilter");

  this->declare_parameter("accelerometerNoise", 0.5);
  this->declare_parameter("magnetometerNoise", 0.5);
  this->declare_parameter("covarianceInitialValue", 10.0);

  staticParams_.filterRate = this->get_parameter("filterRate").as_double();
  staticParams_.filterName = this->get_parameter("filterName").as_string();
  staticParams_.covarianceInitialValue = this->get_parameter("covarianceInitialValue").as_double();
  staticParams_.accelerometerNoise = this->get_parameter("accelerometerNoise").as_double();
  staticParams_.magnetometerNoise = this->get_parameter("magnetometerNoise").as_double();
}

void EstimationNode::initializeFilterInformation()
{
  auto filterRegistry = registry::filterRegistry<double>();
  auto filterResetRegistry = registry::filterResetRegistry<double>();

  filter_ = filterRegistry.at(staticParams_.filterName);
  reset_ = filterResetRegistry.at(staticParams_.filterName + "Reset");
  params_ = std::make_shared<attitude::BaseParams<double>>();
  data_ = std::make_shared<attitude::FilterData<double>>();

  // Default to the AHRS filter
  this->setAHRSParams();

  // Set base parameter information 
  params_->dt = staticParams_.filterRate;
}

void EstimationNode::setAHRSParams()
{
  this->declare_parameter("omegaProcessNoise", 0.5);
  this->declare_parameter("biasProcessNoise", 0.5);
  this->declare_parameter("linearAccelNoise", 0.5);
  this->declare_parameter("magDisturbanceNoise", 0.5);
  this->declare_parameter("gravity", 9.81);
  this->declare_parameter("geomagneticFieldStrength", 50.0);

  std::shared_ptr<attitude::filter::ahrs::AHRSParams<double>> params = std::make_shared<attitude::filter::ahrs::AHRSParams<double>>();

  params->omegaProcessNoise = this->get_parameter("omegaProcessNoise").as_double();
  params->biasProcessNoise = this->get_parameter("biasProcessNoise").as_double();
  params->linearAccelNoise = this->get_parameter("linearAccelNoise").as_double();
  params->magDisturbanceNoise = this->get_parameter("magDisturbanceNoise").as_double();
  params->gravity= this->get_parameter("gravity").as_double();
  params->geomagneticFieldStrength = this->get_parameter("geomagneticFieldStrength").as_double();

  std::shared_ptr<attitude::filter::ahrs::AHRSData<double>> data = std::make_shared<attitude::filter::ahrs::AHRSData<double>>();

  data->P = attitude::filter::Covariance<double, 12>::Identity() * staticParams_.covarianceInitialValue;

  params_ = params;
  data_ = data;
}

} // namespace estimation

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<estimation::EstimationNode>());
  rclcpp::shutdown();

  return 0;
}
