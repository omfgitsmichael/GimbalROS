#pragma once

// Standard includes
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2 Includes
#include <rclcpp/rclcpp.hpp>
#include <messages/msg/attitude_estimation.hpp>
#include <messages/msg/sensor_data.hpp>

// Other Library includes
#include <attitude_libraries/registry/filterFunctionRegistry.h>

namespace estimation{

using namespace std::chrono_literals;
using std::placeholders::_1;

class EstimationNode : public rclcpp::Node
{
  public:
    /**
    * Structure for ROS main static parameters which determine the reset of the params.
    **/
    struct StaticParams {
      double filterRate = 0.01;
      std::string filterName = "AHRSKalmanFilter";
    };

    /**
    * Structure for the necessary CAN data.
    **/
    struct CANData {
      bool systemEngaged = false;
      bool valid = false;
    };

    /**
    * Estimation node class contrstructor.
    * Input:
    * Output:
    **/
    EstimationNode();

  private:
    /**
    * Returns a copy of the CAN data.
    * Input:
    * Output: CANData
    **/
    CANData getCANData() const
    {
      return canData_;
    }

    /**
    * Estimation publisher callback function.
    * Input:
    * Output:
    **/
    void estimationCallback();

    /**
    * CAN data callback function.
    * Input: msg - System engaged message topic.
    * Output:
    **/
    void CANCallback(const messages::msg::SensorData& msg);

    /**
    * Populate the filter data structure with the necessary informarion.
    * Input: data - Copy of the current CAN data.
    * Output:
    **/
    void populateFilterData(const CANData& data);

    /**
    * Reset the state of the filter node.
    * Input:
    * Output:
    **/
    void reset();

    /**
    * Set up the node static parameters.
    * Input:
    * Output:
    **/
    void setStaticParams();

    /**
    * Initializes the filter information.
    * Input:
    * Output:
    **/
    void initializeFilterInformation();

    /**
    * Set up the node AHRS parameters.
    * Input:
    * Output:
    **/
    void setAHRSParams();
    
    // ROS Timers
    rclcpp::TimerBase::SharedPtr filterTimer_;
    
    // Publishers
    rclcpp::Publisher<messages::msg::AttitudeEstimation>::SharedPtr filterPublisher_;

    // Subscribers
    rclcpp::Subscription<messages::msg::SensorData>::SharedPtr systemEngagedSubscriber_;

    // Filter member variables
    registry::filterFunc<double> filter_;
    registry::filterResetFunc<double> reset_;
    std::shared_ptr<attitude::BaseParams<double>> params_;
    std::shared_ptr<attitude::ControllerData<double>> data_;

    // Various member variables
    StaticParams staticParams_;
    CANData canData_;
};

} // namespace estimation
