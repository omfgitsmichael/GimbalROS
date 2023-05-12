#pragma once

// Standard includes
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS Includes
#include <rclcpp/rclcpp.hpp>
#include <messages/msg/control.hpp>
#include <messages/msg/attitude_estimation.hpp>

// Other Library includes
#include <attitude_libraries/controllers/passivityBasedAdaptiveControl.h>

namespace controls{

using namespace std::chrono_literals;
using std::placeholders::_1;

class ControlNode : public rclcpp::Node
{
  public:
    /**
    * Structure for incoming attitude state estimation data.
    **/
    struct EstimationData {
      attitude::Quaternion<double> quat = attitude::Quaternion<double>::Zero();
      attitude::BodyRate<double> omega = attitude::BodyRate<double>::Zero();
      bool valid = false;
    };

    /**
    * Control node class contrstructor.
    * Input:
    * Output:
    **/
    ControlNode();

  private:
    /**
    * Returns a copy of the state estimation data.
    * Input:
    * Output: EstimationData
    **/
    EstimationData getEstimationData() const
    {
      return estimationData_;
    }
  
    /**
    * Controls publisher callback function.
    * Input:
    * Output:
    **/
    void controllerCallback();

    /**
    * Estimation subscriber callback function.
    * Input: msg - estimation message topic name.
    * Output:
    **/
    void estimationCallback(const messages::msg::AttitudeEstimation& msg);

    /**
    * Populate the controller data structure with the necessary informarion.
    * Input:estimation - Copy of the current attitude state estimation data.
    * Output:
    **/
    void populateControllerData(const EstimationData& estimation);
    
    // ROS Timers
    rclcpp::TimerBase::SharedPtr controlTimer_;
    
    // Publishers
    rclcpp::Publisher<messages::msg::Control>::SharedPtr controlPublisher_;

    // Subscribers
    rclcpp::Subscription<messages::msg::AttitudeEstimation>::SharedPtr estimationSubscriber_;

    // Controller params and data
    attitude::control::PassivityParams<double> params_;
    attitude::control::PassivityControlData<double> data_;

    // Various member variables
    EstimationData estimationData_;
};

} // namespace controls
