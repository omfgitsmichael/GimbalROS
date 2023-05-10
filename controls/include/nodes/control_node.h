#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "messages/msg/control.hpp"
#include "messages/msg/attitude_estimation.hpp"

namespace controls{

using namespace std::chrono_literals;
using std::placeholders::_1;

class ControlNode : public rclcpp::Node
{
  public:
    /**
    * Control node class contrstructor.
    * Input:
    * Output:
    **/
    ControlNode();

  private:
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
    
    // ROS Timers
    rclcpp::TimerBase::SharedPtr controlTimer_;
    
    // Publishers
    rclcpp::Publisher<messages::msg::Control>::SharedPtr controlPublisher_;

    // Subscribers
    rclcpp::Subscription<messages::msg::AttitudeEstimation>::SharedPtr estimationSubscriber_;

    // Various Member Variavles
};

} // namespace controls
