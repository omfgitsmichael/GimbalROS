#pragma once

// Standard includes
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2 Includes
#include <rclcpp/rclcpp.hpp>
#include <messages/msg/control.hpp>
#include <messages/msg/attitude_estimation.hpp>
#include <messages/msg/desired_attitude.hpp>
#include <messages/msg/system_engaged.hpp>

// Other Library includes
#include <attitude_libraries/registry/controllerFunctionRegistry.h>

namespace controls {

using namespace std::chrono_literals;
using std::placeholders::_1;

class ControlNode : public rclcpp::Node
{
  public:
    /**
    * Structure for ROS main static parameters which determine the reset of the params.
    **/
    struct StaticParams {
      double controlRate = 0.01;
      std::string controllerName = "PassivityBasedAdaptiveControl";
    };

    /**
    * Structure for incoming attitude state estimation data.
    **/
    struct EstimationData {
      attitude::Quaternion<double> quat = attitude::Quaternion<double>::Zero();
      attitude::BodyRate<double> omega = attitude::BodyRate<double>::Zero();
      bool valid = false;
    };

    /**
    * Structure for incoming desired attitude state estimation data.
    **/
    struct DesiredData {
      attitude::Quaternion<double> quat = attitude::Quaternion<double>::Zero();
      attitude::BodyRate<double> omega = attitude::BodyRate<double>::Zero();
      attitude::BodyRate<double> omegaDot = attitude::BodyRate<double>::Zero();
      bool valid = false;
    };

    /**
    * Structure for the necessary CAN data.
    **/
    struct CANData {
      bool systemEngaged = false;
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
    * Returns a copy of the desired attitude data.
    * Input:
    * Output: DesiredData
    **/
    DesiredData getDesiredData() const
    {
      return desiredData_;
    }

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
    * Controls publisher callback function.
    * Input:
    * Output:
    **/
    void controllerCallback();

    /**
    * Estimation subscriber callback function.
    * Input: msg - estimation message topic.
    * Output:
    **/
    void estimationCallback(const messages::msg::AttitudeEstimation& msg);

    /**
    * Attitude generation subscriber callback function.
    * Input: msg - desired attitude message topic.
    * Output:
    **/
    void attitudeGenerationCallback(const messages::msg::DesiredAttitude& msg);

    /**
    * Controller CAN data callback function.
    * Input: msg - System engaged message topic.
    * Output:
    **/
    void systemEngagedCallback(const messages::msg::SystemEngaged& msg);

    /**
    * Populate the controller data structure with the necessary informarion.
    * Input: estimation - Copy of the current attitude state estimation data.
    * Input: desired - Copy of the current desired attitude state data.
    * Output:
    **/
    void populateControllerData(const EstimationData& estimation, const DesiredData& desired);

    /**
    * Reset the state of the controller node.
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
    * Initializes the controller information.
    * Input:
    * Output:
    **/
    void initializeControllerInformation();

    /**
    * Set up the node passivity based adaptive parameters.
    * Input:
    * Output:
    **/
    void setPassivityBasedAdaptiveControlParams();
    
    // ROS Timers
    rclcpp::TimerBase::SharedPtr controlTimer_;
    
    // Publishers
    rclcpp::Publisher<messages::msg::Control>::SharedPtr controlPublisher_;

    // Subscribers
    rclcpp::Subscription<messages::msg::AttitudeEstimation>::SharedPtr estimationSubscriber_;
    rclcpp::Subscription<messages::msg::DesiredAttitude>::SharedPtr attitudeGenerationSubscriber_;
    rclcpp::Subscription<messages::msg::SystemEngaged>::SharedPtr systemEngagedSubscriber_;

    // Controller member variables
    registry::controllerFunc<double> controller_;
    registry::controllerResetFunc<double> reset_;
    std::shared_ptr<attitude::BaseParams<double>> params_;
    std::shared_ptr<attitude::ControllerData<double>> data_;

    // Various member variables
    StaticParams staticParams_;
    EstimationData estimationData_;
    DesiredData desiredData_;
    CANData canData_;
    attitude::Control<double> previousControl_ = attitude::Control<double>::Zero();
};

} // namespace controls
