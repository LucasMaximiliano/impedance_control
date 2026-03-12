#ifndef COMBINED_CONTROLLER_ROS2_HPP
#define COMBINED_CONTROLLER_ROS2_HPP

#include "combined_controller_base.hpp"
#include "impedance_controller_interfaces/srv/set_gain.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rclcpp/rclcpp.hpp"

//! \class CombinedControllerROS2
//! \brief ROS2 node for controlling the impedance of a robotic grapple with four fingers.
//! Wraps around the CombinedControllerBase class to interface with ROS2 topics for
//! receiving sensor data and sending torque commands.
//! \details This class subscribes to topics for measured joint positions, velocities,
//! and torques, as well as desired joint positions and velocities. The impedance
//! parameters and the torque components i.e. impedance and gravity compensation torques,
//! can be configured/enabled via a service. Finally, it publishes the computed torque
//! commands to a dedicated topic. Each finger of the robotic hand is controlled by an
//! instance of the CombinedControllerBase class.
class CombinedControllerROS2 : public rclcpp::Node
{
public:
  //! \name Constructor & Destructor
  //! \{
  CombinedControllerROS2();
  ~CombinedControllerROS2();
  //! \}

private:
  //! \name Control Loop Callback
  //! \brief Callback function for the control loop timer. This function is called
  //! at a fixed rate defined by the control loop duration in the config file. It
  //! computes the impedance control torques for each finger and publishes the
  //! commands to the appropriate topic.
  void controlLoopCallback();

  //! \name Callback Timer for Control Loop
  //! \brief ROS2 timer for running the control loop at a fixed rate.
  rclcpp::TimerBase::SharedPtr control_loop_timer_;

  //! \name Publisher
  //! \brief ROS2 publisher for sending computed torque commands to all fingers.
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr torque_command_publisher_;
  //! \name Subscribers
  //! \brief ROS2 subscribers for receiving measured joint positions, velocities,
  //! and torques, as well as desired joint positions and velocities for all fingers.
  //! \{
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr measured_position_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr measured_velocity_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr measured_torque_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desired_position_subscription_;   // TODO: Check type with Ian
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr desired_velocity_subscription_;   // TODO: Check type with Ian
  //! \}
  //! \name Services
  //! \brief ROS2 services for dynamically reconfiguring the controller at runtime.
  //! \{
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr impedance_control_enabled_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gravity_compensation_enabled_service_;
  rclcpp::Service<impedance_controller_interfaces::srv::SetGain>::SharedPtr inertia_gain_service_;
  rclcpp::Service<impedance_controller_interfaces::srv::SetGain>::SharedPtr damping_gain_service_;
  rclcpp::Service<impedance_controller_interfaces::srv::SetGain>::SharedPtr stiffness_gain_service_;
  rclcpp::Service<impedance_controller_interfaces::srv::SetGain>::SharedPtr torque_gain_service_;
  //! \}

  //! \name Combined Controller Instances
  //! \brief Combined controller instances of the CombinedControllerBase class
  //! for each finger.
  //! \{
  CombinedControllerBase combined_controller_1_;
  CombinedControllerBase combined_controller_2_;
  CombinedControllerBase combined_controller_3_;
  CombinedControllerBase combined_controller_4_;
  //! \}
};

#endif // COMBINED_CONTROLLER_ROS2_HPP
