#ifndef IMPEDANCE_CONTROLLER_ROS2_HPP
#define IMPEDANCE_CONTROLLER_ROS2_HPP

#include "impedance_controller_base.hpp"
#include "std_msgs/msg/Float32MultiArray.hpp"
#include "std_msgs/msg/Float64MultiArray.hpp"
#include "rclcpp/rclcpp.hpp"

//! \class ImpedanceControllerROS2
//! \brief ROS2 node for controlling the impedance of a robotic grapple with four fingers.
//! Wraps around the ImpedanceControllerBase class to interface with ROS2 topics for
//! receiving sensor data and sending torque commands.
//! \details This class subscribes to topics for measured joint positions, velocities,
//! and torques, as well as desired joint positions and velocities. It publishes the
//! computed torque commands to a dedicated topic. Each finger of the robotic hand is
//! controlled by an instance of the ImpedanceControllerBase class.
class ImpedanceControllerROS2 : public rclcpp::Node
{
    public:
        //! \name Constructor & Destructor
        //! \{
        ImpedanceControllerROS2();
        ~ImpedanceControllerROS2();
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
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desired_position_subscription_; /* TODO: Check type */
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr desired_velocity_subscription_; /* TODO: Check type */
        //! \}
        
        //! \name Impedance Controller Instances
        //! \brief Impedance controller instances of the ImpedanceControllerBase class
        //! for each finger.
        //! \{
        ImpedanceControllerBase impedance_controller_1_;
        ImpedanceControllerBase impedance_controller_2_;
        ImpedanceControllerBase impedance_controller_3_;
        ImpedanceControllerBase impedance_controller_4_;
        //! \}
};

#endif // IMPEDANCE_CONTROLLER_ROS2_HPP