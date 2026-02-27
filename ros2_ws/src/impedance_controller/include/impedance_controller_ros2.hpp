#ifndef IMPEDANCE_CONTROLLER_ROS2_HPP
#define IMPEDANCE_CONTROLLER_ROS2_HPP

#include "impedance_controller_base.hpp"
#include "std_msgs/msg/Float32MultiArray.hpp"
#include "std_msgs/msg/Float64MultiArray.hpp"
#include "rclcpp/rclcpp.hpp"

class ImpedanceControllerROS2 : public rclcpp::Node
{
    public:
        ImpedanceControllerROS2();
        ~ImpedanceControllerROS2();

    private:
        void controlLoopCallback();

        // Publisher:
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr torque_command_publisher_;
        // Subscribers:
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr measured_position_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr measured_velocity_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr measured_torque_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desired_position_subscription_; /* TODO: Check type */
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr desired_velocity_subscription_; /* TODO: Check type */
        // Calback timer for control loop
        rclcpp::TimerBase::SharedPtr control_loop_timer_;
        // Impedance controller instances for each finger
        ImpedanceControllerBase impedance_controller_1_;
        ImpedanceControllerBase impedance_controller_2_;
        ImpedanceControllerBase impedance_controller_3_;
        ImpedanceControllerBase impedance_controller_4_;
};

#endif // IMPEDANCE_CONTROLLER_ROS2_HPP