#include <iostream>
#include <Eigen/Dense>
#include "impedance_controller_base.hpp"

ImpedanceControllerBase::ImpedanceControllerBase(
    const Eigen::Matrix2d& virtual_inertia_matrix,
    const Eigen::Matrix2d& virtual_damping_matrix,
    const Eigen::Matrix2d& virtual_stiffness_matrix,
    double force_feedback_gain)
    : virtual_inertia_matrix_(virtual_inertia_matrix),
      virtual_damping_matrix_(virtual_damping_matrix),
      virtual_stiffness_matrix_(virtual_stiffness_matrix),
      force_feedback_gain_(force_feedback_gain),
      joint_position_measured_(Eigen::Vector2d::Zero()),
      joint_velocity_measured_(Eigen::Vector2d::Zero()),
      joint_torque_measured_(Eigen::Vector2d::Zero()),
      joint_position_desired_(Eigen::Vector2d::Zero()),
      joint_velocity_desired_(Eigen::Vector2d::Zero()),
      joint_acceleration_desired_(Eigen::Vector2d::Zero()),
      cartesian_position_measured_(Eigen::Vector2d::Zero()),
      cartesian_velocity_measured_(Eigen::Vector2d::Zero()),
      cartesian_torque_measured_(Eigen::Vector2d::Zero()),
      cartesian_position_desired_(Eigen::Vector2d::Zero()),
      cartesian_velocity_desired_(Eigen::Vector2d::Zero()),
      cartesian_acceleration_desired_(Eigen::Vector2d::Zero())
{
    std::cout << "[ImpedanceControllerBase] Constructor called with..." << std::endl;
    std::cout << "  Virtual Inertia Matrix:\n" << virtual_inertia_matrix_ << std::endl;
    std::cout << "  Virtual Damping Matrix:\n" << virtual_damping_matrix_ << std::endl;
    std::cout << "  Virtual Stiffness Matrix:\n" << virtual_stiffness_matrix_ << std::endl;
    std::cout << "  Force Feedback Gain: " << force_feedback_gain_ << std::endl;
    std::cout << "  Remaining member variables initialized to zero." << std::endl;
}

ImpedanceControllerBase::~ImpedanceControllerBase()
{
    std::cout << "[ImpedanceControllerBase] Destructor called." << std::endl;
}

Eigen::Vector2d ImpedanceControllerBase::computeImpedanceTorque()
{
    // Compute errors
    Eigen::Vector2d position_error     = cartesian_position_desired_     - cartesian_position_measured_;
    Eigen::Vector2d velocity_error     = cartesian_velocity_desired_     - cartesian_velocity_measured_;
    Eigen::Vector2d acceleration_error = cartesian_acceleration_desired_ - cartesian_acceleration_computed_;

    // Compute commanded torque using the impedance control law
    return torque_commanded_ = (1.0 + force_feedback_gain_) * 
        (virtual_inertia_matrix_ * acceleration_error +
        virtual_damping_matrix_ * velocity_error +
        virtual_stiffness_matrix_ * position_error) +
        force_feedback_gain_ * torque_measured_;
}