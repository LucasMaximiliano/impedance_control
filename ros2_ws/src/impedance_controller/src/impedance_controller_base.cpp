#include <iostream>
#include <Eigen/Dense>
#include "finger_kinematics.hpp"
#include "impedance_controller_base.hpp"

ImpedanceControllerBase::ImpedanceControllerBase(
    const Eigen::Matrix2d& virtual_inertia_matrix,
    const Eigen::Matrix2d& virtual_damping_matrix,
    const Eigen::Matrix2d& virtual_stiffness_matrix,
    const double force_feedback_gain,
    const Eigen::Vector2d& link_mass_vector,
    const Eigen::Vector2d& link_length_vector,
    const Eigen::Vector2d& dist_to_link_com_vector,
    const double max_actuator_torque)
    : virtual_inertia_matrix_(virtual_inertia_matrix),
      virtual_damping_matrix_(virtual_damping_matrix),
      virtual_stiffness_matrix_(virtual_stiffness_matrix),
      force_feedback_gain_(force_feedback_gain),
      link_mass_vector_(link_mass_vector),
      link_length_vector_(link_length_vector),
      dist_to_link_com_vector_(dist_to_link_com_vector),
      max_actuator_torque_(max_actuator_torque),
      joint_position_measured_(Eigen::Vector2d::Zero()),
      joint_velocity_measured_(Eigen::Vector2d::Zero()),
      joint_torque_measured_(Eigen::Vector2d::Zero()),
      joint_position_desired_(Eigen::Vector2d::Zero()),
      joint_velocity_desired_(Eigen::Vector2d::Zero()),
      joint_acceleration_desired_(Eigen::Vector2d::Zero()),
      cartesian_position_measured_(Eigen::Vector2d::Zero()),
      cartesian_velocity_measured_(Eigen::Vector2d::Zero()),
      cartesian_position_desired_(Eigen::Vector2d::Zero()),
      cartesian_velocity_desired_(Eigen::Vector2d::Zero()),
      cartesian_acceleration_desired_(Eigen::Vector2d::Zero())
{
    std::cout << "[ImpedanceControllerBase] Constructor called with..." << std::endl;
    std::cout << "  Virtual Inertia Matrix:\n" << virtual_inertia_matrix_ << std::endl;
    std::cout << "  Virtual Damping Matrix:\n" << virtual_damping_matrix_ << std::endl;
    std::cout << "  Virtual Stiffness Matrix:\n" << virtual_stiffness_matrix_ << std::endl;
    std::cout << "  Force Feedback Gain: " << force_feedback_gain_ << std::endl;
    std::cout << "  Link Mass Vector:\n" << link_mass_vector_ << std::endl;
    std::cout << "  Link Length Vector:\n" << link_length_vector_ << std::endl;
    std::cout << "  Distance to Link COM Vector:\n" << dist_to_link_com_vector_ << std::endl;
    std::cout << "  Maximal actuator torque: " << max_actuator_torque_ << " Nm" << std::endl;
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
    Eigen::Vector2d joint_torque_commanded = (1.0 + force_feedback_gain_) *
        finger_kinematics::jacobian(joint_position_measured_, link_length_vector_).transpose() *
        (virtual_inertia_matrix_ * acceleration_error +
        virtual_damping_matrix_ * velocity_error +
        virtual_stiffness_matrix_ * position_error) +
        force_feedback_gain_ * joint_torque_measured_;

    if (joint_torque_commanded.hasNaN()) {
        std::cerr << "[ImpedanceControllerBase] Warning: Computed joint torque contains NaN values. Setting to zero." << std::endl;
        joint_torque_commanded.setZero();
    } 
    if (joint_torque_commanded.cwiseAbs().maxCoeff() > max_actuator_torque_) {
        std::cerr << "[ImpedanceControllerBase] Warning: Computed joint torque exceeds maximum actuator torque. Clamping values." << std::endl;
        joint_torque_commanded = joint_torque_commanded.cwiseMin(max_actuator_torque_).cwiseMax(-max_actuator_torque_);
    }
    
    return joint_torque_commanded;
}