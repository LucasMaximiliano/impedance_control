#include <iostream>
#include <Eigen/Dense>
#include "combined_controller_base.hpp"
#include "gravity_compensation.hpp"

CombinedControllerBase::CombinedControllerBase(
  const Eigen::Matrix2d & virtual_inertia_matrix,
  const Eigen::Matrix2d & virtual_damping_matrix,
  const Eigen::Matrix2d & virtual_stiffness_matrix,
  const double force_feedback_gain,
  const Eigen::Vector2d & link_mass_vector,
  const Eigen::Vector2d & link_length_vector,
  const Eigen::Vector2d & dist_to_link_com_vector,
  const double max_actuator_torque,
  const int control_loop_duration)
    : ImpedanceControllerBase(
        virtual_inertia_matrix,
        virtual_damping_matrix,
        virtual_stiffness_matrix,
        force_feedback_gain,
        link_mass_vector,
        link_length_vector,
        dist_to_link_com_vector,
        max_actuator_torque,
        control_loop_duration),
      combined_control_enabled_(false),
      impedance_control_enabled_(false), 
      gravity_compensation_enabled_(false)
{
    std::cout << "[CombinedControllerBase] Constructor called with..." << std::endl;
    std::cout << "  Combined control: " << (combined_control_enabled_ ? "Enabled" : "Disabled") << std::endl;
    std::cout << "  Impedance control: " << (impedance_control_enabled_ ? "Enabled" : "Disabled") << std::endl;
    std::cout << "  Gravity compensation: " << (gravity_compensation_enabled_ ? "Enabled" : "Disabled") << std::endl;
}

CombinedControllerBase::~CombinedControllerBase()
{
  std::cout << "[CombinedControllerBase] Destructor called." << std::endl;
}

Eigen::Vector2d CombinedControllerBase::computeCombinedTorque()
{
    Eigen::Vector2d combined_torque = Eigen::Vector2d::Zero();
    // Compute impedance control torque, if enabled
    if (impedance_control_enabled_) {
        combined_torque += computeImpedanceTorque();
    }
    // Compute gravity compensation torque, if enabled
    if (gravity_compensation_enabled_) {
        combined_torque += computeGravityTorque(
            getLinkMasses(),
            getLinkLengths(),
            getDistToLinkCOM(),
            getJointPositionMeasured());
    }
    // Safety checks for NaN values and torque limits
    if (combined_torque.hasNaN()) {
      std::cerr <<
        "[CombinedControllerBase] Warning: Computed joint torque contains NaN values. Setting to zero."
                << std::endl;
      combined_torque.setZero();
    }
    if (combined_torque.cwiseAbs().maxCoeff() > getMaxActuatorTorque()) {
      std::cerr <<
        "[CombinedControllerBase] Warning: Computed joint torque exceeds maximum actuator torque. Clamping value to"
                << getMaxActuatorTorque() << " Nm." << std::endl;
      combined_torque = combined_torque.cwiseMin(getMaxActuatorTorque()).cwiseMax(
        -getMaxActuatorTorque());
    }
    return combined_torque;
}