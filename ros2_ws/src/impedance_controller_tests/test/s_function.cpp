#include <chrono>
#include <string>
#include <Eigen/Dense>
#include "impedance_controller_base.hpp"
#include "impedance_controller_params.hpp"

//! \name s_function
//! \brief Simulink interface for the impedance controller
//! \details This function serves as an interface to the ImpedanceControllerBase class,
//! allowing it to be used in a Simulink model. It initializes the controller on the first call
//! and computes the torque command based on the provided desired and measured Cartesian states,
//! as well as measured joint torques.
//! \param cartesian_position_desired Desired position vector in Cartesian space (2x1)
//! \param cartesian_velocity_desired Desired velocity vector in Cartesian space (2x1)
//! \param cartesian_acceleration_desired Desired acceleration vector in Cartesian space (2x1)
//! \param cartesian_position_measured Measured position vector in Cartesian space (2x1)
//! \param cartesian_velocity_measured Measured velocity vector in Cartesian space (2x1)
//! \param joint_torque_measured Measured joint torque vector (2x1)
//! \return Commanded torque vector (2x1)
Eigen::Vector2d s_function(
    const Eigen::Vector2d& cartesian_position_desired,
    const Eigen::Vector2d& cartesian_velocity_desired,
    const Eigen::Vector2d& cartesian_acceleration_desired,
    const Eigen::Vector2d& cartesian_position_measured,
    const Eigen::Vector2d& cartesian_velocity_measured,
    const Eigen::Vector2d& joint_torque_measured)
{
    // Static initialization (runs only once on first call)
    static ImpedanceControllerBase* impedance_controller = nullptr;
    static auto last_time = std::chrono::high_resolution_clock::now();
    
    if (impedance_controller == nullptr) {
        // TODO: Update the path to the TOML configuration file as needed
        std::string toml_path = "/Users/lucas/dev/seaclear2.0/impedance-control/impedance_control/ros2_ws/src/impedance_controller/config.toml";
        impedance_controller_params::params_t params(toml_path);
        impedance_controller = new ImpedanceControllerBase(
            params.virtual_inertia_matrix_,
            params.virtual_damping_matrix_,
            params.virtual_stiffness_matrix_,
            params.force_feedback_gain_);
    }

    // Finite backward difference method to approximate acceleration numerically
    // TODO: Make sure the velocity is given in m/s
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = current_time - last_time;
    last_time = current_time;

    impedance_controller->cartesian_acceleration_computed_ = (
        cartesian_velocity_measured -
        impedance_controller->cartesian_velocity_measured_) / elapsed.count();

    // Set input variables
    impedance_controller->cartesian_position_desired_ = cartesian_position_desired;
    impedance_controller->cartesian_velocity_desired_ = cartesian_velocity_desired;
    impedance_controller->cartesian_acceleration_desired_ = cartesian_acceleration_desired;
    impedance_controller->cartesian_position_measured_ = cartesian_position_measured;
    impedance_controller->cartesian_velocity_measured_ = cartesian_velocity_measured;
    impedance_controller->joint_torque_measured_ = joint_torque_measured;

    // Compute the torque command
    return impedance_controller->computeImpedanceTorque();
}