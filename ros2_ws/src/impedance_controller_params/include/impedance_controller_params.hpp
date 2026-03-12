#ifndef IMPEDANCE_CONTROLLER_PARAMS_HPP
#define IMPEDANCE_CONTROLLER_PARAMS_HPP

#include <string>
#include <Eigen/Dense>
#include "toml.hpp"

//! \namespace impedance_controller_params
//! \brief Namespace for impedance controller parameters
//! \param toml_path Path to the TOML configuration file
namespace impedance_controller_params {
    //! \{
    //! \name limits
    //! \brief Limits for the impedance controller parameters
    constexpr int    MAX_SENSOJOINT_TORQUE     = 120;  //!< Maximal possible torque for the SensoJoint actuator in Nm (absolute value)
    constexpr double MAX_FORCE_FEEDBACK_GAIN   = 1.0;  //!< Maximal force feedback gain. Above 1 stability is not guaranteed.
    constexpr int    MIN_CONTROL_LOOP_DURATION = 1;    //!< Minimal control loop duration in ms (1 kHz)
    constexpr int    MAX_CONTROL_LOOP_DURATION = 1000; //!< Maximal control loop duration in ms (1 Hz)
    //! \}

    //! \{
    //! \name validators
    //! \brief Validator helpers for the impedance controller parameters
    //! \returns true if the condition is met, otherwise throws a runtime error with the provided message
    bool require_condition(const bool condition, const std::string& message);       //!< Throws a runtime error if the condition is not met
    bool validate_vec(const Eigen::Vector2d& vec, const std::string& name);         //!< Validates that a 2x2 vector is non-negative
    bool validate_mat(const Eigen::Matrix2d& mat, const std::string& name);         //!< Validates that a 2x2 matrix is positive semi-definite
    template <typename T> bool validate_range(const T value, const T min_value,
        const T max_value, const std::string& name);                                //!< Validates that a value is within a specified range
    //! \}
    
    //! \{
    //! \name params_t
    //! \brief Structure to hold impedance controller parameters
    struct params_t {
        Eigen::Matrix2d virtual_inertia_matrix_;    //!< Virtual inertia matrix (2x2)
        Eigen::Matrix2d virtual_damping_matrix_;    //!< Virtual damping matrix (2x2)
        Eigen::Matrix2d virtual_stiffness_matrix_;  //!< Virtual stiffness matrix (2x2)
        double force_feedback_gain_;                //!< Force feedback gain (scalar)
        Eigen::Vector2d link_mass_vector_;          //!< Link mass vector (2x1)
        Eigen::Vector2d link_length_vector_;        //!< Link length vector (2x1) in mm
        Eigen::Vector2d dist_to_com_vector_;        //!< Distance to center of mass vector (2x1)
        int max_actuator_torque_;                   //!< Maximal actuator torque (scalar) in Nm
        int control_loop_duration_;                 //!< Control loop duration (scalar) in ms

        params_t() = delete;
        explicit params_t(const std::string& toml_path);
    };
    //! \}
}

#endif // IMPEDANCE_CONTROLLER_PARAMS_HPP