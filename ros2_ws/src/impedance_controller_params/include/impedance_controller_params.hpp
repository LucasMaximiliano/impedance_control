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
    //! \name params_t
    //! \brief Structure to hold impedance controller parameters
    struct params_t {
        Eigen::Matrix2d virtual_inertia_matrix_;    //!< Virtual inertia matrix (2x2)
        Eigen::Matrix2d virtual_damping_matrix_;    //!< Virtual damping matrix (2x2)
        Eigen::Matrix2d virtual_stiffness_matrix_;  //!< Virtual stiffness matrix (2x2)
        double force_feedback_gain_;                //!< Force feedback gain (scalar)

        params_t() = delete;
        explicit params_t(const std::string& toml_path);
    };
    //! \}
}

#endif // IMPEDANCE_CONTROLLER_PARAMS_HPP