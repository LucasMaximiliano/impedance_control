#ifndef IMPEDANCE_CONTROLLER_BASE_HPP
#define IMPEDANCE_CONTROLLER_BASE_HPP

#include <Eigen/Dense>

class ImpedanceControllerBase
{
    public:

    private:
        //! \name Parameters
        //! \{
        Eigen::Matrix2d virtual_inertia_matrix_;    //!< Virtual inertia matrix (2x2)
        Eigen::Matrix2d virtual_damping_matrix_;    //!< Virtual damping matrix (2x2)
        Eigen::Matrix2d virtual_stiffness_matrix_;  //!< Virtual stiffness matrix (2x2)
        double force_feedback_gain_;                //!< Force feedback gain (scalar)
        //! \}

        //! \name Input Variables
        //! \{
        Eigen::Vector2d position_measured_;     //!< Measured position vector (2x1)
        Eigen::Vector2d velocity_measured_;     //!< Measured velocity vector (2x1)
        Eigen::Vector2d torque_measured_;       //!< Measured torque vector (2x1)
        Eigen::Vector2d position_desired_;      //!< Desired position vector (2x1)
        Eigen::Vector2d velocity_desired_;      //!< Desired velocity vector (2x1)
        Eigen::Vector2d acceleration_desired_;  //!< Desired acceleration vector (2x1)
        //! \}

        //! \name Output Variables
        //! \{
        Eigen::Vector2d torque_commanded_;    //!< Commanded torque vector (2x1)
        //! \}
}

#endif // IMPEDANCE_CONTROLLER_BASE_HPP