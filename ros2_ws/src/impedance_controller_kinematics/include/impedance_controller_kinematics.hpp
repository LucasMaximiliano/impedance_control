#ifndef IMPEDANCE_CONTROLLER_KINEMATICS_HPP
#define IMPEDANCE_CONTROLLER_KINEMATICS_HPP

class ImpedanceControllerKinematics
{
    public:
        //! \name Kinematics Functions
        //! \brief These functions handle forward kinematics and Jacobian computations.
        //! \details To further modularize the codebase, consider offloading these functions
        //! to a standalone Kinematics package in the future.
        //! \{
        Eigen::Vector2d forward(const Eigen::Vector2d& joint_position);
        Eigen::Matrix2d jacobian(const Eigen::Vector2d& joint_position);
        Eigen::Matrix2d dotJacobian(
            const Eigen::Vector2d& joint_position,
            const Eigen::Vector2d& joint_velocity);
        //! \}
};

#endif // IMPEDANCE_CONTROLLER_KINEMATICS_HPP