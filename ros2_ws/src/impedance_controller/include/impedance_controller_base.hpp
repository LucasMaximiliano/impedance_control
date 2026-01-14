#ifndef IMPEDANCE_CONTROLLER_BASE_HPP
#define IMPEDANCE_CONTROLLER_BASE_HPP

#include <Eigen/Dense>

class ImpedanceControllerBase
{
    public:
        //! \name Constructor & Destructor
        //! \param virtual_inertia_matrix Virtual inertia matrix (2x2)
        //! \param virtual_damping_matrix Virtual damping matrix (2x2)
        //! \param virtual_stiffness_matrix Virtual stiffness matrix (2x2)
        //! \param force_feedback_gain Force feedback gain (scalar)
        //! \{
        ImpedanceControllerBase(
            const Eigen::Matrix2d& virtual_inertia_matrix,
            const Eigen::Matrix2d& virtual_damping_matrix,
            const Eigen::Matrix2d& virtual_stiffness_matrix,
            double force_feedback_gain);
        ~ImpedanceControllerBase();
        //! \}

        //! \name Getters
        //! \{
        Eigen::Vector2d getTorqueCommanded() const;
        //! \}

        //! \name Control Law
        //! \brief Computes the commanded torque based on the impedance control law
        //! \details The control law is defined below. For more details, refer to the developer notes.
        //!     \f[
        //!     \tau_c=(1+K_F)J^T(q)[M^d(\ddot{x}^d-\ddot{x})+D^d(\dot{x}^d-\dot{x})+K^d(x^d-x)]+K_F\tau_e
        //!     \f]
        //! \return Commanded torque vector (2x1)
        //! \{
        Eigen::Vector2d controlLaw();
        //! \}
        
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
        Eigen::Vector2d cartesian_position_measured_;       //!< Measured position vector in Cartesian space (2x1)
        Eigen::Vector2d cartesian_velocity_measured_;       //!< Measured velocity vector in Cartesian space (2x1)
        Eigen::Vector2d cartesian_acceleration_computed_;   //!< Computed acceleration vector in Cartesian space (2x1)
        Eigen::Vector2d cartesian_position_desired_;        //!< Desired position vector in Cartesian space (2x1)
        Eigen::Vector2d cartesian_velocity_desired_;        //!< Desired velocity vector in Cartesian space (2x1)
        Eigen::Vector2d cartesian_acceleration_desired_;    //!< Desired acceleration vector in Cartesian space (2x1)
        Eigen::Vector2d joint_torque_measured_;             //!< Measured torque vector in joint space (2x1)
        //! \}

        //! \name Output Variables
        //! \{
        Eigen::Vector2d joint_torque_commanded_;    //!< Commanded torque vector in joint space (2x1)
        //! \}
};

#endif // IMPEDANCE_CONTROLLER_BASE_HPP