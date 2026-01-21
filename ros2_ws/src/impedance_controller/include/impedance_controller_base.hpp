#ifndef IMPEDANCE_CONTROLLER_BASE_HPP
#define IMPEDANCE_CONTROLLER_BASE_HPP

#include <Eigen/Dense>

//! \class ImpedanceControllerBase
//! \brief Base C++ implementation of the impedance controller
//! \details This class implements a Cartesian impedance controller for a 2-DOF
//! planar robotic finger with revolute joints. The controller modulates the
//! dynamic relationship between the end-effector and the environment,
//! allowing for compliant interaction with the environment. It computes the
//! commanded joint torques based on desired and measured Cartesian states,
//! as well as measured joint torques.
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

        //! \name computeImpedanceTorque
        //! \brief Computes the commanded torque based on the impedance control law
        //! \details The control law is defined below. For more details, refer to the developer notes.
        //!     \f[
        //!     \tau_c=(1+K_F)J^T(q)[M^d(\ddot{x}^d-\ddot{x})+D^d(\dot{x}^d-\dot{x})+K^d(x^d-x)]+K_F\tau_e
        //!     \f]
        //! \return Commanded torque vector (2x1)
        //! \{
        Eigen::Vector2d computeImpedanceTorque();
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
};

#endif // IMPEDANCE_CONTROLLER_BASE_HPP