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
            const double force_feedback_gain,
            const Eigen::Vector2d& link_mass_vector,
            const Eigen::Vector2d& link_length_vector,
            const Eigen::Vector2d& dist_to_link_com_vector);
        ~ImpedanceControllerBase();
        //! \}

        //! \name Setters
        //! \{
        void setCartesianPositionMeasured(const Eigen::Vector2d& position) { cartesian_position_measured_ = position; }
        void setCartesianVelocityMeasured(const Eigen::Vector2d& velocity) { cartesian_velocity_measured_ = velocity; }
        void setCartesianAccelerationComputed(const Eigen::Vector2d& acceleration) { cartesian_acceleration_computed_ = acceleration; }
        
        void setCartesianPositionDesired(const Eigen::Vector2d& position) { cartesian_position_desired_ = position; }
        void setCartesianVelocityDesired(const Eigen::Vector2d& velocity) { cartesian_velocity_desired_ = velocity; }
        void setCartesianAccelerationDesired(const Eigen::Vector2d& acceleration) { cartesian_acceleration_desired_ = acceleration; }
        
        void setJointPositionMeasured(const Eigen::Vector2d& position) { joint_position_measured_ = position; }
        void setJointVelocityMeasured(const Eigen::Vector2d& velocity) { joint_velocity_measured_ = velocity; }
        void setJointTorqueMeasured(const Eigen::Vector2d& torque) { joint_torque_measured_ = torque; }
        //! \}

        //! \name Getters
        //! \{
        Eigen::Vector2d getCartesianPositionMeasured() const { return cartesian_position_measured_; }
        Eigen::Vector2d getCartesianVelocityMeasured() const { return cartesian_velocity_measured_; }
        Eigen::Vector2d getCartesianAccelerationComputed() const { return cartesian_acceleration_computed_; }
        
        Eigen::Vector2d getCartesianPositionDesired() const { return cartesian_position_desired_; }
        Eigen::Vector2d getCartesianVelocityDesired() const { return cartesian_velocity_desired_; }
        Eigen::Vector2d getCartesianAccelerationDesired() const { return cartesian_acceleration_desired_; }
        
        Eigen::Vector2d getJointPositionMeasured() const { return joint_position_measured_; }
        Eigen::Vector2d getJointVelocityMeasured() const { return joint_velocity_measured_; }
        Eigen::Vector2d getJointTorqueMeasured() const { return joint_torque_measured_; }
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
        //! \name Impedance Controller Parameters
        //! \{
        Eigen::Matrix2d virtual_inertia_matrix_;    //!< Virtual inertia matrix (2x2)
        Eigen::Matrix2d virtual_damping_matrix_;    //!< Virtual damping matrix (2x2)
        Eigen::Matrix2d virtual_stiffness_matrix_;  //!< Virtual stiffness matrix (2x2)
        double force_feedback_gain_;                //!< Force feedback gain (scalar)
        //! \}

        //! \name Mechanical Robot Parameters
        //! \{
        Eigen::Vector2d link_mass_vector_;         //!< Link mass vector (2x1)
        Eigen::Vector2d link_length_vector_;       //!< Link length vector (2x1)
        Eigen::Vector2d dist_to_link_com_vector_;  //!< Distance to link center of mass vector (2x1)
        //! \}

        //! \name Input Variables
        //! \{        
        Eigen::Vector2d cartesian_position_measured_;       //!< Measured position vector in Cartesian space (2x1)
        Eigen::Vector2d cartesian_velocity_measured_;       //!< Measured velocity vector in Cartesian space (2x1)
        Eigen::Vector2d cartesian_acceleration_computed_;   //!< Computed acceleration vector in Cartesian space (2x1)
        
        Eigen::Vector2d cartesian_position_desired_;        //!< Desired position vector in Cartesian space (2x1)
        Eigen::Vector2d cartesian_velocity_desired_;        //!< Desired velocity vector in Cartesian space (2x1)
        Eigen::Vector2d cartesian_acceleration_desired_;    //!< Desired acceleration vector in Cartesian space (2x1)
        
        Eigen::Vector2d joint_position_measured_;           //!< Measured position vector in joint space (2x1)
        Eigen::Vector2d joint_velocity_measured_;           //!< Measured velocity vector in joint space (2x1)
        Eigen::Vector2d joint_torque_measured_;             //!< Measured torque vector in joint space (2x1)
        
        Eigen::Vector2d joint_position_desired_;            //!< Desired position vector in joint space (2x1)
        Eigen::Vector2d joint_velocity_desired_;            //!< Desired velocity vector in joint space (2x1)
        Eigen::Vector2d joint_acceleration_desired_;        //!< Desired acceleration vector in joint space (2x1)
        //! \}
};

#endif // IMPEDANCE_CONTROLLER_BASE_HPP