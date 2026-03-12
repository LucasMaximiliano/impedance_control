#ifndef COMBINED_CONTROLLER_BASE_HPP
#define COMBINED_CONTROLLER_BASE_HPP

#include <Eigen/Dense>
#include "impedance_controller_base.hpp"

//! \class CombinedControllerBase
//! \brief C++ implementation of the combined impedance and gravity compensation
//! controller for a robotic finger with two joints.
//! \details This class inherits from the ImpedanceControllerBase and adds
//! functionality for enabling/disabling the impedance control and gravity
//! compensation components, as well as methods for setting the gains of the
//! controller. The computeCombinedTorque() method computes the commanded joint
//! torques based on the impedance control law and the gravity compensation term.
//! This class can be easily expanded to include further torque components e.g.
//! a friction compensation or nullspace terms.
class CombinedControllerBase : public ImpedanceControllerBase
{
public:
    //! \name Constructor & Destructor
    //! \{
    CombinedControllerBase(
      const Eigen::Matrix2d & virtual_inertia_matrix,
      const Eigen::Matrix2d & virtual_damping_matrix,
      const Eigen::Matrix2d & virtual_stiffness_matrix,
      const double force_feedback_gain,
      const Eigen::Vector2d & link_mass_vector,
      const Eigen::Vector2d & link_length_vector,
      const Eigen::Vector2d & dist_to_link_com_vector,
      const double max_actuator_torque,
      const int control_loop_duration);
    ~CombinedControllerBase();
    //! \}

    //! \name Setters
    //! \note The setters for the gains overload the ImpedanceControllerBase setters.
    //! They take a scalar value and set the corresponding matrix to a scaled identity 
    //! matrix. This is a common approach for tuning the controller with a single gain
    //! parameter. However, the class can be easily extended to allow setting full
    //! matrices by leveraging wrappers around the original setters, if needed.
    //! \{
    void setCombinedControlEnabled(const bool enable) { combined_control_enabled_ = enable; }
    void setImpedanceControlEnabled(const bool enable) { impedance_control_enabled_ = enable; }
    void setGravityCompensationEnabled(const bool enable) { gravity_compensation_enabled_ = enable; }
    void setVirtualInertiaMatrix(const double gain)
    {
      ImpedanceControllerBase::setVirtualInertiaMatrix(gain * Eigen::Matrix2d::Identity());
    }
    void setVirtualDampingMatrix(const double gain)
    {
      ImpedanceControllerBase::setVirtualDampingMatrix(gain * Eigen::Matrix2d::Identity());
    }
    
    void setVirtualStiffnessMatrix(const double gain)
    {
      ImpedanceControllerBase::setVirtualStiffnessMatrix(gain * Eigen::Matrix2d::Identity());
    }
    //! \}

    //! \name Getters
    //! \{
    bool getCombinedControlEnabled() { return combined_control_enabled_; }
    bool getImpedanceControlEnabled() { return impedance_control_enabled_; }
    bool getGravityCompensationEnabled() { return gravity_compensation_enabled_; }
    //! \}

    //! \name computeCombinedTorque
    //! \brief Computes the combined torque based on the impedance control law and
    //! gravity compensation.
    //! \details For more details on the control law, refer to the developer notes.
    //! This method checks the enabled components and computes the total torque accordingly.
    Eigen::Vector2d computeCombinedTorque();
private:
    //! \name Controller enable flags
    //! \brief Flags for enabling/disabling the torque control components.
    //! \{
    bool combined_control_enabled_;     //!< Flag for enabling/disabling the entire combined controller. (Dis-)connects controller from the actuators.
    bool impedance_control_enabled_;    //!< Flag for enabling/disabling the impedance control component of the controller
    bool gravity_compensation_enabled_; //!< Flag for enabling/disabling the gravity compensation component of the controller
    //! \}
};

#endif // COMBINED_CONTROLLER_BASE_HPP