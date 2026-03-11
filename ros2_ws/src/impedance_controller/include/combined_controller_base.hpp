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
    CombinedControllerBase();
    ~CombinedControllerBase();
    //! \}

    //! \name Setters
    //! \note The setters for the gains overload the ImpedanceControllerBase setters.
    //! They take a scalar value and set the corresponding matrix to a scaled identity 
    //! matrix. This is a common approach for tuning the controller with a single gain
    //! parameter. However, the class can be easily extended to allow setting full
    //! matrices by leveraging wrappers around the original setters, if needed.
    //! \{
    void setImpedanceControlEnabled(const bool enable) { impedance_control_enabled_ = enable; }
    void setGravityCompensationEnabled(const bool enable) { gravity_compensation_enabled_ = enable; }
    void setVirtualInertiaMatrix(const double gain) { virtual_inertia_matrix_ = gain * Eigen::Matrix2d::Identity(); }
    void setVirtualDampingMatrix(const double gain) { virtual_damping_matrix_ = gain * Eigen::Matrix2d::Identity(); }
    void setVirtualStiffnessMatrix(const double gain) { virtual_stiffness_matrix_ = gain * Eigen::Matrix2d::Identity(); }
    //! \}

    //! \name Getters
    //! \{
    void getImpedanceControlEnabled(bool & impedance_ctrl_enabled) const { impedance_ctrl_enabled = impedance_control_enabled_; }
    void getGravityCompensationEnabled(bool & gravity_comp_enabled) const { gravity_comp_enabled = gravity_compensation_enabled_; }
    //! \}

    //! \name computeCombinedTorque
    //! \brief Computes the combined torque based on the impedance control law and
    //! gravity compensation.
    //! \details For more details on the control law, refer to the developer notes.
    //! This method checks the enabled components and computes the total torque accordingly.
    Eigen::Vector2d computeCombinedTorque();
private:
    bool impedance_control_enabled_;
    bool gravity_compensation_enabled_;
}

#endif // COMBINED_CONTROLLER_BASE_HPP