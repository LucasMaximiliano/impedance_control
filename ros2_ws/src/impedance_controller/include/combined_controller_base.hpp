#ifndef COMBINED_CONTROLLER_BASE_HPP
#define COMBINED_CONTROLLER_BASE_HPP

#include <Eigen/Dense>
#include "impedance_controller_base.hpp"

class CombinedControllerBase : public ImpedanceControllerBase
{
public:
    CombinedControllerBase();
    ~CombinedControllerBase();
    void setImpedanceControlEnabled(bool enable) { impedance_control_enabled_ = enable; }
    void setGravityCompensationEnabled(bool enable) { gravity_compensation_enabled_ = enable; }
    void getImpedanceControlEnabled(bool & impedance_ctrl_enabled) const { impedance_ctrl_enabled = impedance_control_enabled_; }
    void getGravityCompensationEnabled(bool & gravity_comp_enabled) const { gravity_comp_enabled = gravity_compensation_enabled_; }
    Eigen::Vector2d computeCombinedTorque();
private:
    bool impedance_control_enabled_;
    bool gravity_compensation_enabled_;
}

#endif // COMBINED_CONTROLLER_BASE_HPP