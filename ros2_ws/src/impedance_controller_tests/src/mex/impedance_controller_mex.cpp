#include "mex.h"
#include "matrix.h"

#include <Eigen/Dense>
#include "impedance_controller_base.hpp"
#include "finger_kinematics.hpp"

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    // Check number of inputs and outputs
    if (nrhs != 11) {
        mexErrMsgTxt("Usage: impedance_torque_cmd = impedance_controller_mex(gain(x4), link_mass(x2), "
            "link_length(x2), link_com_distance(x2), max_actuator_torque (scalar), "
            "joint_position(x2), joint_velocity(x2), "
            "cartesian_target_position(x2), cartesian_target_velocity(x2), "
            "cartesian_target_acceleration(x2), torque_feedback(x2));");
    }
    if (nlhs != 2) {
        mexErrMsgTxt("Two outputs required (impedance_torque_cmd and joint_acceleration).");
    }

    // Check dimension of inputs
    if (!mxIsDouble(prhs[0])  || mxGetNumberOfElements(prhs[0]) != 4 ||
        !mxIsDouble(prhs[1])  || mxGetNumberOfElements(prhs[1]) != 2 ||
        !mxIsDouble(prhs[2])  || mxGetNumberOfElements(prhs[2]) != 2 ||
        !mxIsDouble(prhs[3])  || mxGetNumberOfElements(prhs[3]) != 2 ||
        !mxIsDouble(prhs[4])  || mxGetNumberOfElements(prhs[4]) != 1 ||
        !mxIsDouble(prhs[5])  || mxGetNumberOfElements(prhs[5]) != 2 ||
        !mxIsDouble(prhs[6])  || mxGetNumberOfElements(prhs[6]) != 2 ||
        !mxIsDouble(prhs[7])  || mxGetNumberOfElements(prhs[7]) != 2 ||
        !mxIsDouble(prhs[8])  || mxGetNumberOfElements(prhs[8]) != 2 ||
        !mxIsDouble(prhs[9])  || mxGetNumberOfElements(prhs[9]) != 2 ||
        !mxIsDouble(prhs[10]) || mxGetNumberOfElements(prhs[10]) != 2) {
        mexErrMsgTxt("gain must be a 4x1 double vector, max torque must be a double scalar, and the remaining arguments 2x1 double vectors.");
    }

    // Read inputs
    double* k_ptr     = mxGetPr(prhs[0]);
    double* m_ptr     = mxGetPr(prhs[1]);
    double* l_ptr     = mxGetPr(prhs[2]);
    double* d_ptr     = mxGetPr(prhs[3]);
    double* tau_m_ptr = mxGetPr(prhs[4]);
    double* q_ptr     = mxGetPr(prhs[5]);
    double* qd_ptr    = mxGetPr(prhs[6]);
    double* x_ptr     = mxGetPr(prhs[7]);
    double* xd_ptr    = mxGetPr(prhs[8]);
    double* xdd_ptr   = mxGetPr(prhs[9]);
    double* tau_ptr   = mxGetPr(prhs[10]);

    Eigen::Vector4d k(k_ptr[0], k_ptr[1], k_ptr[2], k_ptr[3]);
    Eigen::Vector2d m(m_ptr[0], m_ptr[1]);
    Eigen::Vector2d l(l_ptr[0], l_ptr[1]);
    Eigen::Vector2d d(d_ptr[0], d_ptr[1]);
    double tau_max = tau_m_ptr[0];
    Eigen::Vector2d q(q_ptr[0], q_ptr[1]);
    Eigen::Vector2d qd(qd_ptr[0], qd_ptr[1]);
    Eigen::Vector2d x(x_ptr[0], x_ptr[1]);
    Eigen::Vector2d xd(xd_ptr[0], xd_ptr[1]);
    Eigen::Vector2d xdd(xdd_ptr[0], xdd_ptr[1]);
    Eigen::Vector2d tau_meas(tau_ptr[0], tau_ptr[1]);

    // Format inputs
    Eigen::Matrix2d virtual_inertia_matrix;
    virtual_inertia_matrix << k(0), 0, 0, k(0);
    Eigen::Matrix2d virtual_damping_matrix;
    virtual_damping_matrix << k(1), 0, 0, k(1);
    Eigen::Matrix2d virtual_stiffness_matrix;
    virtual_stiffness_matrix << k(2), 0, 0, k(2);
    double torque_feedback_gain = k(3);
    Eigen::Vector2d cartesian_position_measured = finger_kinematics::forward(q, l);
    Eigen::Vector2d cartesian_velocity_measured = finger_kinematics::jacobian(q, l) * qd;

    // Static initialization (runs only once on first call)
    static ImpedanceControllerBase* impedance_controller = nullptr;
    static auto last_time = std::chrono::high_resolution_clock::now();
    static Eigen::Vector2d last_qd = qd;
    
    if (impedance_controller == nullptr) {
        impedance_controller = new ImpedanceControllerBase(
            virtual_inertia_matrix,
            virtual_damping_matrix,
            virtual_stiffness_matrix,
            torque_feedback_gain,
            m,
            l,
            d,
            tau_max);
    }

    // Finite backward difference method to approximate acceleration numerically
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = current_time - last_time;
    last_time = current_time;

    impedance_controller->setCartesianAccelerationComputed( 
        (xd - impedance_controller->getCartesianVelocityMeasured()) / elapsed.count());

    Eigen::Vector2d qdd = (qd - last_qd) / elapsed.count();
    last_qd = qd;

    // Set input variables of the controller
    impedance_controller->setCartesianPositionDesired(x);
    impedance_controller->setCartesianVelocityDesired(xd);
    impedance_controller->setCartesianAccelerationDesired(xdd);
    impedance_controller->setJointPositionMeasured(q);
    impedance_controller->setJointVelocityMeasured(qd);
    impedance_controller->setCartesianPositionMeasured(cartesian_position_measured);
    impedance_controller->setCartesianVelocityMeasured(cartesian_velocity_measured);
    impedance_controller->setJointTorqueMeasured(tau_meas);

    // Compute the torque command
    Eigen::Vector2d result = impedance_controller->computeImpedanceTorque();

    // Create MATLAB output
    plhs[0] = mxCreateDoubleMatrix(2, 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(2, 1, mxREAL);
    double* out1 = mxGetPr(plhs[0]);
    double* out2 = mxGetPr(plhs[1]);

    out1[0] = result(0);
    out1[1] = result(1);
    out2[0] = qdd(0);
    out2[1] = qdd(1);
}
