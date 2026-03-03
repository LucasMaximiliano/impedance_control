#include "mex.h"
#include "matrix.h"

#include <Eigen/Dense>
#include "gravity_compensation.hpp"

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    // Check number of inputs and outputs
    if (nrhs != 4) {
        mexErrMsgTxt("Usage: gravity_torque_cmd = gravity_compensation_mex(link_mass(x2), link_length(x2), "
            "dist_to_link_com(x2), joint_position(x2));");
    }
    if (nlhs != 1) {
        mexErrMsgTxt("One output required (gravity_torque_cmd).");
    }

    // Check dimension of inputs
    if (!mxIsDouble(prhs[0]) || mxGetNumberOfElements(prhs[0]) != 2 ||
        !mxIsDouble(prhs[1]) || mxGetNumberOfElements(prhs[1]) != 2 ||
        !mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 2 ||
        !mxIsDouble(prhs[3]) || mxGetNumberOfElements(prhs[3]) != 2) {
        mexErrMsgTxt("All inputs must be 2x1 double vectors.");
    }

    // Read inputs
    double* m_ptr = mxGetPr(prhs[0]);
    double* l_ptr = mxGetPr(prhs[1]);
    double* d_ptr = mxGetPr(prhs[2]);
    double* q_ptr = mxGetPr(prhs[3]);
    
    Eigen::Vector2d m(m_ptr[0], m_ptr[1]);
    Eigen::Vector2d l(l_ptr[0], l_ptr[1]);
    Eigen::Vector2d d(d_ptr[0], d_ptr[1]);
    Eigen::Vector2d q(q_ptr[0], q_ptr[1]);

    // Call your C++ function
    Eigen::Vector2d tau = computeGravityTorque(m, l, d, q);

    // Create MATLAB output
    plhs[0] = mxCreateDoubleMatrix(2, 1, mxREAL);
    double* out = mxGetPr(plhs[0]);

    out[0] = tau(0);
    out[1] = tau(1);
}
