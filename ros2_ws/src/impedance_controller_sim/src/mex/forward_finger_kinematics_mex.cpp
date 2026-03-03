#include "mex.h"
#include "matrix.h"

#include <Eigen/Dense>
#include "finger_kinematics.hpp"

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    // Check number of inputs and outputs
    if (nrhs != 2) {
        mexErrMsgTxt("Usage: cartesian_position = forward_finger_kinematics_mex(joint_position, link_length);");
    }
    if (nlhs != 1) {
        mexErrMsgTxt("One output required (cartesian_position).");
    }

    // Check dimension of inputs
    if (!mxIsDouble(prhs[0]) || mxGetNumberOfElements(prhs[0]) != 2 ||
        !mxIsDouble(prhs[1]) || mxGetNumberOfElements(prhs[1]) != 2) {
        mexErrMsgTxt("joint_position and link_length must be 2x1 double vectors.");
    }

    // Read inputs
    double* q_ptr = mxGetPr(prhs[0]);
    double* l_ptr = mxGetPr(prhs[1]);

    Eigen::Vector2d q(q_ptr[0], q_ptr[1]);
    Eigen::Vector2d l(l_ptr[0], l_ptr[1]);

    // Call your C++ function
    Eigen::Vector2d p = finger_kinematics::forward(q, l);

    // Create MATLAB output
    plhs[0] = mxCreateDoubleMatrix(2, 1, mxREAL);
    double* out = mxGetPr(plhs[0]);

    out[0] = p(0);
    out[1] = p(1);
}
