#ifndef FINGER_KINEMATICS_HPP
#define FINGER_KINEMATICS_HPP

#include <Eigen/Dense>

//! \namespace finger_kinematics
//! \brief Functions for forward kinematics and Jacobian computations for a 2-DOF
//! planar robotic finger with revolute joints.
//! \details This namespace provides functions to compute the forward kinematics,
//! Jacobian matrix, and time derivative of the Jacobian matrix for a 2-DOF
//! planar robotic finger. The finger consists of two links connected by revolute
//! joints, allowing it to operate in a 2D plane.
namespace finger_kinematics {
    inline Eigen::Vector2d forward(
        const Eigen::Vector2d& joint_position,
        const Eigen::Vector2d& link_length)
    {
        double l1 = link_length(0);
        double l2 = link_length(1);

        double q1 = joint_position(0);
        double q2 = joint_position(1);

        double x = l1 * cos(q1) + l2 * cos(q1 + q2);
        double y = l1 * sin(q1) + l2 * sin(q1 + q2);

        return Eigen::Vector2d(x, y);
    }

    inline Eigen::Matrix2d jacobian(
        const Eigen::Vector2d& joint_position,
        const Eigen::Vector2d& link_length)
    {
        double l1 = link_length(0);
        double l2 = link_length(1);

        double q1 = joint_position(0);
        double q2 = joint_position(1);

        double j11 = -l1 * sin(q1) - l2 * sin(q1 + q2);
        double j12 = -l2 * sin(q1 + q2);
        double j21 = l1 * cos(q1) + l2 * cos(q1 + q2);
        double j22 = l2 * cos(q1 + q2);

        Eigen::Matrix2d J;
        J << j11, j12,
             j21, j22;

        return J;
    }

    inline Eigen::Matrix2d dotJacobian(
        const Eigen::Vector2d& joint_position,
        const Eigen::Vector2d& joint_velocity,
        const Eigen::Vector2d& link_length)
    {
        double l1 = link_length(0);
        double l2 = link_length(1);

        double q1 = joint_position(0);
        double q2 = joint_position(1);

        double dot_q1 = joint_velocity(0);
        double dot_q2 = joint_velocity(1);

        double dot_j11 = -l1 * cos(q1) * dot_q1 - l2 * cos(q1 + q2) * (dot_q1 + dot_q2);
        double dot_j12 = -l2 * cos(q1 + q2) * (dot_q1 + dot_q2);
        double dot_j21 = -l1 * sin(q1) * dot_q1 - l2 * sin(q1 + q2) * (dot_q1 + dot_q2);
        double dot_j22 = -l2 * sin(q1 + q2) * (dot_q1 + dot_q2);

        Eigen::Matrix2d dot_J;
        dot_J << dot_j11, dot_j12,
                 dot_j21, dot_j22;
        return dot_J;
    }
}

#endif // FINGER_KINEMATICS_HPP