#ifndef GRAVITY_COMPENSATION_HPP
#define GRAVITY_COMPENSATION_HPP

#include <Eigen/Dense>

Eigen::Vector2d computeGravityTorque(
    const Eigen::Vector2d& link_mass_vector,
    const Eigen::Vector2d& link_length_vector,
    const Eigen::Vector2d& dist_to_link_com_vector,
    const Eigen::Vector2d& joint_position)
{
    const double g = 9.81; // Acceleration due to gravity (m/s^2)

    double m1 = link_mass_vector(0);
    double m2 = link_mass_vector(1);
    double l1 = link_length_vector(0);
    double d1 = dist_to_link_com_vector(0);
    double d2 = dist_to_link_com_vector(1);

    double q1 = joint_position(0);
    double q2 = joint_position(1);

    Eigen::Vector2d gravity_torque = Eigen::Vector2d::Zero();
    gravity_torque(0) = g * (m1 * d1 * cos(q1) + m2 * (l1 * cos(q1) + d2 * cos(q1 + q2)));
    gravity_torque(1) = g * (m2 * d2 * cos(q1 + q2));

    return gravity_torque;
}

#endif // GRAVITY_COMPENSATION_HPP