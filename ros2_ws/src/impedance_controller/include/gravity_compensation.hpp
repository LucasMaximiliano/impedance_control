#ifndef GRAVITY_COMPENSATION_HPP
#define GRAVITY_COMPENSATION_HPP

#include <Eigen/Dense>

//! \name computeGravityTorque
//! \brief Computes the gravity compensation torque for a 2-DOF planar robotic finger
//! \details This function calculates the gravity compensation torque required at each joint
//! of a 2-DOF planar robotic finger with revolute joints. The calculation is based on the
//! link masses, link lengths, distances to the center of mass, and current joint positions.
//! It assumes a standard gravitational acceleration of 9.81 m/s² pointing downwards i.e. in
//! the negative y-direction of the finger base coordinate system S_0.
//! \param link_mass_vector Vector containing the masses of the links (2x1)
//! \param link_length_vector Vector containing the lengths of the links (2x1)
//! \param dist_to_link_com_vector Vector containing the distances from each joint to the center of mass of the respective link (2x1)
//! \param joint_position Vector containing the current joint positions (2x1)
//! \return Gravity compensation torque vector (2x1)
inline Eigen::Vector2d computeGravityTorque(
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