#ifndef COORDINATE_TRANSFORM_HPP
#define COORDINATE_TRANSFORM_HPP

#include <Eigen/Dense>

//! \namespace coordinate_transform
//! \brief Functions for transforming between incoming joint angles, velocities, and torques
//! and the Denavit-Hartenberg (DH) convention used within this package.
//! \details This namespace provides functions to convert the quantities mentioned above from
//! the incoming format (as received from the sensor feedback and trajectory planner) to the
//! DH format, as well as the corresponding inverse transformations.
namespace coordinate_transform {
    //! \{
    //! \name Coordinate transforms in the input direction
    inline Eigen::Vector2d incoming_angles_to_DH_angles(const Eigen::Vector2d incoming_angles)
    {
        Eigen::Vector2d DH_angles;
        DH_angles(0) = M_PI - incoming_angles(0);
        DH_angles(1) = incoming_angles(1);
        return DH_angles;
    }
    inline Eigen::Vector2d incoming_velocities_to_DH_velocities(const Eigen::Vector2d incoming_velocities)
    {
        Eigen::Vector2d DH_velocities;
        DH_velocities(0) = -incoming_velocities(0);
        DH_velocities(1) = incoming_velocities(1);
        return DH_velocities;
    }
    inline Eigen::Vector2d incoming_torques_to_DH_torques(const Eigen::Vector2d incoming_torques)
    {
        Eigen::Vector2d DH_torques;
        DH_torques(0) = -incoming_torques(0);
        DH_torques(1) = incoming_torques(1);
        return DH_torques;
    }
    //! \}
    
    //! \{
    //! \name Coordinate transforms in the output direction
    inline Eigen::Vector2d DH_angles_to_outgoing_angles(const Eigen::Vector2d DH_angles)
    {
        Eigen::Vector2d outgoing_angles;
        outgoing_angles(0) = M_PI - DH_angles(0);
        outgoing_angles(1) = DH_angles(1);
        return outgoing_angles;
    }
    inline Eigen::Vector2d DH_velocities_to_outgoing_velocities(const Eigen::Vector2d DH_velocities)
    {
        Eigen::Vector2d outgoing_velocities;
        outgoing_velocities(0) = -DH_velocities(0);
        outgoing_velocities(1) = DH_velocities(1);
        return outgoing_velocities;
    }
    inline Eigen::Vector2d DH_torques_to_outgoing_torques(const Eigen::Vector2d DH_torques)
    {
        Eigen::Vector2d outgoing_torques;
        outgoing_torques(0) = -DH_torques(0);
        outgoing_torques(1) = DH_torques(1);
        return outgoing_torques;
    }
    //! \}
}

#endif // COORDINATE_TRANSFORM_HPP