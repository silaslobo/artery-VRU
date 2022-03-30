#ifndef ARTERY_PERSONKINEMATICS_H
#define ARTERY_PERSONKINEMATICS_H

#include "artery/utility/Geometry.h"
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <vanetza/units/angular_velocity.hpp>

// forward declaration
namespace traci { class PersonController; }

namespace artery
{
// PersonKinematics stores attributes describing a person's kinematic

struct PersonKinematics
{
    PersonKinematics();

    Position p_position;
    GeoPosition p_geo_position;
    vanetza::units::Velocity p_speed;
    vanetza::units::Acceleration p_acceleration;
    vanetza::units::Angle p_heading; // from north, clockwise
    vanetza::units::AngularVelocity p_yaw_rate;
};

PersonKinematics p_getKinematics(const traci::PersonController&);

} // namespace artery


#endif ARTERY_PERSONKINEMATICS_H