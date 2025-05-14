/*
 * Artery V2X Simulation Framework
 * Copyright 2020 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef ARTERY_KINEMATICS_H_VDOQF76E
#define ARTERY_KINEMATICS_H_VDOQF76E

#include "artery/utility/Geometry.h"
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <vanetza/units/angular_velocity.hpp>

// forward declaration
namespace traci { class EntityController; }

namespace artery
{

/**
 * EntityKinematics stores attributes describing an entity's kinematic
 */
struct EntityKinematics
{
    EntityKinematics();

    Position position;
    GeoPosition geo_position;
    vanetza::units::Velocity speed;
    vanetza::units::Acceleration acceleration;
    vanetza::units::Angle heading; // from north, clockwise
    vanetza::units::AngularVelocity yaw_rate;
};

EntityKinematics getKinematics(const traci::EntityController&);

} // namespace artery

#endif /* ARTERY_KINEMATICS_H_VDOQF76E */
