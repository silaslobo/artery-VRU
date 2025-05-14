#include "artery/application/EntityKinematics.h"
#include "artery/traci/VehicleController.h"
#include "artery/traci/PersonController.h"
#include <limits>

namespace artery
{

EntityKinematics::EntityKinematics() :
    acceleration(vanetza::units::Acceleration::from_value(std::numeric_limits<double>::quiet_NaN())),
    yaw_rate(vanetza::units::AngularVelocity::from_value(std::numeric_limits<double>::quiet_NaN()))
{
}

EntityKinematics getKinematics(const traci::EntityController& controller)
{
    artery::EntityKinematics kinematics;
    kinematics.position = controller.getPosition();
    kinematics.geo_position = controller.getGeoPosition();
    kinematics.speed = controller.getSpeed();
    kinematics.heading = controller.getHeading().getTrueNorth();
    return kinematics;
}

} // namespace artery
