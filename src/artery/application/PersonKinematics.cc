#include "artery/application/PersonKinematics.h"
#include "artery/traci/PersonController.h"
#include <limits>

namespace artery
{
PersonKinematics::PersonKinematics():
    p_acceleration(vanetza::units::Acceleration::from_value(std::numeric_limits<double>::quiet_NaN())),
    p_yaw_rate(vanetza::units::AngularVelocity::from_value(std::numeric_limits<double>::quiet_NaN()))
{        
}

PersonKinematics p_getKinematics(const traci::PersonController& p_controller)
{
    artery::PersonKinematics p_kinematics;
    p_kinematics.p_position = p_controller.getPosition();
    p_kinematics.p_geo_position = p_controller.getGeoPosition();
    p_kinematics.p_speed = p_controller.getSpeed();
    p_kinematics.p_heading = p_controller.getHeading().getTrueNorth();
    return p_kinematics;
} 
} // namespace artery