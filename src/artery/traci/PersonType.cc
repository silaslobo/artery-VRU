#include "artery/traci/PersonType.h"
#include <boost/units/systems/si/acceleration.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/velocity.hpp>

namespace si = boost::units::si;

namespace traci
{
PersonType::PersonType(const traci::API::PersonScope& api, const std::string& id) :
    pm_api(api), pm_id(id)
{
}

const std::string& PersonType::getTypeId() const
{
    return pm_id;
}

auto PersonType::getSpeed() const -> Velocity
{
    return pm_api.getSpeed(pm_id) * si::meter_per_second; 
}



auto PersonType::getLength() const -> Length
{
    return pm_api.getLength(pm_id) * si::meter;
}

}// namespace traci
