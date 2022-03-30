#include "artery/traci/PersonController.h"
#include "artery/traci/Cast.h"
#include "traci/VariableCache.h"
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/systems/si/acceleration.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/velocity.hpp>

namespace si = boost::units::si;

namespace traci
{

PersonController::PersonController(std::shared_ptr<traci::API> p_api, const std::string& p_id) :
    PersonController(p_api, std::make_shared<PersonCache>(p_api, p_id))
{
}

PersonController::PersonController(std::shared_ptr<traci::API> p_api, std::shared_ptr<PersonCache> p_cache) :
    pm_traci(p_api), pm_boundary(p_api->simulation.getNetBoundary()),
    pm_type(p_api->person, p_api->person.getTypeID(p_cache->getId())),
    pm_cache(p_cache)
{
}

const std::string& PersonController::getPersonId() const
{    
    return pm_cache->getId();
} 

const PersonType& PersonController::getPersonType() const
{

    return pm_type;
}

const std::string PersonController::getPersonRoadID() const
{
    return pm_traci->person.getRoadID(pm_cache->getId());
    //return pm_cache->get<libsumo::CMD_GET_EDGE_VARIABLE>();
    
} 

artery::Position PersonController::getPosition() const
{
    return traci::position_cast(pm_boundary, pm_cache->get<libsumo::VAR_POSITION>());
}

auto PersonController::getGeoPosition() const -> artery::GeoPosition
{
    TraCIPosition traci_pos = pm_cache->get<libsumo::VAR_POSITION>();

    TraCIGeoPosition traci_geo = pm_traci->convertGeo(traci_pos);
    artery::GeoPosition geo;
    geo.latitude = traci_geo.latitude * boost::units::degree::degree;
    geo.longitude = traci_geo.longitude * boost::units::degree::degree;
    return geo;
}

auto PersonController::getHeading() const -> artery::Angle
{
    using namespace traci;
    return angle_cast(TraCIAngle { pm_cache->get<libsumo::VAR_ANGLE>() });
}

auto PersonController::getSpeed() const -> Velocity
{
    return pm_cache->get<libsumo::VAR_SPEED>() * si::meter_per_second;
}

void PersonController::setSpeed(Velocity v)
{
    pm_traci->person.setSpeed(pm_cache->getId(), v / si::meter_per_second);
}

const std::string PersonController::getVehicleClass() const //StationType
{
    return pm_cache->get<libsumo::VAR_VEHICLECLASS>();
}

/*

auto PersonController::getMaxSpeed() const -> Velocity
{
    return pm_cache->get<libsumo::VAR_MAXSPEED>() * si::meter_per_second;
}

void PersonController::setMaxSpeed(Velocity v)
{
    pm_traci->vehicle.setMaxSpeed(m_cache->getId(), v / si::meter_per_second);
}

void PersonController::setSpeed(Velocity v)
{
    m_traci->vehicle.setSpeed(m_cache->getId(), v / si::meter_per_second);
}

void PersonController::setSpeedFactor(double f)
{
    m_traci->vehicle.setSpeedFactor(m_cache->getId(), f);
}

void PersonController::setSpeedMode(int m)
{
    m_traci->vehicle.setSpeedMode(m_cache->getId(), m);
}

*/

auto PersonController::getWidth() const -> Length
{
    return pm_cache->get<libsumo::VAR_WIDTH>() * si::meter;
}



auto PersonController::getLength() const -> Length
{
    return pm_cache->get<libsumo::VAR_LENGTH>() * si::meter;
}



/*
void PersonController::changeTarget(const std::string& edge)
{
    m_traci->vehicle.changeTarget(m_cache->getId(), edge);
}
*/

}// namespace traci