#include "artery/traci/Cast.h"
#include "artery/traci/PersonMobility.h"
#include <omnetpp/cwatch.h>

using namespace traci;

namespace artery
{

void PersonMobility::initializeSink(std::shared_ptr<API> api, std::shared_ptr<PersonCache> cache, const Boundary& boundary)
{
    ASSERT(api);
    ASSERT(cache);
    //mCache = cache;
    mTraci = api;
    mNetBoundary = boundary;
    mPersonId = cache->getId();
    mController.reset(new PersonController(api, cache));
}

void PersonMobility::initializePerson(const TraCIPosition& traci_pos, TraCIAngle traci_heading, double traci_speed)
{
    //mPosition = position_cast(mNetBoundary, traci_pos);
    //mHeading = angle_cast(traci_heading);
    //mSpeed = traci_speed;
    const auto opp_pos = position_cast(mNetBoundary, traci_pos);
    const auto opp_angle = angle_cast(traci_heading);
    initialize(opp_pos, opp_angle, traci_speed);
}

void PersonMobility::updatePerson(const TraCIPosition& traci_pos, TraCIAngle traci_heading, double traci_speed)
{
    //mPosition = position_cast(mNetBoundary, traci_pos);
    //mHeading = angle_cast(traci_heading);
    //mSpeed = traci_speed;
    const auto opp_pos = position_cast(mNetBoundary, traci_pos);
    const auto opp_angle = angle_cast(traci_heading);
    update(opp_pos, opp_angle, traci_speed);
    //update(mPosition, mHeading, mSpeed);
}

/*
GeoPosition PersonMobility::getGeoPosition() const
{
    TraCIPosition pos = mCache->get<libsumo::VAR_POSITION>();
    TraCIGeoPosition geo = mTraci->convertGeo(pos);
    GeoPosition result;
    result.latitude = geo.latitude * boost::units::degree::degree;
    result.longitude = geo.longitude * boost::units::degree::degree;
    return result;
}*/

PersonController* PersonMobility::getPersonController()
{
    ASSERT(mController);
    return mController.get();
}

} // namespace artery
