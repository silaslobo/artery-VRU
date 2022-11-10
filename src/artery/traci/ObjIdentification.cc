#include "artery/traci/ObjIdentification.h"
#include "artery/traci/Cast.h"
#include "traci/VariableCache.h"

namespace traci
{

ObjIdentification::ObjIdentification(std::shared_ptr<traci::API> api) :
    ObjIdentification(api, std::make_shared<SimulationCache>(api))
{
}

ObjIdentification::ObjIdentification(std::shared_ptr<traci::API> api, std::shared_ptr<SimulationCache> cache) :
    m_traci(api), m_boundary(api->simulation.getNetBoundary()),
    m_cache(cache)
{
}

int ObjIdentification::getPersonNumber()
{
    mRunningPerson = m_traci->simulation.getDepartedPersonNumber() - m_traci->simulation.getArrivedPersonNumber();
    return mRunningPerson;
}
} // namespace traci