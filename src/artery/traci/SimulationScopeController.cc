#include "artery/traci/SimulationScopeController.h"
#include "artery/traci/Cast.h"
#include "traci/VariableCache.h"

namespace traci
{

SimulationScopeController::SimulationScopeController(std::shared_ptr<traci::API> p_api) :
    SimulationScopeController(p_api, std::make_shared<SimulationCache>(p_api))
{
}

SimulationScopeController::SimulationScopeController(std::shared_ptr<traci::API> p_api, std::shared_ptr<SimulationCache> p_cache) :
    m_traci(p_api), m_type(p_api->simulation), m_cache(p_cache)
{
}

const std::vector<std::string> SimulationScopeController::getDepartedPersonIDList() const
{
    return m_traci->simulation.getDepartedPersonIDList();
}

const std::vector<std::string> SimulationScopeController::getArrivedPersonIDList() const
{
    return m_traci->simulation.getArrivedPersonIDList();
}

}// namespace traci