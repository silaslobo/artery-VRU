#include "artery/traci/SimulationType.h"

namespace traci
{
SimulationType::SimulationType(const traci::API::SimulationScope& api) :
    pm_api(api)
{
}

const std::vector<std::string> SimulationType::getDepartedList() const
{
    return pm_api.getDepartedPersonIDList();
}

}// namespace traci