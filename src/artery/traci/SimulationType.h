#ifndef SIMULATIONTYPE_H_
#define SIMULATIONTYPE_H_

#include "traci/API.h"
#include <string>

namespace traci
{

class SimulationType
{
public:

    SimulationType(const traci::API::SimulationScope&);

    const std::vector<std::string> getDepartedList() const;
    //const std::vector<std::string> getArrivedList() const;

private:
    const traci::API::SimulationScope& pm_api;
};

} // namespace traci

#endif /* SIMULATIONTYPE_H_ */