#ifndef SIMULATIONCONTROLLER_H
#define SIMULATIONCONTROLLER_H

#include "artery/traci/SimulationType.h"
#include "artery/utility/Geometry.h"
#include "traci/API.h"
#include "traci/VariableCache.h"
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/length.hpp>
#include <vanetza/units/velocity.hpp>
#include <string>

namespace traci
{
class SimulationCache;

class SimulationScopeController
{
public:
    
    SimulationScopeController(std::shared_ptr<traci::API>);
    SimulationScopeController(std::shared_ptr<traci::API>, std::shared_ptr<SimulationCache> p_cache);

    const std::vector<std::string> getDepartedPersonIDList() const;
    const std::vector<std::string> getArrivedPersonIDList() const;

    std::shared_ptr<traci::API> getTraCI() { return m_traci; }
    std::shared_ptr<const traci::API> getTraCI() const { return m_traci; }
    
private:
    std::shared_ptr<traci::API> m_traci;    
    std::shared_ptr<SimulationCache> m_cache;   
    SimulationType m_type;

    };
} // namespace traci


#endif /* SIMULATIONCONTROLLER_H */