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

void ObjIdentification::objIdentfy() const
{

    std::vector<std::string> person;
    std::vector<std::string> vehicle;

    person = m_traci->simulation.getDepartedPersonIDList();
    vehicle = m_traci->simulation.getDepartedIDList();

    std::cout << "Time: " << m_traci->simulation.getCurrentTime() << std::endl; 
    std::cout << "Person: " << m_traci->simulation.getDepartedPersonNumber() << std::endl;
    std::cout << "Vehicle: " << m_traci->simulation.getDepartedNumber() << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    //m_objMap.insert(std::pair<int, std::vector<std::string>>(m_traci->simulation.getDepartedPersonNumber(),vehicle));
    

    //return m_objMap;

}

} // namespace traci