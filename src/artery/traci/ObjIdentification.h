#ifndef OBJIDENTIFICATION_H_AXBS5NQM
#define OBJIDENTIFICATION_H_AXBS5NQM

#include "traci/API.h"
#include "traci/VariableCache.h"

namespace traci
{

class SimulationCache;

class ObjIdentification
{
public:

    ObjIdentification(std::shared_ptr<traci::API>);
    ObjIdentification(std::shared_ptr<traci::API>, std::shared_ptr<SimulationCache> cache);

    void objIdentfy() const;

private:

    std::map<int, std::vector<std::string>> m_objMap;
    std::shared_ptr<traci::API> m_traci;
    traci::Boundary m_boundary;
    std::shared_ptr<SimulationCache> m_cache;

};

} // namespace traci

#endif /* OBJIDENTIFICATION_H_AXBS5NQM */