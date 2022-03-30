#include "traci/RegionOfInterestPersonPolicy.h"
#include "traci/BasicNodeManager.h"
#include "traci/API.h"
#include "traci/VariableCache.h"
#include "traci/PersonLifecycle.h"
#include <omnetpp/cxmlelement.h>
#include <cassert>

using namespace omnetpp;

namespace traci
{

Define_Module(RegionOfInterestPersonPolicy)

void RegionOfInterestPersonPolicy::initialize(PersonLifecycle* lifecycle)
{
    BasicNodeManager* manager = dynamic_cast<BasicNodeManager*>(getParentModule());
    if (!manager) {
        throw cRuntimeError("Missing traci::BasicNodeManager as parent module");
    }

    /* validate regions */
    cXMLElement* regions = par("regionsOfInterest").xmlValue();
    if (regions) {
        Boundary boundary { manager->getAPI()->simulation.getNetBoundary() };
        m_regions.initialize(*regions, boundary);
        EV_INFO << "Added " << m_regions.size() << " Regions of Interest to simulation" << endl;
    }

    m_lifecycle = lifecycle;
    m_subscriptions = manager->getSubscriptions();
    manager->subscribe(BasicNodeManager::updateNodeSignal, this);
}

void RegionOfInterestPersonPolicy::receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t signal, unsigned long n, omnetpp::cObject*)
{
    if (signal == BasicNodeManager::updateNodeSignal) {
        checkRegionOfInterest();
    }
}

PersonPolicy::Decision RegionOfInterestPersonPolicy::addPerson(const std::string& id)
{
    assert(m_subscriptions);

    if (m_regions.empty()) {
        return Decision::Continue;
    } else {
        /* check if vehicle is in Region of Interest */
        auto person = m_subscriptions->getPersonCache(id);
        if (m_regions.cover(person->get<libsumo::VAR_POSITION>())) {
            /* vehicle was in region and NOT in vehicle list */
            EV_DEBUG << "Person " << id << " is added: departed within region of interest" << endl;
            return Decision::Continue;
        } else {
            EV_DEBUG << "Person " << id << " is not added: departed outside region of interest" << endl;
            m_outside.insert(id);
            return Decision::Discard;
        }
    }
}

PersonPolicy::Decision RegionOfInterestPersonPolicy::updatePerson(const std::string& id)
{
    assert(m_subscriptions);
    assert(m_lifecycle);

    if (m_regions.empty()) {
        return Decision::Continue;
    } else {
        /* check if vehicle is in Region of Interest */
        auto person = m_subscriptions->getPersonCache(id);
        if (m_regions.cover(person->get<libsumo::VAR_POSITION>())) {
            /* vehicle is known and in RoI */
            return Decision::Continue;
        } else {
            /* known vehicle left Region of Interest */
            EV_DEBUG << "Person " << id << " was removed: left region of interest" << endl;
            m_lifecycle->removePerson(id);
            m_outside.insert(id);
            return Decision::Discard;
        }
    }
}

PersonPolicy::Decision RegionOfInterestPersonPolicy::removePerson(const std::string& id)
{
    auto found = m_outside.find(id);
    if (found == m_outside.end()) {
        return Decision::Continue;
    } else {
        m_outside.erase(found);
        return Decision::Discard;
    }
}

void RegionOfInterestPersonPolicy::checkRegionOfInterest()
{
    assert(m_subscriptions);
    assert(m_lifecycle);

    for (auto it = m_outside.begin(); it != m_outside.end();) {
        auto person = m_subscriptions->getPersonCache(*it);
        if (m_regions.cover(person->get<libsumo::VAR_POSITION>())) {
            EV_DEBUG << "Person " << *it << " is added: entered region of interest" << endl;
            m_lifecycle->addPerson(*it);
            it = m_outside.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace traci
