/*
 * Artery V2X Simulation Framework
 * Copyright 2020 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "traci/InsertionDelayPersonPolicy.h"
#include "traci/PersonLifecycle.h"

using namespace omnetpp;

namespace traci
{

Define_Module(InsertionDelayPersonPolicy)

InsertionDelayPersonPolicy::InsertionDelayPersonPolicy() :
    m_insert_event(new omnetpp::cMessage("TraCI vehicle insertion"))
{
}

InsertionDelayPersonPolicy::~InsertionDelayPersonPolicy()
{
    cancelAndDelete(m_insert_event);
}

void InsertionDelayPersonPolicy::handleMessage(cMessage* msg)
{
    if (msg == m_insert_event && !m_insert_queue.empty()) {
        auto next_insertion = m_insert_queue.right.begin();
        const SimTime& when = next_insertion->first;
        const std::string& id = next_insertion->second;
        ASSERT(when == simTime());

        if (m_lifecycle) {
            m_lifecycle->addPerson(id);
        } else {
            EV_FATAL << "no lifecycle interface registered\n";
        }

        m_insert_queue.right.erase(next_insertion);
        schedulePersonInsertion();
    }
}

void InsertionDelayPersonPolicy::schedulePersonInsertion()
{
    cancelEvent(m_insert_event);
    if (!m_insert_queue.empty()) {
        auto next_insertion = m_insert_queue.right.begin();
        const SimTime& when = next_insertion->first;
        ASSERT(when >= simTime());
        scheduleAt(when, m_insert_event);
    }
}

void InsertionDelayPersonPolicy::initialize(PersonLifecycle* lifecycle)
{
    m_lifecycle = lifecycle;
}

PersonPolicy::Decision InsertionDelayPersonPolicy::addPerson(const std::string& id)
{
    Enter_Method_Silent();
    const SimTime when = simTime() + par("insertionDelay");
    m_insert_queue.insert(InsertionQueue::relation(id, when));
    EV_DETAIL << "person " << id << " will be inserted at time " << when << "\n";
    schedulePersonInsertion();
    return Decision::Discard;
}

PersonPolicy::Decision InsertionDelayPersonPolicy::updatePerson(const std::string& id)
{
    Enter_Method_Silent();
    if (m_insert_queue.left.find(id) == m_insert_queue.left.end()) {
        return Decision::Continue;
    } else {
        return Decision::Discard;
    }
}

PersonPolicy::Decision InsertionDelayPersonPolicy::removePerson(const std::string& id)
{
    Enter_Method_Silent();
    const auto found = m_insert_queue.left.find(id);
    if (found == m_insert_queue.left.end()) {
        return Decision::Continue;
    } else {
        m_insert_queue.left.erase(found);
        schedulePersonInsertion();
        return Decision::Discard;
    }
}

} // namespace traci
