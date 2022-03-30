/*
 * Artery V2X Simulation Framework
 * Copyright 2020 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef TRACI_INSERTIONDELAYPERSONPOLICY_H_CULRKQOD
#define TRACI_INSERTIONDELAYPERSONPOLICY_H_CULRKQOD

#include "traci/PersonPolicy.h"
#include <boost/bimap/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <omnetpp/cmessage.h>
#include <omnetpp/simtime.h>

namespace traci
{

/**
 * This policy inserts vehicles only after some delay.
 */
class InsertionDelayPersonPolicy : public PersonPolicy
{
public:
    InsertionDelayPersonPolicy();
    ~InsertionDelayPersonPolicy();

    void initialize(PersonLifecycle*) override;
    Decision addPerson(const std::string& id) override;
    Decision updatePerson(const std::string& id) override;
    Decision removePerson(const std::string& id) override;

protected:
    void handleMessage(omnetpp::cMessage* msg) override;

private:
    using InsertionQueue = boost::bimaps::bimap<std::string, boost::bimaps::multiset_of<omnetpp::SimTime>>;

    void schedulePersonInsertion();

    PersonLifecycle* m_lifecycle = nullptr;
    omnetpp::cMessage* m_insert_event = nullptr;
    InsertionQueue m_insert_queue;
};

} // namespace traci

#endif /* TRACI_INSERTIONDELAYPERSONPOLICY_H_CULRKQOD */

