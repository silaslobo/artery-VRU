/*
 * Artery V2X Simulation Framework
 * Copyright 2020 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "traci/ExtensibleNodeManager.h"

namespace traci
{

Define_Module(ExtensibleNodeManager)


ExtensibleNodeManager::VehicleLifecycle::VehicleLifecycle(ExtensibleNodeManager* manager, const VehiclePolicy* policy) :
    m_manager(manager), m_policy(policy)
{
}

ExtensibleNodeManager::PersonLifecycle::PersonLifecycle(ExtensibleNodeManager* manager, const PersonPolicy* policy) :
    m_manager(manager), m_policy(policy)
{
}

void ExtensibleNodeManager::VehicleLifecycle::addVehicle(const std::string& id)
{
    m_manager->addVehicle(m_policy, id);
}

void ExtensibleNodeManager::VehicleLifecycle::removeVehicle(const std::string& id)
{
    m_manager->removeVehicle(m_policy, id);
}

void ExtensibleNodeManager::VehicleLifecycle::updateVehicle(const std::string& id)
{
    m_manager->updateVehicle(m_policy, id, m_manager->getVehicleSink(id));
}

void ExtensibleNodeManager::PersonLifecycle::addPerson(const std::string& id)
{
    m_manager->addPerson(m_policy, id);
}

void ExtensibleNodeManager::PersonLifecycle::removePerson(const std::string& id)
{
    m_manager->removePerson(m_policy, id);
}

void ExtensibleNodeManager::PersonLifecycle::updatePerson(const std::string& id)
{
    m_manager->updatePerson(m_policy, id, m_manager->getPersonSink(id));
}


void ExtensibleNodeManager::initialize()
{
    for (omnetpp::cModule::SubmoduleIterator it(this); !it.end(); ++it) {
        VehiclePolicy* policy = dynamic_cast<VehiclePolicy*>(*it);
        if (policy) {
            EV_INFO << "recognized policy " << policy->getNedTypeName() << "\n";
            m_policies.push_back(policy);
        } else {
            PersonPolicy* p_policy = dynamic_cast<PersonPolicy*>(*it);
            EV_INFO << "recognized policy " << p_policy->getNedTypeName() << "\n";
            pm_policies.push_back(p_policy);
        }
    }

    BasicNodeManager::initialize();
}

void ExtensibleNodeManager::finish()
{
    BasicNodeManager::finish();
    m_policies.clear();
    pm_policies.clear();
}

void ExtensibleNodeManager::traciInit()
{
    for (VehiclePolicy* policy : m_policies) {
        EV_DETAIL << "initialize policy " << policy->getNedTypeName() << "\n";
        m_lifecycles.emplace_back(this, policy);
        policy->initialize(&m_lifecycles.back());
    }
    BasicNodeManager::traciInit();
}

void ExtensibleNodeManager::processVehicles()
{
    BasicNodeManager::processVehicles();
    for (const std::string& id : m_remove_vehicles) {
        BasicNodeManager::removeVehicle(id);
    }
    m_remove_vehicles.clear();
}

void ExtensibleNodeManager::processPersons()
{
    BasicNodeManager::processPersons();
    for (const std::string& id : m_remove_persons) {
        BasicNodeManager::removePerson(id);
    }
    m_remove_persons.clear();
}


void ExtensibleNodeManager::addVehicle(const std::string& id)
{
    addVehicle(nullptr, id);
}

void ExtensibleNodeManager::addPerson(const std::string& id)
{
    addPerson(nullptr, id);
}

void ExtensibleNodeManager::addVehicle(const VehiclePolicy* omit, const std::string& id)
{
    for (VehiclePolicy* policy : m_policies) {
        if (policy != omit && policy->addVehicle(id) == VehiclePolicy::Decision::Discard) {
            return;
        }
    }

    BasicNodeManager::addVehicle(id);
}

void ExtensibleNodeManager::addPerson(const PersonPolicy* omit, const std::string& id)
{
    for (PersonPolicy* policy : pm_policies) {
        if (policy != omit && policy->addPerson(id) == PersonPolicy::Decision::Discard) {
            return;
        }
    }

    BasicNodeManager::addPerson(id);
}

void ExtensibleNodeManager::removeVehicle(const std::string& id)
{
    removeVehicle(nullptr, id);
}

void ExtensibleNodeManager::removePerson(const std::string& id)
{
    removePerson(nullptr, id);
}

void ExtensibleNodeManager::removeVehicle(const VehiclePolicy* omit, const std::string& id)
{
    for (VehiclePolicy* policy : m_policies) {
        if (policy != omit && policy->removeVehicle(id) == VehiclePolicy::Decision::Discard) {
            return;
        }
    }

    if (omit == nullptr) {
        BasicNodeManager::removeVehicle(id);
    } else {
        m_remove_vehicles.push_back(id);
    }
}

void ExtensibleNodeManager::removePerson(const PersonPolicy* omit, const std::string& id)
{
    for (PersonPolicy* policy : pm_policies) {
        if (policy != omit && policy->removePerson(id) == PersonPolicy::Decision::Discard) {
            return;
        }
    }

    if (omit == nullptr) {
        BasicNodeManager::removePerson(id);
    } else {
        m_remove_persons.push_back(id);
    }
}

void ExtensibleNodeManager::updateVehicle(const std::string& id, VehicleSink* sink)
{
    updateVehicle(nullptr, id, sink);
}

void ExtensibleNodeManager::updatePerson(const std::string& id, PersonSink* sink)
{
    updatePerson(nullptr, id, sink);
}

void ExtensibleNodeManager::updateVehicle(const VehiclePolicy* omit, const std::string& id, VehicleSink* sink)
{
    for (VehiclePolicy* policy : m_policies) {
        if (policy != omit && policy->updateVehicle(id) == VehiclePolicy::Decision::Discard) {
            return;
        }
    }

    BasicNodeManager::updateVehicle(id, sink);
}

void ExtensibleNodeManager::updatePerson(const PersonPolicy* omit, const std::string& id, PersonSink* sink)
{
    for (PersonPolicy* policy : pm_policies) {
        if (policy != omit && policy->updatePerson(id) == PersonPolicy::Decision::Discard) {
            return;
        }
    }

    BasicNodeManager::updatePerson(id, sink);
}

} // namespace traci
