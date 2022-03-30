/*
 * Artery V2X Simulation Framework
 * Copyright 2020 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef TRACI_EXTENSIBLENODEMANAGER_H_7AC9LLFH
#define TRACI_EXTENSIBLENODEMANAGER_H_7AC9LLFH

#include "traci/BasicNodeManager.h"
#include "traci/VehicleLifecycle.h"
#include "traci/VehiclePolicy.h"
#include "traci/PersonLifecycle.h"
#include "traci/PersonPolicy.h"
#include <list>
#include <vector>

namespace traci
{

/**
 * This node manager is extensible via configurable VehiclePolicy sub-modules.
 *
 * Each VehiclePolicy is invoked on vehicle events (add, update or remove) and
 * can decide on further processing of the particular vehicle. If no policy
 * does actively discard an event, the default action will be performed after
 * interrogation of all policies.
 */
class ExtensibleNodeManager : public BasicNodeManager
{
protected:
    void initialize() override;
    void finish() override;

    void traciInit() override;

    void processVehicles() override;
    void addVehicle(const std::string&) override;
    void updateVehicle(const std::string&, VehicleSink*) override;
    void removeVehicle(const std::string&) override;

    friend class VehicleLifecycle;
    void addVehicle(const VehiclePolicy* omit, const std::string&);
    void removeVehicle(const VehiclePolicy* omit, const std::string&);
    void updateVehicle(const VehiclePolicy* omit, const std::string&, VehicleSink*);

    void processPersons() override;
    void addPerson(const std::string&) override;
    void updatePerson(const std::string&, PersonSink*) override;
    void removePerson(const std::string&) override;

    friend class PersonLifecycle;
    void addPerson(const PersonPolicy* omit, const std::string&);
    void removePerson(const PersonPolicy* omit, const std::string&);
    void updatePerson(const PersonPolicy* omit, const std::string&, PersonSink*);


private:
    class VehicleLifecycle : public traci::VehicleLifecycle
    {
    public:
        VehicleLifecycle(ExtensibleNodeManager* manager, const VehiclePolicy* ext);
        void addVehicle(const std::string& id) override;
        void removeVehicle(const std::string& id) override;
        void updateVehicle(const std::string& id) override;

    private:
        ExtensibleNodeManager* m_manager;
        const VehiclePolicy* m_policy;
    };

    class PersonLifecycle : public traci::PersonLifecycle
    {
    public:
        PersonLifecycle(ExtensibleNodeManager* manager, const PersonPolicy* ext);
        void addPerson(const std::string& id) override;
        void removePerson(const std::string& id) override;
        void updatePerson(const std::string& id) override;

    private:
        ExtensibleNodeManager* m_manager;
        const PersonPolicy* m_policy;
    };

    std::vector<VehiclePolicy*> m_policies;
    std::list<VehicleLifecycle> m_lifecycles;
    std::vector<std::string> m_remove_vehicles;

    std::vector<PersonPolicy*> pm_policies;
    std::list<PersonLifecycle> pm_lifecycles;
    std::vector<std::string> m_remove_persons;
};

} // namespace traci

#endif /* TRACI_EXTENSIBLENODEMANAGER_H_7AC9LLFH */

