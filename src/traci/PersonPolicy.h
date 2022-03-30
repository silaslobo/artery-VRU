#ifndef TRACI_PERSONPOLICY_H_SV6K4QXZ
#define TRACI_PERSONPOLICY_H_SV6K4QXZ

#include <string>
#include <omnetpp/csimplemodule.h>

namespace traci
{

class PersonLifecycle;

/**
 * VehiclePolicy enable custom behaviours regarding the lifecycle of vehicle nodes.
 */
class PersonPolicy : public omnetpp::cSimpleModule
{
public:
    enum class Decision {
        Continue, // continue with next policy
        Discard, // stop processing of policies, perform no action now (add, update or remove)
    };

    /**
     * Initialize policy when lifecycle actions become possible (TraCI connection is active)
     * \param lc lifecycle instance to trigger actions explicitly
     */
    virtual void initialize(PersonLifecycle*) {};

    /**
     * Hosting NodeManager wants to add a new vehicle (departed right now)
     * \param id TraCI ID of vehicle
     * \return decision if vehicle shall be added now
     */
    virtual Decision addPerson(const std::string& id) { return Decision::Continue; }

    /**
     * Hosting NodeManager received updates for a vehicle
     * \param id TraCI ID of vehicle
     * \return decision if vehicle shall be updated (it may not have been added previously)
     */
    virtual Decision updatePerson(const std::string& id) { return Decision::Continue; }

    /**
     * Hosting NodeManager wants to remove a vehicle that arrived at its destination
     * \param id TraCI ID of vehicle
     * \return decision if vehicle shall be removed now
     */
    virtual Decision removePerson(const std::string& id) { return Decision::Continue; }

    virtual ~PersonPolicy() = default;
};

} // namespace traci

#endif /* TRACI_PERSONPOLICY_H_SV6K4QXZ */