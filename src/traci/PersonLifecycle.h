/*
 * Artery V2X Simulation Framework
 * Copyright 2020 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef TRACI_PERSONLIFECYCLE_H_9CW2JCKB
#define TRACI_PERSONLIFECYCLE_H_9CW2JCKB

#include <string>

namespace traci
{

/**
 * VehicleLifecycle is an interface employed by VehiclePolicy.
 * It enables policies to trigger actions (add, update and remove) unsolicitedly,
 * e.g. after expiry of a local timer or a condition.
 */
class PersonLifecycle
{
public:
    virtual void addPerson(const std::string& id) = 0;
    virtual void updatePerson(const std::string& id) = 0;
    virtual void removePerson(const std::string& id) = 0;

    virtual ~PersonLifecycle() = default;
};

} // namespace traci

#endif /* TRACI_PERSONLIFECYCLE_H_9CW2JCKB */

