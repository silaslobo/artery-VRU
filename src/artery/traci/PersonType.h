#ifndef PERSONTYPE_H_
#define PERSONTYPE_H_

#include "traci/API.h"
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/length.hpp>
#include <vanetza/units/velocity.hpp>
#include <string>

namespace traci
{

class PersonType
{
public:
    using Acceleration = vanetza::units::Acceleration;
    using Length = vanetza::units::Length;
    using Velocity = vanetza::units::Velocity;

    PersonType(const traci::API::PersonScope&, const std::string& p_id);

    const std::string& getTypeId() const;
    //std::string getVehicleClass() const;
    Velocity getSpeed() const;
    //Acceleration getMaxAcceleration() const;
    //Acceleration getMaxDeceleration() const;
    Length getLength() const;
    Length getWidth() const;
    //Length getHeight() const;

private:
    std::string pm_id;
    const traci::API::PersonScope& pm_api;
};

} // namespace traci

#endif /* PERSONTYPE_H_ */