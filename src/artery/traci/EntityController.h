#ifndef ENTITYCONTROLLER_H
#define ENTITYCONTROLLER_H

#include "artery/utility/Geometry.h"
#include "traci/API.h"
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/length.hpp>
#include <vanetza/units/velocity.hpp>
#include <string>

namespace traci
{

class EntityController
{
public:
    using Acceleration = vanetza::units::Acceleration;
    using Length = vanetza::units::Length;
    using Velocity = vanetza::units::Velocity;

    EntityController(std::shared_ptr<traci::API> traci)
        : traci_(std::move(traci))
    { }

    virtual artery::Position getPosition() const = 0;
    virtual artery::GeoPosition getGeoPosition() const = 0;
    virtual artery::Angle getHeading() const = 0;
    virtual Velocity getSpeed() const = 0;

    std::shared_ptr<traci::API> getTraCI() { return traci_; }
    std::shared_ptr<const traci::API> getTraCI() const { return traci_; }
    
protected:
    std::shared_ptr<traci::API> traci_;

};

} // namespace traci

#endif /* ENTITYCONTROLLER_H */
