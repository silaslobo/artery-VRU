#ifndef PERSONCONTROLLER_H
#define PERSONCONTROLLER_H

#include "artery/traci/PersonType.h"
#include "artery/utility/Geometry.h"
#include "traci/API.h"
#include "traci/VariableCache.h"
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/length.hpp>
#include <vanetza/units/velocity.hpp>
#include <string>

namespace traci
{
class PersonCache;

class PersonController
{
public:
    using Acceleration = vanetza::units::Acceleration;
    using Length = vanetza::units::Length;
    using Velocity = vanetza::units::Velocity;

    PersonController(std::shared_ptr<traci::API>, const std::string& p_id);
    PersonController(std::shared_ptr<traci::API>, std::shared_ptr<PersonCache> p_cache);

    const std::string& getPersonId() const;
    const std::string getVehicleClass() const;
    const std::string getPersonRoadID() const;
    const PersonType& getPersonType() const;

    artery::Position getPosition() const;
    artery::GeoPosition getGeoPosition() const;
    artery::Angle getHeading() const;
    Velocity getSpeed() const;
    //Velocity getMaxSpeed() const;
    //void setMaxSpeed(Velocity);
    void setSpeed(Velocity);
    //void setSpeedMode(int);
    //void setSpeedFactor(double);

    Length getLength() const;
    Length getWidth() const;
        
    void changeTarget(const std::string& edge);

    std::shared_ptr<traci::API> getTraCI() { return pm_traci; }
    std::shared_ptr<const traci::API> getTraCI() const { return pm_traci; }
    
private:
    std::shared_ptr<traci::API> pm_traci;
    traci::Boundary pm_boundary;
    PersonType pm_type;
    std::shared_ptr<PersonCache> pm_cache;   

    };
} // namespace traci


#endif /* PERSONCONTROLLER_H */