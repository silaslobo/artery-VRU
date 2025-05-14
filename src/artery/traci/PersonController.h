#ifndef PERSONCONTROLLER_H
#define PERSONCONTROLLER_H

#include "artery/traci/EntityController.h"
#include "artery/traci/PersonType.h"
#include "traci/VariableCache.h"

namespace traci
{
class PersonCache;

class PersonController : public EntityController
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
    const libsumo::TraCIPositionVector& getBoundary() const;

    const std::string getvehicle() const;


    artery::Position getPosition() const override;
    artery::GeoPosition getGeoPosition() const override;
    artery::Angle getHeading() const override;
    Velocity getSpeed() const override;
    //Velocity getMaxSpeed() const;
    //void setMaxSpeed(Velocity);
    void setSpeed(Velocity);
    //void setSpeedMode(int);
    //void setSpeedFactor(double);

    Length getLength() const;
    Length getWidth() const;
        
    void changeTarget(const std::string& edge);

private:
    traci::Boundary pm_boundary;
    PersonType pm_type;
    std::shared_ptr<PersonCache> pm_cache;   

    };
} // namespace traci


#endif /* PERSONCONTROLLER_H */
