#ifndef VEHICLECONTROLLER_H_AXBS5NQM
#define VEHICLECONTROLLER_H_AXBS5NQM

#include "artery/traci/EntityController.h"
#include "artery/traci/VehicleType.h"
#include "traci/VariableCache.h"

namespace traci
{

class VehicleCache;

class VehicleController : public EntityController
{
public:
    VehicleController(std::shared_ptr<traci::API>, const std::string& id);
    VehicleController(std::shared_ptr<traci::API>, std::shared_ptr<VehicleCache> cache);

    const std::string& getVehicleId() const;
    std::string getTypeId() const;
    const VehicleType& getVehicleType() const;
    const std::string getVehicleClass() const;

    artery::Position getPosition() const override;
    artery::GeoPosition getGeoPosition() const override;
    artery::Angle getHeading() const override;
    Velocity getSpeed() const override;
    Velocity getMaxSpeed() const;
    void setMaxSpeed(Velocity);
    void setSpeed(Velocity);
    void setSpeedMode(int);
    void setSpeedFactor(double);
    void getParameter() const;

    Length getLength() const;
    Length getWidth() const;

    void changeTarget(const std::string& edge);

private:
    traci::Boundary m_boundary;
    VehicleType m_type;
    std::shared_ptr<VehicleCache> m_cache;
};

} // namespace traci

#endif /* VEHICLECONTROLLER_H_AXBS5NQM */

