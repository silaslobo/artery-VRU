#ifndef CARLA_AVATAR_H_
#define CARLA_AVATAR_H_

#include <memory>
#include <string>

#include <omnetpp/csimplemodule.h>

#include "CoordTransform.h"
#include "traci/VehicleSink.h"

namespace artery
{
namespace carla
{

class CarlaScenarioManager;

/**
 * Kinematic CARLA mirror of a SUMO vehicle ("avatar").
 *
 * Lives inside the host module created by traci::BasicNodeManager for every
 * simulated vehicle and implements traci::VehicleSink, i.e. it is driven by
 * the SUMO position updates the node manager already distributes -- no TraCI
 * polling from CARLA threads.
 *
 * Every update is only *queued* at the CarlaScenarioManager; the CARLA RPCs
 * (spawn / SetTransform / destroy) happen inside the manager's step, right
 * before World::Tick(), keeping the lock-step frame contract intact.
 */
class CarlaAvatar : public omnetpp::cSimpleModule, public traci::VehicleSink
{
public:
    CarlaAvatar();
    ~CarlaAvatar() override;

    // traci::VehicleSink interface
    void initializeSink(std::shared_ptr<traci::API>, std::shared_ptr<traci::VehicleCache>,
        const traci::Boundary&) override;
    void initializeVehicle(const traci::TraCIPosition&, traci::TraCIAngle,
        double speed) override;
    void updateVehicle(const traci::TraCIPosition&, traci::TraCIAngle,
        double speed) override;

protected:
    void initialize() override;
    void finish() override;

private:
    CarlaPose toCarla(const traci::TraCIPosition& position,
        traci::TraCIAngle angle) const;
    CarlaScenarioManager* resolveManager(const std::string& path) const;

    CarlaScenarioManager* m_manager;
    std::string m_vehicleId;
    std::string m_blueprint;
    double m_zOffset;
    bool m_acquired;
};

}  // namespace carla
}  // namespace artery

#endif
