#include "CarlaAvatar.h"

#include "CarlaScenarioManager.h"
#include "traci/VariableCache.h"

using namespace omnetpp;

Define_Module(artery::carla::CarlaAvatar)

namespace artery
{
namespace carla
{

CarlaAvatar::CarlaAvatar()
    : m_manager(nullptr),
      m_zOffset(0.0),
      m_acquired(false)
{
}

CarlaAvatar::~CarlaAvatar()
{
    // finish() normally releases the avatar; this covers aborted runs
    if (m_acquired && m_manager != nullptr) {
        try {
            m_manager->releaseAvatar(m_vehicleId);
        }
        catch (const std::exception&) {
            // the CARLA server may be gone already; destructors must not throw
        }
        m_acquired = false;
    }
}

void CarlaAvatar::initialize()
{
    cSimpleModule::initialize();
    // nothing to do: traci::BasicNodeManager calls initializeSink() and
    // initializeVehicle() *before* callInitialize(), so all setup happens
    // there.
}

void CarlaAvatar::finish()
{
    if (m_acquired && m_manager != nullptr) {
        m_manager->releaseAvatar(m_vehicleId);
        m_acquired = false;
    }
    cSimpleModule::finish();
}

void CarlaAvatar::initializeSink(std::shared_ptr<traci::API>,
    std::shared_ptr<traci::VehicleCache> cache, const traci::Boundary&)
{
    // called by the node manager before initialize(), the module is already
    // attached to the module tree
    m_vehicleId = cache->getId();
    m_blueprint = par("vehicleBlueprint").stdstringValue();
    m_zOffset = par("zOffset").doubleValue();

    const std::string path = par("managerModule").stdstringValue();
    m_manager = resolveManager(path);
    if (m_manager == nullptr) {
        throw cRuntimeError("CarlaAvatar: no CarlaScenarioManager at '%s'",
            path.c_str());
    }
}

CarlaScenarioManager* CarlaAvatar::resolveManager(const std::string& path) const
{
    // a bare name is searched in all ancestor modules (the node module is
    // freshly created by traci::BasicNodeManager, relative '..' paths are
    // awkward before it is fully initialised); anything containing '.', '^'
    // or '/' is interpreted as an OMNeT++ module path
    if (!path.empty() && path.find_first_of(".^/") == std::string::npos) {
        for (cModule* m = getParentModule(); m != nullptr; m = m->getParentModule()) {
            auto* candidate = dynamic_cast<CarlaScenarioManager*>(
                m->getSubmodule(path.c_str()));
            if (candidate != nullptr) {
                return candidate;
            }
        }
        return nullptr;
    }
    return dynamic_cast<CarlaScenarioManager*>(getModuleByPath(path.c_str()));
}

void CarlaAvatar::initializeVehicle(const traci::TraCIPosition& position,
    traci::TraCIAngle angle, double /*speed*/)
{
    m_manager->acquireAvatar(m_vehicleId, m_blueprint, toCarla(position, angle));
    m_acquired = true;
    const auto sensors = par("sensors").stdstringValue();
    if (!sensors.empty()) {
        m_manager->attachAvatarSensors(m_vehicleId, sensors);
    }
}

void CarlaAvatar::updateVehicle(const traci::TraCIPosition& position,
    traci::TraCIAngle angle, double /*speed*/)
{
    if (m_acquired) {
        m_manager->updateAvatar(m_vehicleId, toCarla(position, angle));
    }
}

CarlaPose CarlaAvatar::toCarla(const traci::TraCIPosition& position,
    traci::TraCIAngle angle) const
{
    // SUMO and the OpenDRIVE file share coordinates (netconvert ran with
    // --offset.disable-normalization=true); only the handedness differs
    CarlaPose pose = sumoToCarla(position.x, position.y, position.z, angle.degree);
    pose.z += m_zOffset;
    return pose;
}

}  // namespace carla
}  // namespace artery
