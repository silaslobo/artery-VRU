#ifndef CARLA_SCENARIO_MANAGER_H_8Q4MZF2C
#define CARLA_SCENARIO_MANAGER_H_8Q4MZF2C

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "CoordTransform.h"

// boost::shared_ptr is required by the CARLA 0.9.16 client API: carla::
// SharedPtr<T> is an alias for boost::shared_ptr<T> (LibCarla Memory.h), so
// every handle the client returns is boost-managed.  std::shared_ptr has an
// independent control block, so wrapping those raw pointers would create
// double ownership; CARLA handle storage therefore uses the boost types and
// everything else is std.
#include <boost/shared_ptr.hpp>

#include <omnetpp/cmessage.h>
#include <omnetpp/csimplemodule.h>
#include <omnetpp/simtime.h>

#include "CarlaSensorReading.h"

namespace carla
{
namespace client
{
class Actor;
class Client;
class Sensor;
class Vehicle;
class World;
}  // namespace client
namespace rpc
{
class EpisodeSettings;
}  // namespace rpc
}  // namespace carla

namespace artery
{
namespace carla
{

/**
 * Owns the connection to a CARLA server and drives it in lock-step with the
 * OMNeT++ event loop (the CARLA counterpart of traci::Core).
 *
 * Responsibilities:
 *   - connect to the CARLA server, (optionally) load a map and switch the
 *     server into synchronous mode with a fixed delta time
 *   - spawn a configurable set of sensors; every vehicle in the CARLA world
 *     is a kinematic avatar of an external (SUMO) vehicle
 *   - advance the CARLA world by exactly one frame per OMNeT++ step event
 *   - marshal sensor data arriving on CARLA's streaming threads into the
 *     OMNeT++ event loop and emit it as "carla.sensor" signal
 *
 * Threading: carla::client::Sensor::Listen() callbacks are invoked on CARLA's
 * receiver threads.  They only ever touch the SensorInbox (mutex guarded) and
 * CARLA data structures; all OMNeT++ work happens in handleMessage().
 */
class CarlaScenarioManager : public omnetpp::cSimpleModule
{
public:
    CarlaScenarioManager();
    ~CarlaScenarioManager() override;

    void initialize() override;
    void finish() override;
    void handleMessage(omnetpp::cMessage*) override;

    /// CARLA frame counter of the last completed step
    uint64_t frame() const
    {
        return m_frame;
    }

    // ---------------------------------------------------------------------
    // Avatar API -- kinematic CARLA mirrors of external (SUMO) vehicles.
    //
    // These are called from the OMNeT++ event loop (e.g. by a
    // traci::VehicleSink) and only queue changes; the CARLA RPCs happen in
    // the module's step(), right before World::Tick(), so the mirrored
    // poses are applied within the same lock-step frame.  All poses are in
    // CARLA coordinates (see CoordTransform.h).
    // ---------------------------------------------------------------------

    /// spawn an avatar (or update the request if the key is already known)
    void acquireAvatar(const std::string& key, const std::string& blueprint, const CarlaPose& pose);
    /// queue sensors (same grammar as the "sensors" parameter) for attachment
    /// to an avatar once it exists on the server
    void attachAvatarSensors(const std::string& key, const std::string& spec);
    /// queue a pose update for an existing avatar
    void updateAvatar(const std::string& key, const CarlaPose& pose);
    /// destroy the avatar (no-op if the key is unknown)
    void releaseAvatar(const std::string& key);

    /// residual statistics of the avatar pose verification (see parameter
    /// avatarResidualCheck)
    double residualMax() const
    {
        return m_residualMax;
    }
    double residualMean() const
    {
        return m_residualCount ? m_residualSum / static_cast<double>(m_residualCount) : 0.0;
    }
    uint64_t residualCount() const
    {
        return m_residualCount;
    }

protected:
    void connect();
    /// load a generated map (parameter opendriveFile) via GenerateOpenDriveWorld
    void loadGeneratedMap();
    void spawnSensors();
    void applyAvatarUpdates();
    void measureAvatarResiduals();
    /// compare CARLA's built lane geometry against the generator's lane
    /// centre line samples (parameter mapCheckFile)
    void checkGeneratedGeometry();
    void step();
    void drainInbox(std::vector<RawSensorData> pending);
    void stopSensors();
    void cleanupActors();
    /// release all CARLA objects without letting exceptions escape
    void disconnect();

    /// move pending measurements out of the inbox (event loop thread only)

private:
    omnetpp::cMessage* m_connectEvent;
    omnetpp::cMessage* m_stepEvent;
    omnetpp::SimTime m_stepInterval;
    omnetpp::SimTime m_sensorWaitTimeout;
    omnetpp::SimTime m_timeout;
    omnetpp::SimTime m_tickTimeout;

    std::shared_ptr<SensorInbox> m_inbox;

    std::unique_ptr<::carla::client::Client> m_client;
    std::unique_ptr<::carla::client::World> m_world;
    std::vector<boost::shared_ptr<::carla::client::Sensor>> m_sensors;
    /// names of per-frame sensors (all spawned sensors except event types
    /// like collision); step() waits for all of them to have reported for
    /// the frame that was just ticked (or for sensorWaitTimeout to expire)
    std::set<std::string> m_frameSensorNames;

    struct AvatarEntry
    {
        boost::shared_ptr<::carla::client::Vehicle> actor;  // null until spawned
        CarlaPose pose;  ///< last requested pose (CARLA coordinates)
        std::string blueprint;
        std::string pendingSensors;   ///< sensor spec not yet attached
        bool sensorsAttached {false};
        std::vector<boost::shared_ptr<::carla::client::Sensor>> sensors;
    };
    std::map<std::string, AvatarEntry> m_avatars;
    /// shared implementation of spawnSensors() and the avatar sensors
    void spawnSensorList(::carla::client::Actor* parent, const std::string& text,
        const std::string& name_prefix,
        std::vector<boost::shared_ptr<::carla::client::Sensor>>& out);

    /// actors of released avatars, destroyed at the next step
    std::vector<boost::shared_ptr<::carla::client::Vehicle>> m_avatarsToDestroy;
    double m_residualMax {0.0};
    double m_residualSum {0.0};
    uint64_t m_residualCount {0};

    std::string m_host;
    uint16_t m_port;

    uint64_t m_frame {0};

    /// measurements that arrived after the CARLA frame they belong to had
    /// already been ticked (back-dated by frame distance, see drainInbox)
    uint64_t m_lateReadings {0};
};

}  // namespace carla
}  // namespace artery

#endif /* CARLA_SCENARIO_MANAGER_H_8Q4MZF2C */
