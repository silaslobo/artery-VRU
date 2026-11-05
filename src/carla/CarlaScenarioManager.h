#ifndef CARLA_SCENARIO_MANAGER_H_8Q4MZF2C
#define CARLA_SCENARIO_MANAGER_H_8Q4MZF2C

#include <memory>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <omnetpp/cmessage.h>
#include <omnetpp/csimplemodule.h>
#include <omnetpp/simtime.h>

#include "CarlaSensorReading.h"

namespace carla
{
namespace client
{
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
 *   - spawn an ego vehicle plus a configurable set of sensors
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

    /// CARLA time (seconds since episode start) matching the last step
    double carlaTime() const
    {
        return m_carlaTime;
    }

    /// Convert a CARLA episode timestamp into OMNeT++ simulation time
    omnetpp::SimTime toSimTime(double carlaTimestamp) const;

    /// id of the spawned ego vehicle (-1 if none)
    int32_t egoVehicleId() const
    {
        return m_egoId;
    }

protected:
    void connect();
    void spawnEgoVehicle();
    void spawnSensors();
    void step();
    void drainInbox();
    void stopSensors();
    void cleanupActors();
    /// release all CARLA objects without letting exceptions escape
    void disconnect();

    /// move pending measurements out of the inbox (event loop thread only)
    std::vector<RawSensorData> takePending();

private:
    omnetpp::cMessage* m_connectEvent;
    omnetpp::cMessage* m_stepEvent;
    omnetpp::SimTime m_stepInterval;
    omnetpp::SimTime m_sensorWaitTimeout;
    omnetpp::SimTime m_timeout;
    omnetpp::SimTime m_tickTimeout;
    unsigned m_tickRetries {2u};

    std::shared_ptr<SensorInbox> m_inbox;

    std::unique_ptr<::carla::client::Client> m_client;
    std::unique_ptr<::carla::client::World> m_world;
    /// previous server settings, restored on finish()
    std::unique_ptr<::carla::rpc::EpisodeSettings> m_savedSettings;

    boost::shared_ptr<::carla::client::Vehicle> m_ego;
    std::vector<boost::shared_ptr<::carla::client::Sensor>> m_sensors;

    std::string m_host;
    uint16_t m_port;
    bool m_synchronous;
    int32_t m_egoId {-1};

    uint64_t m_frame {0};
    double m_carlaTime {0.0};
    /// carla timestamp - sim time, established with the first received frame
    double m_carlaEpoch {0.0};
    bool m_epochSet {false};

    size_t m_droppedReadings {0};
};

}  // namespace carla
}  // namespace artery

#endif /* CARLA_SCENARIO_MANAGER_H_8Q4MZF2C */
