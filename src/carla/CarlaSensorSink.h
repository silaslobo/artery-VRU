#ifndef CARLA_SENSOR_SINK_H_5RTW7J3D
#define CARLA_SENSOR_SINK_H_5RTW7J3D

#include <map>
#include <string>

#include <omnetpp/clistener.h>
#include <omnetpp/csimplemodule.h>

namespace artery
{
namespace carla
{

class CarlaSensorReading;

/**
 * Example consumer of the "carla.sensor" signal.
 *
 * Shows how downstream middleware subscribes to CARLA sensor data: the readings
 * are emitted by CarlaScenarioManager inside the OMNeT++ event loop, so any
 * module can receive them through a regular signal hook.
 */
class CarlaSensorSink : public omnetpp::cSimpleModule, public omnetpp::cListener
{
public:
    CarlaSensorSink() = default;

    void initialize(int stage) override;
    void finish() override;

    void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID,
        omnetpp::cObject* obj, omnetpp::cObject* details) override;

    uint64_t count(const std::string& kind) const;

protected:
    virtual void handleReading(omnetpp::cComponent* source, const CarlaSensorReading& reading);

private:
    std::map<std::string, uint64_t> mCounts;
};

}  // namespace carla
}  // namespace artery

#endif /* CARLA_SENSOR_SINK_H_5RTW7J3D */
