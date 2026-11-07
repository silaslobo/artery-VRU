#include "CarlaSensorSink.h"

#include <cstdint>
#include <sstream>
#include <vector>

#include <omnetpp/cmodule.h>
#include <omnetpp/csimulation.h>

#include "CarlaSensorReading.h"

Define_Module(artery::carla::CarlaSensorSink)

using namespace omnetpp;

namespace
{

const simsignal_t sensorSignal = cComponent::registerSignal("carla.sensor");

std::vector<std::string> splitPath(const std::string& path)
{
    std::vector<std::string> names;
    std::string item;
    std::stringstream stream(path);
    while (std::getline(stream, item, '.')) {
        if (!item.empty()) {
            names.push_back(item);
        }
    }
    return names;
}

}  // namespace

namespace artery
{
namespace carla
{

void CarlaSensorSink::initialize(int stage)
{
    cSimpleModule::initialize(stage);

    if (stage == 0) {
        auto path = par("managerModule").stdstringValue();
        cModule* source = getSimulation()->getSystemModule();
        if (!path.empty()) {
            try {
                for (const auto& name : splitPath(path)) {
                    source = source->getSubmodule(name.c_str());
                }
            }
            catch (const std::exception&) {
                throw cRuntimeError(
                    "CarlaSensorSink: could not resolve CARLA manager module '%s'",
                    path.c_str());
            }
        }
        source->subscribe(sensorSignal, this);
        EV_DEBUG << "Subscribed to carla.sensor on " << source->getFullName() << std::endl;
    }
}

void CarlaSensorSink::finish()
{
    getSimulation()->getSystemModule()->unsubscribe(sensorSignal, this);

    uint64_t total = 0;
    for (const auto& entry : mCounts) {
        EV_INFO << "received " << entry.second << " " << entry.first << " measurements"
                << std::endl;
        total += entry.second;
    }
    recordScalar("carla sensor readings", total);
}

uint64_t CarlaSensorSink::count(const std::string& kind) const
{
    auto entry = mCounts.find(kind);
    return entry == mCounts.end() ? 0 : entry->second;
}

void CarlaSensorSink::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj,
    cObject* /* details */)
{
    if (signalID == sensorSignal) {
        const auto* reading = dynamic_cast<const CarlaSensorReading*>(obj);
        if (reading != nullptr) {
            handleReading(source, *reading);
        }
    }
}

void CarlaSensorSink::handleReading(cComponent* source, const CarlaSensorReading& reading)
{
    ++mCounts[reading.kindName()];

    if (par("printReadings")) {
        EV_INFO << getFullName() << " <- " << source->getFullName() << ": " << reading.str()
                << std::endl;
    }
}

}  // namespace carla
}  // namespace artery
