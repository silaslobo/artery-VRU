#ifndef CARLA_IMAGE_SINK_H_K3MV8P2X
#define CARLA_IMAGE_SINK_H_K3MV8P2X

#include <cstdint>
#include <string>

#include <omnetpp/clistener.h>
#include <omnetpp/csimplemodule.h>

namespace artery
{
namespace carla
{

class CarlaSensorReading;

/**
 * Consumer of the "carla.sensor" signal that stores imaging readings
 * (kind == "image") as PNG files.
 *
 * Requires CarlaScenarioManager::copySensorPayload = true, otherwise the
 * readings carry no pixel bytes and nothing can be written.  CARLA images are
 * BGRA8; they are handed to libpng's simplified writer without conversion.
 *
 * Files are named "<simTime_ms>ms_<sensor name>.png" inside outputDir.
 */
class CarlaImageSink : public omnetpp::cSimpleModule, public omnetpp::cListener
{
public:
    CarlaImageSink() = default;

    void initialize(int stage) override;
    void finish() override;

    void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID,
        omnetpp::cObject* obj, omnetpp::cObject* details) override;

protected:
    virtual void handleReading(const CarlaSensorReading& reading);

private:
    std::string mOutputDir;
    unsigned mMaxFrames {0};
    unsigned mWritten {0};
};

/**
 * Write a BGRA8 pixel buffer as an 8 bit RGBA PNG file (libpng simplified API).
 * Returns true on success; on failure returns false and sets error.
 */
bool writeBgraPng(const std::string& path, uint32_t width, uint32_t height,
    const uint8_t* bgra, std::string& error);

}  // namespace carla
}  // namespace artery

#endif /* CARLA_IMAGE_SINK_H_K3MV8P2X */
