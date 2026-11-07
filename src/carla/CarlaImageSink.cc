#include "CarlaImageSink.h"

#include <cctype>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <vector>

#include <png.h>

#include <omnetpp/cmodule.h>
#include <omnetpp/simtime_t.h>

#include "CarlaSensorReading.h"

Define_Module(artery::carla::CarlaImageSink)

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

/// Make a sensor name safe for use in a file name.
std::string sanitize(const std::string& name)
{
    std::string out;
    for (char c : name) {
        out += (std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_') ? c : '_';
    }
    return out;
}

}  // namespace

namespace artery
{
namespace carla
{

bool writeBgraPng(const std::string& path, uint32_t width, uint32_t height,
    const uint8_t* bgra, std::string& error)
{
    png_image image;
    std::memset(&image, 0, sizeof(image));
    image.version = PNG_IMAGE_VERSION;
    image.width = width;
    image.height = height;
    image.format = PNG_FORMAT_BGRA;

    if (png_image_write_to_file(&image, path.c_str(), /*convert_to_8bit*/ 0, bgra,
            /*stride*/ 0, /*colormap*/ nullptr) == 0) {
        error = image.message;
        return false;
    }
    return true;
}

void CarlaImageSink::initialize(int stage)
{
    cSimpleModule::initialize(stage);

    if (stage == 0) {
        mOutputDir = par("outputDir").stdstringValue();
        const long maxFrames = par("maxFrames").longValue();
        mMaxFrames = maxFrames > 0 ? static_cast<unsigned>(maxFrames) : 0;

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
                    "CarlaImageSink: could not resolve CARLA manager module '%s'",
                    path.c_str());
            }
        }
        source->subscribe(sensorSignal, this);
        EV_DEBUG << "Subscribed to carla.sensor on " << source->getFullName()
                 << ", capturing into " << mOutputDir << std::endl;
    }
}

void CarlaImageSink::finish()
{
    getSimulation()->getSystemModule()->unsubscribe(sensorSignal, this);
    EV_INFO << "wrote " << mWritten << " image frames to " << mOutputDir << std::endl;
    recordScalar("carla image frames saved", mWritten);
}

void CarlaImageSink::receiveSignal(cComponent* /* source */, simsignal_t signalID,
    cObject* obj, cObject* /* details */)
{
    if (signalID == sensorSignal) {
        const auto* reading = dynamic_cast<const CarlaSensorReading*>(obj);
        if (reading != nullptr) {
            handleReading(*reading);
        }
    }
}

void CarlaImageSink::handleReading(const CarlaSensorReading& reading)
{
    if (std::strcmp(reading.kindName(), "image") != 0) {
        return;
    }
    if (mMaxFrames > 0 && mWritten >= mMaxFrames) {
        return;
    }

    const auto& payload = reading.payload();
    const uint32_t width = reading.width();
    const uint32_t height = reading.height();
    const size_t expected = static_cast<size_t>(width) * height * 4;  // BGRA8
    if (width == 0 || height == 0 || payload.size() < expected) {
        EV_WARN << "Ignoring image reading from " << reading.sensorName() << " ("
                << width << 'x' << height << ", " << payload.size()
                << " bytes): empty payload? copySensorPayload must be true." << std::endl;
        return;
    }

    std::error_code ec;
    std::filesystem::create_directories(mOutputDir, ec);  // ok if it exists

    // use the simulation instant of the CARLA frame the image was produced
    // in (assigned by the manager); measurements can arrive late
    const int64_t attributedNs = reading.simTimeNs();
    std::ostringstream name;
    name << mOutputDir << '/'
         << static_cast<long long>(attributedNs / 1000000) << "ms_"
         << sanitize(reading.sensorName()) << "_f" << reading.frame() << ".png";

    std::string error;
    if (!writeBgraPng(name.str(), width, height, payload.data(), error)) {
        EV_WARN << "Could not write " << name.str() << ": " << error << std::endl;
        return;
    }
    ++mWritten;
    EV_INFO << "Captured " << name.str() << " (" << width << 'x' << height
            << ", sim t=" << static_cast<double>(attributedNs) * 1e-9 << 's' << ')' << std::endl;
}

}  // namespace carla
}  // namespace artery
