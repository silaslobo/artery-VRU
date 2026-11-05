#ifndef CARLA_SENSOR_READING_H_A9XK2PQ1
#define CARLA_SENSOR_READING_H_A9XK2PQ1

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <omnetpp/cnamedobject.h>

namespace artery
{
namespace carla
{

/**
 * Discriminator for the CARLA sensor data types understood by the integration.
 */
enum class SensorKind
{
    Other = 0,
    Gnss,
    Imu,
    Collision,
    LaneInvasion,
    ObstacleDetection,
    Lidar,
    SemanticLidar,
    Radar,
    Image
};

const char* sensorKindName(SensorKind);

/**
 * Plain, OMNeT++-free container for a single sensor measurement.
 *
 * Instances are filled by CARLA's streaming threads (see CarlaScenarioManager)
 * and only converted into a CarlaSensorReading once they have been handed over
 * to the OMNeT++ event loop.  Therefore this type must not touch any OMNeT++
 * API.
 */
struct RawSensorData
{
    SensorKind kind {SensorKind::Other};
    std::string sensorName;
    std::string sensorType;

    uint64_t frame {0};
    /// CARLA episode timestamp in seconds (not OMNeT++ sim time!)
    double timestamp {0.0};

    /// sensor transform in CARLA (unreal) world coordinates
    double x {0.0}, y {0.0}, z {0.0};
    double pitch {0.0}, yaw {0.0}, roll {0.0};

    // GNSS
    double latitude {0.0}, longitude {0.0}, altitude {0.0};

    // IMU
    double accelX {0.0}, accelY {0.0}, accelZ {0.0};
    double gyroX {0.0}, gyroY {0.0}, gyroZ {0.0};
    double compass {0.0};

    // Collision (and friends)
    double impulseX {0.0}, impulseY {0.0}, impulseZ {0.0};
    int32_t otherActorId {-1};
    std::string otherActorName;

    // imaging sensors
    uint32_t width {0}, height {0};
    double fov {0.0};

    /// measurement count (e.g. lidar points); -1 if unknown
    int64_t count {-1};

    /// raw payload bytes (empty for image sensors unless raw copies are enabled)
    std::vector<uint8_t> payload;
};

/**
 * Thread-safe mailbox between CARLA's streaming threads and the OMNeT++ event loop.
 *
 * The CarlaScenarioManager shares one instance (via shared_ptr) with every
 * sensor callback, so in-flight callbacks stay safe even after the module itself
 * has been torn down.
 */
struct SensorInbox
{
    std::mutex mutex;
    std::condition_variable cv;
    std::deque<RawSensorData> queue;
    bool closed {false};
    /// guard against unbounded growth when the simulation cannot keep up
    size_t highWatermark {10000};
    /// measurements discarded because the queue ran full
    size_t dropped {0};
};

/**
 * Signal payload carrying a parsed CARLA sensor measurement.
 *
 * Instances are owned by the emitting module; receivers must dup() them if they
 * need to keep the data beyond the scope of receiveSignal().
 */
class CarlaSensorReading : public omnetpp::cNamedObject
{
public:
    using Kind = SensorKind;

    explicit CarlaSensorReading(const RawSensorData& data);
    CarlaSensorReading(const CarlaSensorReading& other);
    ~CarlaSensorReading() override;

    CarlaSensorReading& operator=(const CarlaSensorReading&) = delete;

    CarlaSensorReading* dup() const override
    {
        return new CarlaSensorReading(*this);
    }

    std::string str() const override;

    Kind kind() const
    {
        return mKind;
    }
    const char* kindName() const
    {
        return sensorKindName(mKind);
    }

    const std::string& sensorName() const
    {
        return mData.sensorName;
    }
    const std::string& sensorType() const
    {
        return mData.sensorType;
    }

    uint64_t frame() const
    {
        return mData.frame;
    }
    /// CARLA episode timestamp in seconds
    double carlaTimestamp() const
    {
        return mData.timestamp;
    }

    double x() const
    {
        return mData.x;
    }
    double y() const
    {
        return mData.y;
    }
    double z() const
    {
        return mData.z;
    }
    double pitch() const
    {
        return mData.pitch;
    }
    double yaw() const
    {
        return mData.yaw;
    }
    double roll() const
    {
        return mData.roll;
    }

    double latitude() const
    {
        return mData.latitude;
    }
    double longitude() const
    {
        return mData.longitude;
    }
    double altitude() const
    {
        return mData.altitude;
    }

    double accelX() const
    {
        return mData.accelX;
    }
    double accelY() const
    {
        return mData.accelY;
    }
    double accelZ() const
    {
        return mData.accelZ;
    }
    double gyroX() const
    {
        return mData.gyroX;
    }
    double gyroY() const
    {
        return mData.gyroY;
    }
    double gyroZ() const
    {
        return mData.gyroZ;
    }
    double compass() const
    {
        return mData.compass;
    }

    double impulseX() const
    {
        return mData.impulseX;
    }
    double impulseY() const
    {
        return mData.impulseY;
    }
    double impulseZ() const
    {
        return mData.impulseZ;
    }
    int32_t otherActorId() const
    {
        return mData.otherActorId;
    }
    const std::string& otherActorName() const
    {
        return mData.otherActorName;
    }

    uint32_t width() const
    {
        return mData.width;
    }
    uint32_t height() const
    {
        return mData.height;
    }
    double fov() const
    {
        return mData.fov;
    }

    int64_t count() const
    {
        return mData.count;
    }

    const std::vector<uint8_t>& payload() const
    {
        return mData.payload;
    }

private:
    RawSensorData mData;
    Kind mKind;
    mutable std::string mString;
};

}  // namespace carla
}  // namespace artery

#endif /* CARLA_SENSOR_READING_H_A9XK2PQ1 */
