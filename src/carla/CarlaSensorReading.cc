#include "CarlaSensorReading.h"

#include <cstdio>

namespace artery
{
namespace carla
{

const char* sensorKindName(SensorKind kind)
{
    switch (kind) {
    case SensorKind::Gnss:
        return "gnss";
    case SensorKind::Imu:
        return "imu";
    case SensorKind::Collision:
        return "collision";
    case SensorKind::LaneInvasion:
        return "laneInvasion";
    case SensorKind::ObstacleDetection:
        return "obstacleDetection";
    case SensorKind::Lidar:
        return "lidar";
    case SensorKind::SemanticLidar:
        return "semanticLidar";
    case SensorKind::Radar:
        return "radar";
    case SensorKind::Image:
        return "image";
    case SensorKind::Other:
    default:
        return "other";
    }
}

CarlaSensorReading::CarlaSensorReading(const RawSensorData& data)
    : cNamedObject(data.sensorName.c_str(), false), mData(data), mKind(data.kind)
{
}

CarlaSensorReading::CarlaSensorReading(const CarlaSensorReading& other)
    : cNamedObject(other), mData(other.mData), mKind(other.mKind)
{
}

CarlaSensorReading::~CarlaSensorReading() = default;

std::string CarlaSensorReading::str() const
{
    char buffer[256];
    std::snprintf(buffer,
        sizeof(buffer),
        "%s(%s) frame=%llu t=%.3f sim=%.3f pos=(%.2f,%.2f,%.2f)",
        kindName(),
        sensorName().c_str(),
        static_cast<unsigned long long>(mData.frame),
        mData.timestamp,
        static_cast<double>(mData.simTimeNs) * 1e-9,
        mData.x,
        mData.y,
        mData.z);
    mString = buffer;

    switch (mKind) {
    case SensorKind::Gnss:
        std::snprintf(buffer,
            sizeof(buffer),
            "%s lat=%.7f lon=%.7f alt=%.2f",
            mString.c_str(),
            mData.latitude,
            mData.longitude,
            mData.altitude);
        mString = buffer;
        break;
    case SensorKind::Imu:
        std::snprintf(buffer,
            sizeof(buffer),
            "%s accel=(%.2f,%.2f,%.2f) gyro=(%.2f,%.2f,%.2f) compass=%.1f",
            mString.c_str(),
            mData.accelX,
            mData.accelY,
            mData.accelZ,
            mData.gyroX,
            mData.gyroY,
            mData.gyroZ,
            mData.compass);
        mString = buffer;
        break;
    case SensorKind::Collision:
        std::snprintf(buffer,
            sizeof(buffer),
            "%s impulse=(%.2f,%.2f,%.2f) other=%s",
            mString.c_str(),
            mData.impulseX,
            mData.impulseY,
            mData.impulseZ,
            mData.otherActorName.c_str());
        mString = buffer;
        break;
    case SensorKind::Image:
        std::snprintf(buffer,
            sizeof(buffer),
            "%s %ux%u fov=%.1f",
            mString.c_str(),
            mData.width,
            mData.height,
            mData.fov);
        mString = buffer;
        break;
    default:
        if (mData.count >= 0) {
            std::snprintf(buffer, sizeof(buffer), "%s count=%lld", mString.c_str(),
                static_cast<long long>(mData.count));
            mString = buffer;
        }
        break;
    }

    return mString;
}

}  // namespace carla
}  // namespace artery
