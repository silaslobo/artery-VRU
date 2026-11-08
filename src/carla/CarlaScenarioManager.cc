#include "CarlaScenarioManager.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <omnetpp/csimulation.h>

// OMNeT++ defines PI as a macro, which breaks carla/geom/Math.h
#undef PI

#include <carla/client/Actor.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/Vehicle.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/road/Map.h>
#include <carla/road/element/Waypoint.h>
#include <carla/rpc/EpisodeSettings.h>
#include <carla/sensor/data/CollisionEvent.h>
#include <carla/sensor/data/GnssMeasurement.h>
#include <carla/sensor/data/IMUMeasurement.h>
#include <carla/sensor/data/Image.h>
#include <carla/sensor/data/LaneInvasionEvent.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/ObstacleDetectionEvent.h>
#include <carla/sensor/data/RadarMeasurement.h>
#include <carla/sensor/data/SemanticLidarMeasurement.h>

namespace cc = ::carla::client;
namespace cg = ::carla::geom;
namespace csd = ::carla::sensor::data;

Define_Module(artery::carla::CarlaScenarioManager)

using namespace omnetpp;

namespace
{

const simsignal_t initSignal = cComponent::registerSignal("carla.init");
const simsignal_t stepSignal = cComponent::registerSignal("carla.step");
const simsignal_t closeSignal = cComponent::registerSignal("carla.close");
const simsignal_t sensorSignal = cComponent::registerSignal("carla.sensor");
const simsignal_t residualSignal = cComponent::registerSignal("sync.geometryResidual");

::carla::time_duration toCarlaDuration(double seconds)
{
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(seconds));
    return ::carla::time_duration(milliseconds);
}

std::string trim(const std::string& s)
{
    auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return std::string();
    }
    auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

std::vector<std::string> split(const std::string& s, char separator)
{
    std::vector<std::string> result;
    std::string item;
    std::stringstream stream(s);
    while (std::getline(stream, item, separator)) {
        if (!trim(item).empty()) {
            result.push_back(trim(item));
        }
    }
    return result;
}

/**
 * Specification of a single sensor, parsed from the "sensors" parameter.
 *
 * grammar: entry := blueprint ["@" x,y,z[,pitch,yaw[,roll]]]
 *                  ["!" key=value[,key=value]*]
 * entries are separated by ';'.
 */
struct SensorSpec {
    std::string blueprint;
    bool hasTransform {false};
    double x {0.0}, y {0.0}, z {0.0};
    double pitch {0.0}, yaw {0.0}, roll {0.0};
    std::vector<std::pair<std::string, std::string>> attributes;
};

std::vector<SensorSpec> parseSensors(const std::string& text)
{
    std::vector<SensorSpec> specs;
    for (const auto& entry : split(text, ';')) {
        SensorSpec spec;
        std::string remainder = entry;

        auto bang = remainder.find('!');
        if (bang != std::string::npos) {
            for (const auto& attribute : split(remainder.substr(bang + 1), ',')) {
                auto equals = attribute.find('=');
                if (equals != std::string::npos) {
                    spec.attributes.emplace_back(trim(attribute.substr(0, equals)),
                        trim(attribute.substr(equals + 1)));
                }
            }
            remainder = remainder.substr(0, bang);
        }

        auto at = remainder.find('@');
        if (at != std::string::npos) {
            const auto coordinates = split(remainder.substr(at + 1), ',');
            const double values[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            double parsed[6];
            std::copy(std::begin(values), std::end(values), std::begin(parsed));
            for (size_t i = 0; i < coordinates.size() && i < 6; ++i) {
                parsed[i] = std::strtod(coordinates[i].c_str(), nullptr);
            }
            spec.x = parsed[0];
            spec.y = parsed[1];
            spec.z = parsed[2];
            spec.pitch = parsed[3];
            spec.yaw = parsed[4];
            spec.roll = parsed[5];
            spec.hasTransform = true;
            remainder = remainder.substr(0, at);
        }

        spec.blueprint = trim(remainder);
        if (!spec.blueprint.empty()) {
            specs.push_back(spec);
        }
    }
    return specs;
}

cg::Transform makeTransform(const SensorSpec& spec)
{
    cg::Transform transform;
    transform.location = cg::Location(spec.x, spec.y, spec.z);
    transform.rotation = cg::Rotation(spec.pitch, spec.yaw, spec.roll);
    return transform;
}

}  // namespace

namespace artery
{
namespace carla
{
namespace
{

/**
 * Convert CARLA's (immutable, ref-counted) sensor message into the plain
 * RawSensorData container.  Runs on CARLA's streaming threads, so it must not
 * touch any OMNeT++ API.
 */
RawSensorData toRaw(const std::shared_ptr<::carla::sensor::SensorData>& data,
    const std::string& name,
    const std::string& type,
    bool copyPayload)
{
    RawSensorData raw;
    raw.sensorName = name;
    raw.sensorType = type;
    raw.frame = data->GetFrame();
    raw.timestamp = data->GetTimestamp();

    const auto& transform = data->GetSensorTransform();
    raw.x = transform.location.x;
    raw.y = transform.location.y;
    raw.z = transform.location.z;
    raw.pitch = transform.rotation.pitch;
    raw.yaw = transform.rotation.yaw;
    raw.roll = transform.rotation.roll;

    if (auto gnss = std::dynamic_pointer_cast<csd::GnssMeasurement>(data)) {
        raw.kind = SensorKind::Gnss;
        raw.latitude = gnss->GetLatitude();
        raw.longitude = gnss->GetLongitude();
        raw.altitude = gnss->GetAltitude();
    }
    else if (auto imu = std::dynamic_pointer_cast<csd::IMUMeasurement>(data)) {
        raw.kind = SensorKind::Imu;
        const auto accelerometer = imu->GetAccelerometer();
        const auto gyroscope = imu->GetGyroscope();
        raw.accelX = accelerometer.x;
        raw.accelY = accelerometer.y;
        raw.accelZ = accelerometer.z;
        raw.gyroX = gyroscope.x;
        raw.gyroY = gyroscope.y;
        raw.gyroZ = gyroscope.z;
        raw.compass = imu->GetCompass();
    }
    else if (auto collision = std::dynamic_pointer_cast<csd::CollisionEvent>(data)) {
        raw.kind = SensorKind::Collision;
        const auto& impulse = collision->GetNormalImpulse();
        raw.impulseX = impulse.x;
        raw.impulseY = impulse.y;
        raw.impulseZ = impulse.z;
        if (auto other = collision->GetOtherActor()) {
            raw.otherActorId = other->GetId();
            raw.otherActorName = other->GetTypeId();
        }
    }
    else if (auto obstacle = std::dynamic_pointer_cast<csd::ObstacleDetectionEvent>(data)) {
        raw.kind = SensorKind::ObstacleDetection;
        raw.count = static_cast<int64_t>(obstacle->GetDistance());
        if (auto other = obstacle->GetOtherActor()) {
            raw.otherActorId = other->GetId();
            raw.otherActorName = other->GetTypeId();
        }
    }
    else if (std::dynamic_pointer_cast<csd::LaneInvasionEvent>(data)) {
        raw.kind = SensorKind::LaneInvasion;
    }
    else if (auto lidar = std::dynamic_pointer_cast<csd::LidarMeasurement>(data)) {
        raw.kind = SensorKind::Lidar;
        raw.count = static_cast<int64_t>(lidar->size());
        if (copyPayload && !lidar->empty()) {
            const auto* first = reinterpret_cast<const uint8_t*>(&lidar->at(0));
            const auto* last = first + lidar->size() * sizeof(csd::LidarDetection);
            raw.payload.assign(first, last);
        }
    }
    else if (auto semantic = std::dynamic_pointer_cast<csd::SemanticLidarMeasurement>(data)) {
        raw.kind = SensorKind::SemanticLidar;
        raw.count = static_cast<int64_t>(semantic->size());
    }
    else if (auto radar = std::dynamic_pointer_cast<csd::RadarMeasurement>(data)) {
        raw.kind = SensorKind::Radar;
        raw.count = static_cast<int64_t>(radar->size());
    }
    else if (auto image = std::dynamic_pointer_cast<csd::Image>(data)) {
        raw.kind = SensorKind::Image;
        raw.width = image->GetWidth();
        raw.height = image->GetHeight();
        raw.fov = image->GetFOVAngle();
        raw.count = static_cast<int64_t>(image->size());
        if (copyPayload && !image->empty()) {
            const auto* first = reinterpret_cast<const uint8_t*>(&image->at(0));
            const auto* last = first + image->size() * sizeof(csd::Image::pixel_type);
            raw.payload.assign(first, last);
        }
    }

    return raw;
}

}  // namespace

CarlaScenarioManager::CarlaScenarioManager()
    : m_connectEvent(nullptr),
      m_stepEvent(nullptr),
      m_inbox(std::make_shared<SensorInbox>()),
      m_port(2000)
{
}

CarlaScenarioManager::~CarlaScenarioManager()
{
    // the CARLA objects are usually released in finish(); this covers runs
    // that were aborted before finish() was called
    disconnect();

    delete m_connectEvent;
    delete m_stepEvent;
}

void CarlaScenarioManager::initialize()
{
    m_host = par("host").stdstringValue();
    m_port = static_cast<uint16_t>(static_cast<int>(par("port")));
    // note: SimTime has an (int64_t, unit) constructor which would truncate
    // sub-second values, so assign the plain double (seconds) here
    m_timeout = par("timeout").doubleValue();
    m_stepInterval = par("stepInterval").doubleValue();
    m_sensorWaitTimeout = par("sensorWaitTimeout").doubleValue();
    m_tickTimeout = par("tickTimeout").doubleValue();

    m_connectEvent = new cMessage("connect CARLA");
    m_stepEvent = new cMessage("CARLA step");
    // run early, but *after* traci::Core (which uses the minimum priority), so
    // that in every lock-step frame the SUMO positions for the current time
    // are known before the CARLA frame is produced
    m_stepEvent->setSchedulingPriority(std::numeric_limits<short>::min() + 1);

    scheduleAt(par("startTime"), m_connectEvent);
}

void CarlaScenarioManager::finish()
{
    emit(closeSignal, simTime());

    if (m_connectEvent != nullptr) {
        cancelEvent(m_connectEvent);
    }
    if (m_stepEvent != nullptr) {
        cancelEvent(m_stepEvent);
    }

    stopSensors();

    if (m_residualCount > 0) {
        EV_INFO << "avatar pose residual: n=" << m_residualCount
                << " max=" << m_residualMax << " m mean=" << residualMean() << " m"
                << std::endl;
    }
    if (m_lateReadings > 0) {
        EV_INFO << m_lateReadings << " sensor measurements arrived after their CARLA"
                << " frame had been ticked (attributed by frame distance)" << std::endl;
    }

    if (m_world && par("destroyActorsOnFinish")) {
        cleanupActors();
    }

    // the server is deliberately left in synchronous mode: this integration
    // *is* a lock-step co-simulation, and a client that is killed hard would
    // anyway leave the server waiting for tick cues (handled on the next
    // connect, see connect())
    disconnect();
}

void CarlaScenarioManager::disconnect()
{
    // Destructors must not throw; CARLA may very well time out here (e.g. when
    // the server is still waiting for a tick cue), so swallow any error.
    try {
        m_sensors.clear();
        m_avatars.clear();
        m_world.reset();
        m_client.reset();
    }
    catch (const std::exception& e) {
        EV_WARN << "Error while disconnecting from the CARLA server: " << e.what() << std::endl;
    }

    if (m_inbox->dropped > 0) {
        EV_WARN << "Dropped " << m_inbox->dropped
                << " CARLA sensor measurements (queue overflow)" << std::endl;
    }
}

void CarlaScenarioManager::handleMessage(cMessage* msg)
{
    if (msg == m_connectEvent) {
        connect();
        emit(initSignal, simTime());
        scheduleAt(simTime() + m_stepInterval, m_stepEvent);
    }
    else if (msg == m_stepEvent) {
        step();
    }
    else {
        delete msg;
    }
}

void CarlaScenarioManager::connect()
{
    EV_INFO << "Connecting to CARLA server " << m_host << ':' << m_port << std::endl;

    try {
        m_client.reset(new cc::Client(m_host, m_port));
        m_client->SetTimeout(toCarlaDuration(m_timeout.dbl()));

        m_world.reset(new cc::World(m_client->GetWorld()));
        EV_INFO << "Connected to CARLA " << m_client->GetServerVersion() << ", world "
                << m_world->GetSettings().fixed_delta_seconds.value_or(0.0) << "s step"
                << std::endl;

        // A client that died without restoring the settings leaves the server
        // waiting for tick cues, which makes later requests time out.  Nudge it
        // once before touching anything else.
        if (m_world->GetSettings().synchronous_mode) {
            EV_WARN << "CARLA server is in synchronous mode already, sending tick cue"
                    << std::endl;
            try {
                m_world->Tick(toCarlaDuration(m_timeout.dbl()));
            }
            catch (const std::exception& e) {
                EV_WARN << "Could not nudge CARLA server: " << e.what() << std::endl;
            }
        }

        const auto opendriveFile = par("opendriveFile").stdstringValue();
        const auto map = par("map").stdstringValue();
        if (!opendriveFile.empty()) {
            loadGeneratedMap();
        }
        else if (!map.empty()) {
            m_world.reset(new cc::World(m_client->LoadWorld(map)));
        }

        // lock-step is the only mode this integration supports: a co-
        // simulation that is not synchronous is not a co-simulation
        {
            auto settings = m_world->GetSettings();
            settings.synchronous_mode = true;
            // rendering is always on: the camera sensor is this integration's
            // main data source, and pinning the flag also prevents inheriting
            // a no_rendering session left behind by another client
            settings.no_rendering_mode = false;
            settings.fixed_delta_seconds = m_stepInterval.dbl();
            settings.substepping = false;
            try {
                m_world->ApplySettings(settings, toCarlaDuration(5.0));
            }
            catch (const std::exception& e) {
                // libcarla verifies new settings by ticking until the reported
                // frame time matches fixed_delta_seconds.  A timeout only means
                // that this verification failed; the settings themselves are
                // applied, so keep going and drive the world with our own ticks.
                EV_WARN << "CARLA did not confirm the new settings: " << e.what() << std::endl;
            }
            const auto applied = m_world->GetSettings();
            EV_INFO << "CARLA server settings: synchronous=" << applied.synchronous_mode
                    << " fixed_delta=" << applied.fixed_delta_seconds.value_or(0.0)
                    << " no_rendering=" << applied.no_rendering_mode << std::endl;
        }

        // no traffic manager anywhere, by design: it is a client-side library
        // object (TrafficManagerLocal) that would poll the server with its own
        // thread -- incompatible with lock-step, see the SetAutopilot comment
        // in updateAvatar -- and a stock CARLA server runs no TM service
        spawnSensors();
    }
    catch (const cc::TimeoutException& e) {
        throw cRuntimeError("Timeout while talking to CARLA server %s:%u: %s", m_host.c_str(),
            m_port, e.what());
    }
    catch (const std::exception& e) {
        throw cRuntimeError("Could not connect to CARLA server %s:%u: %s", m_host.c_str(), m_port,
            e.what());
    }
}

void CarlaScenarioManager::loadGeneratedMap()
{
    const std::string path = par("opendriveFile").stdstringValue();
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw cRuntimeError("Cannot open OpenDRIVE file '%s'", path.c_str());
    }
    std::stringstream buffer;
    buffer << in.rdbuf();
    const std::string xodr = buffer.str();
    if (xodr.empty()) {
        throw cRuntimeError("OpenDRIVE file '%s' is empty", path.c_str());
    }

    ::carla::rpc::OpendriveGenerationParameters params;
    params.vertex_distance = par("vertexDistance").doubleValue();
    params.max_road_length = par("maxRoadLength").doubleValue();
    params.wall_height = par("wallHeight").doubleValue();
    params.additional_width = par("additionalWidth").doubleValue();
    params.vertex_width_resolution = par("vertexWidthResolution").doubleValue();
    params.simplification_percentage = static_cast<float>(par("simplificationPercentage").doubleValue());
    params.smooth_junctions = par("smoothJunctions");
    params.enable_pedestrian_navigation = par("enablePedestrianNavigation");

    const size_t sentChecksum = std::hash<std::string> {}(xodr);
    EV_INFO << "Generating CARLA world from OpenDRIVE '" << path << "' (" << xodr.size()
            << " bytes, checksum " << sentChecksum << "): vertex_distance="
            << params.vertex_distance << " additional_width=" << params.additional_width
            << " simplification=" << params.simplification_percentage
            << " smooth_junctions=" << params.smooth_junctions
            << " wall_height=" << params.wall_height << std::endl;

    m_world.reset(new cc::World(m_client->GenerateOpenDriveWorld(xodr, params)));

    // prove in the log which map is really loaded
    auto map = m_world->GetMap();
    const auto& geo = map->GetGeoReference();
    EV_INFO << "Loaded CARLA map name='" << map->GetName() << "' geoReference lat="
            << geo.latitude << " lon=" << geo.longitude << " alt=" << geo.altitude
            << std::endl;
    const auto& received = map->GetOpenDrive();
    EV_INFO << "OpenDRIVE round trip: sent " << xodr.size() << " bytes (checksum "
            << sentChecksum << "), server reports " << received.size() << " bytes (checksum "
            << std::hash<std::string> {}(received) << ")" << std::endl;
    if (received.size() == xodr.size()
            && std::hash<std::string> {}(received) == sentChecksum) {
        EV_INFO << "Server stores exactly the OpenDRIVE string that was sent" << std::endl;
    }

    checkGeneratedGeometry();
}

void CarlaScenarioManager::checkGeneratedGeometry()
{
    const std::string path = par("mapCheckFile").stdstringValue();
    if (path.empty()) {
        return;
    }
    std::ifstream in(path);
    if (!in) {
        throw cRuntimeError("Cannot open map check file '%s'", path.c_str());
    }

    struct Sample {
        int road;
        int lane;
        double x, y;  // xodr coordinates
    };
    std::vector<Sample> samples;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream ls(line);
        Sample s;
        double st, z, hdg;
        if (!(ls >> s.road >> s.lane >> st >> s.x >> s.y >> z >> hdg)) {
            continue;
        }
        samples.push_back(s);
    }
    if (samples.empty()) {
        EV_WARN << "Map check file '" << path << "' contains no samples" << std::endl;
        return;
    }

    const ::carla::road::Map& road_map = m_world->GetMap()->GetMap();

    // Determine the coordinate convention empirically: compare the projection
    // deviation of (x, y) vs (x, -y) at a sample away from the x axis.
    bool flip_y = false;
    {
        double dev_plain = -1.0;
        double dev_flip = -1.0;
        for (const auto& s : samples) {
            if (std::fabs(s.y) < 5.0) {
                continue;
            }
            auto dev_at = [&road_map](double x, double y) {
                auto wp = road_map.GetClosestWaypointOnRoad(
                    ::carla::geom::Location(x, y, 0.0));
                if (!wp) {
                    return 1e9;
                }
                const auto t = road_map.ComputeTransform(*wp);
                return std::hypot(t.location.x - x, t.location.y - y);
            };
            dev_plain = dev_at(s.x, s.y);
            dev_flip = dev_at(s.x, -s.y);
            break;
        }
        flip_y = dev_flip < dev_plain;
        EV_INFO << "Map geometry check: CARLA coordinate convention "
                << (flip_y ? "x_carla = x_xodr, y_carla = -y_xodr" : "x/y identical")
                << " (dev plain=" << dev_plain << " flipped=" << dev_flip << ')'
                << std::endl;
    }

    double max_dev = 0.0;
    double sum_dev = 0.0;
    size_t checked = 0;
    size_t id_mismatch = 0;
    for (const auto& s : samples) {
        const double x = s.x;
        const double y = flip_y ? -s.y : s.y;
        auto wp = road_map.GetClosestWaypointOnRoad(::carla::geom::Location(x, y, 0.0));
        if (!wp) {
            ++id_mismatch;
            continue;
        }
        if (wp->road_id != s.road || wp->lane_id != s.lane) {
            // expected close to road boundaries / at overlapping junction lanes
            ++id_mismatch;
        }
        const auto t = road_map.ComputeTransform(*wp);
        const double dev = std::hypot(t.location.x - x, t.location.y - y);
        max_dev = std::max(max_dev, dev);
        sum_dev += dev;
        ++checked;
    }
    if (checked == 0) {
        EV_WARN << "Map geometry check: no sample could be projected onto the "
                << "CARLA map" << std::endl;
        return;
    }
    EV_INFO << "Map geometry check: " << checked << " lane samples, max deviation "
            << max_dev << " m, mean " << (sum_dev / static_cast<double>(checked))
            << " m, road/lane nearest-id mismatches " << id_mismatch
            << " (boundary effects)" << std::endl;
    if (max_dev > 0.2) {
        EV_WARN << "CARLA lane geometry deviates more than 0.2 m from the generated "
                << "OpenDRIVE - check the generation parameters" << std::endl;
    }
}

void CarlaScenarioManager::acquireAvatar(const std::string& key, const std::string& blueprint,
    const CarlaPose& pose)
{
    auto found = m_avatars.find(key);
    if (found != m_avatars.end()) {
        found->second.pose = pose;
        return;
    }
    AvatarEntry entry;
    entry.pose = pose;
    entry.blueprint = blueprint;
    m_avatars.emplace(key, std::move(entry));
}

void CarlaScenarioManager::updateAvatar(const std::string& key, const CarlaPose& pose)
{
    auto found = m_avatars.find(key);
    if (found != m_avatars.end()) {
        found->second.pose = pose;
    }
}

void CarlaScenarioManager::releaseAvatar(const std::string& key)
{
    auto found = m_avatars.find(key);
    if (found == m_avatars.end()) {
        return;
    }
    // attached sensors are stopped and destroyed with the avatar
    for (auto& sensor : found->second.sensors) {
        try {
            sensor->Stop();
            sensor->Destroy();
        }
        catch (const std::exception& e) {
            EV_WARN << "Could not destroy avatar sensor: " << e.what() << std::endl;
        }
    }
    if (found->second.actor) {
        m_avatarsToDestroy.push_back(found->second.actor);
    }
    m_avatars.erase(found);
}

void CarlaScenarioManager::applyAvatarUpdates()
{
    if (!m_world) {
        return;  // connect() has not run yet, requests stay queued
    }

    if (!m_avatarsToDestroy.empty()) {
        for (auto& actor : m_avatarsToDestroy) {
            try {
                actor->Destroy();
            }
            catch (const std::exception& e) {
                EV_WARN << "Could not destroy CARLA avatar: " << e.what() << std::endl;
            }
        }
        m_avatarsToDestroy.clear();
    }

    std::optional<::carla::client::ActorBlueprint> blueprint;
    std::string blueprintId;
    for (auto& entry : m_avatars) {
        auto& avatar = entry.second;
        cg::Transform transform;
        transform.location = cg::Location(avatar.pose.x, avatar.pose.y, avatar.pose.z);
        transform.rotation = cg::Rotation(0.0, avatar.pose.yaw, 0.0);
        try {
            if (!avatar.actor) {
                if (!blueprint || blueprintId != avatar.blueprint) {
                    auto library = m_world->GetBlueprintLibrary();
                    auto found = library->Find(avatar.blueprint);
                    if (found == nullptr) {
                        throw cRuntimeError("CARLA blueprint '%s' not found (avatar)",
                            avatar.blueprint.c_str());
                    }
                    blueprint = *found;
                    blueprintId = avatar.blueprint;
                }
                // A rejected spawn is a programming error, not a transient
                // condition: the vehicle exists on the SUMO side, and the
                // only ways to block the pose here are our own (wrong z,
                // overlapping avatars) -- fail loudly instead of retrying
                auto actor = m_world->TrySpawnActor(*blueprint, transform);
                if (!actor) {
                    throw cRuntimeError("Could not spawn CARLA avatar '%s' at "
                        "(%.2f,%.2f,%.2f): pose rejected by the server", entry.first.c_str(),
                        transform.location.x, transform.location.y, transform.location.z);
                }
                avatar.actor = std::static_pointer_cast<cc::Vehicle>(actor);
                // deliberately no SetAutopilot(false) here: freshly spawned
                // actors have no autopilot, and even the *unregister* path of
                // SetAutopilot constructs a client-side TrafficManager whose
                // background thread polls the server -- fatal in synchronous
                // mode (RPC timeouts throw on that thread and abort our
                // process while the server never notices anything)

                // avatars are kinematic mirrors: the per-tick SetTransform()
                // is the position authority, and physics cannot coexist with
                // teleports (PhysX resolves the teleported-overlap by
                // exploding the body -- in synchronous mode that has been
                // observed to take the whole server down)
                avatar.actor->SetSimulatePhysics(false);
                EV_INFO << "Spawned CARLA avatar '" << entry.first << "' (actor "
                        << avatar.actor->GetId() << ") from " << avatar.blueprint
                        << " at (" << avatar.pose.x << ',' << avatar.pose.y << ','
                        << avatar.pose.yaw << ')' << std::endl;
            }
            else {
                avatar.actor->SetTransform(transform);
            }
            if (!avatar.sensorsAttached && !avatar.pendingSensors.empty()) {
                spawnSensorList(avatar.actor.get(), avatar.pendingSensors,
                    entry.first + '.', avatar.sensors);
                avatar.sensorsAttached = true;
                avatar.pendingSensors.clear();
            }
        }
        catch (const cc::TimeoutException& e) {
            throw cRuntimeError("CARLA avatar update timed out: %s", e.what());
        }
    }
}

void CarlaScenarioManager::measureAvatarResiduals()
{
    if (!par("avatarResidualCheck") || !m_world || m_avatars.empty()) {
        return;
    }
    for (auto& entry : m_avatars) {
        auto& avatar = entry.second;
        if (!avatar.actor) {
            continue;
        }
        try {
            const auto t = avatar.actor->GetTransform();
            CarlaPose actual;
            actual.x = t.location.x;
            actual.y = t.location.y;
            actual.z = t.location.z;
            actual.yaw = t.rotation.yaw;
            const double residual = positionDistance(actual, avatar.pose);
            m_residualMax = std::max(m_residualMax, residual);
            m_residualSum += residual;
            ++m_residualCount;
            emit(residualSignal, residual);
        }
        catch (const std::exception& e) {
            EV_WARN << "Could not query CARLA avatar '" << entry.first
                    << "' pose: " << e.what() << std::endl;
        }
    }
}

void CarlaScenarioManager::spawnSensors()
{
    // manager-level sensors are world-fixed: no parent actor (avatar sensors
    // are attached to their vehicle, see spawnSensorList)
    spawnSensorList(nullptr, par("sensors").stdstringValue(),
        std::string(), m_sensors);
}

void CarlaScenarioManager::spawnSensorList(cc::Actor* parent, const std::string& text,
    const std::string& name_prefix,
    std::vector<std::shared_ptr<cc::Sensor>>& out)
{
    const auto specs = parseSensors(text);
    if (specs.empty()) {
        return;
    }

    auto library = m_world->GetBlueprintLibrary();
    const bool copyPayload = par("copySensorPayload");

    for (const auto& spec : specs) {
        auto found = library->Find(spec.blueprint);
        if (found == nullptr) {
            throw cRuntimeError("CARLA blueprint '%s' not found", spec.blueprint.c_str());
        }
        auto blueprint = *found;
        // Note: the sensor_tick blueprint attribute is deliberately never
        // set -- every sensor emits on every world frame and consumers throttle
        // by dropping arrivals (see README §6, "Data rate control").
        for (const auto& attribute : spec.attributes) {
            if (!blueprint.ContainsAttribute(attribute.first)) {
                EV_WARN << "CARLA blueprint " << spec.blueprint << " has no attribute "
                        << attribute.first << ", ignoring it" << std::endl;
                continue;
            }
            blueprint.SetAttribute(attribute.first, attribute.second);
        }

        std::string name = name_prefix;
        auto dot = spec.blueprint.rfind('.');
        if (dot != std::string::npos && dot + 1 < spec.blueprint.size()) {
            name += spec.blueprint.substr(dot + 1);
        }
        else {
            name += spec.blueprint;
        }

        const auto transform = makeTransform(spec);
        auto actor = m_world->SpawnActor(blueprint, transform, parent);
        auto sensor = std::static_pointer_cast<cc::Sensor>(actor);

        // The callback runs on a CARLA streaming thread: it may only touch
        // CARLA data and the mutex guarded inbox, never OMNeT++ itself.
        auto inbox = m_inbox;
        const std::string sensorName = name;
        const std::string sensorType = spec.blueprint;
        sensor->Listen([inbox, sensorName, sensorType, copyPayload](
                           std::shared_ptr<::carla::sensor::SensorData> data) {
            auto raw = toRaw(data, sensorName, sensorType, copyPayload);
            {
                std::lock_guard<std::mutex> guard(inbox->mutex);
                if (inbox->queue.size() >= inbox->highWatermark) {
                    inbox->queue.pop_front();
                    ++inbox->dropped;
                }
                inbox->queue.push_back(std::move(raw));
            }
            inbox->cv.notify_all();
        });

        // Every non-event sensor emits one measurement per world frame and
        // is therefore waited for by step(); event sensors
        // (collision, lane invasion, obstacle detection) only report
        // occurrences and must never be waited for.
        const bool eventSensor = spec.blueprint.find("collision") != std::string::npos
            || spec.blueprint.find("lane_invasion") != std::string::npos
            || spec.blueprint.find("obstacle_detection") != std::string::npos;
        if (!eventSensor) {
            m_frameSensorNames.insert(name);
        }

        EV_INFO << "Attached sensor " << sensor->GetId() << " (" << spec.blueprint
                << ", name=" << name << ')'
                << std::endl;
        out.push_back(sensor);
    }
}

void CarlaScenarioManager::attachAvatarSensors(const std::string& key,
    const std::string& spec)
{
    if (spec.empty()) {
        return;
    }
    auto found = m_avatars.find(key);
    if (found != m_avatars.end()) {
        if (!found->second.pendingSensors.empty()) {
            found->second.pendingSensors += ';';
        }
        found->second.pendingSensors += spec;
    }
}

void CarlaScenarioManager::step()
{
    // apply queued avatar changes before the frame so that the CARLA world
    // shows the SUMO state of the current simulation instant
    applyAvatarUpdates();

    if (m_world) {
        const auto tickTimeout = toCarlaDuration(m_tickTimeout.dbl());
        try {
            m_frame = m_world->Tick(tickTimeout);
        }
        catch (const cc::TimeoutException& e) {
            // Deliberately a single attempt: a timed-out tick has very likely
            // completed on the server with the response stuck behind the
            // sensor stream (saturated WAN); sending another tick cue would
            // double-tick the server and desynchronise the strict
            // request/response protocol -- the stale response then corrupts
            // every later RPC until CARLA's receiver thread aborts this
            // process.  A server left waiting for a cue by the abort is
            // nudged at connect time, so the next run recovers cleanly.
            throw cRuntimeError("CARLA Tick() timed out after %s: %s -- the tick "
                "may still have completed on the server, so it is deliberately "
                "not re-cued; aborting the run",
                m_tickTimeout.str().c_str(), e.what());
        }


        // Every sensor emits exactly one measurement per frame (sensor_tick
        // is deliberately unsupported), so the frame's measurement set is
        // complete as soon as each registered sensor has been observed once
        // for m_frame.  Waiting blocks the event loop -- the next tick cue
        // goes out only afterwards -- which is exactly the lock-step pacing
        // of the co-simulation (a CV consumer's processing time is part of
        // the loop).  Measurements that miss the deadline are not lost: the
        // next steps publish them, attributed to their true simulation
        // instant (see drainInbox).
        std::vector<RawSensorData> pending;
        {
            std::unique_lock<std::mutex> lock(m_inbox->mutex);
            auto remaining = m_frameSensorNames;
            // wait_for re-evaluates the predicate against a single deadline
            // until it returns true or the timeout expires; the predicate
            // drains the queue into `pending`, retiring sensors that
            // delivered a measurement for the ticked frame
            m_inbox->cv.wait_for(lock,
                std::chrono::duration<double>(m_sensorWaitTimeout.dbl()),
                [this, &remaining, &pending] {
                    while (!m_inbox->queue.empty()) {
                        auto raw = std::move(m_inbox->queue.front());
                        m_inbox->queue.pop_front();
                        if (raw.frame == m_frame) {
                            remaining.erase(raw.sensorName);
                        }
                        pending.push_back(std::move(raw));
                    }
                    return remaining.empty();
                });
        }
        drainInbox(std::move(pending));
    }

    measureAvatarResiduals();

    emit(stepSignal, simTime());

    scheduleAt(simTime() + m_stepInterval, m_stepEvent);
}

void CarlaScenarioManager::drainInbox(std::vector<RawSensorData> pending)
{
    for (auto& raw : pending) {

        // Correlate the measurement with the simulation instant of the CARLA
        // frame it was produced in: in lock-step, frame numbers advance by
        // exactly one per step, and data can only ever lag behind the ticked
        // frame, never lead it -- so a non-matching measurement is older by
        // (m_frame - raw.frame) steps.  The unsigned subtraction stays
        // correct across frame counter wrap-around, and the whole computation
        // runs in exact integer nanoseconds (OMNeT++'s native time unit), so
        // the attributed time is bit-identical to the simTime() of the tick
        // that advanced that frame; consumers can compare
        // reading.simTimeNs() with simTime().inUnit(SIMTIME_NS)
        const uint64_t lateFrames = m_frame - raw.frame;
        const int64_t lateNs = static_cast<int64_t>(lateFrames) * m_stepInterval.inUnit(SIMTIME_NS);
        raw.simTimeNs = (simTime() - omnetpp::SimTime(lateNs, SIMTIME_NS)).inUnit(SIMTIME_NS);
        if (raw.frame != m_frame) {
            ++m_lateReadings;
            EV_DEBUG << "Sensor data from CARLA frame " << raw.frame
                     << " received at frame " << m_frame << " (late by " << lateFrames
                     << " steps), attributed to t=" << static_cast<double>(raw.simTimeNs) * 1e-9
                     << 's' << std::endl;
        }

        CarlaSensorReading reading(raw);
        emit(sensorSignal, static_cast<const cObject*>(&reading));
    }
}

void CarlaScenarioManager::stopSensors()
{
    for (auto& sensor : m_sensors) {
        try {
            sensor->Stop();
        }
        catch (const std::exception& e) {
            EV_WARN << "Could not stop sensor: " << e.what() << std::endl;
        }
    }
    m_sensors.clear();
}

void CarlaScenarioManager::cleanupActors()
{
    try {
        for (auto& sensor : m_sensors) {
            sensor->Destroy();
        }
        m_frameSensorNames.clear();
        for (auto& entry : m_avatars) {
            // stop the streaming callbacks before destroying the parent, or
            // CARLA logs 'sensor object went out of the scope' leaks
            for (auto& sensor : entry.second.sensors) {
                sensor->Stop();
            }
            if (entry.second.actor) {
                entry.second.actor->Destroy();
            }
            entry.second.sensors.clear();
        }
    }
    catch (const std::exception& e) {
        EV_WARN << "Could not destroy CARLA actors: " << e.what() << std::endl;
    }
    m_sensors.clear();
    m_avatars.clear();
}

}  // namespace carla
}  // namespace artery
