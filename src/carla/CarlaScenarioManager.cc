#include "CarlaScenarioManager.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <fstream>
#include <iterator>
#include <limits>
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
 * grammar: entry := blueprint [":" frequency] ["@" x,y,z[,pitch,yaw[,roll]]]
 *                  ["!" key=value[,key=value]*]
 * entries are separated by ';'.
 */
struct SensorSpec {
    std::string blueprint;
    double frequency {0.0};
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

        auto colon = remainder.find(':');
        if (colon != std::string::npos) {
            spec.frequency = std::strtod(remainder.substr(colon + 1).c_str(), nullptr);
            remainder = remainder.substr(0, colon);
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

/// Parse a "x,y,z[,pitch,yaw[,roll]]" string into a CARLA transform.
cg::Transform parseTransform(const std::string& text)
{
    SensorSpec spec;
    const auto fields = split(text, ',');
    double* components[6] = {&spec.x, &spec.y, &spec.z, &spec.pitch, &spec.yaw, &spec.roll};
    for (size_t i = 0; i < fields.size() && i < 6; ++i) {
        *components[i] = std::strtod(fields[i].c_str(), nullptr);
    }
    return makeTransform(spec);
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
RawSensorData toRaw(const boost::shared_ptr<::carla::sensor::SensorData>& data,
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

    if (auto gnss = boost::dynamic_pointer_cast<csd::GnssMeasurement>(data)) {
        raw.kind = SensorKind::Gnss;
        raw.latitude = gnss->GetLatitude();
        raw.longitude = gnss->GetLongitude();
        raw.altitude = gnss->GetAltitude();
    }
    else if (auto imu = boost::dynamic_pointer_cast<csd::IMUMeasurement>(data)) {
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
    else if (auto collision = boost::dynamic_pointer_cast<csd::CollisionEvent>(data)) {
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
    else if (auto obstacle = boost::dynamic_pointer_cast<csd::ObstacleDetectionEvent>(data)) {
        raw.kind = SensorKind::ObstacleDetection;
        raw.count = static_cast<int64_t>(obstacle->GetDistance());
        if (auto other = obstacle->GetOtherActor()) {
            raw.otherActorId = other->GetId();
            raw.otherActorName = other->GetTypeId();
        }
    }
    else if (boost::dynamic_pointer_cast<csd::LaneInvasionEvent>(data)) {
        raw.kind = SensorKind::LaneInvasion;
    }
    else if (auto lidar = boost::dynamic_pointer_cast<csd::LidarMeasurement>(data)) {
        raw.kind = SensorKind::Lidar;
        raw.count = static_cast<int64_t>(lidar->size());
        if (copyPayload && !lidar->empty()) {
            const auto* first = reinterpret_cast<const uint8_t*>(&lidar->at(0));
            const auto* last = first + lidar->size() * sizeof(csd::LidarDetection);
            raw.payload.assign(first, last);
        }
    }
    else if (auto semantic = boost::dynamic_pointer_cast<csd::SemanticLidarMeasurement>(data)) {
        raw.kind = SensorKind::SemanticLidar;
        raw.count = static_cast<int64_t>(semantic->size());
    }
    else if (auto radar = boost::dynamic_pointer_cast<csd::RadarMeasurement>(data)) {
        raw.kind = SensorKind::Radar;
        raw.count = static_cast<int64_t>(radar->size());
    }
    else if (auto image = boost::dynamic_pointer_cast<csd::Image>(data)) {
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
      m_port(2000),
      m_synchronous(true)
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
    m_synchronous = par("synchronousMode");
    m_stepInterval = par("stepInterval").doubleValue();
    m_sensorWaitTimeout = par("sensorWaitTimeout").doubleValue();
    m_tickTimeout = par("tickTimeout").doubleValue();
    m_tickRetries = static_cast<unsigned>(int(par("tickRetries")));

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

    if (m_world && par("destroyActorsOnFinish")) {
        cleanupActors();
    }

    // never leave the server stuck in synchronous mode
    if (m_world && m_synchronous && m_savedSettings) {
        for (unsigned attempt = 0; attempt < 2; ++attempt) {
            try {
                m_world->ApplySettings(*m_savedSettings, toCarlaDuration(10.0));
                break;
            }
            catch (const std::exception& e) {
                EV_WARN << "Could not restore CARLA server settings: " << e.what() << std::endl;
                // the server might just be waiting for the next tick cue
                try {
                    m_world->Tick(toCarlaDuration(5.0));
                }
                catch (const std::exception&) {
                    break;
                }
            }
        }
    }

    disconnect();
}

void CarlaScenarioManager::disconnect()
{
    // Destructors must not throw; CARLA may very well time out here (e.g. when
    // the server is still waiting for a tick cue), so swallow any error.
    try {
        m_sensors.clear();
        m_ego.reset();
        m_avatars.clear();
        m_savedSettings.reset();
        m_world.reset();
        m_client.reset();
    }
    catch (const std::exception& e) {
        EV_WARN << "Error while disconnecting from the CARLA server: " << e.what() << std::endl;
    }

    {
        std::lock_guard<std::mutex> guard(m_inbox->mutex);
        m_inbox->closed = true;
        if (m_inbox->dropped > 0) {
            EV_WARN << "Dropped " << m_inbox->dropped
                    << " CARLA sensor measurements (queue overflow)" << std::endl;
        }
    }
    m_inbox->cv.notify_all();
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
                << m_world->GetSettings().fixed_delta_seconds.get_value_or(0.0) << "s step"
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

        if (m_synchronous) {
            m_savedSettings.reset(new ::carla::rpc::EpisodeSettings(m_world->GetSettings()));

            auto settings = *m_savedSettings;
            settings.synchronous_mode = true;
            settings.no_rendering_mode = par("noRenderingMode");
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
                    << " fixed_delta=" << applied.fixed_delta_seconds.get_value_or(0.0)
                    << " no_rendering=" << applied.no_rendering_mode << std::endl;
        }

        spawnEgoVehicle();
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

void CarlaScenarioManager::spawnEgoVehicle()
{
    if (!par("spawnEgoVehicle")) {
        return;
    }

    const auto blueprintId = par("vehicleBlueprint").stdstringValue();
    auto library = m_world->GetBlueprintLibrary();
    // Find() returns a const handle; work on a copy so that attributes can be set
    auto found = library->Find(blueprintId);
    if (found == nullptr) {
        throw cRuntimeError("CARLA blueprint '%s' not found", blueprintId.c_str());
    }
    auto blueprint = *found;

    const auto spec = par("spawnTransform").stdstringValue();
    const auto from = par("spawnFrom").stdstringValue();
    std::vector<cg::Transform> spawnPoints;
    if (!spec.empty()) {
        spawnPoints.push_back(parseTransform(spec));
    }
    else if (!from.empty()) {
        // generated worlds have no recommended spawn points: take the pose
        // computed by the map generator from its own geometry
        CarlaPose pose;
        if (!resolveSpawnFrom(from, pose)) {
            throw cRuntimeError("spawnFrom '%s' not found in the spawn sidecar "
                                "(wrong road/lane/s?)", from.c_str());
        }
        pose.z += 0.2;
        cg::Transform transform;
        transform.location = cg::Location(pose.x, pose.y, pose.z);
        transform.rotation = cg::Rotation(0.0, pose.yaw, 0.0);
        spawnPoints.push_back(transform);
    }
    else {
        auto map = m_world->GetMap();
        spawnPoints = map->GetRecommendedSpawnPoints();
        if (spawnPoints.empty()) {
            throw cRuntimeError("CARLA map provides no recommended spawn points "
                                "(use spawnFrom for generated maps)");
        }
    }

    cg::Transform transform;
    boost::shared_ptr<cc::Actor> actor;
    const size_t maxAttempts = std::min<size_t>(spawnPoints.size(), 20u);
    for (size_t attempt = 0; attempt < maxAttempts && !actor; ++attempt) {
        transform = spawnPoints.size() > 1 ?
            spawnPoints[getRNG(0)->intRand(spawnPoints.size())] : spawnPoints.front();
        // TrySpawnActor returns a null handle instead of throwing, e.g. when
        // the spawn position is blocked by actors of a previous run
        actor = m_world->TrySpawnActor(blueprint, transform);
    }
    if (!actor) {
        throw cRuntimeError("Could not spawn CARLA vehicle '%s' (spawn position blocked?)",
            blueprintId.c_str());
    }
    m_ego = boost::static_pointer_cast<cc::Vehicle>(actor);
    m_egoId = m_ego->GetId();
    m_ego->SetAutopilot(par("autopilot"));
    EV_INFO << "Spawned ego vehicle " << m_egoId << " (" << blueprintId << ") at (" << transform.location.x
            << ',' << transform.location.y << ',' << transform.location.z << ')' << std::endl;
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

bool CarlaScenarioManager::resolveSpawnFrom(const std::string& spec, CarlaPose& pose) const
{
    // spec: "roadId,laneId,s"; sidecar lines: "road lane s x y hdg cx cy cyaw"
    const auto fields = split(spec, ',');
    if (fields.size() != 3) {
        throw cRuntimeError("spawnFrom must be 'roadId,laneId,s', got '%s'", spec.c_str());
    }
    const int road = std::atoi(fields[0].c_str());
    const int lane = std::atoi(fields[1].c_str());
    const double s = std::strtod(fields[2].c_str(), nullptr);

    std::string file = par("spawnPointsFile").stdstringValue();
    if (file.empty()) {
        file = par("opendriveFile").stdstringValue() + ".spawn";
    }
    std::ifstream in(file);
    if (!in) {
        throw cRuntimeError("Cannot open spawn sidecar '%s' (generated by "
                            "gen_xodr.py alongside the .xodr)", file.c_str());
    }
    std::string line;
    double bestDelta = -1.0;
    int bestRoad = 0, bestLane = 0; 
    double carlaX = 0, carlaY = 0, carlaYaw = 0;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream ls(line);
        int r, l;
        double ps, x, y, hdg, cx, cy, cyaw;
        if (!(ls >> r >> l >> ps >> x >> y >> hdg >> cx >> cy >> cyaw)) {
            continue;
        }
        if (r != road || l != lane) {
            continue;
        }
        const double delta = std::fabs(ps - s);
        if (bestDelta < 0.0 || delta < bestDelta) {
            bestDelta = delta;
            bestRoad = r;
            bestLane = l;
            carlaX = cx;
            carlaY = cy;
            carlaYaw = cyaw;
        }
    }
    if (bestDelta < 0.0) {
        return false;
    }
    EV_INFO << "spawnFrom " << road << ',' << lane << ',' << s << " resolved to s="
            << (bestDelta) << " m offset, CARLA pose (" << carlaX << ',' << carlaY
            << ',' << carlaYaw << ")" << std::endl;
    pose.x = carlaX;
    pose.y = carlaY;
    pose.z = 0.0;
    pose.yaw = carlaYaw;
    return true;
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

    boost::optional<::carla::client::ActorBlueprint> blueprint;
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
                // TrySpawnActor returns null instead of throwing (e.g. when a
                // queued vehicle departs directly behind another one); keep
                // the avatar pending and retry on the next frame
                auto actor = m_world->TrySpawnActor(*blueprint, transform);
                if (!actor) {
                    if (avatar.spawnWarnings < 3) {
                        ++avatar.spawnWarnings;
                        EV_WARN << "Could not spawn CARLA avatar '" << entry.first
                                << "' at (" << avatar.pose.x << ',' << avatar.pose.y
                                << ") yet, retrying next frame" << std::endl;
                    }
                    continue;
                }
                avatar.actor = boost::static_pointer_cast<cc::Vehicle>(actor);
                avatar.actor->SetAutopilot(false);
                if (!par("avatarSimulatePhysics")) {
                    avatar.actor->SetSimulatePhysics(false);
                }
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
    spawnSensorList(m_ego ? m_ego.get() : nullptr, par("sensors").stdstringValue(),
        std::string(), m_sensors);
}

void CarlaScenarioManager::spawnSensorList(cc::Actor* parent, const std::string& text,
    const std::string& name_prefix,
    std::vector<boost::shared_ptr<cc::Sensor>>& out)
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
        // CARLA controls the measurement rate of a sensor through its
        // "sensor_tick" attribute (period in seconds)
        if (spec.frequency > 0.0) {
            if (blueprint.ContainsAttribute("sensor_tick")) {
                blueprint.SetAttribute("sensor_tick", std::to_string(1.0 / spec.frequency));
            }
            else {
                EV_WARN << "CARLA blueprint " << spec.blueprint
                        << " has no sensor_tick attribute, ignoring requested frequency"
                        << std::endl;
            }
        }
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
        auto sensor = boost::static_pointer_cast<cc::Sensor>(actor);

        // The callback runs on a CARLA streaming thread: it may only touch
        // CARLA data and the mutex guarded inbox, never OMNeT++ itself.
        auto inbox = m_inbox;
        const std::string sensorName = name;
        const std::string sensorType = spec.blueprint;
        sensor->Listen([inbox, sensorName, sensorType, copyPayload](
                           boost::shared_ptr<::carla::sensor::SensorData> data) {
            if (inbox->closed) {
                return;
            }
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

        EV_INFO << "Attached sensor " << sensor->GetId() << " (" << spec.blueprint
                << ", frequency=" << spec.frequency << ", name=" << name << ')'
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

    if (m_synchronous && m_world) {
        const auto tickTimeout = toCarlaDuration(m_tickTimeout.dbl());
        for (unsigned attempt = 1; true; ++attempt) {
            try {
                m_frame = m_world->Tick(tickTimeout);
                break;
            }
            catch (const cc::TimeoutException& e) {
                if (attempt > m_tickRetries) {
                    throw cRuntimeError(
                        "CARLA Tick() timed out after %u attempts (server not in "
                        "synchronous mode?): %s",
                        attempt, e.what());
                }
                EV_WARN << "CARLA Tick() timed out, sending another tick cue" << std::endl;
            }
        }

        // sensor data of a frame is streamed asynchronously and may arrive
        // shortly after Tick() returned
        if (m_sensorWaitTimeout.dbl() > 0.0 && !m_sensors.empty()) {
            std::unique_lock<std::mutex> lock(m_inbox->mutex);
            m_inbox->cv.wait_for(lock,
                std::chrono::duration<double>(m_sensorWaitTimeout.dbl()),
                [this] { return !m_inbox->queue.empty() || m_inbox->closed; });
        }
    }

    drainInbox();

    measureAvatarResiduals();

    emit(stepSignal, simTime());

    scheduleAt(simTime() + m_stepInterval, m_stepEvent);
}

std::vector<RawSensorData> CarlaScenarioManager::takePending()
{
    std::vector<RawSensorData> pending;
    std::lock_guard<std::mutex> guard(m_inbox->mutex);
    pending.assign(std::make_move_iterator(m_inbox->queue.begin()),
        std::make_move_iterator(m_inbox->queue.end()));
    m_inbox->queue.clear();
    m_droppedReadings = m_inbox->dropped;
    return pending;
}

void CarlaScenarioManager::drainInbox()
{
    const auto pending = takePending();
    for (const auto& raw : pending) {
        if (!m_epochSet) {
            m_carlaEpoch = raw.timestamp - simTime().dbl();
            m_epochSet = true;
        }
        m_carlaTime = raw.timestamp;

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
        if (m_ego) {
            m_ego->Destroy();
        }
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
    m_ego.reset();
    m_avatars.clear();
}

SimTime CarlaScenarioManager::toSimTime(double carlaTimestamp) const
{
    if (!m_epochSet) {
        return simTime();
    }
    return SimTime(carlaTimestamp - m_carlaEpoch, SIMTIME_S);
}

}  // namespace carla
}  // namespace artery
