#include "CarlaScenarioManager.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
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
    // run before any other module's periodic events, like traci::Core does
    m_stepEvent->setSchedulingPriority(std::numeric_limits<short>::min());

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

        const auto map = par("map").stdstringValue();
        if (map.empty()) {
            m_world.reset(new cc::World(m_client->GetWorld()));
        }
        else {
            m_world.reset(new cc::World(m_client->LoadWorld(map)));
        }
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
    std::vector<cg::Transform> spawnPoints;
    if (spec.empty()) {
        auto map = m_world->GetMap();
        spawnPoints = map->GetRecommendedSpawnPoints();
        if (spawnPoints.empty()) {
            throw cRuntimeError("CARLA map provides no recommended spawn points");
        }
    }
    else {
        spawnPoints.push_back(parseTransform(spec));
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

void CarlaScenarioManager::spawnSensors()
{
    const auto specs = parseSensors(par("sensors").stdstringValue());
    if (specs.empty()) {
        return;
    }

    auto library = m_world->GetBlueprintLibrary();
    const bool copyPayload = par("copySensorPayload");
    cc::Actor* parent = m_ego ? m_ego.get() : nullptr;

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

        std::string name = spec.blueprint;
        auto dot = name.rfind('.');
        if (dot != std::string::npos && dot + 1 < name.size()) {
            name = name.substr(dot + 1);
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
                << ", frequency=" << spec.frequency << ')' << std::endl;
        m_sensors.push_back(sensor);
    }
}

void CarlaScenarioManager::step()
{
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
    }
    catch (const std::exception& e) {
        EV_WARN << "Could not destroy CARLA actors: " << e.what() << std::endl;
    }
    m_sensors.clear();
    m_ego.reset();
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
