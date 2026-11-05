# CARLA integration for Artery (`src/carla`)

This library connects an Artery (OMNeT++) simulation to a running
[CARLA](https://carla.readthedocs.io) simulator, in the same way
[`src/traci`](../traci) connects it to SUMO.

---

## 1. What was done

| Area | Change |
|---|---|
| CARLA C++ client | `cmake/carla/CarlaClient.cmake` builds `libcarla_client.a`, `librpc.a` and Recast/Detour from the pinned submodules via `ExternalProject` and exports `Carla::Client`, `Carla::Rpc`, `Carla::Recast`. Two fixes were needed to make the static CARLA code usable from an OMNeT++ shared library: `-fPIC` for libcarla, and explicit ordering of the Detour archives (see §5). |
| New library | `src/carla/` → `libartery_carla.so`, compiled with C++17 (CARLA headers), added from the root `CMakeLists.txt` with `add_subdirectory(src/carla)`. |
| Manager module | `CarlaScenarioManager` owns `carla::client::Client` + `carla::client::World`, spawns an ego vehicle and sensors, and drives CARLA one frame per OMNeT++ step. |
| Sensor path | `Sensor::Listen()` callbacks marshal measurements into a mutex-guarded inbox; the event-loop thread drains it and emits the `carla.sensor` signal. |
| Build wiring | `src/artery/CMakeLists.txt`: `target_link_libraries(core PUBLIC carla)`, so every Artery run script picks up the library and its NED folder. |
| Example | [`scenarios/carla`](../../scenarios/carla) — `carla.CarlaWorld` network + `omnetpp.ini` + `run_carla_scenario` target, plus a `CarlaSensorSink` consumer module. |

---

## 2. Architecture

```
              CARLA server (UE4, RPC :2000)
                        ^
                        |  Tick() / ApplySettings()        Listen() callbacks
                        |  (OMNeT++ thread)                (CARLA streaming threads)
                        |                                        |
   +--------------------+------------------+         +-----------v------------+
   | CarlaScenarioManager (cSimpleModule)  |         | RawSensorData (plain)  |
   |                                       |         +-----------+------------+
   |  initialize()  -> schedule connect    |                     | push_back + notify
   |  connect()     -> Client, World,      |         +-----------v------------+
   |                    sync settings,     |         | SensorInbox            |
   |                    ego + sensors      |         |  mutex / condvar /     |
   |  step()        -> World->Tick(1 frame)|<--------|  deque<RawSensorData>  |
   |                    drainInbox()       |  swap   |  (shared_ptr, outlives |
   |                    emit(carla.sensor) |         |   the module)          |
   |                    emit(carla.step)   |         +------------------------+
   |  finish()      -> Stop/Destroy,       |
   |                    restore settings   |
   +-------------------+-------------------+
                        |  signal "carla.sensor"  (payload: CarlaSensorReading : cNamedObject)
       -----------------+--------------------------------------------
       |                         |                                  |
  CarlaSensorSink          (middleware/service)              (any module with
  (example consumer,       e.g. subscribe like               a signal hook)
  counts + logs)           VehicleMiddleware consumes
                           MobilityBase::stateChangedSignal)
```

Time base: **one OMNeT++ step event == one CARLA frame**. `stepInterval` is
written to CARLA's `fixed_delta_seconds`, the module schedules its step message
with `std::numeric_limits<short>::min()` scheduling priority (like
`traci::Core`, i.e. before other periodic events), calls `World::Tick()` and
then drains the inbox, so a consumer always sees the measurements belonging to
the frame that has just completed.

---

## 3. Threading model (the critical part)

`carla::client::Sensor::Listen()` invokes its callback on one of CARLA's
asio/receiver threads, never on the OMNeT++ simulation thread. Therefore:

* The callback captures only a `shared_ptr<SensorInbox>` plus the sensor's
  name/type strings — **never `this` and never an OMNeT++ object**.
* Inside the callback the CARLA message (`SharedPtr<sensor::SensorData>`) is
  converted into `RawSensorData`, a plain struct of doubles/strings/bytes. Only
  CARLA and STL code runs there; no RPC, no OMNeT++ API.
* The struct is moved into a `std::deque` under `std::mutex`, guarded by a
  high-water mark (`10000`, overflow counted in `SensorInbox::dropped`), and a
  `condition_variable` is notified.
* `CarlaScenarioManager::step()` (event-loop thread) swaps the queue out under
  the lock, builds a `CarlaSensorReading` per measurement and emits it.
* The inbox is owned by a `shared_ptr`, so a callback that is still in flight
  when the module is destroyed writes into memory that is still valid.
* Shutdown sets `SensorInbox::closed` and notifies, so no thread waits for a
  consumer that is gone.

Synchronisation of the *world clock*: after `Tick()` the frame's sensor data may
still be in flight on the streaming socket, so the module optionally waits up
to `sensorWaitTimeout` on the condition variable for at least one measurement to
arrive. CARLA's own episode timestamp is kept (`carlaTime()`, `frame()`) and can
be mapped to simulation time with `toSimTime(double carlaTimestamp)` (offset
captured with the first received frame).

---

## 4. Public signal contract

| Signal | Type | Emitted |
|---|---|---|
| `carla.init` | `simtime_t` | after connect / settings applied / actors spawned |
| `carla.step` | `simtime_t` | after every CARLA frame has been processed |
| `carla.close` | `simtime_t` | in `finish()` |
| `carla.sensor` | `object` (`CarlaSensorReading`) | once per received measurement, inside the event loop |

`CarlaSensorReading` exposes `kind()` / `kindName()` (`gnss`, `imu`,
`collision`, `laneInvasion`, `obstacleDetection`, `lidar`, `semanticLidar`,
`radar`, `image`, `other`), `sensorName()`, `sensorType()`, `frame()`,
`carlaTimestamp()`, the sensor transform (`x/y/z`, `pitch/yaw/roll`), GNSS
(`latitude/longitude/altitude`), IMU (`accel*`, `gyro*`, `compass`), collision
(`impulse*`, `otherActorId/Name`), image (`width/height/fov`), `count()` and the
optional raw `payload()`.

Ownership: the reading is stack-allocated by the emitter and only valid during
`receiveSignal()`; consumers that keep it must call `dup()`.

Subscribing (see `CarlaSensorSink`):

```cpp
void MyModule::initialize(int stage) {
    cSimpleModule::initialize(stage);
    if (stage == 0) {
        getSystemModule()->subscribe(cComponent::registerSignal("carla.sensor"), this);
    }
}
```
(multiply inherit `omnetpp::cListener`; signals propagate up the module tree, so
subscribing to the network module catches every manager).

---

## 5. Build integration

`src/carla/CMakeLists.txt` follows the `src/traci` pattern — a `SHARED`
OMNeT++ library with `OMNETPP_LIBRARY ON` and `NED_FOLDERS`, installed together
with its NED files. Two CARLA-specific points:

1. **C++ standard.** `set_target_properties(carla PROPERTIES CXX_STANDARD 17)` —
   libcarla headers require C++17 while Artery itself is C++11/14; the newer
   standard is kept local to this target. In `CarlaScenarioManager.cc`,
   `#undef PI` is required before including CARLA headers because OMNeT++ defines
   `PI` as a macro.
2. **Static library order.** `libcarla_client.a` references Detour/DetourCrowd
   symbols. CMake moves the static dependencies of a target's
   `INTERFACE_LINK_LIBRARIES` to the end of the link line, where they no longer
   resolve those references. `CarlaClient.cmake` therefore also exports the plain
   list `CARLA_RECAST_LIBRARIES` (Recast, DetourCrowd, DetourTileCache, Detour)
   which `src/carla` links right after `Carla::Client`. The archives additionally
   have to be built with `-fPIC`, since they end up inside a `.so`.

Because `core` links `carla` PUBLIC, `run_artery.sh` (and the scenario run
targets) automatically get `-n .../src/carla` and `-l libartery_carla.so`.

---

## 6. Module parameters (`CarlaScenarioManager.ned`)

| Parameter | Default | Meaning |
|---|---|---|
| `host`, `port` | `localhost`, `2000` | CARLA RPC endpoint |
| `timeout` | `20s` | timeout for all CARLA RPCs |
| `map` | `""` | map to load; empty keeps the currently loaded world |
| `synchronousMode` | `true` | switch the server to lock-step mode |
| `noRenderingMode` | `false` | headless rendering |
| `stepInterval` | `0.01s` | OMNeT++ step == `fixed_delta_seconds` (CARLA accepts 1 µs … 1/60 s) |
| `tickTimeout`, `tickRetries` | `10s`, `2` | per-frame tick timeout / number of tick cues |
| `sensorWaitTimeout` | `0.02s` | wait for the frame's sensor data after `Tick()` |
| `startTime` | `0s` | connect at this simulation time |
| `spawnEgoVehicle`, `vehicleBlueprint`, `spawnTransform`, `autopilot` | `true`, `vehicle.tesla.model3`, `""`, `true` | ego vehicle (`spawnTransform` empty ⇒ random recommended spawn point, retried on blocked positions) |
| `sensors` | `sensor.other.gnss:10;sensor.other.imu:100;sensor.other.collision` | see grammar below |
| `copySensorPayload` | `false` | copy raw point-cloud/image bytes into the signal |
| `destroyActorsOnFinish` | `true` | remove own actors at the end |

Sensor list grammar (`;`-separated entries):

```
<blueprint>[:<frequency>][@<x>,<y>,<z>[,<pitch>,<yaw>[,<roll>]]][!<attr>=<value>[,<attr>=<value>]*]
```
`<frequency>` is written to the blueprint's `sensor_tick` attribute (period in
seconds); offsets are relative to the ego vehicle when one was spawned,
otherwise absolute in CARLA coordinates.

---

## 7. Hard-won details / limitations

* **`SimTime` truncation** — `SimTime(double, SimTimeUnit)` takes an `int64_t`
  value, so `SimTime(0.01, SIMTIME_S)` is `0`. The parameters are assigned as
  plain doubles instead.
* **`fixed_delta_seconds` range** — values above 1/60 s are silently ignored by
  CARLA (symptom: server keeps reporting `fixed_delta=0` and runs asynchronously).
* **`ApplySettings()` verification** — libcarla ticks up to 30 times waiting for
  two consecutive snapshots whose frame delta equals `fixed_delta_seconds` to the
  last bit; over a WAN that loop can warn or time out although the settings *are*
  applied. The integration treats such a timeout as a warning and continues,
  driving the world with its own ticks.
* **Wedged servers** — a client that is killed leaves the server waiting for
  tick cues (every RPC then times out). On connect the module checks
  `GetSettings().synchronous_mode`, and if it is already on, sends a tick cue to
  unfreeze the world; on `finish()` the original settings are restored (with one
  retry). CARLA teardown runs inside a `try/catch` because destructors must not
  throw (`carla::client::TimeoutException` escaping a destructor aborts the
  process).
* **Blueprint handling** — `BlueprintLibrary::Find()` returns a `const_pointer`
  and `nullptr` when the id is unknown, so the pointer is checked and the
  blueprint is copied before attributes are modified; unknown attributes are
  skipped with a warning (e.g. `frequency` does not exist — use `sensor_tick`).
* Only the co-simulation side is implemented: CARLA positions are *not* yet fed
  into Artery mobility/`NodeManager` (no vehicle insertion or
  `MobilityBase::stateChangedSignal` emulation), and no control commands are
  sent back to CARLA from Artery applications.

---

## 8. Running

```bash
cmake -S . -B build && cmake --build build --target carla -j$(nproc)
cmake --build build --target run_carla_scenario        # or run opp_run manually
```

See [`scenarios/carla/README.md`](../../scenarios/carla/README.md) for the
manual `opp_run` command line and a sample log.
