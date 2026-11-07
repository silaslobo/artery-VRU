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
| Manager module | `CarlaScenarioManager` owns `carla::client::Client` + `carla::client::World`, spawns sensors and mirrors SUMO vehicles as avatars, and drives CARLA one frame per OMNeT++ step. |
| Sensor path | `Sensor::Listen()` callbacks marshal measurements into a mutex-guarded inbox; the event-loop thread drains it and emits the `carla.sensor` signal. |
| Build wiring | `src/artery/CMakeLists.txt`: `target_link_libraries(core PUBLIC carla)`, so every Artery run script picks up the library and its NED folder. |
| Avatar path | `CarlaAvatar` implements `traci::VehicleSink`: every SUMO vehicle automatically gets one, mirrored as a kinematic CARLA actor (`SetTransform()` per frame, physics off). SUMO→CARLA pose conversion lives in the header-only `CoordTransform.h` (unit-tested, CTest `coordtransform`). |
| Map pipeline | `tools/carla_map/gen_xodr.py` (+ `gen_routes.py`) generate one OpenDRIVE intersection consumed by *both* simulators; `CarlaScenarioManager` loads it with `GenerateOpenDriveWorld()` and verifies the built world against the generator (`mapCheckFile`, checksum round trip). |
| Example | [`scenarios/carla`](../../scenarios/carla) — `carla.CarlaSumoWorld` network (SUMO traffic master + CARLA sensor world) + `omnetpp-sumo.ini` + `run_carla_sumo_scenario` target, plus a `CarlaSensorSink` consumer module. |

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
   |                    sensors            |         |  mutex / condvar /     |
   |  step()        -> World->Tick(1 frame)|<--------|  deque<RawSensorData>  |
   |                    drainInbox()       |  swap   |  (shared_ptr, outlives |
   |                    emit(carla.sensor) |         |   the module)          |
   |                    emit(carla.step)   |         +------------------------+
   |  finish()      -> Stop/Destroy        |
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
  depth bound (`SensorInbox::highWatermark`, default 32 — sized for image
  payloads, ~120 MB worst case; overflow counted in `SensorInbox::dropped`);
  on overflow the *oldest* measurement is dropped so consumers always see
  the freshest data.  A `condition_variable` is notified.
* `CarlaScenarioManager::step()` (event-loop thread) swaps the queue out under
  the lock, builds a `CarlaSensorReading` per measurement and emits it.
* The inbox is owned by a `shared_ptr`, so a callback that is still in flight
  when the module is destroyed writes into memory that is still valid; the
  depth bound keeps such stragglers (e.g. when `Sensor::Stop()` failed over
  a lossy link) from accumulating.  All consumer-side waits are bounded by
  their timeout, so no shutdown signalling is needed anywhere.

Synchronisation of the *world clock*: CARLA streams sensor data asynchronously
while OMNeT++ is synchronous, and a wall-clock wait cannot be an OMNeT++ event
— the only bridge is a blocking wait on the simulation thread, which freezes
the event loop (and thereby the next `Tick()` cue, CARLA, and SUMO).  `step()`
performs exactly that wait, after ticking the frame:

* Every sensor emits one measurement per frame (`sensor_tick` is deliberately
  unsupported, see §6), so the frame's measurement set is complete as soon as
  every registered per-frame sensor has been observed once with
  `frame == frame()` — an exact integer frame-number match, wrap-around safe.
* `step()` waits on the inbox condition variable until then, or until
  `sensorWaitTimeout` (wall-clock) expires; the predicate drains the queue
  against a single deadline and retires sensors as their measurements are
  picked up.  Emission happens after the wait, in one batch.
* Because the event loop is frozen during wait *and* signal handling, a
  consumer's (e.g. computer-vision) processing time *is* the loop's pace:
  one tick == one frame == `stepInterval` of simulation time per completed
  cycle — the behaviour of a real-time system.
* Measurements that miss the deadline are not lost: later steps publish them,
  attributed **by frame number** — `simTimeNs = current simTime − (ticked
  frame − measurement frame) × stepInterval`, exact int64 ns, never
  floating-point timestamp arithmetic — so `CarlaSensorReading::simTimeNs()`
  is always exactly the simulation instant the measurement was produced at
  (`m_lateReadings` counts this case and is logged in `finish()`).  CARLA's
  own episode timestamp is deliberately *not* part of the reading contract —
  time is only ever `frame()` plus `simTimeNs()`, both exact integers.

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
`simTimeNs()` / `simTime()` (the simulation instant of the measurement's
CARLA frame, assigned by the manager as described in §3), the sensor transform (`x/y/z`, `pitch/yaw/roll`), GNSS
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

| `noRenderingMode` | `false` | headless rendering |
| `stepInterval` | `0.01s` | OMNeT++ step == `fixed_delta_seconds` (CARLA accepts 1 µs … 1/60 s) |
| `tickTimeout` | `60s` | per-frame tick timeout; a timed-out tick aborts the run (re-cueing could double-tick the server) |
| `sensorWaitTimeout` | `0.02s` | wall-clock time `step()` waits for the frame's complete sensor set |
| `startTime` | `0s` | connect at this simulation time |
| `sensors` | `sensor.other.gnss:10;sensor.other.imu:100;sensor.other.collision` | see grammar below |
| `copySensorPayload` | `false` | copy raw point-cloud/image bytes into the signal |
| `destroyActorsOnFinish` | `true` | remove own actors at the end |

Generated-map parameters (see `tools/carla_map` in the scenario README):

| Parameter | Default | Meaning |
|---|---|---|
| `opendriveFile` | `""` | load this `.xodr` via `GenerateOpenDriveWorld()` instead of the server's current map |
| `mapCheckFile` | `""` | `.lanepoints` sidecar; after generation, verify CARLA's built lanes against it (max/mean deviation, convention detection) |
| `vertexDistance`, `additionalWidth`, `simplificationPercentage`, `smoothJunctions`, `wallHeight` | `0.2`, `0`, `0`, `false`, `0` | mesh generation options; the defaults keep the mesh geometry-accurate to the xodr |
| `avatarResidualCheck` | `true` | record `sync.geometryResidual` per avatar/frame |

Avatar API (used by `CarlaAvatar`, also usable directly):
`acquireAvatar(key, pose)`, `updateAvatar(key, pose)`, `releaseAvatar(key)`,
`attachAvatarSensors(key, spec)` — all queue-only; the CARLA RPCs happen inside
`step()` before `Tick()`, so the lock-step contract is never broken from
TraCI callback threads.

Sensor list grammar (`;`-separated entries):

```
<blueprint>[@<x>,<y>,<z>[,<pitch>,<yaw>[,<roll>]]][!<attr>=<value>[,<attr>=<value>]*]
```
Every spawned sensor emits **one measurement per world frame**; all of them
are therefore part of the set `step()` waits for (see §3; event
sensors like `sensor.other.collision` report occurrences only and are never
waited for).

**Data rate control** — CARLA's `sensor_tick` blueprint attribute is
deliberately *unsupported* (there is no `:<frequency>` grammar).  It would
reduce the data CARLA generates, but it makes synchronization substantially
harder: the server's emission schedule is an unobservable floating-point
accumulation (no phase, ±1 frame jitter against `fixed_delta_seconds`), so a
the barrier in `step()` could only wait on guessed, jittering frame
masks.  To obtain a lower data rate, keep sensors at per-frame emission and
**drop arriving measurements on the consumer side** instead (skip a
`carla.sensor` reading whenever, e.g., `frame % k != 0`): every consumer
sees exactly the frames it wants with exact `simTimeNs()` attribution, and
the barrier logic stays trivially "all sensors, every frame".  In the
target server-side deployment dropping is free anyway — the cost was only
ever WAN bandwidth.

Offsets are absolute in CARLA world coordinates for manager-level sensors;
for avatar sensors they are vehicle-local (`x` = fore).  Blueprint attributes are verified with `ContainsAttribute`
(`sensor.camera.rgb` uses `image_size_x`, `image_size_y` and `fov`; there is
no `width`/`height`).

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
  unfreeze the world.  The server is otherwise deliberately left in synchronous
  mode (lock-step is the only supported mode; no settings are restored).
  CARLA teardown runs inside a `try/catch` because destructors must not
  throw (`carla::client::TimeoutException` escaping a destructor aborts the
  process).
* **Blueprint handling** — `BlueprintLibrary::Find()` returns a `const_pointer`
  and `nullptr` when the id is unknown, so the pointer is checked and the
  blueprint is copied before attributes are modified; unknown attributes are
  skipped with a warning (e.g. `width` does not exist — use `image_size_x`;
  `sensor_tick` is never set, see §6 "Data rate control").
* Traffic participants enter Artery through `traci::Manager` (SUMO is the
  traffic master); CARLA mirrors them. CARLA-side positions are *not* fed back
  into SUMO, and no control commands are sent from Artery applications to
  CARLA actors.
* CARLA and SUMO coordinate frames: `x_carla = x_xodr`, `y_carla = -y_xodr`,
  `yaw_carla = angle_sumo - 90°` (verified empirically at map-load time, see
  `CoordTransform.h` and the `coordtransform` unit test).
* Avatars spawn with `zOffset = 0.02`: the road surface of a generated map is
  z=0, and spawning exactly on it is rejected by `TrySpawnActor`'s overlap
  check (the avatar then silently never appears), while a few centimetres of
  clearance pass and stay invisible on camera.  A rejected spawn aborts the
  run — the vehicle exists on the SUMO side, so a blocked pose is a
  programming error, not a transient condition.

---

## 8. Running

```bash
cmake -S . -B build && cmake --build build -j$(nproc)
cmake --build build --target run_carla_sumo_scenario   # or run opp_run manually
```

See [`scenarios/carla/README.md`](../../scenarios/carla/README.md) for the
manual `opp_run` command line and a sample log.
