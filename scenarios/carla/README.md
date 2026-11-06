# CARLA + SUMO co-simulation scenario

See [`src/carla/README.md`](../../src/carla/README.md) for the architecture of
the CARLA back end (threading, time synchronisation, signal contract).

This scenario runs a **generated intersection** simultaneously in SUMO and
CARLA, in lock-step:

* **SUMO** (via `traci::Manager`) is the traffic master: it simulates every
  vehicle and pedestrian.
* **CARLA** (via `CarlaScenarioManager`) is the sensor world: each SUMO vehicle
  is mirrored onto a kinematic CARLA avatar (`carla.CarlaAvatar`) right before
  `World::Tick()`, and CARLA sensors attached to those avatars flow back into
  OMNeT++ as `carla.sensor` signals (consumed here by `CarlaSensorSink`).
* One SUMO step == one CARLA frame == 10 ms of simulation time
  (`stepInterval` == SUMO `step-length`).

## Map artifacts (`maps/`)

Both simulators load *the same* geometry, generated deterministically from
`tools/carla_map/gen_xodr.py` (configuration-as-code, Python stdlib only):

```
gen_xodr.py ──> intersection.xodr ──┬──> CARLA GenerateOpenDriveWorld()
                     │              │    (+ .spawn / .lanepoints sidecars)
                     └── netconvert ┴──> intersection.net.xml ──> SUMO
                                             │
                        gen_routes.py ──> intersection.rou.xml (movements are
                                          derived from the xodr <junction>)
```

The generated artifacts in `maps/` (everything except `intersection.sumocfg`)
are **not committed** — they are reproducible output of the tool chain.  The
`carla_maps` CMake target re-runs the chain automatically (also on every
`cmake --build build`) whenever a generator changes, and regeneration is
byte-deterministic, so the two simulators cannot drift apart silently.
`netconvert` MUST keep `--offset.disable-normalization=true` (the SUMO net has
to share the xodr coordinate frame).

To change the intersection dimensions, edit the `# PLACEHOLDER dimensions` block
in `tools/carla_map/gen_xodr.py` and re-run the target.

## Run

```bash
cmake -S . -B build && cmake --build build -j$(nproc)

cmake --build build --target run_carla_sumo_scenario
```

Point `*.carlaHost` / `*.carlaPort` in `omnetpp-sumo.ini` at your CARLA server;
it starts SUMO itself via `PosixLauncher`, so only the CARLA server has to run
externally.

To watch SUMO with the GUI (requires an X display), select the GUI run
configuration, e.g. append `-c Config-sumo-gui` to the generated `opp_run`
command line.

## Expected output

```
[INFO]  Connecting to CARLA server big-boss.tail4b138a.ts.net:2000
[INFO]  Connected to CARLA 0.9.16, world 0.01s step
[INFO]  Generating CARLA world from OpenDRIVE 'maps/intersection.xodr' ...
[INFO]  Loaded CARLA map name='Carla/Maps/OpenDriveMap' geoReference lat=55.6761 lon=12.5683 alt=0
[INFO]  Server stores exactly the OpenDRIVE string that was sent
[INFO]  Map geometry check: CARLA coordinate convention x_carla = x_xodr, y_carla = -y_xodr
[INFO]  Map geometry check: 1120 lane samples, max deviation 0.0004 m, ...
[INFO]  TraCI server is SUMO 1.28.0 with API level 22
** Event #3  t=0.01  CarlaSumoWorld.traci.core
** Event #4  t=0.01  CarlaSumoWorld.carlaManager       <- alternating: lock-step
...
[INFO]  Spawned CARLA avatar 'v_e10_to_e15_0' (actor 87) from vehicle.tesla.model3 at (-5,-115.4,90)
[INFO]  sensorSink <- carlaManager: gnss(v_e10_to_e15_0.gnss) ... lat=55.6751 lon=12.5682
...
[INFO]  avatar pose residual: n=140 max=0.13 m mean=0.003 m
[INFO]  received 14 gnss measurements
```

## Verification built into the run

* **Static, map level**: 1120 lane-centre samples from the generator are
  projected onto the map CARLA actually built (`mapCheckFile`); the run asserts
  the deviation is in the sub-millimetre range and detects CARLA's coordinate
  convention empirically.
* **Identity**: the OpenDRIVE string is checksummed before/after upload
  (`GetOpenDrive()` must return exactly what was sent).
* **Dynamic, per-vehicle**: `sync.geometryResidual` (scalar per avatar/frame)
  compares the mirrored CARLA actor pose against the requested SUMO pose;
  a summary is logged in `finish()`.

## Notes

* `stepInterval` is written to CARLA's `fixed_delta_seconds`, which only accepts
  values between 1 µs and 1/60 s; SUMO's `step-length` must be identical.
* The manager switches the server into synchronous mode and restores the
  previous settings when the simulation ends.  If an Artery run is killed hard,
  the server may be left waiting for tick cues; the manager notices this and
  nudges the server with a tick cue when it connects next.
* CARLA avatars have physics disabled — SUMO owns the motion; the CARLA actor
  is re-positioned with `SetTransform()` every frame.  `zOffset` must clear the
  vehicle bounding box (~0.6 m) or the initial spawn collides with the road.
