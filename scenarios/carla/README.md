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
                     │              │    (+ .lanepoints sidecar)
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
[INFO]  Map geometry check: 257 lane samples, max deviation 0.0004 m, ...
[INFO]  TraCI server is SUMO 1.28.0 with API level 22
** Event #3  t=0.01  CarlaSumoWorld.traci.core
** Event #4  t=0.01  CarlaSumoWorld.carlaManager       <- alternating: lock-step
...
[INFO]  Spawned CARLA avatar 'v_e10_to_e13_0' (actor 87) from vehicle.tesla.model3 at (-1.75,-13.4,90)
[INFO]  Spawned CARLA avatar 'v_e12_to_e17_0' (actor 88) from vehicle.tesla.model3 at (13.4,-1.75,180)
[INFO]  sensorSink <- carlaManager: image(rgb) frame=3077 t=3.171 sim=0.290 pos=(0.00,-38.00,30.00) 1280x720 fov=90.0
[INFO]  Captured captures/290ms_rgb_f3077.png (1280x720, sim t=0.29s)
...
[INFO]  avatar pose residual: n=140 max=0.14 m mean=0.013 m
[INFO]  received 147 image measurements
[INFO]  wrote 147 image frames to captures
```

The `sim=` value and the file-name prefix are the *simulation* instants the
measurements belong to, derived from the CARLA frame numbers (never from
wall-clock or float timestamps).  Measurements can arrive while later frames
are already being ticked (WAN latency); the manager back-dates them by the
frame distance, so every image ends up named with its true simulation
instant.

## Verification built into the run

* **Static, map level**: 257 lane-centre samples from the generator are
  projected onto the map CARLA actually built (`mapCheckFile`); the run asserts
  the deviation is in the sub-millimetre range and detects CARLA's coordinate
  convention empirically.
* **Identity**: the OpenDRIVE string is checksummed before/after upload
  (`GetOpenDrive()` must return exactly what was sent).
* **Dynamic, per-vehicle**: `sync.geometryResidual` (scalar per avatar/frame)
  compares the mirrored CARLA actor pose against the requested SUMO pose;
  a summary is logged in `finish()`.

## Camera captures (`captures/`)

`omnetpp-sumo.ini` enables two 1280x720 RGB cameras:

* one fixed observation camera (manager-level `sensors`, absolute CARLA world
  coordinates): high oblique view from south
  of the junction (`0,-38,30`, pitch -55, yaw 90) covering the whole 18 m
  arms;
* one front-windshield camera on the **second** vehicle only
  (`*.node[1].carlaAvatar.sensors`, vehicle-local offset `1.8,0,1.7`; nodes
  are created in departure order).

Both cameras deliver **one image per world tick** and `step()` blocks each
tick until the frame's complete sensor set arrived (or `sensorWaitTimeout =
2s` expired) — the synchronous mode a CV consumer needs; over a WAN every
tick therefore costs up to the image transfer time, which is fine for
bring-up and disappears when the stack runs server-side.  `sensor_tick` is
deliberately unsupported (see `src/carla/README.md` §6 "Data rate control"):
throttle by dropping arrivals on the consumer side, never by asking CARLA
for less.  Resolution is set through the blueprint attributes
`image_size_x`/`image_size_y` (the correct names; others are skipped with a
warning) and the server should run with `-quality-level=Epic`.  The readings
are stored as PNGs by
`carla.CarlaImageSink` (`imageSink` in `CarlaSumoWorld.ned`), which subscribes
to the same `carla.sensor` signal; it requires
`*.carlaManager.copySensorPayload = true` so the readings carry pixel bytes.

To re-capture: just re-run the scenario — every run overwrites nothing, files
are named `<simTime>ms_<sensor>_f<carlaFrame>.png`:

```bash
rm -f captures/*.png                     # optional: start clean
cmake --build build --target run_carla_sumo_scenario
ls captures/
```

Tuning: change positions in the two `sensor.camera.rgb` entries of
`omnetpp-sumo.ini` (grammar `blueprint[@x,y,z,pitch,yaw,roll][!attr=val]`).
Images stream on every tick — one 1280x720 BGRA frame ≈ 3.7 MB per camera and
tick; `CarlaImageSink` also takes `maxFrames` to cap the number of written
files.  Captures are gitignored.

For a quick structural check without an image viewer,
`tools/carla_map/frame_report.py` (stdlib only) classifies pixels and locates
coloured object blobs:

```bash
python3 tools/carla_map/frame_report.py --ascii captures/*.png
```

## Notes

* `stepInterval` is written to CARLA's `fixed_delta_seconds`, which only accepts
  values between 1 µs and 1/60 s; SUMO's `step-length` must be identical.
* The manager switches the server into synchronous (lock-step) mode and keeps
  it there — synchronous is the only supported mode, and the previous settings
  are deliberately *not* restored at the end.  If an Artery run is killed hard,
  the server is left waiting for tick cues either way; the manager notices this
  and nudges the server with a tick cue when it connects next.
* CARLA avatars have physics disabled — SUMO owns the motion; the CARLA actor
  is re-positioned with `SetTransform()` every frame.  `zOffset = 0.02`: the
  road surface of a generated map is z=0 and spawning *exactly* on it makes
  `TrySpawnActor`'s overlap check reject the pose (verified: the avatar then
  silently never appears), while a few centimetres of clearance pass — and
  2 cm is far below what a camera can show as floating (0.6 m very much was).
  A rejected spawn aborts the run instead of retrying: SUMO has the vehicle,
  so a blocked pose is a bug in the scenario or in this module.

## Known issues

* **CARLA 0.9.16 server: the second synchronous-mode client session dies ~60-77
  ticks in.** The client times out in `Tick()`, and afterwards the server
  answers no RPC at all (fresh connects time out too) without ever logging a
  crash.  Ruled out: image / bandwidth load (reproduces with GNSS-only
  sensors), world regeneration (reproduces with the identical OpenDRIVE world
  reused), spawn geometry, avatar physics, and the traffic manager.  On the TM
  specifically: it is a *client-side* library object (`TrafficManagerLocal`)
  that no stock server runs as a service (port 8000 is closed by default), and
  this integration uses no TM at all — see the `SetAutopilot` comment in
  `updateAvatar`.  So there is no TM to put in synchronous mode; the doc note
  about synchronising the TM does not apply here.

  Only the first run after a server restart completes reliably; restart the
  server between runs until it is replaced (a newer CARLA on Unreal Engine 5
  is the planned fix).  The client behaves correctly: a timed-out tick aborts
  the run cleanly rather than re-cueing (which would double-tick the server and
  desynchronise the RPC protocol).
