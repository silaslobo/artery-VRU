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
cmake -S . -B build -DCMAKE_BUILD_TYPE=RELEASE && cmake --build build -j$(nproc)

cmake --build build --target run_carla_sumo_scenario
```

The run target launches OMNeT++ with its default environment, **Qtenv**
(the Qt GUI).  On a headless machine — CI, a container, or when an agent
runs the scenario — select **Cmdenv** instead; the environment is chosen
by the launcher, not by the ini file, so pass it at configure time:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RELEASE -DRUN_FLAGS=-uCmdenv
```

(or invoke the generated `opp_run` command line manually with `-uCmdenv`).
Everything the run is verified by — the pose-residual summary, the frame
count, `End.` — is console output and appears identically in both
environments.

Point `*.carlaHost` / `*.carlaPort` in `omnetpp-sumo.ini` at your CARLA server;
it starts SUMO itself via `PosixLauncher`, so only the CARLA server has to run
externally.  The server is CARLA 1.0 (Unreal Engine 5) and is started there
with

```bash
CUDA_VISIBLE_DEVICES=<GPU index> ./CarlaUnreal.sh -RenderOffScreen -nosound -graphicsadapter=<GPU index>
# the gpu indexes from the environment variable and the flag may not be the same
# CUDA_VISIBLE_DEVICES limits the devices visible to CUDA
# while -graphicsadapter the device used by vulkan
```

Generating the world from `intersection.xodr` runs UE5 meshing, PCG and
navmesh build on the server; measured, it completes in seconds (a full
re-generation round trip is ~3 s, the first generation after a server start
at most a minute).  `carlaManager.timeout = 60s` covers that comfortably —
if generation ever runs into the timeout, suspect a wedged server (see
Known issues), not a slow build.  Every run of the scenario regenerates the
same world.

To watch SUMO with the GUI (requires an X display), select the GUI run
configuration, e.g. append `-c Config-sumo-gui` to the generated `opp_run`
command line.

## Expected output

```
[INFO]  Connecting to CARLA server big-boss.tail4b138a.ts.net:2000
[INFO]  Connected to CARLA 1.0-9ec914a6f, world 0s step
[INFO]  Generating CARLA world from OpenDRIVE 'maps/intersection.xodr' (29223 bytes, checksum ...): vertex_distance=0.2 additional_width=0 simplification=0 smooth_junctions=0 wall_height=0
[INFO]  Loaded CARLA map name='Carla/Maps/OpenDriveMap' geoReference lat=55.6761 lon=12.5683 alt=0
[INFO]  OpenDRIVE round trip: sent 29223 bytes (checksum ...), server reports 29223 bytes (checksum ...)
[INFO]  Server stores exactly the OpenDRIVE string that was sent
[INFO]  Map geometry check: CARLA coordinate convention x/y identical (dev plain=0 flipped=0)
[INFO]  Map geometry check: 256 lane samples, max deviation 0.00057 m, mean 8.7e-05 m, ...
[INFO]  CARLA server settings: synchronous=1 fixed_delta=0.01 no_rendering=0
[INFO]  TraCI server is SUMO 1.28.0 with API level 22
** Event #3  t=0.01  CarlaSumoWorld.traci.core
** Event #4  t=0.01  CarlaSumoWorld.carlaManager       <- alternating: lock-step
...
[INFO]  Spawned CARLA avatar 'v_e10_to_e13_0' (actor 35) from vehicle.tesla.model3 at (-1.75,-13.4,90)
[INFO]  Spawned CARLA avatar 'v_e12_to_e17_0' (actor 36) from vehicle.tesla.model3 at (13.4,-1.75,180)
[INFO]  sensorSink <- carlaManager: image(v_e12_to_e17_0.rgb) frame=2395124 t=5.020 sim=2.720 pos=(3.34,-1.75,1.72) 1280x720 fov=90.0
[INFO]  Captured captures/2720ms_v_e12_to_e17_0_rgb_f2395124.png (1280x720, sim t=2.72s)
...
[INFO]  avatar pose residual: n=600 max=0.068 m mean=0.002 m
[INFO]  received 300 image measurements
[INFO]  wrote 300 image frames to captures
End.
```

Both vehicles depart at t=0 (`gen_routes.py`), so the residual covers every
frame of all 300 ticks (n=600 = 2 avatars x 300 frames) and the single
vehicle camera delivers exactly one image per tick (300 of them).

Note: on CARLA 1.0 (UE5) the map keeps the OpenDRIVE coordinate frame — the
check reports *x/y identical*; on 0.9.16 (UE4) it reported
`x_carla = x_xodr, y_carla = -y_xodr`.  (If the client ever reports
`1.0-<sha>-dirty` against a `1.0-<sha>` server, the CARLA submodule working
tree has local modifications; the stock build keeps it pristine and the
version-mismatch warning does not appear.)

The `sim=` value and the file-name prefix are the *simulation* instants the
measurements belong to, derived from the CARLA frame numbers (never from
wall-clock or float timestamps).  Measurements can arrive while later frames
are already being ticked (WAN latency); the manager back-dates them by the
frame distance, so every image ends up named with its true simulation
instant.

## Verification built into the run

* **Static, map level**: lane-centre samples from the generator are
  projected onto the map CARLA actually built (`mapCheckFile`); the run asserts
  the deviation is in the sub-millimetre range and detects CARLA's coordinate
  convention empirically.
* **Identity**: the OpenDRIVE string is checksummed before/after upload
  (`GetOpenDrive()` must return exactly what was sent).
* **Dynamic, per-vehicle**: `sync.geometryResidual` (scalar per avatar/frame)
  compares the mirrored CARLA actor pose against the requested SUMO pose;
  a summary is logged in `finish()`.

## Camera captures (`captures/`)

`omnetpp-sumo.ini` enables one 1280x720 RGB camera:

* one front-windshield camera on the **second** vehicle only
  (`*.node[1].carlaAvatar.sensors`, vehicle-local offset `1.8,0,1.7`; nodes
  are created in departure order, both vehicles depart at t=0).

There are deliberately no manager-level (static/overhead) cameras: every
sensor is vehicle-mounted, mirroring the deployment the integration targets.

The camera delivers **one image per world tick** and `step()` blocks each
tick until the frame's complete sensor set arrived (or `sensorWaitTimeout =
2s` expired) — the synchronous mode a CV consumer needs; over a WAN every
tick therefore costs up to the image transfer time, which is fine for
bring-up and disappears when the stack runs server-side.  `sensor_tick` is
deliberately unsupported (see `src/carla/README.md` §6 "Data rate control"):
throttle by dropping arrivals on the consumer side, never by asking CARLA
for less.  Resolution is set through the blueprint attributes
`image_size_x`/`image_size_y` (the correct names; others are skipped with a
warning); the stock CARLA 1.0 server render settings (as launched per "Run"
above) are sufficient.  The readings
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

Tuning: change the `sensor.camera.rgb` entries of
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

* **The CARLA 0.9.16 stall is fixed on CARLA 1.0 (UE5).**  On 0.9.16 the
  second synchronous-mode client session reliably stopped answering every RPC
  ~60-77 ticks in (game thread stuck in the `do { RunSome(1u); } while
  (!TickCueReceived());` spin — an `io_context::reset()` race in
  `SyncRunFor()`); only a server restart helped, so runs had to be
  serialised behind restarts.  On CARLA 1.0 the acceptance scenario and
  repeated sync-mode probe sessions (teleported avatars every frame, world
  regeneration included) run back-to-back against the same server instance
  without a single timed-out tick (5 sessions, 3000 teleported frames).

* **`check-signals` (debug OMNeT++ builds)**: `@signal[carla.sensor]` carries
  no `type=` attribute on purpose.  `type=object` is *invalid* in OMNeT++ 5.7
  ("object" is not a registered class) and throws on the first emit whenever
  signal checking is enabled — which is the default in debug builds
  (`check-signals`).  Build with `CMAKE_BUILD_TYPE=RELEASE` (the usual artery
  configuration); the scenario also runs in debug since the declaration fix.

* **The UE5 server still occasionally dies in the render thread**
  (`LowLevelFatalError [RenderingThread.cpp] GameThread timed out waiting
  for RenderThread after 60.00 secs`, then segfault).  It happens far less
  on CARLA 1.0 than our earlier sessions suggested, but it is not fully
  root-caused.  During such a hang the async `version` RPC keeps answering
  while every game-thread RPC times out; afterwards the server refuses
  connections.  Nothing on our side triggers it deterministically (the
  teleport/sync regression passes repeatedly) and the client already
  behaves correctly: a timed-out tick aborts instead of re-cueing.
  Host-side suspects are VRAM pressure and GPU preemption — check for
  competing GPU processes before suspecting the stack.  Deferred for now;
  if it becomes frequent, the next diagnostic step is a backtrace of the
  RHIThread *during* the hang (`thread apply all bt 1`).
