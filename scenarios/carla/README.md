# CARLA co-simulation scenario

See [`src/carla/README.md`](../../src/carla/README.md) for the architecture of
the CARLA back end (threading, time synchronisation, signal contract).

Minimal example for the CARLA back end of Artery (`src/carla`).  It connects to
a running CARLA server, spawns an ego vehicle equipped with a GNSS, an IMU and a
collision sensor, and advances the CARLA world in lock-step with the OMNeT++
event loop.  Every sensor measurement is marshalled into the OMNeT++ event loop
by `CarlaScenarioManager` and published as `carla.sensor` signal, which
`CarlaSensorSink` (a stand-in for real middleware/services) subscribes to.

## Run

```bash
cmake -S . -B build && cmake --build build --target carla -j$(nproc)

cd scenarios/carla
../../build/../omnetpp-5.7.1/bin/opp_run -u Cmdenv \
    -n ../../src/carla:. \
    -l ../../build/src/carla/libartery_carla.so omnetpp.ini
```

(Or use `cmake --build build --target run_carla_scenario` from the build tree.)

Point `*.carlaHost` / `*.carlaPort` in `omnetpp.ini` at your CARLA server.

## Expected output

```
[INFO]  Connecting to CARLA server big-boss.tail4b138a.ts.net:2000
[INFO]  Connected to CARLA 0.9.16, world 0.01s step
[INFO]  CARLA server settings: synchronous=1 fixed_delta=0.01 no_rendering=0
[INFO]  Spawned ego vehicle 34 (vehicle.tesla.model3) at (45.3829,13.4148,0.6)
[INFO]  Attached sensor 35 (sensor.other.gnss, frequency=100)
...
** Event #3  t=0.02
[INFO]  sink <- carlaManager: gnss(gnss) frame=12761 t=1187.078 pos=(...) lat=... lon=... alt=...
[INFO]  sink <- carlaManager: imu(imu) frame=12762 t=1187.088 accel=(...) gyro=(...) compass=...
...
[INFO]  received 48 gnss measurements
[INFO]  received 49 imu measurements
```

One OMNeT++ step equals exactly one CARLA frame: the CARLA frame counter and the
episode timestamp in the log increase by one frame / `stepInterval` per event.

## Notes

* `stepInterval` is written to CARLA's `fixed_delta_seconds`, which only accepts
  values between 1 µs and 1/60 s.
* The manager switches the server into synchronous mode and restores the
  previous settings when the simulation ends.  If an Artery run is killed hard,
  the server may be left waiting for tick cues; the manager notices this and
  nudges the server with a tick cue when it connects next.
