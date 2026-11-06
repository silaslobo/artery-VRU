package carla;

//
// Owns the connection to a CARLA server and drives it in lock-step with the
// OMNeT++ event loop.  Sensor data received from CARLA (on CARLA's streaming
// threads) is marshalled into the OMNeT++ event loop and emitted as
// "carla.sensor" signal, which any other module can subscribe to.
//
// See src/carla/CarlaScenarioManager.h for implementation details.
//
simple CarlaScenarioManager
{
    parameters:
        @class(artery::carla::CarlaScenarioManager);
        @signal[carla.init](type=simtime_t);
        @signal[carla.step](type=simtime_t);
        @signal[carla.close](type=simtime_t);
        @signal[carla.sensor](type=object);
        @signal[sync.geometryResidual](type=double);

        // --- server connection ---
        string host = default("localhost");
        int port = default(2000);
        double timeout @unit(s) @desc("Timeout for all CARLA network operations") = default(20s);

        // --- world / time synchronisation ---
        string map @desc("Map to load, e.g. Town05_Opt. Empty keeps the world currently loaded on the server") = default("");

        // --- generated (custom OpenDRIVE) map ---
        string opendriveFile @desc("Path to an .xodr file. If non-empty the world is generated with GenerateOpenDriveWorld instead of map/GetWorld") = default("");
        string spawnPointsFile @desc("Sidecar with computed spawn poses as written by gen_xodr.py. Empty appends .spawn to opendriveFile") = default("");
        string mapCheckFile @desc("Lane centre line samples (.lanepoints sidecar). If non-empty, the CARLA lane geometry is compared against them after generating the map") = default("");
        double vertexDistance @desc("OpenDRIVE generation: vertex distance of the generated mesh") = default(0.2);
        double maxRoadLength @desc("OpenDRIVE generation: maximum length of road segments") = default(50);
        double wallHeight @desc("OpenDRIVE generation: height of the walls added at the map borders. 0 for geometry-accurate maps") = default(0);
        double additionalWidth @desc("OpenDRIVE generation: extra width added to lanes. 0 keeps the SUMO lanes and the CARLA mesh identical") = default(0);
        double vertexWidthResolution @desc("OpenDRIVE generation: vertex distance of lane borders") = default(4);
        double simplificationPercentage @desc("OpenDRIVE generation: road mesh simplification percentage. 0 for geometry-accurate maps") = default(0);
        bool smoothJunctions @desc("Smooth intersection corners. False keeps the junction geometry as specified") = default(false);
        bool enablePedestrianNavigation = default(true);

        // --- avatars (kinematic mirrors of SUMO vehicles) ---
        bool avatarResidualCheck @desc("After every frame compare the CARLA actor pose with the requested pose and emit sync.geometryResidual") = default(true);
        bool avatarSimulatePhysics @desc("Keep physics enabled for avatar vehicles. Off makes them purely kinematic mirrors") = default(false);
        bool synchronousMode @desc("Switch the CARLA server into synchronous (lock-step) mode") = default(true);
        bool noRenderingMode @desc("Disable rendering in synchronous mode (faster headless co-simulation)") = default(false);
        double stepInterval @unit(s) @desc("OMNeT++ step size, used as CARLA fixed_delta_seconds. CARLA accepts values between 1e-6 s and 1/60 s") = default(0.01s);
        double tickTimeout @unit(s) @desc("Maximum time to wait for the CARLA server to complete one frame") = default(10s);
        int tickRetries @desc("Number of tick cues sent per frame before giving up") = default(2);
        double sensorWaitTimeout @unit(s) @desc("Maximum time to wait for the sensor data belonging to a frame") = default(0.02s);
        double startTime @unit(s) = default(0s);

        // --- ego vehicle ---
        bool spawnEgoVehicle = default(true);
        string vehicleBlueprint = default("vehicle.tesla.model3");
        string spawnTransform @desc("x,y,z plus optional pitch,yaw,roll in CARLA coordinates. Empty picks a random recommended spawn point (or spawnFrom if set)") = default("");
        string spawnFrom @desc("roadId,laneId,s - spawn pose computed from the generator geometry via the sidecar file. Required for generated maps which have no recommended spawn points") = default("");
        bool autopilot = default(true);

        // --- sensors ---
        // semicolon separated list of sensors, each entry:
        //   <blueprint>[:<frequency>][@<x>,<y>,<z>[,<pitch>,<yaw>[,<roll>]]][!<attr>=<value>[,<attr>=<value>]*]
        // The optional frequency is written to the sensor_tick blueprint attribute.
        // relative offsets are applied to the ego vehicle, if one was spawned.
        string sensors = default("sensor.other.gnss:10;sensor.other.imu:100;sensor.other.collision");
        bool copySensorPayload @desc("Copy raw point cloud / image bytes into the signal payload") = default(false);
        bool destroyActorsOnFinish = default(true);
}
