package carla;


//
// Minimal co-simulation world: a single CARLA ego vehicle with GNSS, IMU and
// collision sensors, whose measurements are forwarded to a sink module through
// the "carla.sensor" signal.  No road traffic infrastructure required, so it
// runs against a plain CARLA server.
//
network CarlaWorld
{
    parameters:
        string carlaHost = default("localhost");
        int carlaPort = default(2000);

    submodules:
        carlaManager: CarlaScenarioManager {
            parameters:
                host = default(carlaHost);
                port = default(carlaPort);
        }

        sink: CarlaSensorSink {
            parameters:
                managerModule = "carlaManager";
        }
}
