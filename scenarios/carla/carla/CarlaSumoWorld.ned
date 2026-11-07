package carla;

import traci.Manager;

//
// Combined synchronized world: SUMO is the traffic master (traci.Manager,
// PosixLauncher), CARLA the sensor world.  Every SUMO vehicle gets a
// CarlaVehicleNode whose CarlaAvatar mirrors the SUMO pose onto a kinematic
// CARLA actor right before each World::Tick(), so one SUMO step equals one
// CARLA frame.  SUMO's step-length must equal *.carlaManager.stepInterval
// (and both must be <= 1/60 s for CARLA's fixed_delta_seconds).
//
network CarlaSumoWorld
{
    parameters:
        string carlaHost = default("localhost");
        int carlaPort = default(2000);
        string sumocfg = default("maps/intersection.sumocfg");
        string mapFile = default("maps/intersection.xodr");

    submodules:
        carlaManager: CarlaScenarioManager {
            parameters:
                host = default(carlaHost);
                port = default(carlaPort);
                opendriveFile = default(mapFile);
        }

        traci: Manager {
            parameters:
            nodes.vehicleSinkModule = default(".carlaAvatar");
            nodes.personSinkModule = default("");
            nodes.ignorePersons = default(true);
            mapper.vehicleType = default("carla.CarlaVehicleNode");
            mapper.personType = default("");
            mapper.bicycleType = default("");
            launcher.typename = default("PosixLauncher");
            launcher.sumocfg = default(sumocfg);
        }

        // counts all sensor measurements (manager-level and avatar sensors)
        // forwarded via the carla.sensor signal
        sensorSink: CarlaSensorSink {
            parameters:
                managerModule = "carlaManager";
        }

        // stores image readings (RGB cameras) as PNG files under captures/
        imageSink: CarlaImageSink {
            parameters:
                managerModule = "carlaManager";
                outputDir = default("captures");
        }
}
