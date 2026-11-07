package carla;

//
// Kinematic CARLA mirror of a SUMO vehicle.  Used as the vehicleSinkModule of
// a traci node manager: it receives the SUMO position updates and queues the
// corresponding CARLA actor transforms at the CarlaScenarioManager, which
// applies them right before every World::Tick().
//
simple CarlaAvatar
{
    parameters:
        @class(artery::carla::CarlaAvatar);
        string managerModule @desc("Name of the CarlaScenarioManager module (searched in all ancestors) or an OMNeT++ module path") = default("carlaManager");
        string vehicleBlueprint = default("vehicle.tesla.model3");
        double zOffset @desc("Height above the SUMO plane for the mirrored actor. The road surface of a generated OpenDRIVE map is z=0; a hair of clearance keeps TrySpawnActor's overlap check happy") = default(0.02);
        // semicolon separated list, same grammar as CarlaScenarioManager.sensors
        string sensors @desc("Sensors attached to this vehicle's CARLA avatar. Empty for none") = default("");
}
