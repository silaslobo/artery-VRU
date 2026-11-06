package carla;

//
// Minimal host module instantiated by traci::BasicModuleMapper for each SUMO
// vehicle (mapper.vehicleType = "carla.CarlaVehicleNode",
// nodes.vehicleSinkModule = ".carlaAvatar").  It carries nothing but the
// CARLA avatar; add middleware/nic modules as needed.
//
module CarlaVehicleNode
{
    submodules:
        carlaAvatar: CarlaAvatar {
        }
}
