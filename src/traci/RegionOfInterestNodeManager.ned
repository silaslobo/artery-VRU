package traci;

module RegionOfInterestNodeManager extends ExtensibleNodeManager
{
    parameters:
        xml regionsOfInterest = default(xml("<regions />"));

        numVehiclePolicies = 1;
        numPersonPolicies = 1;
        vehiclePolicy[0].typename = "RegionOfInterestVehiclePolicy";
        vehiclePolicy[0].regionsOfInterest = regionsOfInterest;
        personPolicy[0].typename = "RegionOfInterestPersonPolicy";
        personPolicy[0].regionsOfInterest = regionsOfInterest;
}
