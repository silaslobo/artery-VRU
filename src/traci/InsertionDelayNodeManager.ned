package traci;

module InsertionDelayNodeManager extends ExtensibleNodeManager
{
    parameters:
        volatile double insertionDelay = default(uniform(0s, 1s)) @unit(s);

        numVehiclePolicies = 1;
        vehiclePolicy[0].typename = "InsertionDelayVehiclePolicy";
        vehiclePolicy[0].insertionDelay = insertionDelay;

        numPersonPolicies = 1;
        personPolicy[0].typename = "InsertionDelayPersonPolicy";
        personPolicy[0].insertionDelay = insertionDelay;
}
