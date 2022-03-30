package traci;

module ExtensibleNodeManager extends BasicNodeManager
{
    parameters:
        @class(traci::ExtensibleNodeManager);
        int numVehiclePolicies = default(0);
        int numPersonPolicies = default(0);

    submodules:
        vehiclePolicy[numVehiclePolicies]: <> like VehiclePolicy {
        }
        personPolicy[numPersonPolicies]: <> like PersonPolicy {
        }
}
