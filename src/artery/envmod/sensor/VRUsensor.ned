package artery.envmod.sensor;

simple VRUsensor
{
    parameters:
        string identityRegistryModule;
        double validityPeriod @unit(s) = default(0.55s);
}