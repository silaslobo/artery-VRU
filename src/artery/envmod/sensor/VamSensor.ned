package artery.envmod.sensor;

simple VamSensor
{
    parameters:
        string identityRegistryModule;
        double validityPeriod @unit(s) = default(1.1s);
}
