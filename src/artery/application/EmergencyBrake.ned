package artery.application;

simple EmergencyBrake like ItsG5Service
{
    parameters:
        @class(EmergencyBrake);
        @signal[VamReceived](type=artery::VaObject);
        @signal[CpmReceived](type=CPObject);

        @statistic[reception](titel=VamReceived; source=VamReceived;record=vector(vamStationId)?,vector(vamGenerationDeltaTime)?);
        @statistic[reception](title=CpmReceived; source=CpmReceived; record=vector(cpmStationId)?,vector(cpmGenerationDeltaTime)?);

        double triggerVRU_TTC @unit(s) = default(1000s);

        double speedDelta @unit(mps) = default(100mps);

        // dcc
        string dccAlgorithm = default("LimericDccEntity");
}