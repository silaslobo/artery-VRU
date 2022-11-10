package artery.application;

simple EmergencyBrake like ItsG5Service
{
    parameters:
        @class(EmergencyBrake);
        @signal[VamReceived](type=artery::VaObject);
        @signal[CpmReceived](type=CPObject);
        @signal[VRU_ID_VAM];
        @signal[VRU_ID_CPM];
        @signal[VRU_ID_RADAR];

        //@statistic[reception](titel=VamReceived; source=VamReceived;record=vector(vamStationId)?,vector(vamGenerationDeltaTime)?);
        //@statistic[reception](title=CpmReceived; source=CpmReceived; record=vector(cpmStationId)?,vector(cpmGenerationDeltaTime)?);
        @statistic[VRU_ID_VAM](source=VRU_ID_VAM; record=vector?);
        @statistic[VRU_ID_CPM](source=VRU_ID_CPM; record=vector?);
        @statistic[VRU_ID_RADAR](source=VRU_ID_RADAR; record=vector?);
        

}