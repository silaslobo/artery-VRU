package artery.networking;

simple LimericDccEntity like IDccEntity
{
    parameters:
        @class(LimericDccEntity);
        string radioDriverModule;
        string routerModule;
        string runtimeModule;

        string ChannelProbeProcessor = default("Local");
        string NetworkEntity = default("");

        double targetCbr = default(0.68);
        int queueLength = default(2);
        bool enableDualAlpha = default(false);

        @signal[packetsTransmitted](type=unsigned long);
        @signal[packetsTransmittedDP2](type=unsigned long);           
        @signal[packetsTransmittedDP3](type=unsigned long);

        @signal[packetsDropped](type=unsigned long);
        @signal[packetsDroppedDP2](type=unsigned long);    
        @signal[packetsDroppedDP3](type=unsigned long);

        @statistic[packetsTransmitted](source=packetsTransmitted; record=count,vector?);
        @statistic[packetsTransmittedDP2](source=packetsTransmittedDP2; record=count,vector?);
        @statistic[packetsTransmittedDP3](source=packetsTransmittedDP3; record=count,vector?);

        @statistic[packetsDropped](source=packetsDropped; record=count,vector?);
        @statistic[packetsDroppedDP2](source=packetsDroppedDP2; record=count,vector?);           
        @statistic[packetsDroppedDP3](source=packetsDroppedDP3; record=count,vector?);

    gates:
        output radioDriverData;
}
