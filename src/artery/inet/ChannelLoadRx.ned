package artery.inet;

import inet.linklayer.ieee80211.mac.Rx;

simple ChannelLoadRx extends Rx
{
    parameters:
        @class(ChannelLoadRx);
        @signal[ChannelLoad];
        @statistic[ChannelLoad](record=timeavg,vector?);
        //@statistic[ChannelLoad](source=ChannelLoad, record=vector);

        bool cbrWithTx = default(false);
        bool asyncChannelReport = default(true);
}
