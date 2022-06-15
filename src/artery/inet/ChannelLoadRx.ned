package artery.inet;

import inet.linklayer.ieee80211.mac.Rx;

simple ChannelLoadRx extends Rx
{
    parameters:
        @class(ChannelLoadRx);
        @signal[ChannelLoad](type=double);
        //@statistic[ChannelLoad](record=timeavg,vector?);
        @statistic[ChannelLoad](record=max, vector(max)?);

        bool cbrWithTx = default(false);
        bool asyncChannelReport = default(true);
}
