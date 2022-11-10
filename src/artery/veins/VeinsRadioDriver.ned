package artery.veins;

import artery.nic.IRadioDriver;

simple RadioDriver like IRadioDriver
{
	parameters:
		@class(VeinsRadioDriver);
		@signal[ChannelLoad](type=double);
		double channelLoadReportInterval = default(0.1s) @unit(s);

		@statistic[ChannelLoad](source=ChannelLoad; record=stats?);

	gates:
		inout upperLayer;
		input lowerLayerIn;
		output lowerLayerOut;
		output properties;
}
