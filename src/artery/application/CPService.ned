//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package artery.application;

simple CPService like ItsG5Service
{
    parameters:
        @signal[CpmReceived](type=CPObject);
        @signal[CpmSent](type=CPObject);
        @signal[ratioFilter](type=double);
        @signal[delayDCC](type=simtime_t);
        @signal[distCovered](type=double);
        @signal[headingVariation](type=double);
        @signal[SpeedVariation](type=double);
        @signal[limericDelta](type=double);
        @signal[RatioRessourceUsed](type=double);
        @signal[Tonpp](type=long);
        @signal[objectAge](type=simtime_t);
        @signal[deltaPositionObject](type=double);
        @signal[removedObjExcessiveSize](type=long);
        @signal[resourceNotUsed](type=double);
        @signal[VRUid](type=long);
        @signal[VRUIdTime](type=simtime_t);

        @statistic[reception](title=CpmReceived; source=CpmReceived; record=vector(cpmStationId)?,vector(cpmGenerationDeltaTime)?);
        @statistic[transmission](source=CpmSent;record=vector(cpmStationId)?,vector(cpmGenerationDeltaTime)?);
        @statistic[cpmSizeByte](source=sizeCPM(CpmSent);record=stats?);
        @statistic[cpmSent](source=CpmSent;record = stats(nbObjects)?);
        @statistic[objSent](source=CpmSent; record = vector(objIdSent)?, count(objIdSent)?);
        @statistic[ratioFilter](source=ratioFilter; record = stats?);
        @statistic[delayDcc](source=delayDCC; record = stats?);
        @statistic[resourceNotUsed](source=resourceNotUsed; record = stats?);

        @statistic[VRUdetection](source=VRUid; record=vector?);

        @statistic[headingCPM](title=headingObject; source=CpmReceived; record=vector(headingObject)?);
        @statistic[speedCPM](title=speedObject; source=CpmReceived; record=vector(speedObject)?);
        @statistic[distXObject](title=distXObject; source=CpmReceived; record=vector(distXObject)?);
        @statistic[distYObject](title=distYObject; source=CpmReceived; record=vector(distYObject)?);
        @statistic[timeMeasurement](title=timeMeasurement; source=CpmReceived; record=vector(objTimeSent)?);


        @statistic[distCovered](source=distCovered; record = stats?);
        @statistic[headingVariation](source=headingVariation; record = stats?);
        @statistic[SpeedVariation](source=SpeedVariation; record = stats?);
        @statistic[limericDelta](source=limericDelta; record = stats?);
        @statistic[RatioRessourceUsed](source=RatioRessourceUsed; record = stats?);
        @statistic[Tonpp](source=Tonpp; record = stats?);
        @statistic[objectAge](source=objectAge; record = stats?);
        @statistic[deltaPositionObject](source=deltaPositionObject; record = stats?);
        @statistic[removedObjExcessiveSize](source=removedObjExcessiveSize; record = count?);


        // evaluate DCC transmission interval restrictions
        bool withDccRestriction = default(true);

        // generation interval boundaries
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(1.0s);

        // generation rules
        string triggeringCondition = default("PeriodicIfObjToSend");

        // dcc
        string dccAlgorithm = default("LimericDccEntity");

        // generate at fixed rate (using minInterval, optionally restricted by DCC)
        bool fixedRate = default(false);

        // change in orientation triggering CPM generation (in degree)
        double headingDelta = default(4.0);

        // change in position triggering CPM generation
        double positionDelta @unit(m) = default(4.0m);

        // change in speed triggering CPM generation (in meter/second)
        double speedDelta @unit(mps) = default(0.5mps);

        //Filter
        bool v2xCapabilities = default(false);
        bool objectDynamicsLocal = default(false);
        bool objectDynamicsV2X = default(false);
        bool fovSensors = default(false);
        bool perceptionQuality = default(false);
        bool updatingTime = default(false);
        bool etsiFilter = default(false);

        bool otherDynamicThresh = default(false);
        bool useMatchingSize = default(false);
        bool useThreshMatchingSize = default(false);
        double threshMatchingSize = default(0.1);

        double startFilteringThresh = default(0.1);
        bool generateForSensor = default(true);
        bool preFilterLowDynamics = default(false);
}
