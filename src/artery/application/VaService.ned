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


// ============================================================

//	ATTENTION:
//	Currently being modified and tested to enable clustering 

// ============================================================
	


package artery.application;

simple VaService like ItsG5Service
{
    parameters:
        @class(VaService);
        @signal[VamReceived](type=artery::VaObject);
        @signal[VamSent](type=artery::VaObject);
        @signal[VamSentCounter];
        @signal[VamReceivedCounter];

        @statistic[reception](source=VamReceived;record=vector(vamStationId)?,vector(vamGenerationDeltaTime)?);
        @statistic[transmission](source=VamSent;record=vector(vamStationId)?,vector(vamGenerationDeltaTime)?);


        @statistic[VamSentCounter](record=count,vector?);
        @statistic[VamReceivedCounter](record=count,vector?);

        // evaluate DCC transmission interval restrictions
        bool withDccRestriction = default(true);

        // VRU roles
        bool vruIdle = default(false);
        bool vruActiveStandalone = default(true);
        bool vruActiveClusterLeader = default(false);
        bool vruPassive = default(false);

        // cluster Information
        bool clusterContainer = default(false);
        double maxVruDistance @unit(m) = default(5.0m);
        double maxClusterVelocityDifference = default(0.05);
        int numCreateCluster = default(3);
        int minClusterSize = default(1);
        int maxClusterSize = default(20);
        string boundingBoxShape = default("Polygon");
        
        // cluster timings
        double timeClusterUniquenessThreshold @unit(s) = default(30.0s);
        double timeClusterBreakupWarning @unit(s) = default(3.0s);
        double timeClusterJoinNotification @unit(s) = default(3.0s);
        double timeClusterJoinSuccess @unit(s) = default(0.5s);
        double timeClusterIdChangeNotification @unit(s) = default(3.0s);
        double timeClusterIdPersist @unit(s) = default(3.0s);
        double timeClusterContinuity @unit(s) = default(2.0s);
        double timeClusterLeaveNotification @unit(s) = default(1.0s);
        double timeCombinedVruClusterOpportunity @unit(s) = default(15.0s);
        

        // generation interval boundaries
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(5.0s);

        // generate at fixed rate (using minInterval, optionally restricted by DCC)
        bool fixedRate = default(false);

        // change in orientation triggering VAM generation (in degree)
        double headingDelta = default(0.2);

        // change in position triggering VAM generation
        double positionDelta @unit(m) = default(4.0m);

        // change in speed triggering VAM generation (in meter/second)
        double speedDelta @unit(mps) = default(0.5mps);

        // length of path history
        volatile int pathHistoryLength = default(23);
        
        bool ignorePersons = false;
}
