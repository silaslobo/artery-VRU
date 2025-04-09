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
        @signal[DCCToff](type=double);

        @statistic[reception](source=VamReceived;record=vector(vamStationId)?,vector(vamGenerationDeltaTime)?);
        @statistic[transmission](source=VamSent;record=vector(vamStationId)?,vector(vamGenerationDeltaTime)?);
        @statistic[VAMlatitude](source=VamSent;record=vector(vamStationId)?,vector(vamLatitude)?);
        @statistic[VAMlongitude](source=VamSent;record=vector(vamStationId)?,vector(vamLongitude)?);
    
        @statistic[VamSentCounter](record=count,vector?);
        @statistic[VamReceivedCounter](record=count,vector?);

        @statistic[DCCToff](source=DCCToff;record=count,vector?);

        @signal[VamTriggerPosition];
        @signal[VamTriggerHeading];
        @signal[VamTriggerSpeed];

        @statistic[PositionTrigger](source=VamTriggerPosition;record=count,vector?);
        @statistic[HeadingTrigger](source=VamTriggerHeading;record=count,vector?);
        @statistic[SpeedTrigger](source=VamTriggerSpeed;record=count,vector?);
        
        @signal[VruStationID];
        @statistic[VruStationID](source=VruStationID;record=vector);

        @signal[vamDP2];
        @signal[vamDP3];

        @statistic[vamDP2](source=vamDP2;record=count,vector?);
        @statistic[vamDP3](source=vamDP3;record=count,vector?);


        @signal[VamClusterCreation];
        @signal[VamCluster];
        @signal[VamJoin];
        @signal[VamBreak];
        @signal[VamLeave];
        @signal[VamTime];
        @signal[VamReceivedJoinId];
        @signal[VamReceivedLeaveId];
        @signal[ClusterMember];

        @statistic[ClusterCreationCounter](source=VamClusterCreation;record=count,vector?);
        @statistic[ClusterCounter](source=VamCluster;record=count,vector?);
        @statistic[VamJoinCounter](source=VamJoin;record=count,vector?);
        @statistic[VamBreakCounter](source=VamBreak;record=count,vector?);
        @statistic[VamLeaveCounter](source=VamLeave;record=count,vector?);
        @statistic[VamTimerCounter](source=VamTime;record=count,vector?);

        @statistic[VamReceivedJoinID](source=VamReceivedJoinId;record=count,vector?);
        @statistic[VamReceivedLeaveID](source=VamReceivedLeaveId;record=count,vector?);
        @statistic[VamClusterMember](source=ClusterMember;record=count,vector?);

        @signal[LeaveReasonCounter];
        @signal[BreakupReasonCounter];
        @signal[Location];

        @statistic[LeaveReasonCounter](source=LeaveReasonCounter;record=count,vector?);
        @statistic[BreakupReasonCounter](source=BreakupReasonCounter;record=count,vector?);
        

        @signal[MessageSize];
        @statistic[MessageSize](source=MessageSize;record=count,vector?);

        // Geometric evaluations
        @signal[BoundingBoxArea](type=double);
        @signal[BoundingBoxPerimeter](type=double);
        @signal[LargestDistToLeader](type=double);
        @signal[BoundingBoxRadius](type=double);

        @statistic[BoundingBoxArea](source=BoundingBoxArea;record=vector);
        @statistic[BoundingBoxPerimeter](source=BoundingBoxPerimeter;record=vector);
        @statistic[LargestDistToLeader](source=LargestDistToLeader;record=vector);
        @statistic[BoundingBoxRadius](source=BoundingBoxRadius;record=vector);

        // Cluster Parameter evaluations
        @signal[CardinalitySize](type=unsigned long);
        @statistic[CardinalitySize](source=CardinalitySize;record=vector);

        // Parameters received from leader's CIC
        @signal[DistFromInfoContainer];
        @signal[CardFromInfoContainer];

        @statistic[DistFromInfoContainer](source=DistFromInfoContainer; record=vector);
        @statistic[CardFromInfoContainer](source=CardFromInfoContainer; record=vector);
        
        // evaluate DCC transmission interval restrictions
        bool withDccRestriction = default(true);

        // cluster Information
        bool withClusterSupport = default(false);

        // generation interval boundaries
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(5.0s);

        // generate at fixed rate (using minInterval, optionally restricted by DCC)
        bool fixedRate = default(false);

        // change in orientation triggering VAM generation (in degree)
        double headingDelta = default(4.0);

        // change in position triggering VAM generation
        double positionDelta @unit(m) = default(4.0m);

        // change in speed triggering VAM generation (in meter/second)
        double speedDelta @unit(mps) = default(0.5mps);

        // length of path history
        volatile int pathHistoryLength = default(23);
        
        bool ignorePersons = false;

        // CLUSTERING PARAMETERS: based on ETSI TS 103 300-3 V2.2.1 (2023-02) Section 8

        // Parameters for VRU clustering decisions 
        int numCreateCluster = default(3);
        double maxClusterDistance @unit(m) = default(5.0m);
        double maxClusterVelocityDifference = default(0.05);
        double maxCombinedClusterDistance @unit(m) = default(2.0m);
        int minClusterSize = default(1);
        int maxClusterSize = default(20);
        int numClusterVAMRepeat = default(3);

        // Clustering behavior on the simulation 
        string defaultBoundingBoxShape = default("Polygon"); // Available options: Circular, Rectangular, Polygon or Autoselect
        bool enableBoundingBoxPadding = default(false);
        
        // Cluster membership parameters 
        double timeClusterUniquenessThreshold  = default(30.0);
        double timeClusterBreakupWarning  = default(3.0);
        double timeClusterJoinNotification  = default(3.0);
        double timeClusterJoinSuccess = default(0.5);
        double timeClusterIdChangeNotification  = default(3.0);
        double timeClusterIdPersist  = default(3.0);
        double timeClusterContinuity  = default(2.0);
        double timeClusterLeaderWaiting  = default(2.0);
        double timeClusterLeaveNotification  = default(1.0);
        double timeCombinedVruClusterOpportunity  = default(3.0);

        // Custom max interval between cluster VAMs due to timeClusterContinuity parameter
        double clusterMaxInterval = default(2.0);
        
}
