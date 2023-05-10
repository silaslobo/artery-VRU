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

simple ClusterManagement 
{
    parameters:
        @class(ClusterManagement);
        // based on ETSI TS 103 300-3 V2.2.1 (2023-02) Section 8

        // Parameters for VRU clustering decisions 
        int numCreateCluster = default(3);
        double maxClusterDistance @unit(m) = default(5.0m);
        double maxClusterVelocityDifference = default(0.05);
        double maxCombinedClusterDistance @unit(m) = default(2.0m);
        int minClusterSize = default(1);
        int maxClusterSize = default(20);
        int numClusterVAMRepeat = default(3);
        
        // Cluster membership parameters 
        double timeClusterUniquenessThreshold @unit(s) = default(30.0s);
        double timeClusterBreakupWarning @unit(s) = default(3.0s);
        double timeClusterJoinNotification @unit(s) = default(3.0s);
        double timeClusterJoinSuccess @unit(s) = default(0.5s);
        double timeClusterIdChangeNotification @unit(s) = default(3.0s);
        double timeClusterIdPersist @unit(s) = default(3.0s);
        double timeClusterContinuity @unit(s) = default(2.0s);
        double timeClusterLeaderWaiting @unit(s) = default(2.0s);
        double timeClusterLeaveNotification @unit(s) = default(1.0s);
        double timeCombinedVruClusterOpportunity @unit(s) = default(15.0s);

        // Clustering behavior on the simulation
        bool clusterContainer = default(false);
        string boundingBoxShape = default("Polygon");
}