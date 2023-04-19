#include "artery/application/ClusterOperations.h"
#include "artery/application/VaObject.h"
#include "artery/application/VaService.h"
#include "artery/application/Constants.hpp"
#include <vanetza/asn1/vam.hpp>

namespace artery
{

using namespace omnetpp;

void ClusterOperations::joinNotification(vanetza::asn1::Vam& message, ClusterId_t& targetClusterId, omnetpp::SimTime timeJoinNotification){
    VruClusterOperationContainer_t* vcoc = buildGenericOperationContainer(message);
	vcoc->clusterJoinInfo->clusterId = targetClusterId;
	vcoc->clusterJoinInfo->joinTime = timeJoinNotification.dbl();
}

void ClusterOperations::leaveNotification(vanetza::asn1::Vam& message, ClusterId_t& clusterId, omnetpp::SimTime timeLeaveNotification){
    VruClusterOperationContainer_t* vcoc = buildGenericOperationContainer(message);
	vcoc->clusterLeaveInfo->clusterId = clusterId;
	vcoc->clusterLeaveInfo->clusterLeaveReason = 1;
}

void ClusterOperations::breakupWarning(vanetza::asn1::Vam& message, omnetpp::SimTime timeBreakupWarning){
    VruClusterOperationContainer_t* vcoc = buildGenericOperationContainer(message);
	vcoc->clusterBreakupInfo->breakupTime = timeBreakupWarning.dbl();
	vcoc->clusterBreakupInfo->clusterBreakupReason = 1;
}


VruClusterOperationContainer_t* ClusterOperations::buildGenericOperationContainer(vanetza::asn1::Vam& message){
	
	VruAwareness_t &vam = (*message).vam;
	VruClusterOperationContainer_t* &vcoc = vam.vamParameters.vruClusterOperationContainer;
	return vanetza::asn1::allocate<VruClusterOperationContainer_t>();
}

}// namespace artery