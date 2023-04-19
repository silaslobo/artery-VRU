#ifndef ARTERY_CLUSTER_OPERATIONS_H_
#define ARTERY_CLUSTER_OPERATIONS_H_

#include <vanetza/asn1/vam.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>

namespace artery
{
class Timer;
class PersonDataProvider;

class ClusterOperations{
    
    public:
        void joinNotification(vanetza::asn1::Vam&, ClusterId_t&, omnetpp::SimTime);
        void leaveNotification(vanetza::asn1::Vam&, ClusterId_t&, omnetpp::SimTime);
        void breakupWarning(vanetza::asn1::Vam&, omnetpp::SimTime);
    
    private:
        VruClusterOperationContainer_t* buildGenericOperationContainer(vanetza::asn1::Vam& message);

};

}// namespace artery

#endif /* ARTERY_CLUSTER_OPERATIONS_H_ */