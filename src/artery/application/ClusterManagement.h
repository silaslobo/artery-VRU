#ifndef ARTERY_CLUSTERMANAGEMENT_H_
#define ARTERY_CLUSTERMANAGEMENT_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VaObject.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/vam.hpp>
#include <omnetpp/simtime.h>
#include <omnetpp.h>
#include <string>
#include "artery/application/PersonDataProvider.h"
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/envmod/EnvironmentModelObject.h>

namespace artery{

using namespace omnetpp;

class PersonDataProvider;
class LocalEnvironmentModel;
class ItsG5BaseService;
class ClusterManagement : public cSimpleModule 
{
    public:

        enum VruRoleType {VruIdle, VruActiveStandalone, VruActiveClusterLeader, VruPassive};
        ClusterManagement();
        //ClusterManagement(const PersonDataProvider* pdp, artery::LocalDynamicMap* ldm, const traci::PersonController* pc, VruRoleType initialVruRole = VruActiveStandalone);

        virtual void initialize() override;
        void simObjects(const PersonDataProvider* pdp, artery::LocalDynamicMap* ldm, const traci::PersonController* pc);

        void setCurrentVruRole(VruRoleType newVruRole);
        VruRoleType getCurrentVruRole();
        bool isVruIdle();
        bool isVruStandalone();
        bool isVruClusterLeader();
        bool isVruPassive();

        // Operation conditions check
        bool vruShouldJoin(const vanetza::asn1::Vam&);
        bool vruShouldLeave(const vanetza::asn1::Vam&);
        bool vruShouldCreateCluster();

        // Cluster parameter getters
        std::string boundingBoxShape() const {return mBoundingBoxShape;}
	    unsigned numCreateCluster() const {return mNumCreateCluster;}
	    unsigned minClusterSize() const {return mMinClusterSize;}
	    unsigned maxClusterSize() const {return mMaxClusterSize;}
	    vanetza::units::Length maxVruDistance() const {return mMaxClusterDistance;}
	    double maxClusterVelocityDifference() const {return mMaxClusterVelocityDifference;}
        
        omnetpp::SimTime timeClusterUniquenessThreshold() const {return mTimeClusterUniquenessThreshold;}
        omnetpp::SimTime timeClusterBreakupWarning() const {return mTimeClusterBreakupWarning;}
        omnetpp::SimTime timeClusterJoinNotification() const {return mTimeClusterJoinNotification;} 
        omnetpp::SimTime timeClusterJoinSuccess() const {return mTimeClusterJoinSuccess;}
        omnetpp::SimTime timeClusterIdChangeNotification() const {return mTimeClusterIdChangeNotification;} 
        omnetpp::SimTime timeClusterIdPersist() const {return mTimeClusterIdPersist;}
        omnetpp::SimTime timeClusterContinuity() const {return mTimeClusterContinuity;} 
        omnetpp::SimTime timeClusterLeaveNotification() const {return mTimeClusterLeaveNotification;}
        omnetpp::SimTime timeCombinedVruClusterOpportunity() const {return mTimeCombinedVruClusterOpportunity;}  

        void updateVruClusterMap();
        std::map<LocalDynamicMap::StationID, artery::VaObject> getVruClusterMap() const {return mVruClusterMap;}

        //long generateRandomClusterID() {return omnetpp::intuniform(getRNG(0), 0, 255);}


    private:
        VruRoleType currentVruRole;

        // Geometry
        typedef boost::geometry::model::point <long,2,boost::geometry::cs::cartesian>pointXY;
        typedef boost::geometry::model::polygon<pointXY, false> Polygon_t; 
		typedef boost::geometry::model::multi_polygon<Polygon_t> Circle_t;
        artery::ClusterManagement::Polygon_t reconstructPolygon(const PolyPointList_t& receivedOffsetList, libsumo::TraCIPosition& leaderPos);
		artery::ClusterManagement::Circle_t buildCircle(const double radius, pointXY& leaderPos);
        std::vector<std::tuple<long,long>> clusterPoly(const std::map<LocalDynamicMap::StationID, artery::VaObject>);

        // Parameter validation
        double velocityDifference(SpeedValue_t, SpeedValue_t);
        bool checkClusteringConditions(vanetza::units::Length distance, double velocityDifference, long cardinality, bool withinBoundingBox = false);

        // Clustering Containers Parsing
        typedef struct ClusterParameters{

            vanetza::units::Length distance;
            double velocityDifference;
            long cardinality;
            bool withinBoundingBox;

        }ClusterParameters_t;

        artery::ClusterManagement::ClusterParameters_t parseInformationContainer(const vanetza::asn1::Vam&);

        // Cluster timing
		omnetpp::SimTime mTimeClusterUniquenessThreshold;
		omnetpp::SimTime mTimeClusterBreakupWarning;
		omnetpp::SimTime mTimeClusterJoinNotification;
		omnetpp::SimTime mTimeClusterJoinSuccess;
		omnetpp::SimTime mTimeClusterIdChangeNotification;
		omnetpp::SimTime mTimeClusterIdPersist;
		omnetpp::SimTime mTimeClusterContinuity;
		omnetpp::SimTime mTimeClusterLeaderWaiting;
		omnetpp::SimTime mTimeClusterLeaveNotification;
		omnetpp::SimTime mTimeCombinedVruClusterOpportunity;  

        //ClusterManagement
        unsigned mNumCreateCluster;
		vanetza::units::Length mMaxClusterDistance;
		double mMaxClusterVelocityDifference;
		vanetza::units::Length mMaxCombinedClusterDistance;
		unsigned mMinClusterSize;
		unsigned mMaxClusterSize;
		unsigned mNumClusterVAMRepeat;

        std::map<LocalDynamicMap::StationID, artery::VaObject> mVruClusterMap;
        //vanetza::units::Length mMaxVruDistance;
        std::vector<vanetza::units::Length> mNearbyVruDistances;

        std::string mBoundingBoxShape;

        LocalDynamicMap* mLocalDynamicMap;
        const PersonDataProvider* mPersonDataProvider;
        const traci::PersonController* mPersonController;
		std::shared_ptr<const traci::API> api;

};

} // namespace artery

#endif /* ARTERY_CLUSTERMANAGEMENT_H_ */