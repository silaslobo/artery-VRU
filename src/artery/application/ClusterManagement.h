#ifndef ARTERY_CLUSTERMANAGEMENT_H_
#define ARTERY_CLUSTERMANAGEMENT_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VaObject.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/vam.hpp>
#include <omnetpp/simtime.h>
#include <omnetpp.h>
#include <string>
#include "artery/application/EntityDataProvider.h"
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/envmod/EnvironmentModelObject.h>

namespace artery{

using namespace omnetpp;

class LocalEnvironmentModel;
class ItsG5BaseService;

typedef struct ClusterManagementParameters{
			
			// Cluster timing
			omnetpp::SimTime TimeClusterUniquenessThreshold;
			omnetpp::SimTime TimeClusterBreakupWarning;
			omnetpp::SimTime TimeClusterJoinNotification;
			omnetpp::SimTime TimeClusterJoinSuccess;
			omnetpp::SimTime TimeClusterIdChangeNotification;
			omnetpp::SimTime TimeClusterIdPersist;
			omnetpp::SimTime TimeClusterContinuity;
			omnetpp::SimTime TimeClusterLeaderWaiting;
			omnetpp::SimTime TimeClusterLeaveNotification;
			omnetpp::SimTime TimeCombinedVruClusterOpportunity;  

			//Cluster pparameters
			unsigned NumCreateCluster;
			vanetza::units::Length MaxClusterDistance;
			double MaxClusterVelocityDifference;
			vanetza::units::Length MaxCombinedClusterDistance;
			unsigned MinClusterSize;
			unsigned MaxClusterSize;
			unsigned NumClusterVAMRepeat;

			std::string DefaultBoundingBoxShape;
            bool EnableBoundingBoxPadding;

} ClusterManagementParameters_t;

typedef struct InformationContainerData{
	std::string shape;
	unsigned cardinalitySize;
	std::tuple<long,long> nodeOffset;
	std::vector<std::tuple<long,long>> polyOffsets;
	const EntityDataProvider* leaderDataProvider;
	double circleRadius;
	long clusterId;
    double orientationAngle;
    double rectangleWidth;
    double rectangleLength;

} InformationContainerData_t;

typedef std::map<LocalDynamicMap::StationID, artery::VaObject> VruClusterMap_t;
class ClusterManagement 
{
    public:

        enum VruRoleType {VruIdle, VruActiveStandalone, VruActiveClusterLeader, VruPassive};
        ClusterManagement();
        
        void initialize(ClusterManagementParameters_t&);
        void simObjects(const EntityDataProvider* pdp, artery::LocalDynamicMap* ldm, const traci::PersonController* pc);

        void setCurrentVruRole(VruRoleType newVruRole);
        VruRoleType getCurrentVruRole();
        bool isVruIdle();
        bool isVruStandalone();
        bool isVruClusterLeader();
        bool isVruPassive();

        // Operation conditions check
        bool vruShouldJoin(const vanetza::asn1::Vam&);
        bool vruShouldLeave_VamParameters(const vanetza::asn1::Vam&);
        bool vruShouldLeave_ElapsedTime(const omnetpp::SimTime&);
        bool vruShouldCreateCluster();
        bool clusterShouldBreakup_VamParameters(const vanetza::asn1::Vam&);
        bool clusterShouldBreakup_ElapsedTime(const omnetpp::SimTime&);

        void registerClusterId(long clusterId){mClusterId = clusterId;}
        long getClusterId(){return mClusterId;}

        void updateReceivedIdMap(long receivedId, const omnetpp::SimTime& T_now);
        void refreshReceivedIdMap(const omnetpp::SimTime& T_now);
        bool isClusterIdUnique(long proposedId);

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
        omnetpp::SimTime timeClusterLeaderWaiting() const {return mTimeClusterLeaderWaiting;}
        omnetpp::SimTime timeClusterLeaveNotification() const {return mTimeClusterLeaveNotification;}
        omnetpp::SimTime timeCombinedVruClusterOpportunity() const {return mTimeCombinedVruClusterOpportunity;}  

        // Timestamps to evaluate Cluster Operation triggers
        omnetpp::SimTime mLastLeaderVamTimestamp;
        omnetpp::SimTime mClusterCreationTimestamp;

        void setLeaveReason(long leaveReason){mLeaveReason = leaveReason;}
        long getLeaveReason(){return mLeaveReason;}

        void setBreakupReason(long breakupReason){mBreakupReason = breakupReason;}
        long getBreakupReason(){return mBreakupReason;}

        void createVruClusterMap();
        void updateVruClusterMap(const vanetza::asn1::Vam&, std::string);
        void clearVruClusterMap() {mClusterMap.clear();}
        VruClusterMap_t getVruClusterMap() const {return mClusterMap;} 
        unsigned getCurrentClusterSize() const {return getVruClusterMap().size()+1;}
        double getSmallestDistanceToVru() const {return mSmallestDistanceToVru;}

        double getBoundingBoxArea() const {return mBoundingBoxArea;}
        double getBoundingBoxPerimeter() const {return mBoundingBoxPerimeter;}
        double getLargetDistanceToLeader() const {return mLargestDistanceToLeader;}

        double getDistFromCIC() const {return mDistFromCIC.value();}
        double getCardFromCIC() const {return mCardFromCIC;}

        InformationContainerData_t generateDataForNewCluster(long);
        InformationContainerData_t generateDataForCurrentCluster();

    private:
        VruRoleType currentVruRole;
        long mClusterId;
        std::map<long, omnetpp::SimTime> mReceivedIdMap;

        // Geometry definitions
        typedef boost::geometry::model::point <double,2,boost::geometry::cs::cartesian>pointXY;
        typedef boost::geometry::model::polygon<pointXY, false> Polygon_t; 
		typedef boost::geometry::model::multi_polygon<Polygon_t> MultiPoly_t;
        typedef boost::geometry::model::box<pointXY> Box_t;

        // Build shapes from received VAM data
        artery::ClusterManagement::Polygon_t reconstructPolygon(const PolyPointList_t& receivedOffsetList, libsumo::TraCIPosition& leaderPos);
        artery::ClusterManagement::Polygon_t reconstructRectangle(const pointXY center, long halfWidth, long halfLength, long angle);
        
        // Convert various NodeOffset types to a tuple
        std::tuple<long,long> getOffsetValues(const OffsetPoint_t&);

        artery::ClusterManagement::pointXY getCurrentVruXY();
        
        // VruClusterMap manipulation
        artery::ClusterManagement::Polygon_t generatePolygonFromMap(const VruClusterMap_t);
        std::tuple<long,long> calculateOffset(const pointXY, const pointXY);
        std::vector<std::tuple<long,long>> calculatePolyOffsetList(const Polygon_t);
        artery::ClusterManagement::pointXY calculateCenterFromBasePoly(const Polygon_t);
        double calculateRadius(const VruClusterMap_t clusterMap, const pointXY center);
        std::string selectBoundingBoxShape(const Polygon_t);
        std::tuple<Polygon_t, double> getRotatedRectangle(const Polygon_t);

        // Rectangle helper methods
        double radiansToWGS84Angle(double radians);
        double WGS84AngleToRadians(double WGSAngle);
        artery::ClusterManagement::Polygon_t translatePoly(const Polygon_t, double offsetX, double offsetY);
        artery::ClusterManagement::Polygon_t rotatePoly(const Polygon_t, double angle);

        // Bounding Box padding
        double calculatePaddingAmount(const VruClusterMap_t);
        artery::ClusterManagement::Polygon_t applyPaddingToPoly(const Polygon_t, double paddingAmount);

        // Geometries metrics 
        double mBoundingBoxArea;
        void setBoundingBoxArea(double area){mBoundingBoxArea = area;}
        double mBoundingBoxPerimeter;
        void setBoundingBoxPerimeter(double perimeter){mBoundingBoxPerimeter = perimeter;}
        double mLargestDistanceToLeader;
        void setLargestDistanceToLeader(double largestDistance){mLargestDistanceToLeader = largestDistance;}

        vanetza::units::Length mDistFromCIC;
        long mCardFromCIC;

        // Parameter validation
        double velocityDifference(SpeedValue_t, SpeedValue_t);
        bool checkClusteringConditions(vanetza::units::Length distance, double velocityDifference, long cardinality, bool withinBoundingBox = false);

        long mLeaveReason;
        long mBreakupReason;

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

        VruClusterMap_t mInitialClusterMap;
        VruClusterMap_t mClusterMap;
        double mSmallestDistanceToVru;

        std::string mBoundingBoxShape;
        bool mEnableBoundingBoxPadding;

        LocalDynamicMap* mLocalDynamicMap;
        const EntityDataProvider* mPersonDataProvider;
        const traci::PersonController* mPersonController;
		std::shared_ptr<const traci::API> api;

};

} // namespace artery

#endif /* ARTERY_CLUSTERMANAGEMENT_H_ */