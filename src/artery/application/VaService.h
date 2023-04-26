/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef ARTERY_VASERVICE_H_
#define ARTERY_VASERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/vam.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>
#include <string>
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/envmod/EnvironmentModelObject.h>

/*
	ATTENTION:

	Currently being modified and tested to enable clustering 
	
*/


namespace traci { class PersonController; class Boundary;}

namespace artery
{
class NetworkInterfaceTable;
class Timer;
class PersonDataProvider;
class LocalEnvironmentModel;

typedef struct BoundingBoxInfo{
	std::string shape;
	unsigned cardinalitySize;
	std::vector<std::tuple<long,long>> polyOffsets;
	const PersonDataProvider* leaderDataProvider;
	double circleRadius;
	long leaderId;

} BoundingBoxInfo_t;

class VaService : public ItsG5BaseService
{
	public:
		VaService();
		void initialize() override;
		void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
		void trigger() override;

	private:
			
		typedef boost::geometry::model::point <double,2,boost::geometry::cs::geographic <boost::geometry::degree> >point;
		typedef boost::geometry::model::point <long,2,boost::geometry::cs::cartesian>pointXY;
		typedef boost::geometry::srs::spheroid <double> stype;
		typedef boost::geometry::strategy::distance::vincenty <stype> vincenty_type;
		typedef boost::geometry::model::polygon<pointXY, false> Polygon_t; 
		typedef boost::geometry::model::multi_polygon<Polygon_t> Circle_t;

		BoundingBoxInfo_t mBoundingBoxInfo;

		void checkTriggeringConditions(const omnetpp::SimTime&);
		bool checkHeadingDelta() const;
		bool checkPositionDelta() const;
		bool checkSpeedDelta() const;
		bool checkClusteringConditions(vanetza::units::Length distance, double velocityDifference, long cardinality, bool withinBoundingBox = false);
		void sendVam(const omnetpp::SimTime&);
		void retrieveInformationFromVAM(const vanetza::asn1::Vam*);
		artery::VaService::Polygon_t reconstructPolygon(const PolyPointList_t& receivedOffsetList, libsumo::TraCIPosition& leaderPos);
		artery::VaService::Circle_t buildCircle(const double radius, pointXY& leaderPos);
		double velocityDifference(SpeedValue_t, SpeedValue_t);
		
		void clusterManagement(const omnetpp::SimTime&);
		void checkSensorData(const omnetpp::SimTime&);	
		std::vector<std::tuple<long,long>> clusterPoly(const std::map<LocalDynamicMap::StationID, artery::VaObject>);

		omnetpp::SimTime genVamDcc();

		ChannelNumber mPrimaryChannel = channel::CCH;
		const NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
		const PersonDataProvider* mPersonDataProvider = nullptr;
		const Timer* mTimer = nullptr;
		
		LocalDynamicMap* mLocalDynamicMap = nullptr;
		std::map<LocalDynamicMap::StationID, artery::VaObject> mVruClusterMap;
		LocalEnvironmentModel* mLocalEnvironmentModel = nullptr;

		omnetpp::SimTime mGenVamMin;
		omnetpp::SimTime mGenVamMax;
		omnetpp::SimTime mGenVam;
		unsigned mGenVamLowDynamicsCounter;
		unsigned mGenVamLowDynamicsLimit;
		Position mLastVamPosition;
		vanetza::units::Velocity mLastVamSpeed;
		vanetza::units::Angle mLastVamHeading;
		omnetpp::SimTime mLastVamTimestamp;
		omnetpp::SimTime mLastLowVamTimestamp;
		vanetza::units::Angle mHeadingDelta;
		vanetza::units::Length mPositionDelta;
		vanetza::units::Velocity mSpeedDelta;
		bool mDccRestriction;
		bool mFixedRate;
		bool mVruIdle;
		bool mVruActiveStandalone;
		bool mVruActiveClusterLeader;
		bool mVruPassive;
		bool mClusterContainer;
		vanetza::units::Length mMaxVruDistance;

		bool mVruShouldJoin = false;
		bool mVruShouldLeave = false;
		long mTargetClusterId;

		std::string mBoundingBoxShape;

		double vamReceiveCounter = 0;
		double vamSentCounter = 0;
		double clusterVamSentCounter = 0;
		float geoBase = 10000000;

		//ClusterManagement
		unsigned mClusterId;
		unsigned mNumCreateCluster;
		unsigned mMaxClusterDistance;
		double mMaxClusterVelocityDifference;
		unsigned mMaxCombinedClusterDistance;
		unsigned mMinClusterSize;
		unsigned mMaxClusterSize;
		unsigned mNumClusterVAMRepeat;

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

		//access to control Person connection among Artery, Omnetpp, and SUMO
		void personSumoData(const omnetpp::SimTime&);
		const traci::PersonController* mPersonController = nullptr;
		std::shared_ptr<const traci::API> api;
};

vanetza::asn1::Vam createVRUAwarenessMessage(const PersonDataProvider&, uint16_t genDeltaTime);
void addLowFrequencyContainer(vanetza::asn1::Vam&, unsigned pathHistoryLength = 0);
void addClusterInformationContainer(vanetza::asn1::Vam&, BoundingBoxInfo_t&);
void addClusterOperationContainer(vanetza::asn1::Vam&);	
void addJoinOperationContainer(vanetza::asn1::Vam&, long clusterId, omnetpp::SimTime timeToJoin);	

} // namespace artery

#endif /* ARTERY_VASERVICE_H_ */