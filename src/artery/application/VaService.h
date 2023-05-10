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
#include "artery/application/ClusterManagement.h"

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
class ClusterManagement;
typedef struct InformationContainerData{
	std::string shape;
	unsigned cardinalitySize;
	std::vector<std::tuple<long,long>> polyOffsets;
	const PersonDataProvider* leaderDataProvider;
	double circleRadius;
	long leaderId;

} InformationContainerData_t;
class VaService : public ItsG5BaseService
{
	public:
		VaService();
		void initialize() override;
		void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
		void trigger() override;

	private:
			
		typedef boost::geometry::model::point <double,2,boost::geometry::cs::geographic <boost::geometry::degree> >point;
		typedef boost::geometry::srs::spheroid <double> stype;
		typedef boost::geometry::strategy::distance::vincenty <stype> vincenty_type;

		InformationContainerData_t mInformationContainerData;

		void checkTriggeringConditions(const omnetpp::SimTime&);
		bool checkHeadingDelta() const;
		bool checkPositionDelta() const;
		bool checkSpeedDelta() const;
		void sendVam(const omnetpp::SimTime&);
		void retrieveInformationFromVAM(const vanetza::asn1::Vam&);
		
		//void clusterManagement(const omnetpp::SimTime&);
		void checkSensorData(const omnetpp::SimTime&);	
		//std::vector<std::tuple<long,long>> clusterPoly(const std::map<LocalDynamicMap::StationID, artery::VaObject>);

		omnetpp::SimTime genVamDcc();

		ChannelNumber mPrimaryChannel = channel::CCH;
		const NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
		const PersonDataProvider* mPersonDataProvider = nullptr;
		const Timer* mTimer = nullptr;
		ClusterManagement mClusterManagement;
		
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

		double vamReceiveCounter = 0;
		double vamSentCounter = 0;

		float geoBase = 10000000;

		//Cluster Management
		bool mClusterContainer;
		double clusterVamSentCounter = 0;
		unsigned mTargetClusterId;    
		std::string mBoundingBoxShape; 
		enum VruClusterOperation {noVruOperation, VruShouldJoin, VruShouldLeave}; 
		VruClusterOperation currentVruClusterOperation;
		void setVruClusterOperation(VruClusterOperation newVruClusterOperation);

		//access to control Person connection among Artery, Omnetpp, and SUMO
		void personSumoData(const omnetpp::SimTime&);
		const traci::PersonController* mPersonController = nullptr;
		std::shared_ptr<const traci::API> api;
};

vanetza::asn1::Vam createVRUAwarenessMessage(const PersonDataProvider&, uint16_t genDeltaTime);
void addLowFrequencyContainer(vanetza::asn1::Vam&, unsigned pathHistoryLength = 0);
void addClusterInformationContainer(vanetza::asn1::Vam&, InformationContainerData_t&);
void addClusterJoinOperationContainer(vanetza::asn1::Vam&);	
void addClusterLeaveOperationContainer(vanetza::asn1::Vam&);	

//TODO: Consider moving this method to an utilities class
SpeedValue_t buildVAMSpeedValue(const vanetza::units::Velocity& v);

} // namespace artery

#endif /* ARTERY_VASERVICE_H_ */