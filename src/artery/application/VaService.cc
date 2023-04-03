#include "artery/application/VaObject.h"
#include "artery/application/VaService.h"
#include "artery/application/Constants.hpp"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/PersonDataProvider.h"
#include <artery/envmod/LocalEnvironmentModel.h>
#include "artery/traci/PersonController.h"
#include "artery/envmod/sensor/Sensor.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <boost/geometry/strategies/geographic/distance.hpp>
#include <boost/geometry/strategies/geographic/parameters.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/strategies/convex_hull.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/asn1/vam.hpp>
#include <chrono>
#include <bitset>
#include "traci/API.h"
#include "traci/sumo/utils/traci/TraCIAPI.h"
#include "traci/Boundary.h"
#include "artery/traci/Cast.h"
#include "traci/Core.h"

/*
	ATTENTION:

	Currently being modified and tested to enable clustering 

	General functionalities yet to implement.
	TODO: handle join event
	TODO: handle leave event
	TODO: handle breakup event 
	
*/
namespace artery
{
namespace bg = boost::geometry;
using namespace omnetpp;


static const simsignal_t scSignalVamReceived = cComponent::registerSignal("VamReceived");
static const simsignal_t scSignalVamSent = cComponent::registerSignal("VamSent");
static const simsignal_t SignalVamReceived = cComponent::registerSignal("VamReceivedCounter");
static const simsignal_t SignalVamSent = cComponent::registerSignal("VamSentCounter");

static const auto scLowFrequencyContainerInterval = std::chrono::milliseconds(500);

SpeedValue_t buildVAMSpeedValue(const vanetza::units::Velocity& v)
{
	static const vanetza::units::Velocity lower { 0.0 * boost::units::si::meter_per_second };
	static const vanetza::units::Velocity upper { 163.82 * boost::units::si::meter_per_second };

	SpeedValue_t speed = SpeedValue_unavailable;
	if (v >= upper) {
		speed = 16382; // see CDD A.74 (TS 102 894 v1.2.1)
	} else if (v >= lower) {
		speed = Constants::round(v, Constants::centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
	}
	return speed;
}


Define_Module(VaService)

VaService::VaService() :
		mGenVamMin { 100, SIMTIME_MS },
		mGenVamMax { 5000, SIMTIME_MS },
		mGenVam(mGenVamMax),
		mGenVamLowDynamicsCounter(0),
		mGenVamLowDynamicsLimit(3)
{
}

void VaService::initialize()
{
	ItsG5BaseService::initialize();
	mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
	mPersonDataProvider = &getFacilities().get_const<PersonDataProvider>();
	mTimer = &getFacilities().get_const<Timer>();
	
	// avoid unreasonable high elapsed time values for newly inserted vehicles
	mLastVamTimestamp = simTime();

	// first generated CAM shall include the low frequency container
	//mLastLowVamTimestamp = mLastVamTimestamp - artery::simtime_cast(scLowFrequencyContainerInterval);

	mLocalDynamicMap = &getFacilities().get_mutable<artery::LocalDynamicMap>();
	
	//mLocalEnvironmentModel = &getFacilities().get_mutable<LocalEnvironmentModel>();	
	mPersonController = &getFacilities().get_const<traci::PersonController>();
	api = mPersonController->getTraCI();

	// generation rate boundaries
	mGenVamMin = par("minInterval");
	mGenVamMax = par("maxInterval");
	mGenVam = mGenVamMax;

	// vehicle dynamics thresholds
	mHeadingDelta = vanetza::units::Angle { par("headingDelta").doubleValue() * vanetza::units::degree };
	mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
	mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;

	mDccRestriction = par("withDccRestriction");
	mFixedRate = par("fixedRate");

	// look up primary channel for CA
	mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::VRU);

	// Clustering options and parameters
	mBoundingBoxShape = par("boundingBoxShape").stringValue();
	mNumCreateCluster = par("numCreateCluster").intValue();
	mMinClusterSize = par("minClusterSize").intValue();
	mMaxClusterSize = par("maxClusterSize").intValue();
	mMaxVruDistance = par("maxVruDistance").doubleValue() * vanetza::units::si::meter;
	mMaxClusterVelocityDifference = par("maxClusterVelocityDifference").doubleValue();

	// VRU Roles
	mVruIdle = par("vruIdle");
	mVruActiveStandalone = par("vruActiveStandalone");
	mVruActiveClusterLeader = par("vruActiveClusterLeader");
	mVruPassive = par("vruPassive");
	mClusterContainer = par("clusterContainer");

	// Clustering timing parameters
	mTimeClusterJoinNotification = par("timeClusterJoinNotification");
	mTimeClusterJoinSuccess = par("timeClusterJoinSuccess");
	mTimeClusterContinuity = par("timeClusterContinuity");
	mTimeClusterLeaveNotification = par("timeClusterLeaveNotification");
}


void VaService::trigger()
{
	Enter_Method("trigger");
	checkTriggeringConditions(simTime());
}

void VaService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
	Enter_Method("indicate");

	Asn1PacketVisitor<vanetza::asn1::Vam> visitor;
	const vanetza::asn1::Vam* vam = boost::apply_visitor(visitor, *packet);
	if (vam && vam->validate()) {
		VaObject obj = visitor.shared_wrapper;
		emit(scSignalVamReceived, &obj);
		mLocalDynamicMap->updateAwareness(obj);
		vamReceiveCounter ++;
		emit(SignalVamReceived,vamReceiveCounter);

		retrieveInformationFromVAM(*vam);

	}
}

void VaService::retrieveInformationFromVAM(const vanetza::asn1::Vam &vam)
{
	VruAwareness_t vam_data = (*vam).vam;
	
	// Cluster Information Container
	if(vam_data.vamParameters.vruClusterInformationContainer){
		int clusterCardinalitySize = vam_data.vamParameters.vruClusterInformationContainer->clusterCardinalitySize;

		if(clusterCardinalitySize > mMinClusterSize && clusterCardinalitySize < mMaxClusterSize){

			// Builds cluster Bounding Box and checks if node is within it
			PolyPointList_t& receivedOffsetList = vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.choice.clusterPolygon.polyPointList;
			std::vector<pointXY> reconstructedPositions;

			auto vruLeaderLat = vam_data.vamParameters.basicContainer.referencePosition.latitude/std::pow(10.0, 7);
			auto vruLeaderLon = vam_data.vamParameters.basicContainer.referencePosition.longitude/std::pow(10.0, 7);
			libsumo::TraCIPosition leaderTraCIPos = api->simulation.convertGeo(vruLeaderLon,vruLeaderLat, true);
			
			std::map<int,OffsetPoint_t*> offsetMap;
			for(int iter = 0; iter < receivedOffsetList.list.count; iter++){
				offsetMap.insert(std::pair<int, OffsetPoint_t*>(iter, receivedOffsetList.list.array[iter]));
			}

			bool firstIteration = true;
			pointXY previousPos;
			for(auto& offsetPoint : offsetMap){

				if(firstIteration){
					previousPos = pointXY(leaderTraCIPos.x, leaderTraCIPos.y);
					firstIteration = false;
				}

				Node_XY_22b_t* node = &offsetPoint.second->nodeOffsetPointxy.choice.node_XY2;
				pointXY currentPos = pointXY(bg::get<0>(previousPos) + node->x/100, bg::get<1>(previousPos) + node->y/100);
				reconstructedPositions.push_back(currentPos);
				previousPos = currentPos;
			}

			Polygon_t reconstructedPolygon;
			bg::assign_points(reconstructedPolygon, reconstructedPositions);

			libsumo::TraCIPosition vruEgoTraCIPos = position_cast(traci::Boundary(api->simulation.getNetBoundary()), mPersonDataProvider->position());
			
			if(bg::within(pointXY(vruEgoTraCIPos.x, vruEgoTraCIPos.y),reconstructedPolygon)){
				std::cout << "Node " << mPersonDataProvider->station_id() << " is in cluster area" << std::endl;
			}
		}
	}

}

void VaService::checkTriggeringConditions(const SimTime& T_now)
{	
	// provide variables named like in EN 302 637-2 V1.3.2 (section 6.1.3)
	SimTime& T_GenVam = mGenVam;
	const SimTime& T_GenVamMin = mGenVamMin;
	const SimTime& T_GenVamMax = mGenVamMax;
	const SimTime T_GenVamDcc = mDccRestriction ? genVamDcc() : T_GenVamMin;
	const SimTime T_elapsed = T_now - mLastVamTimestamp;

	if (T_elapsed >= T_GenVamDcc) {
		if (mFixedRate) {
			clusterManagement(simTime());
			sendVam(T_now);			
		} else if (T_elapsed >= T_GenVam) {
			clusterManagement(simTime());
			sendVam(T_now);
			if (++mGenVamLowDynamicsCounter >= mGenVamLowDynamicsLimit) {
				T_GenVam = T_GenVamMax;
			}
		}
	}

}

bool VaService::checkPositionDelta() const
{
	return (distance(mLastVamPosition, mPersonDataProvider->position()) > mPositionDelta);
}

bool VaService::checkSpeedDelta() const
{
	return abs(mLastVamSpeed - mPersonDataProvider->speed()) > mSpeedDelta;
}

bool VaService::checkClusteringConditions(vanetza::units::Length distance, double velocityDifference, long cardinality)
{
	return (distance <= mMaxVruDistance && velocityDifference <= mMaxClusterVelocityDifference && cardinality < mMaxClusterSize);
}

void VaService::sendVam(const SimTime& T_now)
{
	uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mPersonDataProvider->updated()));
	auto vam = createVRUAwarenessMessage(*mPersonDataProvider, genDeltaTimeMod);

	mLastVamPosition = mPersonDataProvider->position();
	mLastVamSpeed = mPersonDataProvider->speed();

	//mLastVamHeading = mPersonDataProvider->heading();
	mLastVamTimestamp = T_now;
	if (T_now - mLastLowVamTimestamp >= artery::simtime_cast(scLowFrequencyContainerInterval)) {
		//addLowFrequencyContainer(vam, par("pathHistoryLength"));
		mLastLowVamTimestamp = T_now;
	}

	if(mClusterContainer && mVruActiveClusterLeader){
		addClusterInformationContainer(vam, mBoundingBoxInfo);
		clusterVamSentCounter ++;
	}

	using namespace vanetza;
	btp::DataRequestB request;
	request.destination_port = btp::ports::VAM;
	request.gn.its_aid = aid::VRU;
	request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
	request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

	VaObject obj(std::move(vam));
	emit(scSignalVamSent, &obj);

	vamSentCounter ++;
	emit (SignalVamSent, vamSentCounter);

	using VamByteBuffer = convertible::byte_buffer_impl<asn1::Vam>;
	std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
	std::unique_ptr<convertible::byte_buffer> buffer { new VamByteBuffer(obj.shared_ptr()) };
	payload->layer(OsiLayer::Application) = std::move(buffer);
	this->request(request, std::move(payload));	
}

void VaService::clusterManagement(const SimTime& T_now)
{		
	if (mVruActiveStandalone) {
		
		// TODO: Consider moving the range verification to its own method
		auto lDM = mLocalDynamicMap->localMap();
		//check if there are VRUs on the LDM
		if ((*lDM).size() > 0) {

			vanetza::units::Length smallestDistance = mMaxVruDistance;
			
			//calculate distance between egoVRU and received VRU
			auto VruEgoLon = ((Constants::round(mPersonDataProvider->longitude(), Constants::microdegree) * Longitude_oneMicrodegreeEast) / std::pow( 10.0, 7 ));
			auto VruEgoLat = ((Constants::round(mPersonDataProvider->latitude(), Constants::microdegree) * Latitude_oneMicrodegreeNorth) / std::pow( 10.0, 7 )); 
			auto VruEgoSpeed = buildVAMSpeedValue(mPersonDataProvider->speed());
		
			//collect data from the VRUs in the LDM and update mVruClusterMap		
			for (auto const& itr : *lDM) {
											
				auto mVlatitude = ((itr.second.object.asn1()->vam.vamParameters.basicContainer.referencePosition.latitude) / std::pow( 10.0, 7 ));				
				auto mVlongitude = ((itr.second.object.asn1()->vam.vamParameters.basicContainer.referencePosition.longitude) / std::pow( 10.0, 7 ));			
								
				vanetza::units::Length distanceVRU = bg::distance(point(VruEgoLon,VruEgoLat), point(mVlongitude,mVlatitude)) * vanetza::units::si::meter;	

				auto mVSpeed = itr.second.object.asn1()->vam.vamParameters.vruHighFrequencyContainer->speed.speedValue;
				double clusterVelocityDifference = (VruEgoSpeed > 0) ? std::abs(static_cast<double>(VruEgoSpeed - mVSpeed))/VruEgoSpeed : 0.0;

				//update mVruClusterMap when distance, velocity and cardinality constraints are met		
				if(checkClusteringConditions(distanceVRU,clusterVelocityDifference,mVruClusterMap.size()+1)) {	

					// obtains the minimum distance between Ego and the remaining nodes
					if(distanceVRU < smallestDistance){smallestDistance = distanceVRU;}

					uint32_t stationID = itr.second.object.asn1()->header.stationID;
					artery::VaObject awr = itr.second.object;
					mVruClusterMap.insert(std::pair<LocalDynamicMap::StationID, artery::VaObject>(stationID, awr));
				}
			}

			// If it is the first VAM of a cluster, the bounding box must include only the leader
			if(clusterVamSentCounter == 0){
				mBoundingBoxInfo.shape = "Circular";
				mBoundingBoxInfo.cardinalitySize = mMinClusterSize;
				mBoundingBoxInfo.circleRadius = (smallestDistance/vanetza::units::si::meter)*Radius_oneMeter*.5;
				mBoundingBoxInfo.leaderId = omnetpp::intuniform(getRNG(0), 0, 255);

			}else if (mVruClusterMap.size() > mNumCreateCluster) {
				mBoundingBoxInfo.shape = mBoundingBoxShape;
				mBoundingBoxInfo.cardinalitySize = mVruClusterMap.size() + 1;	
				mBoundingBoxInfo.polyOffsets = clusterPoly(mVruClusterMap);			
			}
				
			//Calculate the distance among ego VRU and VRUs on the mVruClusterMap 
			for (auto const& IT : mVruClusterMap){
				auto objLat = ((IT.second.asn1()->vam.vamParameters.basicContainer.referencePosition.latitude) / geoBase);	
				auto objLon = ((IT.second.asn1()->vam.vamParameters.basicContainer.referencePosition.longitude) / geoBase);
				auto vruDistance = bg::distance(point(VruEgoLon,VruEgoLat), point(objLon,objLat));				
			}			
			
			mClusterContainer = true;			
			mVruActiveStandalone = true; //set as false 
			mVruActiveClusterLeader = true;					
		}	
	}
	mVruClusterMap.clear();
}

std::vector<std::tuple<long,long>>  VaService::clusterPoly(const std::map<LocalDynamicMap::StationID, artery::VaObject> vruMap)
{
	std::map<LocalDynamicMap::StationID, artery::VaObject> clusterMap = vruMap;
	std::vector<pointXY> clusterPositions;
	pointXY previousPos, vruEgoPos;
	std::vector<std::tuple<long,long>> distancesList;

	libsumo::TraCIPosition vruEgoTraCIPos = position_cast(traci::Boundary(api->simulation.getNetBoundary()), mPersonDataProvider->position());
	vruEgoPos = pointXY(vruEgoTraCIPos.x, vruEgoTraCIPos.y);
	clusterPositions.push_back(vruEgoPos);

	//egoHeading
	auto egoVruHeading = Constants::round(mPersonDataProvider->heading(), Constants::decidegree);	

	for (auto const& itr : clusterMap) {
		auto vruLon = ((itr.second.asn1()->vam.vamParameters.basicContainer.referencePosition.longitude) / std::pow( 10.0, 7 ));
		auto vruLat = ((itr.second.asn1()->vam.vamParameters.basicContainer.referencePosition.latitude) / std::pow( 10.0, 7 ));
		libsumo::TraCIPosition vruTraCIPos = api->simulation.convertGeo(vruLon,vruLat, true);
		clusterPositions.push_back(pointXY(vruTraCIPos.x, vruTraCIPos.y));				
	}

	// Adds leader node again to close polygon
	clusterPositions.push_back(pointXY(vruEgoTraCIPos.x, vruEgoTraCIPos.y));

	// Using boost geometry, simplifies the original polygon with convex hull algorithm
	Polygon_t clusterPolygon, hull;
	bg::assign_points(clusterPolygon, clusterPositions);

	std::cout << "Original poly: " << bg::dsv(clusterPolygon) << "\n";

	bg::convex_hull(clusterPolygon, hull);

	std::cout << "Hull poly: " << bg::dsv(hull) << "\n";

	// replaces original vertices with simplified polygon from convex hull
	clusterPositions = hull.outer();

	// Obtains list of offsets
	bool firstIteration = true;
	for (auto const& currentPos : clusterPositions){

		long deltaX , deltaY;
		
		// First value is obtained from the offset between vruEgo and starting point of the polygon
		// If the starting point is the vruEgo, a null first offset is expected
		if(firstIteration){
			firstIteration = false;
			previousPos = vruEgoPos;
			std::cout << "offset list leader: ";
		}

		// Offset in centimeters assuming a cartesian plane
		deltaX = std::round(bg::get<0>(currentPos) - bg::get<0>(previousPos))*100; 
		deltaY = std::round(bg::get<1>(currentPos) - bg::get<1>(previousPos))*100;

		std::cout << "(" << std::to_string(deltaX) << "," << std::to_string(deltaY) << "),";

		distancesList.push_back(std::make_tuple(deltaX, deltaY));
		previousPos = currentPos;
	}
	std::cout << std::endl;

	return distancesList;
}

SimTime VaService::genVamDcc()
{
	// network interface may not be ready yet during initialization, so look it up at this later point
	auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
	vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
	if (!trc) {
		throw cRuntimeError("No DCC TRC found for VA's primary channel %i", mPrimaryChannel);
	}
	
	static const vanetza::dcc::TransmissionLite va_tx(vanetza::dcc::Profile::DP2, 0);
	vanetza::Clock::duration interval = trc->interval(va_tx);
	SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
	return std::min(mGenVamMax, std::max(mGenVamMin, dcc));
}

vanetza::asn1::Vam createVRUAwarenessMessage(const PersonDataProvider& pdp, uint16_t genDeltaTime)
{
	vanetza::asn1::Vam message;

	ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 2;
	header.messageID = ItsPduHeader__messageID_vam;
	header.stationID = pdp.getStationId();

	VruAwareness_t& vam = (*message).vam;
	vam.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
	BasicContainer_t& basic = vam.vamParameters.basicContainer;

	basic.stationType = StationType_pedestrian;
	basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
	basic.referencePosition.longitude = Constants::round(pdp.longitude(), Constants::microdegree) * Longitude_oneMicrodegreeEast;
	basic.referencePosition.latitude = Constants::round(pdp.latitude(), Constants::microdegree) * Latitude_oneMicrodegreeNorth;
	basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
			SemiAxisLength_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
			SemiAxisLength_unavailable;
	
	//High Frequency Container
	
	VruHighFrequencyContainer*& hfc = message->vam.vamParameters.vruHighFrequencyContainer;
	hfc = vanetza::asn1::allocate<VruHighFrequencyContainer>();
	
	hfc->heading.headingValue = Constants::round(pdp.heading(), Constants::decidegree);
	hfc->heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
	hfc->speed.speedValue = buildVAMSpeedValue (pdp.speed());
	hfc->speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;

	const double lonAccelValue = pdp.acceleration() / vanetza::units::si::meter_per_second_squared;
	if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0){
		hfc->longitudinalAcceleration.longitudinalAccelerationValue = lonAccelValue * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
	} else{
		hfc->longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
	}
	
	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid High Frequency VAM: %s", error.c_str());
	}

	return message;
}

void addClusterInformationContainer(vanetza::asn1::Vam& message, BoundingBoxInfo_t& boxInfo){

	VruAwareness_t &vam = (*message).vam;

	VruClusterInformationContainer_t* &vcic = vam.vamParameters.vruClusterInformationContainer;
	vcic = vanetza::asn1::allocate<VruClusterInformationContainer_t>();

	vcic->clusterId = boxInfo.leaderId;

	if(boxInfo.shape == "Circular"){
		vcic->clusterBoundingBoxShape.present = ClusterBoundingBoxShape_PR_clusterCircle;
		vcic->clusterBoundingBoxShape.choice.clusterCircle.radius = boxInfo.circleRadius;

	}else if(boxInfo.shape == "Rectangular"){
		vcic->clusterBoundingBoxShape.present = ClusterBoundingBoxShape_PR_clusterRectangle;
		vcic->clusterBoundingBoxShape.choice.clusterRectangle.nodeCenterPoint->nodeOffsetPointxy.present = NodeOffsetPointXY_PR_node_XY2;
		// TODO: Implement proper size scaling of the rectangle's width and length
		vcic->clusterBoundingBoxShape.choice.clusterRectangle.semiMinorRangeLength = 17;
		vcic->clusterBoundingBoxShape.choice.clusterRectangle.semiMajorRangeLength = 17;
		vcic->clusterBoundingBoxShape.choice.clusterRectangle.semiMajorRangeOrientation = Constants::round(boxInfo.leaderDataProvider->heading(), Constants::decidegree);

	}else if(boxInfo.shape == "Polygon"){
		vcic->clusterBoundingBoxShape.present = ClusterBoundingBoxShape_PR_clusterPolygon;

		for (auto &offsetValue : boxInfo.polyOffsets){
			OffsetPoint* offsetPoint = vanetza::asn1::allocate<OffsetPoint>();
			offsetPoint->nodeOffsetPointxy.present = NodeOffsetPointXY_PR_node_XY2;
			offsetPoint->nodeOffsetPointxy.choice.node_XY2.x = std::get<0>(offsetValue);
			offsetPoint->nodeOffsetPointxy.choice.node_XY2.y = std::get<1>(offsetValue);
			ASN_SEQUENCE_ADD(&vcic->clusterBoundingBoxShape.choice.clusterPolygon.polyPointList, offsetPoint);
		}
	}

	vcic->clusterCardinalitySize = boxInfo.cardinalitySize;

	vcic->clusterProfiles.buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
	vcic->clusterProfiles.size = 1;
	vcic->clusterProfiles.bits_unused = 0;
	vcic->clusterProfiles.buf[0] = static_cast<uint8_t>(e_ClusterProfiles::ClusterProfiles_pedestrian);

}

void addClusterOperationContainer(vanetza::asn1::Vam& message){
	VruClusterOperationContainer*& vcoc = message->vam.vamParameters.vruClusterOperationContainer;
	vcoc = vanetza::asn1::allocate<VruClusterOperationContainer>();

}

} // namespace artery
