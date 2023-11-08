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
#include <vanetza/facilities/cam_functions.hpp>
#include <chrono>
#include "traci/API.h"
#include "traci/sumo/utils/traci/TraCIAPI.h"
#include "traci/Boundary.h"
#include "artery/traci/Cast.h"
#include "traci/Core.h"
#include "artery/application/ClusterManagement.h"


/*
	ATTENTION:

	Currently being modified and tested to enable clustering 
	
*/
namespace artery
{
namespace bg = boost::geometry;
using namespace omnetpp;

static const simsignal_t scSignalVamReceived = cComponent::registerSignal("VamReceived");
static const simsignal_t scSignalVamSent = cComponent::registerSignal("VamSent");
static const simsignal_t SignalVamReceived = cComponent::registerSignal("VamReceivedCounter");
static const simsignal_t SignalVamSent = cComponent::registerSignal("VamSentCounter");

static const simsignal_t SignalVamPosition = cComponent::registerSignal("VamTriggerPosition");
static const simsignal_t SignalVamHeading = cComponent::registerSignal("VamTriggerHeading");
static const simsignal_t SignalVamSpeed = cComponent::registerSignal("VamTriggerSpeed");

static const simsignal_t scSignalBoxArea = cComponent::registerSignal("BoundingBoxArea");
static const simsignal_t scSignalBoxPerimeter = cComponent::registerSignal("BoundingBoxPerimeter");
static const simsignal_t scClusterCardinality = cComponent::registerSignal("CardinalitySize");

static const simsignal_t SignalVamCluster = cComponent::registerSignal("VamCluster");
static const simsignal_t SignalVamJoin = cComponent::registerSignal("VamJoin");
static const simsignal_t SignalVamLeave = cComponent::registerSignal("VamLeave");
static const simsignal_t SignalVamBreak = cComponent::registerSignal("VamBreak");
static const simsignal_t SignalVamClusterCreation = cComponent::registerSignal("VamClusterCreation");
static const simsignal_t SignalVamTime = cComponent::registerSignal("VamTime");

static const simsignal_t SignalVamReceivedJoinID = cComponent::registerSignal("VamReceivedJoinId");
static const simsignal_t SignalClusterMember = cComponent::registerSignal("ClusterMember");

static const simsignal_t SignalLeaveReasonCounter = cComponent::registerSignal("LeaveReasonCounter");
static const simsignal_t SignalBreakupReasonCounter = cComponent::registerSignal("BreakupReasonCounter");

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

	// Clustering
	mClusterSupport = par("withClusterSupport");

	if(mClusterSupport){
		//Clustering options and parameters
		mClusterManagementParameters.NumCreateCluster = par("numCreateCluster").intValue();
		mClusterManagementParameters.MaxClusterDistance = par("maxClusterDistance").doubleValue() * vanetza::units::si::meter;
		mClusterManagementParameters.MaxClusterVelocityDifference = par("maxClusterVelocityDifference").doubleValue();
		mClusterManagementParameters.MaxCombinedClusterDistance = par("maxCombinedClusterDistance").doubleValue() * vanetza::units::si::meter;
		mClusterManagementParameters.MinClusterSize = par("minClusterSize").intValue();
		mClusterManagementParameters.MaxClusterSize = par("maxClusterSize").intValue();
		mClusterManagementParameters.NumClusterVAMRepeat = par("numClusterVAMRepeat").intValue();

		// Clustering timing parameters
		mClusterManagementParameters.TimeClusterUniquenessThreshold = par("timeClusterUniquenessThreshold");
		mClusterManagementParameters.TimeClusterBreakupWarning = par("timeClusterBreakupWarning");
		mClusterManagementParameters.TimeClusterJoinNotification = par("timeClusterJoinNotification");
		mClusterManagementParameters.TimeClusterJoinSuccess = par("timeClusterJoinSuccess");
		mClusterManagementParameters.TimeClusterIdChangeNotification = par("timeClusterIdChangeNotification");
		mClusterManagementParameters.TimeClusterIdPersist = par("timeClusterIdPersist");
		mClusterManagementParameters.TimeClusterContinuity = par("timeClusterContinuity");
		mClusterManagementParameters.TimeClusterLeaderWaiting = par("timeClusterLeaderWaiting");
		mClusterManagementParameters.TimeClusterLeaveNotification = par("timeClusterLeaveNotification");
		mClusterManagementParameters.TimeCombinedVruClusterOpportunity = par("timeCombinedVruClusterOpportunity"); 

		mClusterManagementParameters.DefaultBoundingBoxShape = par("defaultBoundingBoxShape").stringValue();
		mClusterManagementParameters.EnableBoundingBoxPadding = par("enableBoundingBoxPadding");

		mClusterManagement.initialize(mClusterManagementParameters);
		mClusterManagement.simObjects(mPersonDataProvider, mLocalDynamicMap, mPersonController);
		
		mClusterGenVamMax = par("clusterMaxInterval");

		currentVruClusterOperation = noVruOperation;
	}
}


void VaService::trigger()
{
	Enter_Method("trigger");
	checkTriggeringConditions(simTime());
}

void VaService::checkTriggeringConditions(const SimTime& T_now)
{	
	// provide variables named like in EN 302 637-2 V1.3.2 (section 6.1.3)
	SimTime& T_GenVam = mGenVam;
	const SimTime& T_GenVamMin = mGenVamMin;
	SimTime T_GenVamMax;
	const SimTime T_GenVamDcc = mDccRestriction ? genVamDcc() : T_GenVamMin;
	const SimTime T_elapsed = T_now - mLastVamTimestamp;

	if(mClusterSupport && mClusterManagement.isVruClusterLeader()){
		if(mClusterManagement.clusterShouldBreakup_ElapsedTime(T_now) && mClusterManagement.getCurrentClusterSize() < mClusterManagement.numCreateCluster()){
			setVruClusterOperation(BreakupCluster);
			sendVam(T_now);
			return;
		}else{
			T_GenVamMax = mClusterGenVamMax;
		}
	}else{
		T_GenVamMax = mGenVamMax;
	}

	if (!mClusterManagement.isVruPassive() || !mClusterSupport){
		if (T_elapsed >= T_GenVamDcc) {
			if (mFixedRate) {
				sendVam(T_now);
			}else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
				if (checkHeadingDelta()){
					emit(SignalVamHeading, 1);
				}
				if (checkPositionDelta()){
					emit(SignalVamPosition, 1);
				}
				if (checkSpeedDelta()){
					emit(SignalVamSpeed, 1);
				}
				sendVam(T_now);
				T_GenVam = std::min(T_elapsed, T_GenVamMax); /*< if middleware update interval is too long */
				mGenVamLowDynamicsCounter = 0;
			}
			else if (T_elapsed >= T_GenVam)
			{
				sendVam(T_now);
				emit(SignalVamTime, 1);
				if (++mGenVamLowDynamicsCounter >= mGenVamLowDynamicsLimit)
				{
					T_GenVam = T_GenVamMax;
				}
			}
		}
	}else if(mClusterManagement.vruShouldLeave_ElapsedTime(T_now)){
		setVruClusterOperation(VruShouldLeave);
		sendVam(T_now);
	}
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


void VaService::setVruClusterOperation(VruClusterOperation newVruClusterOperation){
	currentVruClusterOperation = newVruClusterOperation;
}

void VaService::retrieveInformationFromVAM(const vanetza::asn1::Vam &vam)
{
	VruAwareness_t vam_data = (*vam).vam;

	auto vruLon = vam_data.vamParameters.basicContainer.referencePosition.longitude/std::pow(10.0, 7);
	auto vruLat = vam_data.vamParameters.basicContainer.referencePosition.latitude/std::pow(10.0, 7);

	/**
	* @note compute the position error from the leader in the message 
	*/

	omnetpp::SimTime T_now = simTime();

	// POSSIBLE USE CASES
	// - Standalone VRUs inspecting VAMs from Cluster Leaders -> JoinOp
	// - Passive VRUs inspecting VAMs from Cluster Leaders -> LeaveOp
	// - Cluster Leader checks VAMs from other Leaders for ID uniqueness

	if (mClusterSupport){
		if(vam_data.vamParameters.vruClusterInformationContainer){
			if(mClusterManagement.isVruStandalone() && mClusterManagement.vruShouldJoin(vam)){
				mTargetClusterId = vam_data.vamParameters.vruClusterInformationContainer->clusterId;
				setVruClusterOperation(VruShouldJoin);
			}else if(mClusterManagement.isVruPassive() && vam_data.vamParameters.vruClusterInformationContainer->clusterId == mTargetClusterId){
				if(mClusterManagement.vruShouldLeave_VamParameters(vam)){
					setVruClusterOperation(VruShouldLeave);
				}

				mClusterManagement.mLastLeaderVamTimestamp = T_now;
			}
		}

		// POSSIBLE USE CASES
		// - Cluster Leader looking for Join/Leave VAMs
		// - Passive VRU checking for Breakup warnings from the Leader
		if(vam_data.vamParameters.vruClusterOperationContainer){	
			if (vam_data.vamParameters.vruClusterOperationContainer->clusterJoinInfo){		
				if(mClusterManagement.isVruClusterLeader() && vam_data.vamParameters.vruClusterOperationContainer->clusterJoinInfo->clusterId == mClusterManagement.getClusterId()){
					mClusterManagement.updateVruClusterMap(vam, "include_VRU");
					emit(SignalVamReceivedJoinID,vam->header.stationID);
					for (auto const& itr : mClusterManagement.getVruClusterMap()){						
						emit(SignalClusterMember, itr.first);
					}
					setVruClusterOperation(AddVruToCluster);
				}
			}else if (vam_data.vamParameters.vruClusterOperationContainer->clusterLeaveInfo){		
				if(mClusterManagement.isVruClusterLeader() && vam_data.vamParameters.vruClusterOperationContainer->clusterLeaveInfo->clusterId == mClusterManagement.getClusterId()){
					mClusterManagement.updateVruClusterMap(vam, "remove_VRU");
					if(mClusterManagement.getCurrentClusterSize() < mClusterManagementParameters.MinClusterSize){
						setVruClusterOperation(BreakupCluster);
					}else{
						setVruClusterOperation(RemoveVruFromCluster);
					}
				}
			}else if(vam_data.vamParameters.vruClusterOperationContainer->clusterBreakupInfo){
				if(mClusterManagement.isVruPassive() && vam_data.vamParameters.vruClusterInformationContainer->clusterId == mTargetClusterId){
					//mPersonDataProvider->setStationId(Identity::randomStationId(getRNG(0)));
					setVruClusterOperation(VruShouldLeave);
				}
			}
		}else{
			if(mClusterManagement.isVruClusterLeader()){
				setVruClusterOperation(KeepCluster);
			}
		}
	}
}
	
bool VaService::checkHeadingDelta() const
{
	return !vanetza::facilities::similar_heading(mLastVamHeading, mPersonDataProvider->heading(), mHeadingDelta);
}

bool VaService::checkPositionDelta() const
{
	return (distance(mLastVamPosition, mPersonDataProvider->position()) > mPositionDelta);
}

bool VaService::checkSpeedDelta() const
{
	return abs(mLastVamSpeed - mPersonDataProvider->speed()) > mSpeedDelta;
}


void VaService::sendVam(const SimTime& T_now)
{	
	uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mPersonDataProvider->updated()));
	auto vam = createVRUAwarenessMessage(*mPersonDataProvider, genDeltaTimeMod);
	mLastVamPosition = mPersonDataProvider->position();
	mLastVamSpeed = mPersonDataProvider->speed();

	mLastVamHeading = mPersonDataProvider->heading();
	mLastVamTimestamp = T_now;
	if (T_now - mLastLowVamTimestamp >= artery::simtime_cast(scLowFrequencyContainerInterval)) {
		//addLowFrequencyContainer(vam, par("pathHistoryLength"));
		mLastLowVamTimestamp = T_now;
	}

	// New InfoContainer: Cluster creation 

	if (mClusterSupport){
		if(mClusterManagement.isVruStandalone() && currentVruClusterOperation == noVruOperation){		
			mClusterManagement.createVruClusterMap();
			if(mClusterManagement.vruShouldCreateCluster()){
				mClusterManagement.mClusterCreationTimestamp = T_now;
				mClusterManagement.setCurrentVruRole(mClusterManagement.VruActiveClusterLeader);
				auto clusterCreationID = omnetpp::intuniform(getRNG(0), 1, 255);
				auto initialInfoContainerData = mClusterManagement.generateDataForNewCluster(clusterCreationID);
				addClusterInformationContainer(vam, initialInfoContainerData);
				emit(SignalVamClusterCreation, clusterCreationID);
				//clusterVamSentCounter++;
				previousContainerData = initialInfoContainerData;
				emit(scSignalBoxArea, mClusterManagement.getBoundingBoxArea());
				emit(scSignalBoxPerimeter, mClusterManagement.getBoundingBoxPerimeter());
				emit(scClusterCardinality, mClusterManagement.minClusterSize());
			}
		}


		// Update InfoContainer: Cluster size alteration
		if(mClusterManagement.isVruClusterLeader()){
			if(currentVruClusterOperation == AddVruToCluster || currentVruClusterOperation == RemoveVruFromCluster){
				VruClusterMap_t clusterMap = mClusterManagement.getVruClusterMap();
				// TODO: How should we treat clusters with size 2?
				if(mClusterManagement.getCurrentClusterSize() >= mClusterManagementParameters.NumCreateCluster){
					auto clusterInfoData = mClusterManagement.generateDataForCurrentCluster();
					emit(scSignalBoxArea,mClusterManagement.getBoundingBoxArea());
					emit(scSignalBoxPerimeter,mClusterManagement.getBoundingBoxPerimeter());
					emit(scClusterCardinality, mClusterManagement.getCurrentClusterSize());
					addClusterInformationContainer(vam, clusterInfoData);
					//clusterVamSentCounter++;
					emit(SignalVamCluster,1);
					previousContainerData = clusterInfoData;
				}
			} else if(currentVruClusterOperation == KeepCluster){
				emit(scSignalBoxArea, mClusterManagement.getBoundingBoxArea());
				emit(scSignalBoxPerimeter, mClusterManagement.getBoundingBoxPerimeter());
				emit(scClusterCardinality, mClusterManagement.getCurrentClusterSize());
				addClusterInformationContainer(vam, previousContainerData);
				//clusterVamSentCounter++;
				emit(SignalVamCluster,1);
			}
		}

		// Operation Container to join a Cluster
		if (mClusterManagement.isVruStandalone() && currentVruClusterOperation == VruShouldJoin){
			addClusterJoinOperationContainer(vam, T_now < 1 ? 1: T_now);
			emit(SignalVamJoin, mTargetClusterId);
		}

		if (mClusterManagement.isVruPassive() && currentVruClusterOperation == VruShouldLeave){
			addClusterLeaveOperationContainer(vam);
			emit(SignalVamLeave, mTargetClusterId);
		}

		if(mClusterManagement.isVruClusterLeader() && currentVruClusterOperation == BreakupCluster){
			emit(scSignalBoxArea, mClusterManagement.getBoundingBoxArea());
			emit(scSignalBoxPerimeter, mClusterManagement.getBoundingBoxPerimeter());
			emit(scClusterCardinality, mClusterManagement.getCurrentClusterSize());
			addClusterInformationContainer(vam, previousContainerData);
			addClusterBreakupOperationContainer(vam);
			emit(SignalVamBreak, 1);
		}
	}

	using namespace vanetza;
	btp::DataRequestB request;
	request.destination_port = btp::ports::VAM;
	request.gn.its_aid = aid::VRU;
	request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP0));
	request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

	VaObject obj(std::move(vam));
	emit(scSignalVamSent, &obj);

	//vamSentCounter ++;
	emit (SignalVamSent, 1);

	using VamByteBuffer = convertible::byte_buffer_impl<asn1::Vam>;
	std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
	std::unique_ptr<convertible::byte_buffer> buffer { new VamByteBuffer(obj.shared_ptr()) };
	payload->layer(OsiLayer::Application) = std::move(buffer);
	this->request(request, std::move(payload));	
}

SimTime VaService::genVamDcc()
{
	// network interface may not be ready yet during initialization, so look it up at this later point
	auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
	vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
	if (!trc) {
		throw cRuntimeError("No DCC TRC found for VA's primary channel %i", mPrimaryChannel);
	}
	
	static const vanetza::dcc::TransmissionLite va_tx(vanetza::dcc::Profile::DP0, 0);
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

void addClusterInformationContainer(vanetza::asn1::Vam& message, InformationContainerData_t& infoContainerData){

	VruAwareness_t &vam = (*message).vam;
	VruClusterInformationContainer_t* &vcic = vam.vamParameters.vruClusterInformationContainer;
	vcic = vanetza::asn1::allocate<VruClusterInformationContainer_t>();

	vcic->clusterId = infoContainerData.clusterId;

	if(infoContainerData.shape == "Circular"){
		vcic->clusterBoundingBoxShape.present = ClusterBoundingBoxShape_PR_clusterCircle;
		vcic->clusterBoundingBoxShape.choice.clusterCircle.radius = infoContainerData.circleRadius;
		vcic->clusterBoundingBoxShape.choice.clusterCircle.nodeCenterPoint = selectNodeType(infoContainerData.nodeOffset);

	}else if(infoContainerData.shape == "Rectangular"){
		vcic->clusterBoundingBoxShape.present = ClusterBoundingBoxShape_PR_clusterRectangle;
		vcic->clusterBoundingBoxShape.choice.clusterRectangle.nodeCenterPoint = selectNodeType(infoContainerData.nodeOffset);
		vcic->clusterBoundingBoxShape.choice.clusterRectangle.semiMinorRangeLength = infoContainerData.rectangleWidth;
		vcic->clusterBoundingBoxShape.choice.clusterRectangle.semiMajorRangeLength = infoContainerData.rectangleLength;
		vcic->clusterBoundingBoxShape.choice.clusterRectangle.semiMajorRangeOrientation = infoContainerData.orientationAngle;

	}else if(infoContainerData.shape == "Polygon"){
		vcic->clusterBoundingBoxShape.present = ClusterBoundingBoxShape_PR_clusterPolygon;
		for (auto &offsetValue : infoContainerData.polyOffsets){
			ASN_SEQUENCE_ADD(&vcic->clusterBoundingBoxShape.choice.clusterPolygon.polyPointList, selectNodeType(offsetValue));
		}
	}

	vcic->clusterCardinalitySize = infoContainerData.cardinalitySize;

	vcic->clusterProfiles.buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
	vcic->clusterProfiles.size = 1;
	vcic->clusterProfiles.bits_unused = 0;
	vcic->clusterProfiles.buf[0] = static_cast<uint8_t>(e_ClusterProfiles::ClusterProfiles_pedestrian);

}

OffsetPoint* selectNodeType(std::tuple<long,long> offsetTuple){

	OffsetPoint* offsetPoint = vanetza::asn1::allocate<OffsetPoint>();

	long absOffsetX = abs(std::get<0>(offsetTuple));
	long absOffsetY = abs(std::get<1>(offsetTuple));
 	long largestOffsetValue = absOffsetX > absOffsetY ? absOffsetX : absOffsetY;

	// Offset constraints given in cm
	if(largestOffsetValue <= 511L){
		offsetPoint->nodeOffsetPointxy.present = NodeOffsetPointXY_PR_node_XY1;
		offsetPoint->nodeOffsetPointxy.choice.node_XY1.x = std::get<0>(offsetTuple);
		offsetPoint->nodeOffsetPointxy.choice.node_XY1.y = std::get<1>(offsetTuple);
	}else if(largestOffsetValue <= 1023L){
		offsetPoint->nodeOffsetPointxy.present = NodeOffsetPointXY_PR_node_XY2;
		offsetPoint->nodeOffsetPointxy.choice.node_XY2.x = std::get<0>(offsetTuple);
		offsetPoint->nodeOffsetPointxy.choice.node_XY2.y = std::get<1>(offsetTuple);
	}else if(largestOffsetValue <= 2047L){
		offsetPoint->nodeOffsetPointxy.present = NodeOffsetPointXY_PR_node_XY3;
		offsetPoint->nodeOffsetPointxy.choice.node_XY3.x = std::get<0>(offsetTuple);
		offsetPoint->nodeOffsetPointxy.choice.node_XY3.y = std::get<1>(offsetTuple);
	}else if(largestOffsetValue <= 4095L){
		offsetPoint->nodeOffsetPointxy.present = NodeOffsetPointXY_PR_node_XY4;
		offsetPoint->nodeOffsetPointxy.choice.node_XY4.x = std::get<0>(offsetTuple);
		offsetPoint->nodeOffsetPointxy.choice.node_XY4.y = std::get<1>(offsetTuple);
	}else if(largestOffsetValue <= 8191L){
		offsetPoint->nodeOffsetPointxy.present = NodeOffsetPointXY_PR_node_XY5;
		offsetPoint->nodeOffsetPointxy.choice.node_XY5.x = std::get<0>(offsetTuple);
		offsetPoint->nodeOffsetPointxy.choice.node_XY5.y = std::get<1>(offsetTuple);
	}else if(largestOffsetValue <= 32767L){
		offsetPoint->nodeOffsetPointxy.present = NodeOffsetPointXY_PR_node_XY6;
		offsetPoint->nodeOffsetPointxy.choice.node_XY6.x = std::get<0>(offsetTuple);
		offsetPoint->nodeOffsetPointxy.choice.node_XY6.y = std::get<1>(offsetTuple);
	}
	else{
		offsetPoint->nodeOffsetPointxy.present = NodeOffsetPointXY_PR_NOTHING;
	}

	return offsetPoint;
}

void VaService::addClusterJoinOperationContainer(vanetza::asn1::Vam& message, const SimTime& T_now){
	omnetpp::SimTime joinTime = T_now;
	VruClusterOperationContainer*& vcoc = message->vam.vamParameters.vruClusterOperationContainer;
	vcoc = vanetza::asn1::allocate<VruClusterOperationContainer>();

	ClusterJoinInfo*& joinInfo = vcoc->clusterJoinInfo; 
	joinInfo = vanetza::asn1::allocate<ClusterJoinInfo>();
	joinInfo->clusterId = mTargetClusterId;
	joinInfo->joinTime = joinTime.dbl();
	setVruClusterOperation(noVruOperation);
	mClusterManagement.setCurrentVruRole(mClusterManagement.VruPassive);	
}

void VaService::addClusterLeaveOperationContainer(vanetza::asn1::Vam& message){
	VruClusterOperationContainer*& vcoc = message->vam.vamParameters.vruClusterOperationContainer;
	vcoc = vanetza::asn1::allocate<VruClusterOperationContainer>();

	ClusterLeaveInfo*& leaveInfo = vcoc->clusterLeaveInfo;
	leaveInfo = vanetza::asn1::allocate<ClusterLeaveInfo>();
	leaveInfo->clusterId = mTargetClusterId;
	leaveInfo->clusterLeaveReason = mClusterManagement.getLeaveReason();
	emit(SignalLeaveReasonCounter, mClusterManagement.getLeaveReason());
	setVruClusterOperation(noVruOperation);
	mClusterManagement.setCurrentVruRole(mClusterManagement.VruActiveStandalone);
}

void VaService::addClusterBreakupOperationContainer(vanetza::asn1::Vam& message){
	VruClusterOperationContainer*& vcoc = message->vam.vamParameters.vruClusterOperationContainer;
	vcoc = vanetza::asn1::allocate<VruClusterOperationContainer>();

	ClusterBreakupInfo*& breakupInfo = vcoc->clusterBreakupInfo;
	breakupInfo = vanetza::asn1::allocate<ClusterBreakupInfo>();
	breakupInfo->breakupTime = mClusterManagementParameters.TimeClusterBreakupWarning.dbl();
	breakupInfo->clusterBreakupReason = ClusterBreakupReason_notProvided;

	emit(SignalBreakupReasonCounter,1);

	mClusterManagement.clearVruClusterMap();
	setVruClusterOperation(noVruOperation);
	mClusterManagement.setCurrentVruRole(mClusterManagement.VruActiveStandalone);
}

} // namespace artery
