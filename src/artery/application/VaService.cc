#include "artery/application/VaObject.h"
#include "artery/application/VaService.h"
#include "artery/application/Constants.hpp"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/PersonDataProvider.h"
#include "artery/traci/PersonController.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/asn1/vam.hpp>
#include <chrono>


namespace artery
{

using namespace omnetpp;


static const simsignal_t scSignalVamReceived = cComponent::registerSignal("VamReceived");
static const simsignal_t scSignalVamSent = cComponent::registerSignal("VamSent");
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
		mGenVamMax { 1000, SIMTIME_MS },
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
}


void VaService::trigger()
{
	Enter_Method("trigger");
	checkTriggeringConditions(simTime());
//	personSumoData(simTime());
}

void VaService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
	Enter_Method("indicate");

	Asn1PacketVisitor<vanetza::asn1::Vam> visitor;
	const vanetza::asn1::Vam* vam = boost::apply_visitor(visitor, *packet);
	if (vam && vam->validate()) {
		VaObject obj = visitor.shared_wrapper;
		emit(scSignalVamReceived, &obj);
		//mLocalDynamicMap->updateAwareness(obj);
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
			sendVam(T_now);			
		} else if (T_elapsed >= T_GenVam) {
			sendVam(T_now);
			if (++mGenVamLowDynamicsCounter >= mGenVamLowDynamicsLimit) {
				T_GenVam = T_GenVamMax;
			}
		}
	}
}

/*
void VaService::personSumoData(const SimTime& simulationTimeNow)
{	
	mPersonController = &getFacilities().get_mutable<traci::PersonController>();

	if (mPersonController->getPersonRoadID() == ":IN_6020_c1"){		
		mPersonController->setSpeed(3 * vanetza::units::si::meter_per_second);
	}
}*/


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
	//mLastVamHeading = mPersonDataProvider->heading();
	mLastVamTimestamp = T_now;
	if (T_now - mLastLowVamTimestamp >= artery::simtime_cast(scLowFrequencyContainerInterval)) {
		//addLowFrequencyContainer(vam, par("pathHistoryLength"));
		mLastLowVamTimestamp = T_now;
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
		throw cRuntimeError("No DCC TRC found for CA's primary channel %i", mPrimaryChannel);
	}

	static const vanetza::dcc::TransmissionLite va_tx(vanetza::dcc::Profile::DP2, 0);
	vanetza::Clock::duration interval = trc->interval(va_tx);
	SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
	return std::min(mGenVamMax, std::max(mGenVamMin, dcc));
}

vanetza::asn1::Vam createVRUAwarenessMessage(const PersonDataProvider& vdp, uint16_t genDeltaTime)
{
	vanetza::asn1::Vam message;

	ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 2;
	header.messageID = ItsPduHeader__messageID_vam;
	header.stationID = vdp.station_id();

	VruAwareness_t& vam = (*message).vam;
	vam.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
	BasicContainer_t& basic = vam.vamParameters.basicContainer;

	basic.stationType = StationType_pedestrian;
	basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
	basic.referencePosition.longitude = Constants::round(vdp.longitude(), Constants::microdegree) * Longitude_oneMicrodegreeEast;
	basic.referencePosition.latitude = Constants::round(vdp.latitude(), Constants::microdegree) * Latitude_oneMicrodegreeNorth;
	basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
			SemiAxisLength_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
			SemiAxisLength_unavailable;

	
	//High Frequency Container
	
	VruHighFrequencyContainer*& hfc = message->vam.vamParameters.vruHighFrequencyContainer;
	hfc = vanetza::asn1::allocate<VruHighFrequencyContainer>();
	
	hfc->heading.headingValue = Constants::round(vdp.heading(), Constants::decidegree);
	hfc->heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
	hfc->speed.speedValue = buildVAMSpeedValue (vdp.speed());
	hfc->speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;

	const double lonAccelValue = vdp.acceleration() / vanetza::units::si::meter_per_second_squared;
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



} // namespace artery
