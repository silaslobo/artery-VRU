#include "artery/application/CPObject.h"
#include "artery/application/CPService.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/PersonDataProvider.h"
#include "artery/traci/ObjIdentification.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include "artery/inet/InetMobility.h"
#include "artery/application/MultiChannelPolicy.h"
#include "DCCFacility.h"
#include "artery/application/Constants.hpp"
#include "artery/envmod/sensor/Sensor.h"
#include <limits.h>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <vanetza/asn1/cam.hpp>
#include <memory>
#include "artery/envmod/sensor/FovSensor.h"
#include "artery/application/StationType.h"

#include <chrono>
#include <artery/cpm/compiled/ListOfVehicleSensorProperties.h>
#include <artery/networking/LimericDccEntity.h>


namespace artery {

using namespace omnetpp;

static const simsignal_t scSignalCpmReceived = cComponent::registerSignal("CpmReceived");
static const simsignal_t scSignalCpmSent = cComponent::registerSignal("CpmSent");
static const simsignal_t CamReceivedSignal = cComponent::registerSignal("CamReceived");


/** Signals for statistics collection purpose. */
static const simsignal_t scSignalRatioFilter = cComponent::registerSignal("ratioFilter");
static const simsignal_t scSignalDccTime = cComponent::registerSignal("delayDCC");
static const simsignal_t scSignalDist = cComponent::registerSignal("distCovered");
static const simsignal_t scSignalHeading = cComponent::registerSignal("headingVariation");
static const simsignal_t scSignalSpeed = cComponent::registerSignal("SpeedVariation");
static const simsignal_t scSignalLimericDelta = cComponent::registerSignal("limericDelta");
static const simsignal_t scSignalTonpp = cComponent::registerSignal("Tonpp");
static const simsignal_t scSignalRatioRessourceUsed = cComponent::registerSignal("RatioRessourceUsed");
static const simsignal_t scSignalRatioObjectAge = cComponent::registerSignal("objectAge");
static const simsignal_t scSignalDeltaPositionObject = cComponent::registerSignal("deltaPositionObject");
static const simsignal_t scSignalRemovedObjExcessiveSize = cComponent::registerSignal("removedObjExcessiveSize");
static const simsignal_t scSignalResourceNotUsed = cComponent::registerSignal("resourceNotUsed");

/** DCC Profile for the CP service.
* @note CA is DP2, CP supposed to be either DP2 or DP3
* @note If DP3 -> CAM privilidged on CPM and CPM will face higher drop rate in case of congestion
*/
const auto DCCPROFILECP = vanetza::dcc::Profile::DP2;

const long GENERATIONDELTATIMEMAX = 65535;
const long TIMEOFMEASUREMENTMAX = 1500;
const size_t MAXCPMSIZE = 1100;


Define_Module(CPService)


CPService::CPService() :
        mGenCpmMin{100, SIMTIME_MS},
        mGenCpmMax{1000, SIMTIME_MS},
        mGenCpm(mGenCpmMax),
        mGenCpmLowDynamicsCounter(0),
        mGenCpmLowDynamicsLimit(3),
        mCPSensor(nullptr),
        mCASensor(nullptr) {
}


CPService::~CPService() {
}


void CPService::initialize() {
    ItsG5BaseService::initialize();
    mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
    //mPersonDataProvider = &getFacilities().get_const<PersonDataProvider>();
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();

    mStationId = mVehicleDataProvider->station_id();
    /** @note Only for debugging purpose */
    WATCH(mStationId);

    mTimer = &getFacilities().get_const<Timer>();
    // Avoid unreasonable high elapsed time values for newly inserted vehicles
    mLastCpmTimestamp = simTime();
    mLastSensorInfoCont = simTime();

    // Generation rate boundaries
    mGenCpmMin = par("minInterval");
    mGenCpmMax = par("maxInterval");

    // Vehicle dynamics thresholds
    mHeadingDelta = vanetza::units::Angle{par("headingDelta").doubleValue() * vanetza::units::degree};
    mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
    mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;

    mDccAlg = par("dccAlgorithm").stdstringValue();

    // Generation rules
    mTriggeringCondition = par("triggeringCondition").stdstringValue();
    mDccRestriction = par("withDccRestriction");
    mFixedRate = par("fixedRate");

    mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::CP);

    mLocalEnvironmentModel = &getFacilities().get_mutable<LocalEnvironmentModel>();

    // Objects filters
    mFiltersEnabled = std::vector<bool>{par("v2xCapabilities"), par("objectDynamicsLocal"),
                                        par("objectDynamicsV2X"),
                                        par("fovSensors"), par("perceptionQuality"), par("updatingTime"),
                                        par("etsiFilter")};

    mFilterObj.initialize(mPersonDataProvider, mLocalEnvironmentModel, mFiltersEnabled, mHeadingDelta,
                          mPositionDelta, mSpeedDelta, &mSensorsId, mGenCpmMin, mGenCpmMax);

    mOtherDynamicThresh = par("otherDynamicThresh");

    mUseMatchingSize = par("useMatchingSize");
    mUseThreshMatchingSize =  par("useThreshMatchingSize");
    mThreshMatchingSize =  par("threshMatchingSize");

    mStartFilteringThresh =  par("startFilteringThresh");
    mGenerateForSensor = par("generateForSensor");
    mPreFilterLowDynamics = par("preFilterLowDynamics");

    //@note
    this->subscribe(CamReceivedSignal);
}


void CPService::trigger() {
    Enter_Method("trigger");
    //Can't intialize in initialize() as the sensor vector is not fulfilled yet
    if (mSensorsId.empty())
        fulfillmSensorsId();

    removeExpiredObject();
    collectObjectAge();

    //Stat: get the delta position, heading and speed SINCE LAST TRANSMISSION
    //Avoid the first values when created
    if (mInitied) {
        emit(scSignalDist, distance(mLastCpmPosition, mVehicleDataProvider->position()) / boost::units::si::meter);
        vanetza::units::Angle angleDiff = abs(
                computeDifference<vanetza::units::Angle>(mLastCpmHeading, mVehicleDataProvider->heading(),
                                                         vanetza::units::Angle(10.0 * vanetza::units::degree),
                                                         vanetza::units::Angle(360.0 * vanetza::units::degree)));
        emit(scSignalHeading, angleDiff.value());
        emit(scSignalSpeed,
             abs(mLastCpmSpeed - mVehicleDataProvider->speed()) / vanetza::units::si::meter_per_second);
    }


    if (mDccAlg == "LimericDccEntity") {
        vanetza::UnitInterval dutyCycleLimit = getLimericDelta();
        emit(scSignalLimericDelta, dutyCycleLimit.value());
    }

    //TODO remove
    //std::cout << "Current time: " << simTime() << "  Last delta update" << getLimericLastDeltaUpdatingTime() << std::endl;
    //std::cout << "Now: " << simTime() << "\tLast delta update: " << getLimericLastDeltaUpdatingTime() << std::endl;
    checkTriggeringConditions(simTime());
    //InfoObject::printObjectsReceivedMap(mObjectsReceived);
}


void CPService::removeExpiredObject(){

    for (auto it = mObjectsReceived.begin(); it != mObjectsReceived.end();){

        if (it->second.getLastTrackingTime().last() + mCPSensor->getValidityPeriod() < mPersonDataProvider->updated()) {
            it = mObjectsReceived.erase(it);    // or "it = m.erase(it)" since C++11
        }
        else
            ++it;
    }
}


void CPService::indicate(const vanetza::btp::DataIndication &ind, std::unique_ptr<vanetza::UpPacket> packet) {
    Enter_Method("indicate");

    //Can't intialize in initialize() as the sensor vector is not fulfilled yet
    if (mSensorsId.empty())
        fulfillmSensorsId();

    Asn1PacketVisitor<artery::cpm::Cpm> visitor;
    const artery::cpm::Cpm *cpm = boost::apply_visitor(visitor, *packet);

    if (cpm && cpm->validate()) {
        //printCPM(*cpm);
        CPObject obj = visitor.shared_wrapper;
        retrieveInformationFromCPM(cpm);
        //InfoObject::printObjectsReceivedMap(mObjectsReceived);

        emit(scSignalCpmReceived, &obj);
    }
}


//TODO avoid creation of object everytime -> needed?
/** Create objects with data required for filtering (heading, position, speed, timestamp, etc..) from the CPM received
* @param: asnc struct of CPM
* @return: /
*/
void CPService::retrieveInformationFromCPM(const artery::cpm::Cpm *cpm) {

    artery::cpm::Cpm cpm_data = (*cpm);

    //Get info of the emitter vehicle
    uint32_t stationID = cpm_data->header.stationID;
    omnetpp::SimTime generationTime = mTimer->getTimeFor(
            mTimer->reconstructMilliseconds(cpm_data->cpm.generationDeltaTime));

    if (mObjectsReceived.find(stationID) == mObjectsReceived.end() || //First time object perceived
        mObjectsReceived.at(stationID).getLastTrackingTime().last() + mCPSensor->getValidityPeriod() <= simTime() ||
        //Object is expired
        generationTime >
        mObjectsReceived.at(stationID).getLastTrackingTime().last()) { // the CPM received is more recent

        OriginatingVehicleContainer_t originVeh = cpm_data->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer;
        LocalEnvironmentModel::TrackingTime newTracking(generationTime);

        //Retrieve heading, position and velocity
        vanetza::units::Angle headingReceived(originVeh.heading.headingValue * Constants::decidegree);

        /** @note For simplicity, in management container, the position (x,y) is given instead of (longitude, latitude) */
        Position posReceivedStation(
                (double) cpm_data->cpm.cpmParameters.managementContainer.referencePosition.longitude /
                DistanceValue_oneMeter,
                -(double) cpm_data->cpm.cpmParameters.managementContainer.referencePosition.latitude /
                DistanceValue_oneMeter);

        vanetza::units::Velocity speedReceived(originVeh.speed.speedValue * Constants::centimeter_per_second);


        if (mObjectsReceived.find(stationID) != mObjectsReceived.end()) {


            /*auto dist = distance(posReceivedStation, mObjectsReceived[stationID].getLastPosition()) /
                        boost::units::si::meter;
            if(dist > 20.0){
                std::cout << "Distance computed " << simTime() << " " << dist << std::endl;
                std::cout << "Station " << stationID << std::endl;
                std::cout << "Position received (" << posReceivedStation.x / boost::units::si::meter << " " << posReceivedStation.y / boost::units::si::meter << std::endl;
                std::cout << "Position previous (" << mObjectsReceived[stationID].getLastPosition().x / boost::units::si::meter << " " << mObjectsReceived[stationID].getLastPosition().y / boost::units::si::meter << std::endl;
                std::cout <<  mObjectsReceived[stationID] << std::endl;
            }*/

            //TODO remove when found out why
            /*if(distance(posReceivedStation, mObjectsReceived[stationID].getLastPosition()) /
               boost::units::si::meter >= 50.0){
                auto dist = distance(posReceivedStation, mObjectsReceived[stationID].getLastPosition()) /
                            boost::units::si::meter;
                std::cout << "Problem with object: " << std::endl;
                std::cout << "Distance with object is: " << dist << std::endl;
                std::cout << "Last tracking time: " << mObjectsReceived[stationID].getLastTrackingTime().last() << std::endl;
                std::cout << "Last vehicle update: " << mVehicleDataProvider->updated() << std::endl;
            }

            assert(distance(posReceivedStation, mObjectsReceived[stationID].getLastPosition()) /
                   boost::units::si::meter < 50.0);
            */
            emit(scSignalDeltaPositionObject,
                 distance(posReceivedStation, mObjectsReceived[stationID].getLastPosition()) /
                 boost::units::si::meter);
        }

        mObjectsReceived[stationID] = InfoObject(true, newTracking, mSensorsId.at(mCPSensor), headingReceived, true,
                                                 posReceivedStation, speedReceived);
    }


    //Get info of the objects received:
    ListOfPerceivedObjectContainer_t *objectsContainer = cpm_data->cpm.cpmParameters.perceivedObjectContainer;
    for (int i = 0; objectsContainer != nullptr && i < objectsContainer->list.count; i++) {

        PerceivedObjectContainer *objCont = objectsContainer->list.array[i];

        /** @note Skip message received about myself */
        if (objCont->objectID == mPersonDataProvider->station_id()) {
            //std::cout << "Skip myself" << std::endl;
            continue;
        }
        

        omnetpp::SimTime objectPerceptTime = mTimer->getTimeFor(mTimer->reconstructMilliseconds(
                cpm_data->cpm.generationDeltaTime - objCont->timeOfMeasurement));

        if (mObjectsReceived.find(objCont->objectID) == mObjectsReceived.end() || //First time object perceived
            mObjectsReceived.at(objCont->objectID).getLastTrackingTime().last() + mCPSensor->getValidityPeriod() <=
            simTime() || //Object is expired
            objectPerceptTime > mObjectsReceived.at(
                    objCont->objectID).getLastTrackingTime().last()) { // the CPM received is more recent

            LocalEnvironmentModel::TrackingTime newTracking(objectPerceptTime);

            vanetza::units::Velocity speedX(objCont->xSpeed.value * Constants::centimeter_per_second);
            vanetza::units::Velocity speedY(objCont->ySpeed.value * Constants::centimeter_per_second);

            vanetza::units::Angle headingReceived = VehicleDataProvider::computeHeading(speedX, speedY);

            bool headingAvalaible = headingReceived != -1 * vanetza::units::si::radian;

            /** @note Change the axis to point to the south (OMNeT++ frame) */
            ReferencePosition_t refPosSender = cpm_data->cpm.cpmParameters.managementContainer.referencePosition;

            Position posReceived(
                    ((double) objCont->xDistance.value + refPosSender.longitude) / DistanceValue_oneMeter,
                    -((double) objCont->yDistance.value + refPosSender.latitude) / DistanceValue_oneMeter);

            vanetza::units::Velocity speedReceived = boost::units::sqrt(
                    boost::units::pow<2>(speedX) + boost::units::pow<2>(speedY));

            if (mObjectsReceived.find(objCont->objectID) == mObjectsReceived.end()) {
                mObjectsReceived[objCont->objectID] = InfoObject(false, newTracking, mSensorsId.at(mCPSensor),
                                                                 headingReceived, headingAvalaible, posReceived,
                                                                 speedReceived); //don't know if object has V2X capabilities, default is false
            } else {
                //TODO remove
                //auto dist = distance(posReceived, mObjectsReceived[objCont->objectID].getLastPosition()) /
                //            boost::units::si::meter;
               /* if(dist > 50.0){
                    std::cout << "Distance computed " << simTime() << " " << dist << std::endl;
                    std::cout << "Position received (" << posReceived.x / boost::units::si::meter << " " << posReceived.y / boost::units::si::meter << std::endl;
                    std::cout << "Position previous (" << mObjectsReceived[objCont->objectID].getLastPosition().x / boost::units::si::meter << " " << mObjectsReceived[objCont->objectID].getLastPosition().y / boost::units::si::meter << std::endl;
                }
                assert(distance(posReceived, mObjectsReceived[objCont->objectID].getLastPosition()) /
                       boost::units::si::meter < 50.0);
                */
                emit(scSignalDeltaPositionObject,
                     distance(posReceived, mObjectsReceived[objCont->objectID].getLastPosition()) /
                     boost::units::si::meter);

                mObjectsReceived[objCont->objectID] = InfoObject(
                        mObjectsReceived[objCont->objectID].getHasV2XCapabilities(),
                        newTracking, mSensorsId.at(mCPSensor), headingReceived,
                        headingAvalaible, posReceived, speedReceived);
            }
        }
    }
}


/**
* Depending the triggering policy, determine if a new message should be generated
*/
void CPService::checkTriggeringConditions(const SimTime &T_now) {
    // provide variables named like in EN 302 637-2 V1.3.2 (section 6.1.3)
    //std::cout << "Time to wait: " << genCpmDcc() << std::endl;
    bool messageGenerated = false;
    SimTime &T_GenCpm = mGenCpm;
    const SimTime &T_GenCpmMin = mGenCpmMin;
    const SimTime &T_GenCpmMax = mGenCpmMax;
    const SimTime T_GenCpmDcc = mDccRestriction ? genCpmDcc() : mGenCpmMin;
    const SimTime T_elapsed = T_now - mLastCpmTimestamp;

    /// @note Same as generation rules for the CA service
    if (mTriggeringCondition == "CamKinematics") {
        if (T_elapsed >= T_GenCpmDcc) {
            if (mFixedRate) {
                sendCpm(T_now);
                messageGenerated = true;
            } else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
                sendCpm(T_now);
                messageGenerated = true;
                T_GenCpm = std::min(T_elapsed, T_GenCpmMax); /*< if middleware update interval is too long */
                mGenCpmLowDynamicsCounter = 0;
            } else if (T_elapsed >= T_GenCpm) {
                sendCpm(T_now);
                messageGenerated = true;
                if (++mGenCpmLowDynamicsCounter >= mGenCpmLowDynamicsLimit) {
                    T_GenCpm = T_GenCpmMax;
                }
            }
        }
    }

        /// @note Fixed periodic transmission, still limited by DCC if DCC enabled
    else if (mTriggeringCondition == "PeriodicFixed") {
        if (T_elapsed >= T_GenCpmDcc) {
            sendCpm(T_now);
            messageGenerated = true;
        }
    }

        /// @note Correspond to the current standard development
    else if (mTriggeringCondition == "PeriodicIfObjToSend") {
        if (T_elapsed >= T_GenCpmDcc) {
            bool sensorTriggered = mGenerateForSensor & T_now - mLastSensorInfoCont >= SimTime(1, SIMTIME_S);
            if (isObjectToSend(T_now) || T_elapsed >= mGenCpmMax ||
                sensorTriggered) { // TODO: need this rule actually? || T_now - mLastSensorInfoCont >= SimTime(1, SIMTIME_S)
                //std::cout << "Send info at " << T_now << std::endl;
                sendCpm(T_now);
                messageGenerated = true;
            }
        }
    } else if (mTriggeringCondition == "LimericAdapted") {
        if (T_elapsed >= T_GenCpmDcc) {
            messageGenerated = sendLimericAdaptedCPM(T_now);
        }
    }
}


/**
* Check if the heading of the vehicle has changed more than mHeadingDelta parameter since last message generated
* @return True if a new message should be generated
*/
bool CPService::checkHeadingDelta() const {
    return !vanetza::facilities::similar_heading(mLastCpmHeading, mVehicleDataProvider->heading(), mHeadingDelta);
}

/**
* Check if the position of the vehicle has changed more than mPositionDelta parameter since last message generated
* @return True if a new message should be generated
*/
bool CPService::checkPositionDelta() const {
    return (distance(mLastCpmPosition, mVehicleDataProvider->position()) > mPositionDelta);
}

/**
* Check if the speed of the vehicle has changed more than mPositionDelta parameter since last message generated
* @return True if a new message should be generated
*/
bool CPService::checkSpeedDelta() const {
    return abs(mLastCpmSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;
}


/**
* Create CPM and define parameters for BTP and Geonet
* @param T_now: generation time of cpm
*/
void CPService::sendCpm(const SimTime &T_now) {
    uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
    auto cpm = createCollectivePerceptionMessage(genDeltaTimeMod);

    if (T_now - mLastSensorInfoCont >= SimTime(1, SIMTIME_S)) {
        //std::cout << "include CP sensors info " << T_now << std::endl;
        addSensorInformation(mLocalEnvironmentModel, cpm, mSensorsId);
        mLastSensorInfoCont = T_now;
    }
    //std::cout << "Time now: " << T_now << std::endl;
    //InfoObjet::printObjectsReceivedMap(mObjectsReceived);

    addPerceivedObjectContainer(mLocalEnvironmentModel, cpm, T_now);

    requestCPMTransmission(T_now, cpm);
}


void CPService::requestCPMTransmission(const SimTime &T_now, const artery::cpm::Cpm &cpm) {

    //Stat collection
    emit(scSignalDccTime, genCpmDcc());

    vanetza::Clock::duration t_on_pp = getTonpp(&cpm);
    emit(scSignalTonpp, std::chrono::duration_cast<std::chrono::microseconds>(t_on_pp).count());
    emit(scSignalRatioRessourceUsed, getRatioTimeAllowedUsed(&cpm));
    emit(scSignalResourceNotUsed, 0.1 - std::fmod(getDCCToff(), 0.1));

    mLastCpmPosition = mVehicleDataProvider->position();
    mLastCpmSpeed = mVehicleDataProvider->speed();
    mLastCpmHeading = mVehicleDataProvider->heading();
    mLastCpmTimestamp = T_now;

    mInitied = true;

    using namespace vanetza;
    btp::DataRequestB request;
    request.destination_port = btp::ports::CPM;
    request.gn.its_aid = aid::CP;
    request.gn.transport_type = geonet::TransportType::SHB; //Single Hop Broadcast
    request.gn.maximum_lifetime = geonet::Lifetime{geonet::Lifetime::Base::One_Second, 1};
    request.gn.traffic_class.tc_id(static_cast<unsigned>(DCCPROFILECP));
    request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    CPObject obj(std::move(cpm));
    emit(scSignalCpmSent, &obj);

    using CpmByteBuffer = convertible::byte_buffer_impl<artery::cpm::Cpm>;
    std::unique_ptr<geonet::DownPacket> payload{new geonet::DownPacket()};
    std::unique_ptr<convertible::byte_buffer> buffer{new CpmByteBuffer(obj.shared_ptr())};
    payload->layer(OsiLayer::Application) = std::move(buffer);

    this->request(request, std::move(payload));
}


/** Assign an unique id for each sensor of the vehicle (use in CPM) and check that some sensors are available
*  @param: /
*  @return: /
*/
void CPService::fulfillmSensorsId() {
    std::vector<Sensor *> sensors = mLocalEnvironmentModel->getSensors();

    //Check that at least some sensors are available and that some of them are for perception, i.e., radar.
    if (sensors.size() == 0 ||
        boost::size(pfilterBySensorCategory(mLocalEnvironmentModel->allPersonObjects(), "Radar")) == 0)
        EV_WARN << "No sensors for local perception currently used along the CP service" << std::endl;

    for (int i = 0; i < sensors.size(); i++) {
        mSensorsId.insert(std::pair<Sensor *, Identifier_t>(sensors[i], i));

        if (!mCPSensor && sensors[i]->getSensorCategory() == "CP")
            mCPSensor = sensors[i];

        if (!mCASensor && sensors[i]->getSensorCategory() == "CA")
            mCASensor = sensors[i];
    }
}


/**
* Create the CPM structure and fill it ith the management and station data container.
* @param vdp
* @param genDeltaTime
* @return the asn1 structure of the cpm
*/
artery::cpm::Cpm CPService::createCollectivePerceptionMessage(uint16_t genDeltaTime) {
    artery::cpm::Cpm message;

    ItsPduHeader_t &header = (*message).header;
    header.protocolVersion = 1;
    header.messageID = ItsPduHeader__messageID_cpm;
    header.stationID = mVehicleDataProvider->station_id();

    CollectivePerceptionMessage_t &cpm = (*message).cpm;
    cpm.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;

    CpmManagementContainer_t &managementContainer = cpm.cpmParameters.managementContainer;
    //StationDataContainer mandatory for vehicles
    StationDataContainer_t *&stationDataContainer = cpm.cpmParameters.stationDataContainer;
    stationDataContainer = vanetza::asn1::allocate<StationDataContainer_t>();

    managementContainer.stationType = (StationType_t) mVehicleDataProvider->getStationType();
    managementContainer.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
    managementContainer.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;

    /** @note To avoid useless computation, the cpm position is given in the omnetpp frame but with y pointing to the north */
    //managementContainer.referencePosition.longitude = round(mVehicleDataProvider->longitude(), Constants::microdegree) * Longitude_oneMicrodegreeEast;
    //managementContainer.referencePosition.latitude = round(mVehicleDataProvider->latitude(), Constants::microdegree) * Latitude_oneMicrodegreeNorth;
    managementContainer.referencePosition.longitude =
            mVehicleDataProvider->position().x / boost::units::si::meter * DistanceValue_oneMeter; // x values
    if(managementContainer.referencePosition.longitude > abs(1800000000)){
        std::cout << "!!! Error: longitude =  " << managementContainer.referencePosition.longitude << std::endl;
        managementContainer.referencePosition.longitude = 0;
    }

    /** Change the axis to point to the north (Standard frame) */
    managementContainer.referencePosition.latitude =
            -mVehicleDataProvider->position().y / boost::units::si::meter * DistanceValue_oneMeter; // y values

    if(managementContainer.referencePosition.latitude > abs(900000001)){
        std::cout << "!!! Error: Latitude =  " << managementContainer.referencePosition.latitude << std::endl;
        managementContainer.referencePosition.latitude = 0;
    }

    managementContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    managementContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
            SemiAxisLength_unavailable;
    managementContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
            SemiAxisLength_unavailable;

    stationDataContainer->present = StationDataContainer_PR_originatingVehicleContainer;
    OriginatingVehicleContainer_t &originVehCont = stationDataContainer->choice.originatingVehicleContainer;

    originVehCont.heading.headingValue = Constants::round(mVehicleDataProvider->heading(), Constants::decidegree);
    originVehCont.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
    originVehCont.speed.speedValue =
            Constants::round(mVehicleDataProvider->speed(), Constants::centimeter_per_second) *
            SpeedValue_oneCentimeterPerSec;
    originVehCont.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;

    //TODO more correct to check between orientation and heading?
    originVehCont.driveDirection = mVehicleDataProvider->speed().value() >= 0.0 ?
                                   DriveDirection_forward : DriveDirection_backward;

    const double lonAccelValue =
            mVehicleDataProvider->acceleration() / vanetza::units::si::meter_per_second_squared;
    // extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well)
    if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0) {
        LongitudinalAcceleration_t *&longitudinalAcceleration = originVehCont.longitudinalAcceleration;
        longitudinalAcceleration = vanetza::asn1::allocate<LongitudinalAcceleration_t>();
        longitudinalAcceleration->longitudinalAccelerationValue =
                lonAccelValue * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
        longitudinalAcceleration->longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
    }

    YawRate_t *&yawRate = originVehCont.yawRate;
    yawRate = vanetza::asn1::allocate<YawRate_t>();
    yawRate->yawRateValue = Constants::round(mVehicleDataProvider->yaw_rate(), Constants::degree_per_second) *
                            YawRateValue_degSec_000_01ToLeft * 100.0;
    if (yawRate->yawRateValue == LONG_MIN || abs(yawRate->yawRateValue) >= YawRateValue_unavailable) {
        yawRate->yawRateValue = YawRateValue_unavailable;
    }
    yawRate->yawRateConfidence = YawRateConfidence_unavailable;

    std::string error;
    if (!message.validate(error)) {
        throw cRuntimeError("Invalid Station Data Container in CPM: %s", error.c_str());
    }

    return message;
}


void CPService::addSensorInformation(LocalEnvironmentModel *&localEnvironmentModel, artery::cpm::Cpm &message,
                                     std::map<const Sensor *, Identifier_t> sensorsId) {
    CollectivePerceptionMessage_t &cpm = (*message).cpm;
    std::vector<Sensor *> sensors = localEnvironmentModel->getSensors();

    ListOfSensorInformationContainer_t *&seqSensInfCont = cpm.cpmParameters.sensorInformationContainer;
    seqSensInfCont = NULL;

    /*
    for (int i = 0; i < sensors.size(); i++) {
        if (sensors[i]->getSensorCategory() == "Radar") {
            createSensorInformationContainer(seqSensInfCont, sensors[i], sensorsId.at(sensors[i]),
                                             SensorType_radar);
        }
    }*/
}


void CPService::createSensorInformationContainer(ListOfSensorInformationContainer_t *&seqSensInfCont,
                                                 Sensor *&sensor, Identifier_t id, SensorType_t sensorType) {

    if (!seqSensInfCont) {
        seqSensInfCont = vanetza::asn1::allocate<ListOfSensorInformationContainer_t>();
    }

    SensorInformationContainer_t *sensorInfoCont = vanetza::asn1::allocate<SensorInformationContainer_t>();
    sensorInfoCont->id = id;
    sensorInfoCont->type = sensorType;

    sensorInfoCont->details.present = SensorDetails_PR_vehicleSensor;
    VehicleSensor_t &vehicleSensor = sensorInfoCont->details.choice.vehicleSensor;

    std::pair<long, long> positionPair = artery::relativePosition(sensor->position());

    vehicleSensor.refPointId = 0;
    vehicleSensor.xSensorOffset = positionPair.first;//positionPair.first / boost::units::si::meter;
    vehicleSensor.ySensorOffset = positionPair.second;//positionPair.second / boost::units::si::meter;

    //In our case only add 1 vehicle sensor properties for each sensor
    VehicleSensorProperties_t *vehicleSensorProp = vanetza::asn1::allocate<VehicleSensorProperties_t>();

    vehicleSensorProp->range = sensor->getFoV()->range.value() * Range_oneMeter;


    const double openingAngleDeg = sensor->getFoV()->angle / boost::units::degree::degrees;
    const double sensorPositionDeg = artery::relativeAngle1(sensor->position()) / boost::units::degree::degrees;

    //angle anti-clockwise
    vehicleSensorProp->horizontalOpeningAngleStart =
            std::fmod(std::fmod((sensorPositionDeg - 0.5 * openingAngleDeg),
                                (double) 360) + 360, 360) * CartesianAngleValue_oneDegree;
    vehicleSensorProp->horizontalOpeningAngleEnd = std::fmod(std::fmod((sensorPositionDeg + 0.5 * openingAngleDeg),
                                                                       (double) 360) + 360, 360) *
                                                   CartesianAngleValue_oneDegree;


    int result = ASN_SEQUENCE_ADD(&vehicleSensor.vehicleSensorProperties, vehicleSensorProp);
    if (result != 0) {
        perror("asn_set_add() failed");
        exit(EXIT_FAILURE);
    }


    result = ASN_SEQUENCE_ADD(seqSensInfCont, sensorInfoCont);
    if (result != 0) {
        perror("asn_set_add() failed");
        exit(EXIT_FAILURE);
    }
}


void CPService::addPerceivedObjectContainer(LocalEnvironmentModel *localEnvironmentModel, artery::cpm::Cpm &message,
                                            const omnetpp::SimTime &T_now) {
    if (mTriggeringCondition != "PeriodicIfObjToSend") {
        mObjectsToSend.clear();
        std::size_t countObject = mFilterObj.filterObjects(mObjectsToSend, mObjectsPrevSent, genCpmDcc(), mCPSensor,
                                                           mObjectsReceived, T_now);
    }

    generateASN1Objects(message, T_now, mObjectsToSend);
    checkCPMSize(T_now, mObjectsToSend, message);

    //Add object in the list of previously sent
    completeMyPrevObjSent(T_now, mObjectsToSend);

    //std::cout << "Send CPM with " << objectsToSend.size() << " objects" << std::endl;
    double nbRadarObj = (double) boost::size(pfilterBySensorCategory(mLocalEnvironmentModel->allPersonObjects(), "Radar"));
    if (nbRadarObj != 0)
        emit(scSignalRatioFilter, (double) 1 - (double) mObjectsToSend.size() / nbRadarObj);

}


void CPService::generateASN1Objects(artery::cpm::Cpm &message, const omnetpp::SimTime &T_now,
                                    InfoObject::ObjectsToSendMap objToSend) {

    //TODO: check for memory leaking here
    CollectivePerceptionMessage_t &cpm = (*message).cpm;
    ListOfPerceivedObjectContainer_t *&perceivedObjectContainers = cpm.cpmParameters.perceivedObjectContainer;
    vanetza::asn1::free(asn_DEF_ListOfPerceivedObjectContainer, perceivedObjectContainers);
    perceivedObjectContainers = nullptr;

    if (!objToSend.empty()) {
        perceivedObjectContainers = vanetza::asn1::allocate<ListOfPerceivedObjectContainer_t>();
        for (auto &obj : objToSend) {
            if (obj.first.expired()) continue;
            PerceivedObjectContainer_t *objContainer = createPerceivedObjectContainer(obj.first, obj.second, cpm);
            ASN_SEQUENCE_ADD(perceivedObjectContainers, objContainer);
        }
    }
}


PerceivedObjectContainer_t *
CPService::createPerceivedObjectContainer(const std::weak_ptr<artery::EnvironmentModelPerson> &object,
                                          InfoObject &infoObj,
                                          CollectivePerceptionMessage_t &cpm) {
    const auto &vdObj = object.lock()->getPersonData();
    

    PerceivedObjectContainer_t *objContainer = vanetza::asn1::allocate<PerceivedObjectContainer_t>();

    objContainer->objectID = vdObj.getStationId();
    objContainer->sensorID = new Identifier_t(infoObj.getSensorId());

    //Compute relative time between CPM generation and time of observation of the object
    //std::cout << "Time perception:" << (uint16_t) countTaiMilliseconds(mTimer->getTimeFor(infoObj.getLastTrackingTime().last())) << "\n";
    //std::cout << "cpm.generationDeltaTime:" << cpm.generationDeltaTime << "\n";
    //std::cout << "computeDifference:" << computeDifference<uint16_t> (cpm.generationDeltaTime, (uint16_t) countTaiMilliseconds(mTimer->getTimeFor(infoObj.getLastTrackingTime().last())), 1500, 65535) << "\n";

    /** @note: u_int16_t to take the modulo */
    objContainer->timeOfMeasurement = computeDifference<long>(cpm.generationDeltaTime,
                                                              (u_int16_t) countTaiMilliseconds(mTimer->getTimeFor(
                                                                      infoObj.getLastTrackingTime().last())),
                                                              TIMEOFMEASUREMENTMAX, GENERATIONDELTATIMEMAX);
    //Need to give relative position because the relative position is between (-132768..132767) cm
    //Change axis y from south to north
    objContainer->xDistance.value =
            ((vdObj.position().x - mVehicleDataProvider->position().x) / boost::units::si::meter) *
            DistanceValue_oneMeter;
    objContainer->xDistance.confidence = DistanceConfidence_oneMeter;
    objContainer->yDistance.value =
            -((vdObj.position().y - mVehicleDataProvider->position().y) / boost::units::si::meter) *
            DistanceValue_oneMeter;
    objContainer->yDistance.confidence = DistanceConfidence_oneMeter;

    
    /** @note: prevent teleportation **/
    if(abs(objContainer->xDistance.value) > 132767 || abs(objContainer->yDistance.value) > 132767){
        objContainer->xDistance.value = 0;
        objContainer->yDistance.value = 0;
    }

    /** @note xSpeed and ySpeed should be computed relatively to the ego speed. For simplicity, we consider the
     * speed of the vehicle detected directly.
     */
    const inet::Coord direction{sin(vdObj.heading()), cos(vdObj.heading())};
    inet::Coord speed =
            direction * (vdObj.speed() / vanetza::units::si::meter_per_second) * 100; //Conversion in cm/s
    objContainer->xSpeed.value = speed.x;
    objContainer->xSpeed.confidence = SpeedConfidence_equalOrWithinOneMeterPerSec;
    objContainer->ySpeed.value = speed.y;
    objContainer->ySpeed.confidence = SpeedConfidence_equalOrWithinOneMeterPerSec;

    if(abs(objContainer->xSpeed.value) > 16383 || abs(objContainer->ySpeed.value) > 16383){
        objContainer->xSpeed.value = 0;
        objContainer->ySpeed.value = 0;

    }

    objContainer->planarObjectDimension1 = vanetza::asn1::allocate<ObjectDimension_t>();
    objContainer->planarObjectDimension1->value =
            object.lock()->getLength() / boost::units::si::meter * ObjectDimensionValue_oneMeter;
    objContainer->planarObjectDimension1->confidence = 0;

    objContainer->planarObjectDimension2 = vanetza::asn1::allocate<ObjectDimension_t>();
    objContainer->planarObjectDimension2->value =
            object.lock()->getWidth() / boost::units::si::meter * ObjectDimensionValue_oneMeter;
    objContainer->planarObjectDimension2->confidence = 0;

    objContainer->dynamicStatus = vanetza::asn1::allocate<DynamicStatus_t>();
    *(objContainer->dynamicStatus) = DynamicStatus_dynamic;

    objContainer->classification = vanetza::asn1::allocate<StationType_t>();
    *(objContainer->classification) = StationType_pedestrian;

    return objContainer;
}


SimTime CPService::genCpmDcc() {
    // network interface may not be ready yet during initialization, so look it up at this later point
    auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
    vanetza::dcc::TransmitRateThrottle *trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
    if (!trc) {
        throw cRuntimeError("No DCC TRC found for CP's primary channel %i", mPrimaryChannel);
    }
    static const vanetza::dcc::TransmissionLite cp_tx(DCCPROFILECP, 0);
    vanetza::Clock::duration delay = trc->interval(cp_tx);
    SimTime dcc{std::chrono::duration_cast<std::chrono::milliseconds>(delay).count(), SIMTIME_MS};
    //TODO revove
    //std::cout << "time to wait before next transmission: " << dcc << std::endl;
    return std::min(mGenCpmMax, std::max(mGenCpmMin, dcc));
}


void CPService::receiveSignal(cComponent *, simsignal_t signal, cObject *obj, cObject *) {
    //Can't intialize in initialize() as the sensor vector is not fulfilled yet
    if (mSensorsId.empty())
        fulfillmSensorsId();

    if (signal == CamReceivedSignal) {
        auto *cam = dynamic_cast<CaObject *>(obj);
        if (cam) {
            //Get info of the emitter vehicle
            uint32_t stationID = cam->asn1()->header.stationID;
            omnetpp::SimTime generationTime = mTimer->getTimeFor(
                    mTimer->reconstructMilliseconds(cam->asn1()->cam.generationDeltaTime));

            if (mObjectsReceived.find(stationID) == mObjectsReceived.end() || //First time object perceived
                mObjectsReceived.at(stationID).getLastTrackingTime().last() + mCPSensor->getValidityPeriod() <
                simTime() || //Object is expired
                generationTime >
                mObjectsReceived.at(stationID).getLastTrackingTime().last()) { // the CAM received is more recent

                BasicVehicleContainerHighFrequency_t originVeh = cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
                LocalEnvironmentModel::TrackingTime newTracking(generationTime);

                //Retrieve heading, position and velocity
                vanetza::units::Angle headingReceived(originVeh.heading.headingValue * Constants::decidegree);

                /** @note For simplicity, in basicContainer container, the position (x,y) is given instead of (longitude, latitude) */
                Position posReceivedStation(
                        (double) cam->asn1()->cam.camParameters.basicContainer.referencePosition.longitude /
                        DistanceValue_oneMeter,
                        -(double) cam->asn1()->cam.camParameters.basicContainer.referencePosition.latitude /
                        DistanceValue_oneMeter);

                vanetza::units::Velocity speedReceived(
                        originVeh.speed.speedValue * Constants::centimeter_per_second);

                if (mObjectsReceived.find(stationID) != mObjectsReceived.end()) {

                   /* if(distance(posReceivedStation, mObjectsReceived[stationID].getLastPosition()) /

                       boost::units::si::meter >= 50.0){
                        auto dist = distance(posReceivedStation, mObjectsReceived[stationID].getLastPosition()) /
                                    boost::units::si::meter;
                        std::cout << "Problem with object: " << std::endl;
                        std::cout << "Distance with object is: " << dist << std::endl;
                        std::cout << "Last tracking time: " << mObjectsReceived[stationID].getLastTrackingTime().last() << std::endl;
                        std::cout << "Last vehicle update: " << mVehicleDataProvider->updated() << std::endl;
                    }

                    //assert(distance(posReceivedStation, mObjectsReceived[stationID].getLastPosition()) /
                    //       boost::units::si::meter < 50.0);
                    */
                    emit(scSignalDeltaPositionObject,
                         distance(posReceivedStation, mObjectsReceived[stationID].getLastPosition()) /
                         boost::units::si::meter);
                }
                mObjectsReceived[stationID] = InfoObject(true, newTracking, mSensorsId.at(mCASensor),
                                                         headingReceived, true,
                                                         posReceivedStation, speedReceived);
            }


        } else {
            EV_ERROR << "received signal has no CaObject";
        }
    }
}


bool CPService::isObjectToSend(const SimTime &T_now) {
    mObjectsToSend.clear();
    std::size_t countObject = mFilterObj.filterObjects(mObjectsToSend, mObjectsPrevSent, genCpmDcc(), mCPSensor,
                                                       mObjectsReceived, T_now);
    if (countObject > 0 && mObjectsToSend.empty()) {
        emit(scSignalRatioFilter, (double) 1.0);
    }
    return !mObjectsToSend.empty();
}

/** Print information of a CPM message
 * @param CPM struct from asnc
 * @return /
 */
void CPService::printCPM(const artery::cpm::Cpm &message) {
        const CPM_t &cpm = (*message);

        std::cout << "\n--- CPM at: " << simTime() << " ---" << std::endl;
        //Print header
        std::cout << "Header:\n\tprotocolVersion: " << cpm.header.protocolVersion
                  << "\n\tmessageID: " << cpm.header.messageID << "\n\tstationID: " << cpm.header.stationID
                  << std::endl;

        //Generation delta time
        std::cout << "generationDeltaTime: " << cpm.cpm.generationDeltaTime << std::endl;

        //CPM parameters
        std::cout << "-- CpmParameters --" << std::endl;

        //Management container
        CpmManagementContainer_t cpmManag = cpm.cpm.cpmParameters.managementContainer;
        std::cout << "CpmManagementContainer:\n\tstationType: " << cpmManag.stationType
                  << "\n\treferencePosition:\n\t\tlongitude: " << cpmManag.referencePosition.longitude
                  << "\n\t\tlatitude: " << cpmManag.referencePosition.latitude << std::endl;

        //Station data container
        StationDataContainer_t *cpmStationDC = cpm.cpm.cpmParameters.stationDataContainer;
        if (cpmStationDC) {
            std::cout << "StationDataContainer:\n\ttype: vehicle (fixed)"
                      << "\n\theading: " << cpmStationDC->choice.originatingVehicleContainer.heading.headingValue
                      << "\n\tspeed: " << cpmStationDC->choice.originatingVehicleContainer.speed.speedValue
                      << std::endl;

        }

        //Sensors list:
        std::cout << "-- List of sensors --" << std::endl;
        ListOfSensorInformationContainer_t *sensorsContainer = cpm.cpm.cpmParameters.sensorInformationContainer;
        for (int i = 0; sensorsContainer != nullptr && i < sensorsContainer->list.count; i++) {
            SensorInformationContainer_t *sensCont = sensorsContainer->list.array[i];
            std::cout << "Sensor " << i << ": \n\tId: " << sensCont->id
                      << "\n\tType: " << sensCont->type;

            if (sensCont->details.present == SensorDetails_PR_vehicleSensor) {
                VehicleSensor_t sensDetails = sensCont->details.choice.vehicleSensor;

                std::cout << "\n\tReference point: " << sensDetails.refPointId
                          << "\n\tX Sensor offset: " << sensDetails.xSensorOffset
                          << "\n\tY Sensor offset: " << sensDetails.ySensorOffset;

                ListOfVehicleSensorProperties_t sensorProperties = sensDetails.vehicleSensorProperties;
                for (int j = 0; j < sensorProperties.list.count; j++) {
                    VehicleSensorProperties_t *sensProp = sensorProperties.list.array[j];
                    std::cout << "\n\tRange: " << sensProp->range / Range_oneMeter
                              << "\n\tHor. op. angle start: "
                              << sensProp->horizontalOpeningAngleStart / CartesianAngleValue_oneDegree
                              << "\n\tHor. op. angle end: "
                              << sensProp->horizontalOpeningAngleEnd / CartesianAngleValue_oneDegree;
                }
            }

            std::cout << std::endl << std::endl;
        }

        //Perceived object container
        std::cout << "-- List of Objects --" << std::endl;
        ListOfPerceivedObjectContainer_t *objectsContainer = cpm.cpm.cpmParameters.perceivedObjectContainer;
        for (int i = 0; objectsContainer != nullptr && i < objectsContainer->list.count; i++) {
            PerceivedObjectContainer *objCont = objectsContainer->list.array[i];
            std::cout << "Object " << i << ": \n\tobjectId: " << objCont->objectID
                      << "\n\ttimeOfMeasurement: " << objCont->timeOfMeasurement
                      << "\n\txDistance: " << objCont->xDistance.value
                      << "\n\tyDistance: " << objCont->yDistance.value
                      << "\n\txSpeed: " << objCont->xSpeed.value
                      << "\n\tySpeed: " << objCont->ySpeed.value
                      << std::endl << std::endl;
        }
    }


// Compute Toff
double CPService::getRatioTimeAllowedUsed(const artery::cpm::Cpm *message) {
    if (mDccAlg == "LimericDccEntity") {
        vanetza::UnitInterval dutyCycleLimit = getLimericDelta();

        if (message) {
            vanetza::Clock::duration t_on_pp = getTonpp(message);
            const auto interval = std::chrono::duration_cast<vanetza::Clock::duration>(
                    t_on_pp / dutyCycleLimit.value());
            double timeToWait = (double) std::chrono::duration_cast<std::chrono::microseconds>(interval).count();
            timeToWait /= 1000000;
            return timeToWait;
        }
    }
    return 0.0;
}


vanetza::Clock::duration CPService::getTonpp(const artery::cpm::Cpm *message){
    std::size_t v2xMessageSize = message->size();
    std::size_t btpHeaderSize = 4;
    std::size_t geonetSHBHeader = 40;

    vanetza::dcc::TransmissionLite transmission(DCCPROFILECP, v2xMessageSize + btpHeaderSize + geonetSHBHeader);
    return transmission.channel_occupancy();
}


// Compute Toff
void CPService::getToff(artery::cpm::Cpm *msg, vanetza::UnitInterval &delta, bool computeBestDelta, double &ToffFinal,
                        const omnetpp::SimTime& T_now){
    //Init all duty cycles
    delta = getLimericDelta();
    omnetpp::SimTime deltaUpdatingPeriod = SimTime(200, SIMTIME_MS); //Default in the standard

    //Init time of checking + get last time delta has been updated
    omnetpp::SimTime lastDeltaUpdate = getLimericLastDeltaUpdatingTime();
    omnetpp::SimTime timeEstimation = T_now;

    //Init all Toff
    ToffFinal = 0.0;
    omnetpp::SimTime Toff = SimTime(std::min(getRatioTimeAllowedUsed(msg), 1.0));
    //std::cout << std::min(getRatioTimeAllowedUsed(msg), 1.0) << std::endl;
    omnetpp::SimTime tRemaining;

    while(ToffFinal < 1.0 && Toff > lastDeltaUpdate + deltaUpdatingPeriod - timeEstimation){

        tRemaining = Toff + timeEstimation - lastDeltaUpdate - deltaUpdatingPeriod;
        ToffFinal += Toff.dbl() - tRemaining.dbl();

        if (computeBestDelta) {
            delta = 0.984 * delta + 0.0005;
        } else {
            delta = 0.9 * delta - 0.00025;
        }

        delta = vanetza::UnitInterval(std::min(std::max(delta.value(),0.0006), 0.03));

        //Apply equation B.2.
        const auto ratio = std::chrono::duration_cast<vanetza::Clock::duration>(
                getTonpp(msg) / delta.value());
        double timeToWait = (double) std::chrono::duration_cast<std::chrono::microseconds>(ratio).count();
        timeToWait /= 1000000;

        assert(timeToWait > 0.0);
        Toff = SimTime(std::min(timeToWait * (tRemaining.dbl() / Toff.dbl()), 1.0 - ToffFinal));

        lastDeltaUpdate = timeEstimation; //Both terms will cancel each other afterwards.
    }
    ToffFinal += Toff.dbl();
}



void CPService::collectObjectAge(){
	for(const LocalEnvironmentModel::TrackedPerson& obj : mLocalEnvironmentModel->allPersonObjects()){
		const artery::LocalEnvironmentModel::Tracking& tracking_ptr = obj.second;
		const LocalEnvironmentModel::Tracking::TrackingMap& sensorsDetection =  tracking_ptr.sensors();

		bool detectedByRadars = false;

		for(const auto& tracker : sensorsDetection) {
			if (tracker.first->getSensorCategory() == "Radar") {
				detectedByRadars = true;
				break;
			}
		}

		if(!detectedByRadars){
			const PersonDataProvider &vd = obj.first.lock()->getPersonData();

			if (mObjectsReceived.find(vd.station_id()) != mObjectsReceived.end()) {
				InfoObject &infoObjectAI = mObjectsReceived.at(vd.station_id());

				//Remove the entry if expired
				if (mObjectsReceived.at(vd.station_id()).getLastTrackingTime().last() +
					mCPSensor->getValidityPeriod() < mPersonDataProvider->updated()) {
					mObjectsReceived.erase(vd.station_id());
				} else {
                    emit(scSignalRatioObjectAge, simTime() - mObjectsReceived.at(vd.station_id()).getLastTrackingTime().last());
                }
			}
		}
	}
}


void CPService::completeMyPrevObjSent(const omnetpp::SimTime& T_now, InfoObject::ObjectsToSendMap objToSend){
    //Add object in the list of previously sent
    for(auto obj : objToSend) {
        obj.second.setLastTimeSent(T_now);
        if (mObjectsPrevSent.find(obj.first) != mObjectsPrevSent.end()) {
            mObjectsPrevSent[obj.first] = obj.second;
        } else {
            mObjectsPrevSent.insert(obj);
        }
    }
}


/**
* Create CPM and define parameters for BTP and Geonet
* @param T_now: generation time of cpm
*/
bool CPService::sendLimericAdaptedCPM(const SimTime& T_now)
{
    assert(par("v2xCapabilities") || par("objectDynamicsLocal")); // We need these two filters in case we want to use the full filtering
    bool needToSendSensorInfo = false;
	uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
	auto cpm = createCollectivePerceptionMessage(genDeltaTimeMod);

	//Add sensor information?
	if(T_now - mLastSensorInfoCont >= SimTime(1, SIMTIME_S)) {
		//std::cout << "include CP sensors info " << T_now << std::endl;
		mLastSensorInfoCont = T_now;
		addSensorInformation(mLocalEnvironmentModel, cpm, mSensorsId);
        needToSendSensorInfo = true;
	}

	//Step 1: Filter all objects with low dynamics
	InfoObject::ObjectsToSendMap objToSendWithoutLowDynamics;
	InfoObject::ObjectsToSendMap objToSendNoFiltering;
	bool removeObjectWithLowDynamics = mPreFilterLowDynamics;
	if(mPreFilterLowDynamics) {
        mFilterObj.changeDeltas(mHeadingDelta * 0.2, mPositionDelta * 0.2, mSpeedDelta * 0.2);
    }
    mFilterObj.getObjToSendNoFilter(objToSendWithoutLowDynamics, removeObjectWithLowDynamics, mObjectsPrevSent, T_now);
    mFilterObj.getObjToSendNoFilter(objToSendNoFiltering, false, mObjectsPrevSent, T_now);

    //If nothing to send but we still want to send Sensor information
    if(objToSendWithoutLowDynamics.size() == 0){
        if(needToSendSensorInfo && mGenerateForSensor){
            requestCPMTransmission(T_now, cpm);
            return true;
        }
        return false; // Do not generate a cpm if you have nothing to send
    }

    //Compute Toff for the computed cpm
	generateASN1Objects(cpm, T_now, objToSendWithoutLowDynamics);
    double Toff;
    vanetza::UnitInterval worstDelta;
    getToff(&cpm, worstDelta, false, Toff, T_now);

	//TODO: comment all Test output
/*	//Test:
	std::cout << "\nCheck for conditons:" << std::endl;
	std::cout << "CPM size:" << cpm.size() << std::endl;
    std::cout << "delta:" << getLimericDelta().value() << std::endl;
    std::cout << "getLimericLastDeltaUpdatingTime:" << getLimericLastDeltaUpdatingTime() << std::endl;
    std::cout << "now: " << T_now << std::endl;
	std::cout << "Toff:" << Toff << std::endl;
*/
    //We are in the case where we are sure that we detect at least one object
    if(Toff > mStartFilteringThresh) {

        mObjectsToSend.clear();
        std::size_t countObject = mFilterObj.filterObjects(mObjectsToSend, mObjectsPrevSent, genCpmDcc(), mCPSensor,
                                                           mObjectsReceived, T_now);

        bool sentMessage = false;
        if(mObjectsToSend.size() > 0 || (needToSendSensorInfo && mGenerateForSensor)){
            // Used for second enhancement
            if(mUseMatchingSize && (!mUseThreshMatchingSize || Toff <= mThreshMatchingSize)) {
                matchSpleepingTime(cpm, objToSendWithoutLowDynamics, T_now);
            }
            else
                generateASN1Objects(cpm, T_now, mObjectsToSend);

            checkCPMSize(T_now, mObjectsToSend, cpm);
            completeMyPrevObjSent(T_now, mObjectsToSend);
            requestCPMTransmission(T_now, cpm);
            sentMessage = true;
        }

        emit(scSignalRatioFilter, (double) 1.0 - mObjectsToSend.size() / objToSendNoFiltering.size());

        return sentMessage;

    }
    else {

        checkCPMSize(T_now, objToSendWithoutLowDynamics, cpm);
        emit(scSignalRatioFilter, (double) 1.0 - objToSendWithoutLowDynamics.size() / objToSendNoFiltering.size());

        completeMyPrevObjSent(T_now, objToSendWithoutLowDynamics);
        requestCPMTransmission(T_now, cpm);
    }
    return true;
}


bool CPService::matchSpleepingTime(artery::cpm::Cpm& cpm, InfoObject::ObjectsToSendMap& objToSendWithoutLowDynamics, const SimTime& T_now) {

    //Generate the CPM with all non filtered objects
	generateASN1Objects(cpm, T_now, mObjectsToSend);
	double ToffBest, ToffWorst, ToffAlg;
    vanetza::UnitInterval bestCaseDelta, worstCaseDelta;

    vanetza::Clock::duration t_on_pp = getTonpp(&cpm);

    //TODO: comment all Test output
    //Test:
    /*
    std::cout << "\nCheck for conditons:" << std::endl;
    std::cout << "CPM size:" << cpm.size() << std::endl;
    std::cout << "Tonpp:" << std::chrono::duration_cast<std::chrono::microseconds>(t_on_pp).count() << std::endl;
    std::cout << "delta:" << getLimericDelta().value() << std::endl;
    std::cout << "getLimericLastDeltaUpdatingTime:" << getLimericLastDeltaUpdatingTime() << std::endl;
    std::cout << "now: " << T_now << std::endl;
    */
    //get Toff in best and worst case
	getToff(&cpm, bestCaseDelta, true, ToffBest, T_now);
	getToff(&cpm, worstCaseDelta, false, ToffWorst, T_now);
    int periodToffBest = std::ceil(ToffBest*10);
    int periodToffWorst = std::ceil(ToffWorst*10);

    /*
    std::cout << "Best Toff = " << ToffBest << std::endl;
    std::cout << "Worst case Toff = " << ToffWorst << std::endl;
    std::cout << "Best case delta = " << bestCaseDelta.value() << std::endl;
    std::cout << "Worst case delta = " << worstCaseDelta.value() << std::endl;
    std::cout << "Best case 100ms period " << std::ceil(ToffBest*10) << " Worst case 100ms period " << std::ceil(ToffWorst*10) << std::endl;
    */

    if(ToffBest < 1.0 && periodToffBest == periodToffWorst){
        double Toff = ToffWorst;

        double availableTimeForFilteredObjects =
                0.1 - std::fmod(Toff, 0.1); //0.1 comes from the frequency of generation rules checks

        //If there is no remaining leftover time for an objects, than avoid useless operation
        if (availableTimeForFilteredObjects == 0.0)
            return mObjectsToSend.size() > 0;

        //TODO: comment
        //std::cout << "Toff* = " << Toff << std::endl;
        //std::cout << "Available time: " << availableTimeForFilteredObjects << std::endl;

        //Todo: comment
        //std::cout << "Worst case delta: " << worstCaseDelta.value() << std::endl;

        if (worstCaseDelta.value() >= 0.0006) {

            int allowedExtraSize = availableTimeForFilteredObjects * worstCaseDelta.value() * (6 * std::pow(10, 6)) / 8;    //6*10^6 = data bitrate
            //std::cout << "Allowed extra size: " << allowedExtraSize << std::endl;

            size_t initialCpmSize = cpm.size();
            //std::cout << "Initial CP size:" << initialCpmSize << std::endl;

            bool stillEnoughSpace = true;
            // We keep trying till an object can't added or all the objects have already been added
            while (stillEnoughSpace && mObjectsToSend.size() < objToSendWithoutLowDynamics.size()) {
                //std::cout << "Current nb of objects to send: " << mObjectsToSend.size() << std::endl;

                //Choose a random number that identifies which object should be inserted
                int rand = std::rand() % (objToSendWithoutLowDynamics.size() - mObjectsToSend.size()) + 1;

                //Go through all the objects no filtered
                for(auto obj : objToSendWithoutLowDynamics) {
                    //Check if it is one of the object filtered
                    if (mObjectsToSend.find(obj.first) == mObjectsToSend.end()) {
                        rand--;
                    }

                    //Object chosen randomly
                    if (rand == 0) {
                        mObjectsToSend.insert(obj);
                        generateASN1Objects(cpm, T_now, mObjectsToSend);
                        getToff(&cpm, worstCaseDelta, false, ToffAlg, T_now);

                        //if size added is bigger than the allowed one, remove the object and stop the process of looking for new object
                        //std::cout << "Current CPM size: " << cpm.size() << std::endl;
                        if (std::ceil(ToffAlg*10) != periodToffWorst) {
                            stillEnoughSpace = false;
                            mObjectsToSend.erase(obj.first);
                            // Setup the previous state of the CPM
                            generateASN1Objects(cpm, T_now, mObjectsToSend);
                        }
                        break;
                    }
                }
            }

        }
    }
//TODO: comment
/*
    //std::cout << "Final nb of objects to send: " << mObjectsToSend.size() << "/" << objToSendNoFiltering.size() << std::endl;
	//std::cout << "Final cp size: " << cpm.size() << std::endl;
    vanetza::UnitInterval deltaFinal;
    getToff(&cpm, deltaFinal, false, ToffTosee, T_now);
    std::cout << "Final Toff estimated: " << ToffAlg << std::endl;
*/
    return mObjectsToSend.size() > 0;
}


vanetza::UnitInterval CPService::getLimericDelta(){
    auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
    LimericDccEntity* limericDcc = (LimericDccEntity*) &netifc->getDccEntity();
    if (!limericDcc) {
        throw cRuntimeError("No Limeric module found (CP service)", mPrimaryChannel);
    }
    return limericDcc->getPermittedDutyCycle(); //Interesting delta to debug adapted limeric= 0.0012 and 0.0014 -> return vanetza::UnitInterval(0.0014)
}


omnetpp::SimTime CPService::getLimericLastDeltaUpdatingTime(){
    auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
    LimericDccEntity* limericDcc = (LimericDccEntity*) &netifc->getDccEntity();
    if (!limericDcc) {
        throw cRuntimeError("No Limeric module found (CP service)", mPrimaryChannel);
    }
    return limericDcc->getLastDeltaUpdatingTime();
}


void CPService::checkCPMSize(const SimTime& T_now, InfoObject::ObjectsToSendMap& objToSend, artery::cpm::Cpm& cpm){
	bool removedObject = false;
	while(cpm.size() > MAXCPMSIZE){
		InfoObject::ObjectsToSendMap::iterator item = objToSend.begin();
		std::advance(item, std::rand() % objToSend.size());
		objToSend.erase(item);
		generateASN1Objects(cpm, T_now, objToSend);
		removedObject = true;
	}

	if(removedObject)
		emit(scSignalRemovedObjExcessiveSize, 1);
}


double CPService::getDCCToff(){
    // network interface may not be ready yet during initialization, so look it up at this later point
    auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
    vanetza::dcc::TransmitRateThrottle *trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
    if (!trc) {
        throw cRuntimeError("No DCC TRC found for CP's primary channel %i", mPrimaryChannel);
    }
    static const vanetza::dcc::TransmissionLite cp_tx(DCCPROFILECP, 0);
    vanetza::Clock::duration delay = trc->interval(cp_tx);
    SimTime dcc{std::chrono::duration_cast<std::chrono::milliseconds>(delay).count(), SIMTIME_MS};

    return dcc.dbl();
}



} // namespace artery


