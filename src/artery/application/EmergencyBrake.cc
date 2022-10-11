#include "artery/application/EmergencyBrake.h"
#include "artery/application/VaService.h"
#include "artery/application/CPObject.h"
#include "artery/application/VaObject.h"
#include "artery/application/CPService.h"
#include "artery/application/Constants.hpp"
#include "artery/traci/VehicleController.h"
#include "artery/application/PersonDataProvider.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/envmod/sensor/Sensor.h"
#include "artery/inet/PowerLevelRx.h"
#include <vanetza/asn1/vam.hpp>
#include <artery/cpm/cpm.hpp>
#include "artery/application/Asn1PacketVisitor.h"
#include <boost/units/systems/si/prefixes.hpp>
#include "artery/application/MultiChannelPolicy.h"
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include <vanetza/btp/data_request.hpp>
#include <omnetpp/checkandcast.h>
#include "artery/utility/Geometry.h"
#include <boost/geometry.hpp>
#include <boost/geometry/strategies/spherical/distance_haversine.hpp> 
#include <vector>
#include <chrono>
#include "artery/application/Timer.h"
#include <memory>
#include <string>
#include <iostream>  
#include <artery/networking/LimericDccEntity.h>



namespace bg = boost::geometry;
typedef bg::model::point <double,2,bg::cs::geographic <bg::degree> >point;
typedef bg::srs::spheroid <double> stype;
typedef bg::strategy::distance::vincenty <stype> vincenty_type;

namespace artery
{

using namespace omnetpp;

Define_Module(EmergencyBrake)

static const simsignal_t scSignalVamReceived = cComponent::registerSignal("VamReceived");
static const simsignal_t scSignalCpmReceived = cComponent::registerSignal("CpmReceived");


void EmergencyBrake::initialize()
{
    ItsG5BaseService::initialize();
    mVehicleController = &getFacilities().get_mutable<traci::VehicleController>();
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
    mLocalEnvironmentModel = &getFacilities().get_mutable<LocalEnvironmentModel>();
    trigger();

    mTimer = &getFacilities().get_const<Timer>();

    //setVehicleBehavior();

    mtriggerVRU_TTC = par("triggerVRU_TTC").doubleValue() * vanetza::units::si::second;
    mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;  

}

void EmergencyBrake::setVehicleBehavior()
{
   ;
}

void EmergencyBrake::trigger()
{
    checkSensorData(simTime());
}


void EmergencyBrake::checkSensorData(const SimTime &T_now)
{
    Enter_Method("sensorData");
    auto TrackedPersons = mLocalEnvironmentModel->allPersonObjects();
    
    
    PowerLevelRx* getCHLoad;
    //auto teste = getCHLoad->getChannelLoad().cbr();
   

    timeNow = T_now;

    //////////////////////////////////////////////////////////////
    ////////  Calculate the Latency of Object's Detection  ////////
    //////////////////////////////////////////////////////////////

    std::map<const std::string, omnetpp::SimTime>::iterator mEnvItr;
    
    for (auto itr : TrackedPersons){
            for (auto range : itr.second.sensors()){
            
            auto const &personID = itr.first.lock()->getPersonId(); 
             
            mEnvItr = mSensorDetection.find(personID); //look for personID in the map

            if (mEnvItr != mSensorDetection.end()){
                ; //do nothing
            } else{
                mSensorDetection.insert(std::pair<std::string, omnetpp::SimTime>(personID, range.second.first()));
            }
        }
    }

    
    if (T_now > 99.8){
        //std::cout << "Vam Received: " << vamReceived << std::endl;
        //std::cout << "CPM Received: " << cpmReceived << std::endl;
        for (auto printItr : mobjDetectedMap){        
            std::cout << printItr.first << ";" << printItr.second << std::endl; 
        }
    }


    ///////////////////////////////////////////////////////////////
    /////////////////////  Calculate the VPP  /////////////////////
    ///////////////////////////////////////////////////////////////

//  std::cout << T_now << ";" << TrackedPersons.size() << std::endl;

    /*
    objDetected.push_back(TrackedPersons);
    
    float elementSum = 0;
    
    
    if (T_now > 99.8){
        for (auto const& itr : objDetected){
            elementSum += itr;
            std::cout << T_now << ";" << elementSum << std::endl;
        }
    }*/
}


void EmergencyBrake::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
	Enter_Method("indicate");

    Asn1PacketVisitor<artery::cpm::Cpm> visitor_cpm;
    const artery::cpm::Cpm* cpm = boost::apply_visitor(visitor_cpm, *packet);

    if (cpm && cpm->validate()) {
            
        CPObject obj = visitor_cpm.shared_wrapper;
        CPM_Content(cpm);
        emit(scSignalCpmReceived, &obj);
    }

    Asn1PacketVisitor<vanetza::asn1::Vam> visitor;
	const vanetza::asn1::Vam* vam = boost::apply_visitor(visitor, *packet);
    
	if (vam && vam->validate()) {
		VaObject obj = visitor.shared_wrapper;
        VAM_Content(vam);
        emit(scSignalVamReceived, &obj);

        VaObject* va_obj = dynamic_cast<VaObject*>(&obj);
	}
}



void EmergencyBrake::receiveSignal(cComponent*, omnetpp::simsignal_t signal, cObject *obj, cObject*)
{
    std::cout << "Signal received" << std::endl;
    
    if (signal == scSignalCpmReceived){
        std::cout << "scSignalCPmReceived" << std::endl;    
    }
}

void EmergencyBrake::VAM_Content(const vanetza::asn1::Vam *vam)
{
    ++vamReceived;

    vanetza::asn1::Vam vam_data = (*vam);
    vanetza::asn1::Vam VAM;

    //get VRU id
    uint32_t stationID = vam_data->header.stationID;
    double VRU_lat = (vam_data->vam.vamParameters.basicContainer.referencePosition.latitude);
    double VRU_lon = (vam_data->vam.vamParameters.basicContainer.referencePosition.longitude);

    VRU_latitude = VRU_lat / 10000000;
    VRU_longitude = VRU_lon / 10000000;

    VruHighFrequencyContainer*& VAM_hfc = vam_data->vam.vamParameters.vruHighFrequencyContainer;
    vanetza::units::Velocity VRUspeed (VAM_hfc->speed.speedValue * Constants::centimeter_per_second);

    checkDistanceConditions();

    auto personID = std::to_string(stationID);
    triggerObjMap(personID);

}


void EmergencyBrake::CPM_Content(const artery::cpm::Cpm *cpm)
{
    ++cpmReceived;

    artery::cpm::Cpm cpm_data = (*cpm);
    artery::cpm::Cpm CPM;

    uint32_t stationID = cpm_data->header.stationID; 
    
    omnetpp::SimTime generationTime = mTimer->getTimeFor(mTimer->reconstructMilliseconds(cpm_data->cpm.generationDeltaTime));
    ListOfPerceivedObjectContainer_t *objectsContainer = cpm_data->cpm.cpmParameters.perceivedObjectContainer;


    std::map<const std::string, omnetpp::SimTime>::iterator mobjItr;
    std::map<const std::string, omnetpp::SimTime>::iterator mEnvCheckItr;
    
    for (int i = 0; objectsContainer != nullptr && i < objectsContainer->list.count; i++) {        
        PerceivedObjectContainer *objCont = objectsContainer->list.array[i];
        auto personID = std::to_string(objCont->objectID);   
        

        triggerObjMap(personID);
        /*
        mEnvCheckItr = mPerceptionEnv.find(personID); //Check if CPM PersonID is on Env Map

        if (mEnvCheckItr != mPerceptionEnv.end()) {

            mobjItr = mobjDetctedMap.find(personID);
            if (mobjItr != mobjDetctedMap.end()) { //Check if the element is already in ObjMapCPM
                ;
            } else {
                auto latency = timeNow - mEnvCheckItr->second;
                mobjDetctedMap.insert(std::pair<std::string, omnetpp::SimTime>(personID, latency));
            }        
        }   */      
    }
}

void EmergencyBrake::triggerObjMap(const std::string &stationID)
{
    std::string personID = stationID;

    std::map<const std::string, omnetpp::SimTime>::iterator mobjItr;
    std::map<const std::string, omnetpp::SimTime>::iterator mSensorCheckItr;

    mSensorCheckItr = mSensorDetection.find(personID); //Check if CPM PersonID is on Env Map

    if (mSensorCheckItr != mSensorDetection.end()) {

        //detectionTime by the sensor
        omnetpp::SimTime sensorDetectionTime = mSensorCheckItr->second;


        mobjItr = mobjDetectedMap.find(personID);
        if (mobjItr != mobjDetectedMap.end()) { //Check if the element is already in ObjMapCPM
            ;
        } else {         
            if (sensorDetectionTime <= timeNow) {
                mobjDetectedMap.insert(std::pair<std::string, omnetpp::SimTime>(personID, sensorDetectionTime));
            } else {
                mobjDetectedMap.insert(std::pair<std::string, omnetpp::SimTime>(personID, timeNow));
            }   
        }        
    } else {
        mobjDetectedMap.insert(std::pair<std::string, omnetpp::SimTime>(personID, timeNow));
    }         
}   

void EmergencyBrake::checkDistanceConditions()
{
    //calculate Lat and Long from VUT
    artery::GeoPosition VUT_GeoPosition =  mVehicleController->getGeoPosition();
    double VUT_lat = Constants::round(VUT_GeoPosition.latitude, Constants::microdegree); 
    double VUT_lon = Constants::round(VUT_GeoPosition.longitude, Constants::microdegree);

    VUT_latitude = VUT_lat / 1000000;
    VUT_longitude = VUT_lon / 1000000;

    
    //Calculate the distance between 2 Points using Haversine Formula
    //point (longitude, latitude)
    auto distance_VRU_VUT_H = bg::distance(point(VUT_longitude, VUT_latitude), point(VRU_longitude, VRU_latitude), vincenty_type()) * vanetza::units::si::meter;

    //Calculate TTC
    mVUT_Speed = mVehicleDataProvider->speed();
    TTC = (distance_VRU_VUT_H / mVUT_Speed);


    if (TTC <= mtriggerVRU_TTC){
        AEB();
    }
}

void EmergencyBrake::AEB()
{       
    EV_INFO << "AEB has been triggered" << endl;
}

} //namespace artery