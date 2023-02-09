#include "artery/application/EmergencyBrake.h"
#include "artery/application/VaService.h"
#include "artery/application/CPObject.h"
#include "artery/application/VaObject.h"
#include "artery/application/CPService.h"
#include "artery/application/Constants.hpp"
#include "artery/traci/VehicleController.h"
#include "artery/traci/SimulationScopeController.h"
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
#include <unordered_map>

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
static const simsignal_t scSignalVRU_ID_VAM = cComponent::registerSignal("VRU_ID_VAM");
static const simsignal_t scSignalVRU_ID_CPM = cComponent::registerSignal("VRU_ID_CPM");
static const simsignal_t scSignalVRU_ID_RADAR = cComponent::registerSignal("VRU_ID_RADAR");

void EmergencyBrake::initialize()
{
    ItsG5BaseService::initialize();
    mVehicleController = &getFacilities().get_mutable<traci::VehicleController>();
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
    mLocalEnvironmentModel = &getFacilities().get_mutable<LocalEnvironmentModel>();
    trigger();

    mTimer = &getFacilities().get_const<Timer>();


}

void EmergencyBrake::trigger()
{
    checkSensorData(simTime());
}

void EmergencyBrake::checkSensorData(const SimTime &T_now)
{
    Enter_Method("sensorData");
    auto TrackedPersons = mLocalEnvironmentModel->allPersonObjects();
   
    timeNow = T_now;

    //////////////////////////////////////////////////////////////
    ////////  Calculate the Latency of Object's Detection  ////////
    //////////////////////////////////////////////////////////////
    
    for (auto itr : TrackedPersons){
            for (auto range : itr.second.sensors()){
            
            auto const &personID = itr.first.lock()->getPersonId();
            u_int32_t vruID = std::atoi(personID.c_str());
            std::map<const u_int32_t, omnetpp::SimTime>::iterator mEnvItr;
            mEnvItr = mSensorDetection.find(vruID); //look for personID in the map            
            if (mEnvItr != mSensorDetection.end()){
                ; //do nothing
            } else{
                mSensorDetection.insert(std::pair<u_int32_t, omnetpp::SimTime>(vruID, range.second.first()));  
                emit(scSignalVRU_ID_RADAR, vruID);    
            }
        }
    }

    /*
    if (T_now > 99.8){
        for (auto printItr : mobjDetectedMap_vam){        
            std::cout << printItr.first << ";" << printItr.second << std::endl; 
        }

        for (auto printItr : mobjDetectedMap_cpm){        
            std::cout << printItr.first << ";" << printItr.second << std::endl; 
        }
    }*/



    ///////////////////////////////////////////////////////////////
    /////////////////////  Calculate the VPP  /////////////////////
    ///////////////////////////////////////////////////////////////


    if ((timeNow - lastCPMUpdate) > 0.1){
        lastCPMUpdate = timeNow;
        objDetected_cpm.clear();
    }
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

void EmergencyBrake::VAM_Content(const vanetza::asn1::Vam *vam)
{
    Enter_Method("VAM_Content");

    ++vamReceived;

    vanetza::asn1::Vam vam_data = (*vam);
    vanetza::asn1::Vam VAM;

    //get VRU id
    uint32_t stationID = vam_data->header.stationID;
    double VRU_lat = (vam_data->vam.vamParameters.basicContainer.referencePosition.latitude);
    double VRU_lon = (vam_data->vam.vamParameters.basicContainer.referencePosition.longitude);

    VruHighFrequencyContainer*& VAM_hfc = vam_data->vam.vamParameters.vruHighFrequencyContainer;
    vanetza::units::Velocity VRUspeed (VAM_hfc->speed.speedValue * Constants::centimeter_per_second);

    triggerObjMap_vam(stationID);

    omnetpp::cComponent::emit(scSignalVRU_ID_VAM,stationID);    
}


void EmergencyBrake::CPM_Content(const artery::cpm::Cpm *cpm)
{
    Enter_Method("CPM_Content");
    ++cpmReceived;

    artery::cpm::Cpm cpm_data = (*cpm);
    artery::cpm::Cpm CPM;

    uint32_t stationID = cpm_data->header.stationID; 
    auto msgID = std::to_string(cpm_data->header.messageID);

    omnetpp::SimTime generationTime = mTimer->getTimeFor(mTimer->reconstructMilliseconds(cpm_data->cpm.generationDeltaTime));
    ListOfPerceivedObjectContainer_t *objectsContainer = cpm_data->cpm.cpmParameters.perceivedObjectContainer;


    std::map<const std::string, omnetpp::SimTime>::const_iterator mobjItr;
    std::map<const std::string, omnetpp::SimTime>::const_iterator mEnvCheckItr;
    
    
    for (int i = 0; objectsContainer != nullptr && i < objectsContainer->list.count; i++) {        
        PerceivedObjectContainer *objCont = objectsContainer->list.array[i];
        auto personID = std::to_string(objCont->objectID);   

        triggerObjMap_cpm(objCont->objectID);

        if ( std::find(objDetected_cpm.begin(), objDetected_cpm.end(), objCont->objectID) != objDetected_cpm.end() ){
            ;
        }else{
            objDetected_cpm.push_back(objCont->objectID);
        }        

        omnetpp::cComponent::emit(scSignalVRU_ID_CPM,objCont->objectID);         
    }

}

void EmergencyBrake::triggerObjMap_vam(const u_int32_t &stationID)
{   
    std::map<const u_int32_t, omnetpp::SimTime>::iterator mobjItr;
    std::map<const u_int32_t, omnetpp::SimTime>::iterator mSensorCheckItr;

    if (mobjDetectedMap_vam.find(stationID) != mobjDetectedMap_vam.end()){
        ;
    } else {
        mSensorCheckItr = mSensorDetection.find(stationID); //Check if CPM PersonID is on Env Map        
        if (mSensorCheckItr != mSensorDetection.end()){
            omnetpp::SimTime sensorDetectionTime = mSensorCheckItr->second;
            if (sensorDetectionTime < timeNow){
                mobjDetectedMap_vam.insert(std::pair<u_int32_t, omnetpp::SimTime>(stationID, sensorDetectionTime));            
            } else {
                mobjDetectedMap_vam.insert(std::pair<u_int32_t, omnetpp::SimTime>(stationID, timeNow));
            }
        } else {
            mobjDetectedMap_vam.insert(std::pair<u_int32_t, omnetpp::SimTime>(stationID, timeNow));                  
        }
    }     
}

void EmergencyBrake::triggerObjMap_cpm(const u_int32_t &stationID)
{ 
    std::map<const u_int32_t, omnetpp::SimTime>::iterator mobjItr;
    std::map<const u_int32_t, omnetpp::SimTime>::iterator mSensorCheckItr;

    if (mobjDetectedMap_cpm.find(stationID) != mobjDetectedMap_cpm.end()){
        ;
    } else {
        mSensorCheckItr = mSensorDetection.find(stationID); //Check if CPM PersonID is on Env Map        
        if (mSensorCheckItr != mSensorDetection.end()){
            omnetpp::SimTime sensorDetectionTime = mSensorCheckItr->second;
            if (sensorDetectionTime < timeNow){
                mobjDetectedMap_cpm.insert(std::pair<u_int32_t, omnetpp::SimTime>(stationID, sensorDetectionTime)); 
            } else {
                mobjDetectedMap_cpm.insert(std::pair<u_int32_t, omnetpp::SimTime>(stationID, timeNow));
            }
        } else {
            mobjDetectedMap_cpm.insert(std::pair<u_int32_t, omnetpp::SimTime>(stationID, timeNow));             
        }
    }    
}

} //namespace artery