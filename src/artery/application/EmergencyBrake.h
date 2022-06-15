#ifndef EMERGENCYBRAKESERVICE_H_
#define EMERGENCYBRAKESERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/PersonDataProvider.h"
#include "artery/utility/Channel.h"
#include <omnetpp/simtime.h>
#include "artery/application/NetworkInterface.h"
#include <vanetza/asn1/vam.hpp>
#include "artery/cpm/cpm.hpp"
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/envmod/EnvironmentModelObject.h>
#include "artery/application/InfoObject.h"
#include "artery/cpm/compiled/ListOfSensorInformationContainer.h"
#include "artery/application/InfoObject.h"
#include <map>
#include <vanetza/common/clock.hpp>
#include "artery/nic/ChannelLoadSampler.h"

// forward declaration
namespace traci { class VehicleController; class ObjIdentification; class SimulationScopeController;}

namespace artery
{

class VehicleDataProvider;
class Timer;

class EmergencyBrake : public ItsG5BaseService
{
    public:

        void initialize() override;
        void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
        void trigger() override;

    private:
        ChannelNumber mPrimaryChannel = channel::CCH;
        void VAM_Content(const vanetza::asn1::Vam*);
        void CPM_Content(const artery::cpm::Cpm*);
        void receiveSignal(cComponent*, omnetpp::simsignal_t signal, cObject *obj, cObject*);

        vanetza::units::Velocity mVUT_Speed;
        vanetza::units::Velocity mVRU_Speed;
        //vanetza::units::Length mPositionDelta;
        vanetza::units::Duration TTC;
        vanetza::units::Duration mtriggerVRU_TTC;
        vanetza::units::Velocity mSpeedDelta;

        int vamReceived = 0;
        int cpmReceived = 0;

        LocalEnvironmentModel* mLocalEnvironmentModel;
        const Timer* mTimer = nullptr;
        Sensor* mCPSensor;

        ChannelLoadSampler mChannelLoadSampler;

        omnetpp::SimTime timeNow;

        std::vector<float> objDetected;

        std::map<const std::string, omnetpp::SimTime> mobjDetectedMap;
        std::map<const std::string, omnetpp::SimTime> mSensorDetection;

        void setVehicleBehavior();
        void AEB();
        void checkDistanceConditions();
        void triggerObjMap(const std::string&);
        void checkSensorData(const omnetpp::SimTime&);
        
        double VRU_latitude; //latitude from VRU
        double VRU_longitude; //longitude from VRU
        double VUT_latitude; //latitude from VRU
        double VUT_longitude; //longitude from VRU

        //SpeedValue_t vruSpeed;

        traci::VehicleController* mVehicleController = nullptr;
        traci::SimulationScopeController* mSimulationScope = nullptr;
        const VehicleDataProvider* mVehicleDataProvider = nullptr;   
};

} //namespace artery

#endif /* EMERGENCYBRAKESERVICE_H_ */