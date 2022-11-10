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
#include <unordered_map>
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

        int vamReceived = 0;
        int cpmReceived = 0;

        LocalEnvironmentModel* mLocalEnvironmentModel;
        const Timer* mTimer = nullptr;

        ChannelLoadSampler mChannelLoadSampler;

        omnetpp::SimTime timeNow;

        std::vector<float> objDetected;
        std::vector<u_int32_t> objDetected_cpm;
        omnetpp::SimTime lastCPMUpdate = 0;

        std::map<const u_int32_t, omnetpp::SimTime> mobjDetectedMap_vam;
        std::map<const u_int32_t, omnetpp::SimTime> mobjDetectedMap_cpm;
        std::map<const u_int32_t, omnetpp::SimTime> mSensorDetection;

        void triggerObjMap_vam(const u_int32_t&);
        void triggerObjMap_cpm(const u_int32_t&);
        void checkSensorData(const omnetpp::SimTime&);

        traci::VehicleController* mVehicleController = nullptr;
        const VehicleDataProvider* mVehicleDataProvider = nullptr;   
};

} //namespace artery

#endif /* EMERGENCYBRAKESERVICE_H_ */