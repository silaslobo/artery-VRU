#include "artery/application/CaService.h"
#include "artery/application/cam/Cam_UseCase.h"
#include "artery/application/cam/EmergencyBrake_UseCase.h"
#include "artery/application/SampleBufferAlgorithm.h"
#include "artery/application/VehicleDataProvider.h"
#include <vanetza/btp/data_request.hpp>
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/velocity.hpp>
#include <algorithm>
#include <iostream>
#include "artery/traci/VehicleController.h"
#include "traci/sumo/utils/traci/TraCIAPI.h"


namespace artery
{

namespace cam
{

//Define_Module(EmergencyBrake_UseCase)


void EmergencyBrake_UseCase::initialize(int stage)
{
    Cam_UseCase::initialize(stage);
    if (stage == 0)
    {
        using boost::units::si::meter_per_second;
        using boost::units::si::meter_per_second_squared;

        mAccelerationSampler.setDuration(par("sampleDuration"));
        mAccelerationSampler.setInterval(par("sampleInterval"));
        mSpeedThreshold = par("speedThreshold").doubleValue() * meter_per_second;
        mDecelerationThreshold = par("decelerationThreshold").doubleValue() * meter_per_second_squared;        
    }    
}




void EmergencyBrake_UseCase::indicate(const artery::CaObject& cam_msg)
{
    Enter_Method("Emergency_Brake");
    auto veh = mService->getId();
    EV_DEBUG << veh;

   
}




} // namespace cam
} // namespace artery


