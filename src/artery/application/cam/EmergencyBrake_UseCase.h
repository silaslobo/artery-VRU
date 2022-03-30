#ifndef ARTERY_EMERGENCYBRAKE_H_
#define ARTERY_EMERGENCYBRAKE_H_

#include "artery/application/Sampling.h"
#include "artery/application/cam/Cam_UseCase.h"
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/velocity.hpp>
#include "traci/sumo/utils/traci/TraCIAPI.h"

namespace artery
{
namespace cam
{

class EmergencyBrake_UseCase : public Cam_UseCase
{
    public:        
        void indicate(const artery::CaObject&);// override {};
        void handleStoryboardTrigger (const StoryboardSignal&) override{};    
        
        bool checkConditions();
        bool checkEgoSpeed() const;

        void setEgospeed();
        void setEgoModel();
        void slowDown();  

    protected:
        void initialize(int) override;

    private:


        traci::VehicleController* mVehicleController = nullptr;
        TraCIAPI::VehicleScope* setVehicle = nullptr;
        
        SkipEarlySampler<vanetza::units::Acceleration> mAccelerationSampler;
        vanetza::units::Velocity mSpeedThreshold;
        vanetza::units::Acceleration mDecelerationThreshold;

};
} // namespace cam
} // namespace artery

#endif /* ARTERY_EMERGENCYBRAKE_H_ */

