#ifndef CAM_USECASE_H_
#define CAM_USECASE_H_

#include "artery/application/ItsG5BaseService.h"
#include <omnetpp/csimplemodule.h>
#include <vanetza/asn1/cam.hpp>
#include <functional>
#include <omnetpp/clistener.h>

namespace traci { class VehicleController; }

namespace artery
{

class CamObject;
class CaService;
class StoryboardSignal;
class VehicleDataProvider;

namespace cam
{
class Cam_UseCase : public ItsG5BaseService
{
    public:
        /*
        *Evaluate use case triggering conditions
        *Generate CAMs if necessary
        */
        virtual void check()=0;
    
        /*
        *Forwards the received msg by CamService to EmergencyBrake_UseCase
        *MSG responding to this can be triggered
        */
        virtual void indicate(const artery::CaObject&) = 0;
    
        /*
        *Send Storyboard trigger to EmergencyBrake_UseCase
        */
        virtual void handleStoryboardTrigger(const StoryboardSignal&) = 0;

        void initialize(int) override;
        int numInitStages () const override { return 1; }
            
    protected:
	
        CaService* mService = nullptr;
        const VehicleDataProvider* mVdp = nullptr;
        using TrafficCondition = std::function<bool(void)>;

        virtual vanetza::asn1::Cam createMessageSkeleton();
};
} //namespace cam
} //namespace artery


#endif /* CAM_USECASE_H_ */
