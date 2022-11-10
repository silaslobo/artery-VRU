#include "artery/envmod/sensor/VRUsensor.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/application/VaObject.h"
#include "artery/application/CPObject.h"
#include "artery/application/Middleware.h"
#include "artery/utility/IdentityRegistry.h"
#include <inet/common/ModuleAccess.h>

using namespace omnetpp;

namespace artery
{

static const simsignal_t VamReceivedSignal = cComponent::registerSignal("VamReceived");
static const simsignal_t CPmReceivedSignal = cComponent::registerSignal("CpmReceived");

Define_Module(VRUsensor)

void VRUsensor::initialize()
{
    mValidityPeriod = par("validityPeriod");
    BaseSensor::initialize();
    mIdentityRegistry = inet::getModuleFromPar<IdentityRegistry>(par("identityRegistryModule"), this);
    getMiddleware().subscribe(VamReceivedSignal, this);
    getMiddleware().subscribe(CPmReceivedSignal, this);
}

void VRUsensor::finish()
{
    getMiddleware().unsubscribe(VamReceivedSignal, this);
    getMiddleware().unsubscribe(CPmReceivedSignal, this);
    BaseSensor::finish();
}

void VRUsensor::measurement()
{
    Enter_Method("measurement");
}

void VRUsensor::receiveSignal(cComponent*, simsignal_t signal, cObject *obj, cObject*)
{
    if (signal == VamReceivedSignal) {
        auto* vam = dynamic_cast<VaObject*>(obj);
        if (vam) {
            uint32_t stationID = vam->asn1()->header.stationID;
            auto identity = mIdentityRegistry->lookup<IdentityRegistry::application>(stationID);
            if (identity) {
                auto person = mGlobalEnvironmentModel->getPerson(identity->traci);
                SensorDetection detection;
                detection.persons.push_back(person);
                mLocalEnvironmentModel->complementObjects(detection, *this);
            } else {
                EV_WARN << "Unknown identity for station ID " << stationID;
            }
        } /*else {
            EV_ERROR << "received signal has no VaObject";
        }*/
    }

    if (signal == CPmReceivedSignal) {
        auto* cpm = dynamic_cast<CPObject*>(obj);
        if (cpm){
            ListOfPerceivedObjectContainer_t* objectsContainer = cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer;
            for(int i = 0 ; objectsContainer != nullptr && i < objectsContainer->list.count ; i++) {
                PerceivedObjectContainer* objCont = objectsContainer->list.array[i];
                auto identityPerson = mIdentityRegistry->lookup<IdentityRegistry::application>(objCont->objectID);
                if (identityPerson) {
                    auto person = mGlobalEnvironmentModel->getPerson(identityPerson->traci);
                    SensorDetection detection;
                    detection.persons.push_back(person);
                    mLocalEnvironmentModel->complementObjects(detection, *this);
                } else {
                EV_WARN << "Unknown identity for station ID ";
                }
            }               
        } 
    }
}

omnetpp::SimTime VRUsensor::getValidityPeriod() const
{
    return mValidityPeriod;
}

const std::string& VRUsensor::getSensorCategory() const
{
    static const std::string category = "VRU";
    return category;
}

SensorDetection VRUsensor::detectObjects() const
{
    // return empty sensor detection because VAM objects are added upon VAM reception signal
    SensorDetection detection;
    return detection;
}

} // namespace artery
