#include "artery/envmod/sensor/VamSensor.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/application/VaObject.h"
#include "artery/application/Middleware.h"
#include "artery/utility/IdentityRegistry.h"
#include <inet/common/ModuleAccess.h>

using namespace omnetpp;

namespace artery
{

static const simsignal_t VamReceivedSignal = cComponent::registerSignal("VamReceived");

Define_Module(VamSensor)

void VamSensor::initialize()
{
    mValidityPeriod = par("validityPeriod");
    BaseSensor::initialize();
    mIdentityRegistry = inet::getModuleFromPar<IdentityRegistry>(par("identityRegistryModule"), this);
    getMiddleware().subscribe(VamReceivedSignal, this);
}

void VamSensor::finish()
{
    getMiddleware().unsubscribe(VamReceivedSignal, this);
    BaseSensor::finish();
}

void VamSensor::measurement()
{
    Enter_Method("measurement");
}

void VamSensor::receiveSignal(cComponent*, simsignal_t signal, cObject *obj, cObject*)
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
        } else {
            EV_ERROR << "received signal has no VaObject";
        }
    }
}

omnetpp::SimTime VamSensor::getValidityPeriod() const
{
    return mValidityPeriod;
}

const std::string& VamSensor::getSensorCategory() const
{
    static const std::string category = "VA";
    return category;
}

SensorDetection VamSensor::detectObjects() const
{
    // return empty sensor detection because VAM objects are added upon VAM reception signal
    SensorDetection detection;
    return detection;
}

} // namespace artery
