#ifndef ENVMOD_VRUSENSOR_H_
#define ENVMOD_VRUSENSOR_H_

#include "artery/envmod/sensor/BaseSensor.h"
#include <vanetza/asn1/vam.hpp>
#include <artery/cpm/cpm.hpp>
#include <omnetpp/clistener.h>

namespace artery
{

class IdentityRegistry;

class VRUsensor : public BaseSensor, public omnetpp::cListener
{
public:
    void measurement() override;
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject* = nullptr) override;
    omnetpp::SimTime getValidityPeriod() const override;
    SensorPosition position() const override { return SensorPosition::VIRTUAL; }
    const std::string& getSensorCategory() const override;
    const std::string getSensorName() const override {  return mSensorName; }
    void setSensorName(const std::string& name) override { mSensorName = name; }
    SensorDetection detectObjects() const override;

protected:
    void initialize() override;
    void finish() override;

private:
    IdentityRegistry* mIdentityRegistry;
    omnetpp::SimTime mValidityPeriod;
    std::string mSensorName;
};

} // namespace artery

#endif /* ENVMOD_VRUSENSOR_H_ */
