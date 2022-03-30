#include <artery/application/CPObject.h>
#include <omnetpp.h>
#include <cassert>
#include <artery/cpm/cpm.hpp>
#include <artery/cpm/compiled/ListOfPerceivedObjectContainer.h>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include "Constants.hpp"
#include <vanetza/facilities/cam_functions.hpp>
#include "VehicleDataProvider.h"


namespace artery
{

using namespace artery::cpm;

Register_Abstract_Class(CPObject)
	
CPObject::CPObject(artery::cpm::Cpm&& cpm) :
    m_cpm_wrapper(std::make_shared<Cpm>(std::move(cpm)))
{
}

CPObject& CPObject::operator=(Cpm&& cpm)
{
    m_cpm_wrapper = std::make_shared<Cpm>(std::move(cpm));
    return *this;
}

CPObject::CPObject(const Cpm& cpm) :
    m_cpm_wrapper(std::make_shared<Cpm>(cpm))
{
}

CPObject& CPObject::operator=(const Cpm& cpm)
{
    m_cpm_wrapper = std::make_shared<Cpm>(cpm);
    return *this;
}

CPObject::CPObject(const std::shared_ptr<const Cpm>& ptr) :
    m_cpm_wrapper(ptr)
{
    assert(m_cpm_wrapper);
}

CPObject& CPObject::operator=(const std::shared_ptr<const Cpm>& ptr)
{
    m_cpm_wrapper = ptr;
    assert(m_cpm_wrapper);
    return *this;
}

std::shared_ptr<const Cpm> CPObject::shared_ptr() const
{
    assert(m_cpm_wrapper);
    return m_cpm_wrapper;
}

const artery::cpm::Cpm& CPObject::asn1() const
{
    return *m_cpm_wrapper;
}

using namespace omnetpp;

class CpmStationIdResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto cpm = dynamic_cast<CPObject*>(object)) {
            const auto id = cpm->asn1()->header.stationID;
            fire(this, t, id, details);
        }
    }
};

Register_ResultFilter("cpmStationId", CpmStationIdResultFilter)


class CpmGenerationDeltaTimeResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto cpm = dynamic_cast<CPObject*>(object)) {
            const auto genDeltaTime = cpm->asn1()->cpm.generationDeltaTime;
            fire(this, t, genDeltaTime, details);
        }
    }
};

Register_ResultFilter("cpmGenerationDeltaTime", CpmGenerationDeltaTimeResultFilter)



class NbObjectsResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto cpm = dynamic_cast<CPObject*>(object)) {
            long nb = 0;
            if(cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer)
                nb = cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer->list.count;

            fire(this, t, nb, details);
        }
    }
};

Register_ResultFilter("nbObjects", NbObjectsResultFilter)

class ObjectsSentIdFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto cpm = dynamic_cast<CPObject*>(object)) {
            long nb = 0;
            if(cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer){
                nb = cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer->list.count;
                for(int i = 0 ; i < nb ; i++){
                    const auto id = cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer->list.array[i]->objectID;
                    fire(this, t, id, details);
                }
            }
        }
    }
};

Register_ResultFilter("objIdSent", ObjectsSentIdFilter)


class ObjectsSentTimeFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto cpm = dynamic_cast<CPObject*>(object)) {
            long nb = 0;
            if(cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer){
                nb = cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer->list.count;
                for(int i = 0 ; i < nb ; i++){
                    const auto genDeltaTime = cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer->list.array[i]->timeOfMeasurement;
                    fire(this, t, genDeltaTime, details);

                }
            }
        }
    }
};

Register_ResultFilter("objTimeSent", ObjectsSentTimeFilter)


class sizeCPMFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto cpm = dynamic_cast<CPObject*>(object)) {

            fire(this, t, cpm->asn1().size(), details);
        }
    }
};

Register_ResultFilter("sizeCPM", sizeCPMFilter)



class headingObjectFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto cpm = dynamic_cast<CPObject*>(object)) {
            long nb = 0;

            OriginatingVehicleContainer_t originVeh = cpm->asn1()->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer;
            vanetza::units::Angle headingReceived(originVeh.heading.headingValue * Constants::decidegree);
            boost::units::quantity<boost::units::degree::plane_angle> headingDegreeSender {headingReceived};
            fire(this, t, headingDegreeSender.value(), details);

            if(cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer){

                nb = cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer->list.count;
                for(int i = 0 ; i < nb ; i++){
                    const auto objCont = cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer->list.array[i];
                    boost::units::quantity<boost::units::si::velocity> speedX(objCont->xSpeed.value * Constants::centimeter_per_second);
                    boost::units::quantity<boost::units::si::velocity> speedY(objCont->ySpeed.value * Constants::centimeter_per_second);

                    boost::units::quantity<boost::units::si::plane_angle> heading = VehicleDataProvider::computeHeading(speedX, speedY);
                    boost::units::quantity<boost::units::degree::plane_angle> headingDegree { heading };

                    fire(this, t, headingDegree.value(), details);
                }
            }
        }
    }
};

Register_ResultFilter("headingObject", headingObjectFilter)


class speedObjectFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto cpm = dynamic_cast<CPObject*>(object)) {
            long nb = 0;

            OriginatingVehicleContainer_t originVeh = cpm->asn1()->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer;
            vanetza::units::Velocity speedReceived(originVeh.speed.speedValue * Constants::centimeter_per_second);
            fire(this, t, speedReceived.value(), details);

            if(cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer){

                nb = cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer->list.count;
                for(int i = 0 ; i < nb ; i++){

                    const auto objCont = cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer->list.array[i];
                    vanetza::units::Velocity speedX(objCont->xSpeed.value * Constants::centimeter_per_second);
                    vanetza::units::Velocity speedY(objCont->ySpeed.value * Constants::centimeter_per_second);
                    vanetza::units::Velocity speedReceived = boost::units::sqrt(boost::units::pow<2>(speedX) + boost::units::pow<2>(speedY));

                    fire(this, t, speedReceived.value(), details);
                }
            }
        }
    }
};

Register_ResultFilter("speedObject", speedObjectFilter)


class distXObjectFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto cpm = dynamic_cast<CPObject*>(object)) {
            long nb = 0;
            if(cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer){
                nb = cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer->list.count;
                for(int i = 0 ; i < nb ; i++){

                    const auto objCont = cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer->list.array[i];
                    fire(this, t, (double) objCont->xDistance.value / DistanceValue_oneMeter, details);
                }
            }
        }
    }
};

Register_ResultFilter("distXObject", distXObjectFilter)



class distYObjectFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto cpm = dynamic_cast<CPObject*>(object)) {
            long nb = 0;
            if(cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer){
                nb = cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer->list.count;
                for(int i = 0 ; i < nb ; i++){

                    const auto objCont = cpm->asn1()->cpm.cpmParameters.perceivedObjectContainer->list.array[i];
                    fire(this, t, (double) objCont->yDistance.value  / DistanceValue_oneMeter, details);
                }
            }
        }
    }
};

Register_ResultFilter("distYObject", distYObjectFilter)


} // namespace artery
