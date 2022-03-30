//
// Created by rosk on 13.02.19.
//

#ifndef ARTERY_INFOOBJECT_H
#define ARTERY_INFOOBJECT_H

#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Geometry.h"
#include <artery/cpm/cpm.hpp>
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/envmod/EnvironmentModelPerson.h>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>
#include "artery/application/Constants.hpp"


namespace artery
{

class InfoObject
{
public:
    using ObjectsToSendMap = std::map<const LocalEnvironmentModel::Person, InfoObject, std::owner_less<LocalEnvironmentModel::Person>>;
    using ObjectsReceivedMap = std::map<const uint32_t, InfoObject>;

    InfoObject();
    InfoObject(bool, LocalEnvironmentModel::TrackingTime, Identifier_t&,
                    vanetza::units::Angle, bool, Position, vanetza::units::Velocity);

    bool getHasV2XCapabilities() const { return mHasV2XCapabilities;}
    LocalEnvironmentModel::TrackingTime& getLastTrackingTime() { return mLastTrackingTime; }
    size_t getNumberOfSensors() const { return mNumberOfSensors; }
    Identifier_t getSensorId() const { return mSensorsId; }
    vanetza::units::Angle getLastHeading(){return mLastCpmHeading;}
    Position getLastPosition(){return mLastCpmPosition;}
    vanetza::units::Velocity getLastVelocity(){return mLastCpmSpeed;}
    bool getHeadingAvailable(){return mHeadingAvailable;}
    void setLastTrackingTime(LocalEnvironmentModel::TrackingTime lastTrackingTime) {mLastTrackingTime = lastTrackingTime;}
    void setNumberOfSensors(size_t numberOfSensors) {mNumberOfSensors = numberOfSensors;}
    void setSensorId(Identifier_t& id) {mSensorsId = id;}
    void setHasV2XCapabilities(bool hasV2XCapabilities) {mHasV2XCapabilities = hasV2XCapabilities;}
    static void printObjectsReceivedMap(ObjectsReceivedMap objReceived);
    static void printObjectsToSendMap(ObjectsToSendMap objMap);
    void setLastTimeSent(omnetpp::SimTime time) { mLastTimeSent = time;}
    omnetpp::SimTime getLastTimeSent() { return mLastTimeSent; }


private:
    LocalEnvironmentModel::TrackingTime mLastTrackingTime;
    size_t mNumberOfSensors;
    Identifier_t mSensorsId;
    vanetza::units::Angle mLastCpmHeading;
    Position mLastCpmPosition;
    vanetza::units::Velocity mLastCpmSpeed;
    bool mHasV2XCapabilities = false;
    bool mHeadingAvailable = false;
    omnetpp::SimTime mLastTimeSent;
};

std::ostream& operator<<(std::ostream& os, InfoObject& infoObj);

} //end namespace artery

#endif //ARTERY_INFOOBJECTD_H
