//
// Created by rosk on 13.02.19.
//

#include "InfoObject.h"

namespace artery
{

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
    boost::units::quantity<U> v { q };
    return std::round(v.value());
}

InfoObject::InfoObject(){}

InfoObject::InfoObject(bool hasV2XCapabilities, LocalEnvironmentModel::TrackingTime lastTrackingTime, Identifier_t& id,
        vanetza::units::Angle lastCpmHeading, bool headAvailable, Position lastCpmPosition,
        vanetza::units::Velocity lastCpmSpeed):
        mHasV2XCapabilities(hasV2XCapabilities), mLastTrackingTime(lastTrackingTime), mNumberOfSensors(1), mSensorsId(id),
        mLastCpmHeading(lastCpmHeading), mHeadingAvailable(headAvailable), mLastCpmPosition(lastCpmPosition),
        mLastCpmSpeed(lastCpmSpeed)
{
}

std::ostream& operator<<(std::ostream& os, InfoObject& infoObj){
    os << "Info of the object: " << std::endl;
    os << "\tHas V2X capabilities: " << infoObj.getHasV2XCapabilities() << std:: endl;
    os << "\tLast Tracked time: " << infoObj.getLastTrackingTime().last() << std::endl;
    os << "\tNumber of sensors in the detection: " << infoObj.getNumberOfSensors() << std::endl;
    os << "\tSensor used in the detection: " << infoObj.getSensorId() << std::endl;
    boost::units::quantity<boost::units::degree::plane_angle> heading { infoObj.getLastHeading() };
    os << "\tLast heading perceived: " << heading.value() << std::endl;
    os << "\tLast position perceived: (" << infoObj.getLastPosition().x / boost::units::si::meter
       << ", " << infoObj.getLastPosition().y /  boost::units::si::meter << ")" << std::endl;
    os << "\tLast velocity perceived: " << infoObj.getLastVelocity() / vanetza::units::si::meter_per_second << std::endl;
}

void InfoObject::printObjectsReceivedMap(ObjectsReceivedMap objReceived){
    std::cout << "Number of objects: " << objReceived.size() << std::endl;
    for(auto& mapObj : objReceived){
        std::cout << "id Object: " << mapObj.first << std::endl;
        std::cout << mapObj.second << std::endl;
    }
}

void InfoObject::printObjectsToSendMap(ObjectsToSendMap objMap){
    std::cout << "Number of objects: " << objMap.size() << std::endl;
    for(auto& mapObj : objMap){
        std::cout << "id Object: " << mapObj.first.lock()->getPersonData().station_id() << std::endl;
        std::cout << mapObj.second << std::endl;
    }
}


} // end namespace artery